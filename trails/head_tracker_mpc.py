#!/usr/bin/env python3
"""Model Predictive Control (MPC) visual servo for pan/tilt head tracking.

This is a fresh, from-scratch implementation based on the standard MPC
formulation used in papers like:
- Nebeluk et al., "Predictive tracking of an object by a pan–tilt camera",
  Nonlinear Dynamics (2023)
- Yang et al., "Online Predictive Visual Servo Control for Constrained
  Target Tracking", Drones (2024)

Core idea
---------
Instead of reactive "error → move", MPC plans a sequence of moves over a
finite horizon that minimizes predicted image error while respecting
constraints (max rate, max accel, joint limits).

State-space model (per axis, then stacked for 2-axis)
-----------------------------------------------------
We model each axis as a 2nd-order discrete-time system:
  x_k = [θ, θ̇]^T   (position, velocity)
  u_k = Δθ̇        (acceleration command, i.e., rate change)

Discrete dynamics (Euler, dt = sample period):
  θ_{k+1}   = θ_k + dt * θ̇_k
  θ̇_{k+1}  = θ̇_k + dt * u_k

Or in matrix form:
  x_{k+1} = A_d x_k + B_d u_k

The image error is modeled as:
  e = C_cam * [θ_pan, θ_tilt]^T + e_target
where C_cam is your calibrated 2×2 matrix (dx_dpan, dx_dtilt, dy_dpan, dy_dtilt).

MPC cost (over horizon N)
-------------------------
  J = Σ_{k=0}^{N-1} [ e_k^T Q e_k + u_k^T R u_k ] + e_N^T Q_f e_N

We minimize J subject to:
  - dynamics
  - |θ̇| ≤ v_max  (rate limit)
  - |u| ≤ a_max  (accel limit)
  - θ_min ≤ θ ≤ θ_max (joint limits)

Implementation
--------------
We form a dense QP and solve it with scipy. For real-time on RPi (~30 Hz),
horizon N=10–20 is tractable.

Usage
-----
  python3 rpi/head_tracker_mpc.py
  python3 rpi/head_tracker_mpc.py --dry-run

"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, str(Path(__file__).parent.parent / "shared"))
from motor_control import MotorController


# -----------------------------------------------------------------------------
# UDP packet parsing (same as other trackers)
# -----------------------------------------------------------------------------
@dataclass
class FacePacket:
    x: float
    y: float
    detected: bool
    confidence: float
    stability: float
    timestamp: float


class UdpFaceReceiver:
    def __init__(self, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)

    def _parse(self, msg: str) -> Optional[FacePacket]:
        msg = msg.strip()
        if "," in msg and not msg.startswith("{"):
            parts = msg.split(",")
            if len(parts) >= 2:
                try:
                    return FacePacket(
                        x=float(parts[0]),
                        y=float(parts[1]),
                        detected=True,
                        confidence=1.0,
                        stability=1.0,
                        timestamp=time.time(),
                    )
                except ValueError:
                    return None

        if not msg.startswith("{"):
            return None

        try:
            data = json.loads(msg)
        except json.JSONDecodeError:
            return None

        detected = bool(data.get("detected", data.get("face_detected", False)))
        x = data.get("x", data.get("x_offset", data.get("offset_x", 0.0)))
        y = data.get("y", data.get("y_offset", data.get("offset_y", 0.0)))

        try:
            x = float(x)
            y = float(y)
        except (TypeError, ValueError):
            return None

        return FacePacket(
            x=np.clip(x, -1.0, 1.0),
            y=np.clip(y, -1.0, 1.0),
            detected=detected,
            confidence=float(data.get("confidence", 1.0) or 0.0),
            stability=float(data.get("stability", 1.0) or 0.0),
            timestamp=float(data.get("timestamp", time.time()) or time.time()),
        )

    def get_latest(self) -> Optional[FacePacket]:
        latest: Optional[FacePacket] = None
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break
            pkt = self._parse(data.decode("utf-8", errors="ignore"))
            if pkt is not None:
                latest = pkt
        return latest


# -----------------------------------------------------------------------------
# MPC solver
# -----------------------------------------------------------------------------
class PanTiltMPC:
    """Linear MPC for 2-axis pan/tilt visual servo.

    State vector (4 elements):
        x = [pan_pos, pan_vel, tilt_pos, tilt_vel]

    Control vector (2 elements):
        u = [pan_accel, tilt_accel]

    Measurement (2 elements):
        e = C_cam @ [pan_pos, tilt_pos] + e_target
    where e_target is the face offset when motors are at origin (what we measure).
    """

    def __init__(
        self,
        dt: float,
        horizon: int,
        C_cam: np.ndarray,
        Q: np.ndarray,
        R: np.ndarray,
        Q_f: np.ndarray,
        pan_lim: tuple[float, float],
        tilt_lim: tuple[float, float],
        vel_max_pan: float,
        vel_max_tilt: float,
        accel_max_pan: float,
        accel_max_tilt: float,
    ):
        self.dt = dt
        self.N = horizon
        self.C_cam = C_cam  # 2×2

        # State-space matrices for one axis: x = [pos, vel], u = accel
        # x_{k+1} = A1 x_k + B1 u_k
        A1 = np.array([[1.0, dt], [0.0, 1.0]])
        B1 = np.array([[0.5 * dt * dt], [dt]])

        # Stack for 2-axis (pan, tilt)
        self.A = np.block([[A1, np.zeros((2, 2))], [np.zeros((2, 2)), A1]])  # 4×4
        self.B = np.block([[B1, np.zeros((2, 1))], [np.zeros((2, 1)), B1]])  # 4×2

        # Output matrix: e = C_cam @ [pan_pos, tilt_pos]
        # C extracts positions from state: [pan_pos, tilt_pos] = C_pos @ x
        self.C_pos = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]])  # 2×4
        self.C = C_cam @ self.C_pos  # 2×4

        self.Q = Q  # 2×2 (error weight)
        self.R = R  # 2×2 (control weight)
        self.Q_f = Q_f  # 2×2 (terminal weight)

        # Limits
        self.pan_min, self.pan_max = pan_lim
        self.tilt_min, self.tilt_max = tilt_lim
        self.vel_max = np.array([vel_max_pan, vel_max_tilt])
        self.accel_max = np.array([accel_max_pan, accel_max_tilt])

        # Precompute prediction matrices for QP
        self._build_prediction_matrices()

    def _build_prediction_matrices(self):
        """Build matrices such that:
        X = Sx @ x0 + Su @ U
        E = C_bar @ X + E_target_bar
        where X stacks states over horizon, U stacks controls.
        """
        N = self.N
        nx, nu = 4, 2

        # Sx: X = Sx @ x0  (free response)
        Sx = np.zeros((N * nx, nx))
        Apow = np.eye(nx)
        for k in range(N):
            Apow = Apow @ self.A if k > 0 else self.A
            Sx[k * nx : (k + 1) * nx, :] = Apow

        # Su: X = Su @ U  (forced response)
        Su = np.zeros((N * nx, N * nu))
        for k in range(N):
            for j in range(k + 1):
                Apow = np.linalg.matrix_power(self.A, k - j)
                Su[k * nx : (k + 1) * nx, j * nu : (j + 1) * nu] = Apow @ self.B

        self.Sx = Sx
        self.Su = Su

        # C_bar: stacked output matrix
        C_bar = np.zeros((N * 2, N * nx))
        for k in range(N):
            C_bar[k * 2 : (k + 1) * 2, k * nx : (k + 1) * nx] = self.C
        self.C_bar = C_bar

        # Q_bar, R_bar: stacked cost matrices
        Q_bar = np.zeros((N * 2, N * 2))
        for k in range(N - 1):
            Q_bar[k * 2 : (k + 1) * 2, k * 2 : (k + 1) * 2] = self.Q
        Q_bar[(N - 1) * 2 : N * 2, (N - 1) * 2 : N * 2] = self.Q_f
        self.Q_bar = Q_bar

        R_bar = np.zeros((N * nu, N * nu))
        for k in range(N):
            R_bar[k * nu : (k + 1) * nu, k * nu : (k + 1) * nu] = self.R
        self.R_bar = R_bar

    def solve(self, x0: np.ndarray, e_meas: np.ndarray) -> np.ndarray:
        """Solve MPC given current state x0 and measured image error e_meas.

        Args:
            x0: current state [pan_pos, pan_vel, tilt_pos, tilt_vel]
            e_meas: current image error [ex, ey] (normalized, from Jetson)

        Returns:
            u0: optimal first control [pan_accel, tilt_accel]
        """
        N, nu = self.N, 2

        # Predicted target error (assuming target stationary, e_target constant)
        # e_k = C @ x_k + e_target
        # At k=0: e_meas = C @ x0 + e_target  =>  e_target = e_meas - C @ x0
        e_target = e_meas - self.C @ x0
        E_target_bar = np.tile(e_target, N)

        # Predicted error: E = C_bar @ (Sx @ x0 + Su @ U) + E_target_bar
        # E = (C_bar @ Sx) @ x0 + (C_bar @ Su) @ U + E_target_bar
        Phi = self.C_bar @ self.Sx  # N*2 × 4
        Psi = self.C_bar @ self.Su  # N*2 × N*2

        # Cost: J = E^T Q_bar E + U^T R_bar U
        # E = Psi @ U + (Phi @ x0 + E_target_bar)
        # Let c = Phi @ x0 + E_target_bar
        c = Phi @ x0 + E_target_bar

        # J = (Psi U + c)^T Q_bar (Psi U + c) + U^T R_bar U
        # J = U^T (Psi^T Q_bar Psi + R_bar) U + 2 c^T Q_bar Psi U + c^T Q_bar c
        # H = Psi^T Q_bar Psi + R_bar
        # g = Psi^T Q_bar c
        H = Psi.T @ self.Q_bar @ Psi + self.R_bar
        g = Psi.T @ self.Q_bar @ c

        # Symmetrize H (numerical)
        H = 0.5 * (H + H.T) + 1e-6 * np.eye(H.shape[0])

        # Bounds on U (accel limits)
        u_lb = np.tile(-self.accel_max, N)
        u_ub = np.tile(self.accel_max, N)

        # State constraint helpers
        # X = Sx @ x0 + Su @ U
        # We need to constrain:
        # - positions: pan_min ≤ pan_pos ≤ pan_max, tilt_min ≤ tilt_pos ≤ tilt_max
        # - velocities: -vel_max ≤ vel ≤ vel_max

        # Extract position rows from Sx, Su
        pos_indices = [k * 4 + i for k in range(N) for i in [0, 2]]  # pan_pos, tilt_pos
        vel_indices = [k * 4 + i for k in range(N) for i in [1, 3]]  # pan_vel, tilt_vel

        Sx_pos = self.Sx[pos_indices, :]
        Su_pos = self.Su[pos_indices, :]
        Sx_vel = self.Sx[vel_indices, :]
        Su_vel = self.Su[vel_indices, :]

        # Position limits
        pos_min = np.tile([self.pan_min, self.tilt_min], N)
        pos_max = np.tile([self.pan_max, self.tilt_max], N)
        # Sx_pos @ x0 + Su_pos @ U >= pos_min  =>  Su_pos @ U >= pos_min - Sx_pos @ x0
        # Sx_pos @ x0 + Su_pos @ U <= pos_max  =>  Su_pos @ U <= pos_max - Sx_pos @ x0

        # Velocity limits
        vel_min = np.tile(-self.vel_max, N)
        vel_max = np.tile(self.vel_max, N)

        def cost(U):
            return 0.5 * U @ H @ U + g @ U

        def grad(U):
            return H @ U + g

        # Use scipy's L-BFGS-B with bounds (fast for box constraints)
        # For state constraints, we'll use a soft penalty (keeps it fast)
        penalty_weight = 1000.0

        def cost_with_penalty(U):
            J = 0.5 * U @ H @ U + g @ U

            # Position penalty
            pos_pred = Sx_pos @ x0 + Su_pos @ U
            pos_viol_lo = np.maximum(0.0, pos_min - pos_pred)
            pos_viol_hi = np.maximum(0.0, pos_pred - pos_max)
            J += penalty_weight * (np.sum(pos_viol_lo**2) + np.sum(pos_viol_hi**2))

            # Velocity penalty
            vel_pred = Sx_vel @ x0 + Su_vel @ U
            vel_viol_lo = np.maximum(0.0, vel_min - vel_pred)
            vel_viol_hi = np.maximum(0.0, vel_pred - vel_max)
            J += penalty_weight * (np.sum(vel_viol_lo**2) + np.sum(vel_viol_hi**2))

            return J

        # Initial guess: zero
        U0 = np.zeros(N * nu)

        # Bounds for accel
        bounds = [(u_lb[i], u_ub[i]) for i in range(N * nu)]

        result = minimize(
            cost_with_penalty,
            U0,
            method="L-BFGS-B",
            bounds=bounds,
            options={"maxiter": 50, "ftol": 1e-6},
        )

        U_opt = result.x
        u0 = U_opt[:nu]  # First control
        return u0


# -----------------------------------------------------------------------------
# Main tracker loop
# -----------------------------------------------------------------------------
def run(config_path: Path, *, dry_run: bool) -> int:
    cfg = json.loads(config_path.read_text())

    receiver = UdpFaceReceiver(int(cfg["network"]["udp_port"]))
    motors = MotorController(str(config_path))

    if not dry_run and not motors.is_ready():
        print("[ERROR] Moonraker not ready")
        return 2

    motors_cfg = cfg.get("motors", {})
    pan_cfg = motors_cfg.get("pan", {})
    tilt_cfg = motors_cfg.get("tilt", {})

    track_cfg = cfg.get("tracking", {})
    mpc_cfg = cfg.get("tracking_mpc", {})
    dec = cfg.get("tracking_decoupled", {})

    # Calibration matrix: C_cam maps motor degrees to image offset
    dx_dpan = float(dec.get("dx_dpan", 0.17))
    dx_dtilt = float(dec.get("dx_dtilt", 0.0))
    dy_dpan = float(dec.get("dy_dpan", 0.0))
    dy_dtilt = float(dec.get("dy_dtilt", 0.036))

    C_cam = np.array([[dx_dpan, dx_dtilt], [dy_dpan, dy_dtilt]])

    det = np.linalg.det(C_cam)
    if abs(det) < 1e-6:
        print("[ERROR] Calibration matrix near-singular; run --calibrate on decoupled tracker")
        return 3

    # MPC parameters
    dt = float(mpc_cfg.get("dt", 0.033))  # ~30 Hz
    horizon = int(mpc_cfg.get("horizon", 15))
    
    # Cost weights - note: image error is normalized [0,1], motor angles are degrees
    # We scale q_error to make error reduction aggressive enough
    q_error = float(mpc_cfg.get("q_error", 5000.0))  # error weight (high = aggressive)
    r_accel = float(mpc_cfg.get("r_accel", 0.1))  # accel weight (low = allow fast response)
    q_terminal = float(mpc_cfg.get("q_terminal", 10000.0))  # terminal weight

    Q = np.diag([q_error, q_error])
    R = np.diag([r_accel, r_accel])
    Q_f = np.diag([q_terminal, q_terminal])

    # Limits
    pan_lim = (float(pan_cfg.get("min", -22.0)), float(pan_cfg.get("max", 22.0)))
    tilt_lim = (float(tilt_cfg.get("min", -3.0)), float(tilt_cfg.get("max", 5.0)))

    vel_max_pan = float(mpc_cfg.get("vel_max_pan", dec.get("max_pan_speed", 60.0)))
    vel_max_tilt = float(mpc_cfg.get("vel_max_tilt", dec.get("max_tilt_speed", 45.0)))

    accel_max_pan = float(mpc_cfg.get("accel_max_pan", 400.0))
    accel_max_tilt = float(mpc_cfg.get("accel_max_tilt", 300.0))

    # Other params
    deadzone = float(mpc_cfg.get("deadzone", track_cfg.get("deadzone", 0.08)))
    min_conf = float(mpc_cfg.get("min_confidence", 0.40))
    meas_ema = float(mpc_cfg.get("meas_ema", 0.7))
    return_to_center_delay = float(track_cfg.get("return_to_center_delay", 3.0))

    # Command pacing
    segment_dt = float(mpc_cfg.get("segment_dt", 0.05))
    queue_limit_s = float(mpc_cfg.get("queue_limit_s", 0.20))

    # Build MPC controller
    mpc = PanTiltMPC(
        dt=dt,
        horizon=horizon,
        C_cam=C_cam,
        Q=Q,
        R=R,
        Q_f=Q_f,
        pan_lim=pan_lim,
        tilt_lim=tilt_lim,
        vel_max_pan=vel_max_pan,
        vel_max_tilt=vel_max_tilt,
        accel_max_pan=accel_max_pan,
        accel_max_tilt=accel_max_tilt,
    )

    # Initial state
    pan_pos = float(pan_cfg.get("center", 0.0))
    tilt_pos = float(tilt_cfg.get("center", 0.0))
    pan_vel = 0.0
    tilt_vel = 0.0

    if not dry_run:
        motors.move_pan(pan_pos, speed=pan_cfg.get("speed"), sync=False)
        motors.move_tilt(tilt_pos, speed=tilt_cfg.get("speed"), sync=False)

    # Filtered measurement
    ex_f, ey_f = 0.0, 0.0
    filt_init = False

    last_face_time = time.time()
    last_cmd_time = 0.0
    commanded_until = 0.0
    last_print = 0.0

    print("=" * 72)
    print("MPC HEAD TRACKER")
    print(f"UDP port: {cfg['network']['udp_port']} | dry_run={dry_run}")
    print(f"dt={dt:.3f}s | horizon={horizon} | deadzone={deadzone}")
    print(f"Q_error={q_error} R_accel={r_accel} Q_term={q_terminal}")
    print(f"vel_max: pan={vel_max_pan:.1f} tilt={vel_max_tilt:.1f} deg/s")
    print(f"accel_max: pan={accel_max_pan:.0f} tilt={accel_max_tilt:.0f} deg/s^2")
    print(f"C_cam=[[{dx_dpan:.4f},{dx_dtilt:.4f}],[{dy_dpan:.4f},{dy_dtilt:.4f}]] det={det:.6f}")
    print("=" * 72)

    loop_dt = dt
    try:
        while True:
            loop_start = time.time()

            pkt = receiver.get_latest()
            have_face = pkt is not None and pkt.detected and pkt.confidence >= min_conf

            if not have_face:
                # Decelerate to stop
                pan_vel *= 0.8
                tilt_vel *= 0.8

                if time.time() - last_face_time > return_to_center_delay:
                    tgt_pan = float(pan_cfg.get("center", 0.0))
                    tgt_tilt = float(tilt_cfg.get("center", 0.0))
                    if abs(pan_pos - tgt_pan) > 0.5 or abs(tilt_pos - tgt_tilt) > 0.5:
                        pan_pos, tilt_pos = tgt_pan, tgt_tilt
                        pan_vel, tilt_vel = 0.0, 0.0
                        if not dry_run:
                            motors.move_pan_tilt(pan_pos, tilt_pos, pan_speed=15, tilt_speed=12, sync=False)
                        print("[CENTER] no face -> returning to center")
                    last_face_time = time.time()

                time.sleep(0.02)
                continue

            last_face_time = time.time()

            # Filter measurement
            ex, ey = float(pkt.x), float(pkt.y)
            if not filt_init:
                ex_f, ey_f = ex, ey
                filt_init = True
            else:
                ex_f = meas_ema * ex_f + (1.0 - meas_ema) * ex
                ey_f = meas_ema * ey_f + (1.0 - meas_ema) * ey

            # Deadzone
            if abs(ex_f) < deadzone and abs(ey_f) < deadzone:
                # In deadzone: decelerate
                pan_vel *= 0.9
                tilt_vel *= 0.9
                if time.time() - last_print > 1.0:
                    print(f"[HOLD] e=({ex_f:+.3f},{ey_f:+.3f}) pos=({pan_pos:+.2f},{tilt_pos:+.2f}) conf={pkt.confidence:.2f}")
                    last_print = time.time()
                time.sleep(loop_dt)
                continue

            # Command pacing
            now = time.time()
            queued_s = max(0.0, commanded_until - now)
            if queued_s >= queue_limit_s:
                time.sleep(loop_dt * 0.5)
                continue

            # Current state
            x0 = np.array([pan_pos, pan_vel, tilt_pos, tilt_vel])
            e_meas = np.array([ex_f, ey_f])

            # Solve MPC
            t_solve_start = time.time()
            u_opt = mpc.solve(x0, e_meas)
            t_solve = time.time() - t_solve_start

            # Apply first control (accel) to update velocity
            pan_accel, tilt_accel = u_opt[0], u_opt[1]

            # Integrate: new velocity
            pan_vel_new = np.clip(pan_vel + pan_accel * dt, -vel_max_pan, vel_max_pan)
            tilt_vel_new = np.clip(tilt_vel + tilt_accel * dt, -vel_max_tilt, vel_max_tilt)

            # Integrate: new position (for segment_dt)
            pan_pos_new = np.clip(pan_pos + pan_vel_new * segment_dt, pan_lim[0], pan_lim[1])
            tilt_pos_new = np.clip(tilt_pos + tilt_vel_new * segment_dt, tilt_lim[0], tilt_lim[1])

            step_pan = pan_pos_new - pan_pos
            step_tilt = tilt_pos_new - tilt_pos

            # Skip tiny moves
            if abs(step_pan) < 0.01 and abs(step_tilt) < 0.01:
                pan_vel, tilt_vel = pan_vel_new, tilt_vel_new
                time.sleep(loop_dt)
                continue

            # Compute speed for Klipper (deg/s)
            pan_speed = max(1.0, min(abs(pan_vel_new) * 1.2, vel_max_pan))
            tilt_speed = max(1.0, min(abs(tilt_vel_new) * 1.2, vel_max_tilt))

            if dry_run:
                if now - last_print > 0.3:
                    print(
                        f"[MPC] e=({ex_f:+.3f},{ey_f:+.3f}) u=({pan_accel:+.1f},{tilt_accel:+.1f}) "
                        f"v=({pan_vel_new:+.1f},{tilt_vel_new:+.1f}) pos=({pan_pos_new:+.2f},{tilt_pos_new:+.2f}) "
                        f"solve={t_solve*1000:.1f}ms"
                    )
                    last_print = now
            else:
                motors.move_pan_tilt(pan_pos_new, tilt_pos_new, pan_speed=pan_speed, tilt_speed=tilt_speed, sync=False)

            # Update state
            pan_pos, tilt_pos = pan_pos_new, tilt_pos_new
            pan_vel, tilt_vel = pan_vel_new, tilt_vel_new

            # Update queue estimate
            est_t = max(abs(step_pan) / max(1.0, pan_speed), abs(step_tilt) / max(1.0, tilt_speed))
            last_cmd_time = now
            commanded_until = max(commanded_until, now) + est_t

            # Periodic status
            if now - last_print > 1.0:
                print(
                    f"[STAT] e=({ex_f:+.3f},{ey_f:+.3f}) v=({pan_vel:+.1f},{tilt_vel:+.1f}) "
                    f"pos=({pan_pos:+.2f},{tilt_pos:+.2f}) solve={t_solve*1000:.1f}ms queued={queued_s:.2f}s"
                )
                last_print = now

            # Sleep to maintain loop rate
            elapsed = time.time() - loop_start
            time.sleep(max(0.0, loop_dt - elapsed))

    except KeyboardInterrupt:
        print("\n[STOP] exiting")
        return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="MPC visual servo head tracker")
    ap.add_argument(
        "--config",
        default=str(Path(__file__).parent.parent / "config.json"),
        help="Path to config.json",
    )
    ap.add_argument("--dry-run", action="store_true", help="Do not move motors")

    args = ap.parse_args()
    return run(Path(args.config), dry_run=bool(args.dry_run))


if __name__ == "__main__":
    raise SystemExit(main())
