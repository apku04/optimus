#!/usr/bin/env python3
"""MPC visual servo with direct Klipper socket connection.

This is the same MPC algorithm as head_tracker_mpc.py but uses direct
communication to Klipper via Unix socket instead of Moonraker HTTP API.

Key improvement: command latency drops from 10-50ms to ~0.1ms, allowing
for much faster, smoother control updates.

Usage
-----
  python3 rpi/head_tracker_mpc_direct.py
  python3 rpi/head_tracker_mpc_direct.py --dry-run

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
from collections import deque

sys.path.insert(0, str(Path(__file__).parent.parent / "shared"))
from klipper_direct import KlipperDirect


# -----------------------------------------------------------------------------
# UDP packet parsing
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

    def solve(self, x0: np.ndarray, e_meas: np.ndarray, e_vel: Optional[np.ndarray] = None) -> np.ndarray:
        """Solve MPC given current state x0 and measured image error e_meas.

        Args:
            x0: current state [pan_pos, pan_vel, tilt_pos, tilt_vel]
            e_meas: current image error [ex, ey] (normalized, from Jetson)
            e_vel: target velocity in image [ex_dot, ey_dot] for extrapolation (optional)

        Returns:
            u0: optimal first control [pan_accel, tilt_accel]
        """
        N, nu = self.N, 2

        # Target position at k=0
        e_target = e_meas - self.C @ x0
        
        # Build predicted target trajectory over horizon
        # If we have target velocity, extrapolate linearly (like the paper)
        if e_vel is not None and (abs(e_vel[0]) > 0.01 or abs(e_vel[1]) > 0.01):
            E_target_bar = np.zeros(N * 2)
            for k in range(N):
                # Linear extrapolation: e_target(k) = e_target(0) + k * dt * e_vel
                e_extrap = e_target + (k + 1) * self.dt * e_vel
                E_target_bar[k * 2] = e_extrap[0]
                E_target_bar[k * 2 + 1] = e_extrap[1]
        else:
            # No velocity info: assume stationary target
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

        # Velocity limits
        vel_min = np.tile(-self.vel_max, N)
        vel_max_arr = np.tile(self.vel_max, N)

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
            vel_viol_hi = np.maximum(0.0, vel_pred - vel_max_arr)
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
# Main tracker loop with direct Klipper connection
# -----------------------------------------------------------------------------
def run(config_path: Path, *, dry_run: bool) -> int:
    cfg = json.loads(config_path.read_text())

    receiver = UdpFaceReceiver(int(cfg["network"]["udp_port"]))
    
    # Use direct Klipper connection instead of Moonraker
    klipper = KlipperDirect()
    
    if not dry_run:
        if not klipper.connect():
            print("[ERROR] Failed to connect to Klipper socket")
            return 2
        
        # Enable steppers
        klipper.enable_stepper("stepper_0", True)
        klipper.enable_stepper("stepper_1", True)
        klipper.set_position("stepper_0", 0)
        klipper.set_position("stepper_1", 0)

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
    
    q_error = float(mpc_cfg.get("q_error", 5000.0))
    r_accel = float(mpc_cfg.get("r_accel", 0.1))
    q_terminal = float(mpc_cfg.get("q_terminal", 10000.0))

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
        klipper.move_stepper("stepper_0", pan_pos, speed=pan_cfg.get("speed", 50))
        klipper.move_stepper("stepper_1", tilt_pos, speed=tilt_cfg.get("speed", 40))

    # Filtered measurement
    ex_f, ey_f = 0.0, 0.0
    filt_init = False

    # Target trajectory extrapolation (like the paper)
    ex_hist = deque(maxlen=5)
    ey_hist = deque(maxlen=5)
    ex_vel = 0.0
    ey_vel = 0.0

    last_face_time = time.time()
    last_print = 0.0
    cmd_count = 0
    total_cmd_time = 0.0

    print("=" * 72)
    print("MPC HEAD TRACKER (DIRECT KLIPPER)")
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
                            klipper.move_pan_tilt(pan_pos, tilt_pos, pan_speed=15, tilt_speed=12)
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
            
            # Estimate target velocity (for setpoint extrapolation like the paper)
            ex_hist.append(ex_f)
            ey_hist.append(ey_f)
            if len(ex_hist) >= 3:
                # Simple finite difference: v ≈ (e[k] - e[k-2]) / (2*dt)
                ex_vel = (ex_hist[-1] - ex_hist[-3]) / (2 * dt)
                ey_vel = (ey_hist[-1] - ey_hist[-3]) / (2 * dt)
                # Smooth velocity estimate
                ex_vel = 0.7 * ex_vel + 0.3 * ex_vel  # Could track prev ex_vel for smoothing
                ey_vel = 0.7 * ey_vel + 0.3 * ey_vel
            else:
                ex_vel, ey_vel = 0.0, 0.0

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

            # Current state
            x0 = np.array([pan_pos, pan_vel, tilt_pos, tilt_vel])
            e_meas = np.array([ex_f, ey_f])
            e_vel_vec = np.array([ex_vel, ey_vel])

            # Solve MPC with target trajectory extrapolation
            t_solve_start = time.time()
            u_opt = mpc.solve(x0, e_meas, e_vel_vec)
            t_solve = time.time() - t_solve_start

            # Apply first control (accel) to update velocity
            pan_accel, tilt_accel = u_opt[0], u_opt[1]

            # Integrate: new velocity
            pan_vel_new = np.clip(pan_vel + pan_accel * dt, -vel_max_pan, vel_max_pan)
            tilt_vel_new = np.clip(tilt_vel + tilt_accel * dt, -vel_max_tilt, vel_max_tilt)

            # Integrate: new position
            pan_pos_new = np.clip(pan_pos + pan_vel_new * dt, pan_lim[0], pan_lim[1])
            tilt_pos_new = np.clip(tilt_pos + tilt_vel_new * dt, tilt_lim[0], tilt_lim[1])

            step_pan = pan_pos_new - pan_pos
            step_tilt = tilt_pos_new - tilt_pos

            # Skip tiny moves
            if abs(step_pan) < 0.005 and abs(step_tilt) < 0.005:
                pan_vel, tilt_vel = pan_vel_new, tilt_vel_new
                time.sleep(loop_dt)
                continue

            # Compute speed for Klipper (deg/s) - use velocity magnitude
            pan_speed = max(1.0, min(abs(pan_vel_new) * 1.5, vel_max_pan))
            tilt_speed = max(1.0, min(abs(tilt_vel_new) * 1.5, vel_max_tilt))

            if dry_run:
                now = time.time()
                if now - last_print > 0.3:
                    print(
                        f"[MPC] e=({ex_f:+.3f},{ey_f:+.3f}) u=({pan_accel:+.1f},{tilt_accel:+.1f}) "
                        f"v=({pan_vel_new:+.1f},{tilt_vel_new:+.1f}) pos=({pan_pos_new:+.2f},{tilt_pos_new:+.2f}) "
                        f"solve={t_solve*1000:.1f}ms"
                    )
                    last_print = now
            else:
                # Send commands via direct socket (very fast!)
                t_cmd_start = time.time()
                klipper.move_pan_tilt(pan_pos_new, tilt_pos_new, pan_speed=pan_speed, tilt_speed=tilt_speed)
                t_cmd = time.time() - t_cmd_start
                cmd_count += 1
                total_cmd_time += t_cmd

            # Update state
            pan_pos, tilt_pos = pan_pos_new, tilt_pos_new
            pan_vel, tilt_vel = pan_vel_new, tilt_vel_new

            # Periodic status
            now = time.time()
            if now - last_print > 1.0:
                avg_cmd = (total_cmd_time / cmd_count * 1000) if cmd_count > 0 else 0
                print(
                    f"[STAT] e=({ex_f:+.3f},{ey_f:+.3f}) v=({pan_vel:+.1f},{tilt_vel:+.1f}) "
                    f"pos=({pan_pos:+.2f},{tilt_pos:+.2f}) solve={t_solve*1000:.1f}ms cmd={avg_cmd:.2f}ms"
                )
                last_print = now
                cmd_count = 0
                total_cmd_time = 0.0

            # Sleep to maintain loop rate
            elapsed = time.time() - loop_start
            time.sleep(max(0.0, loop_dt - elapsed))

    except KeyboardInterrupt:
        print("\n[STOP] exiting")
        if not dry_run:
            klipper.emergency_stop()
            klipper.disconnect()
        return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="MPC visual servo (direct Klipper)")
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
