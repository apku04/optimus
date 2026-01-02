#!/usr/bin/env python3
"""MPC tracker with move queue blending.

Instead of waiting for each move to complete, we queue multiple small
moves with SYNC=0. Klipper's motion planner will blend them together
into smoother motion via its look-ahead algorithm.

The key insight: when you send moves faster than they execute, Klipper
queues them and can optimize the transitions between moves.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Deque
from collections import deque

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, str(Path(__file__).parent.parent / "shared"))
from klipper_direct import KlipperDirect


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


class PanTiltMPC:
    """Linear MPC for 2-axis pan/tilt visual servo."""

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
        self.C_cam = C_cam

        A1 = np.array([[1.0, dt], [0.0, 1.0]])
        B1 = np.array([[0.5 * dt * dt], [dt]])

        self.A = np.block([[A1, np.zeros((2, 2))], [np.zeros((2, 2)), A1]])
        self.B = np.block([[B1, np.zeros((2, 1))], [np.zeros((2, 1)), B1]])

        self.C_pos = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]])
        self.C = C_cam @ self.C_pos

        self.Q = Q
        self.R = R
        self.Q_f = Q_f

        self.pan_min, self.pan_max = pan_lim
        self.tilt_min, self.tilt_max = tilt_lim
        self.vel_max = np.array([vel_max_pan, vel_max_tilt])
        self.accel_max = np.array([accel_max_pan, accel_max_tilt])

        self._build_prediction_matrices()

    def _build_prediction_matrices(self):
        N = self.N
        nx, nu = 4, 2

        Sx = np.zeros((N * nx, nx))
        Apow = np.eye(nx)
        for k in range(N):
            Apow = Apow @ self.A if k > 0 else self.A
            Sx[k * nx : (k + 1) * nx, :] = Apow

        Su = np.zeros((N * nx, N * nu))
        for k in range(N):
            for j in range(k + 1):
                Apow = np.linalg.matrix_power(self.A, k - j)
                Su[k * nx : (k + 1) * nx, j * nu : (j + 1) * nu] = Apow @ self.B

        self.Sx = Sx
        self.Su = Su

        C_bar = np.zeros((N * 2, N * nx))
        for k in range(N):
            C_bar[k * 2 : (k + 1) * 2, k * nx : (k + 1) * nx] = self.C
        self.C_bar = C_bar

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
        N, nu = self.N, 2

        e_target = e_meas - self.C @ x0
        E_target_bar = np.tile(e_target, N)

        Phi = self.C_bar @ self.Sx
        Psi = self.C_bar @ self.Su

        c = Phi @ x0 + E_target_bar

        H = Psi.T @ self.Q_bar @ Psi + self.R_bar
        g = Psi.T @ self.Q_bar @ c

        H = 0.5 * (H + H.T) + 1e-6 * np.eye(H.shape[0])

        u_lb = np.tile(-self.accel_max, N)
        u_ub = np.tile(self.accel_max, N)

        pos_indices = [k * 4 + i for k in range(N) for i in [0, 2]]
        vel_indices = [k * 4 + i for k in range(N) for i in [1, 3]]

        Sx_pos = self.Sx[pos_indices, :]
        Su_pos = self.Su[pos_indices, :]
        Sx_vel = self.Sx[vel_indices, :]
        Su_vel = self.Su[vel_indices, :]

        pos_min = np.tile([self.pan_min, self.tilt_min], N)
        pos_max = np.tile([self.pan_max, self.tilt_max], N)
        vel_min = np.tile(-self.vel_max, N)
        vel_max_arr = np.tile(self.vel_max, N)

        penalty_weight = 1000.0

        def cost_with_penalty(U):
            J = 0.5 * U @ H @ U + g @ U

            pos_pred = Sx_pos @ x0 + Su_pos @ U
            pos_viol_lo = np.maximum(0.0, pos_min - pos_pred)
            pos_viol_hi = np.maximum(0.0, pos_pred - pos_max)
            J += penalty_weight * (np.sum(pos_viol_lo**2) + np.sum(pos_viol_hi**2))

            vel_pred = Sx_vel @ x0 + Su_vel @ U
            vel_viol_lo = np.maximum(0.0, vel_min - vel_pred)
            vel_viol_hi = np.maximum(0.0, vel_pred - vel_max_arr)
            J += penalty_weight * (np.sum(vel_viol_lo**2) + np.sum(vel_viol_hi**2))

            return J

        U0 = np.zeros(N * nu)
        bounds = [(u_lb[i], u_ub[i]) for i in range(N * nu)]

        result = minimize(
            cost_with_penalty,
            U0,
            method="L-BFGS-B",
            bounds=bounds,
            options={"maxiter": 50, "ftol": 1e-6},
        )

        U_opt = result.x
        return U_opt  # Return FULL trajectory, not just first control


class MoveQueueManager:
    """Manages a queue of moves sent to Klipper with look-ahead."""
    
    def __init__(self, klipper: KlipperDirect, max_queue_time: float = 0.3):
        self.klipper = klipper
        self.max_queue_time = max_queue_time
        self.queued_until = time.time()
        self.last_pan = 0.0
        self.last_tilt = 0.0
        
    def queue_move(
        self,
        pan: float,
        tilt: float,
        pan_speed: float,
        tilt_speed: float,
        move_time: float,
    ) -> bool:
        """Queue a move if queue isn't too full."""
        now = time.time()
        queue_depth = max(0.0, self.queued_until - now)
        
        if queue_depth > self.max_queue_time:
            return False
        
        # Send moves with SYNC=0 to allow queuing
        pan_cmd = f"MANUAL_STEPPER STEPPER=stepper_0 MOVE={pan:.4f} SPEED={pan_speed:.2f} SYNC=0"
        tilt_cmd = f"MANUAL_STEPPER STEPPER=stepper_1 MOVE={tilt:.4f} SPEED={tilt_speed:.2f} SYNC=0"
        
        self.klipper.gcode(pan_cmd + "\n" + tilt_cmd)
        
        self.queued_until = max(self.queued_until, now) + move_time
        self.last_pan = pan
        self.last_tilt = tilt
        
        return True
    
    def get_queue_depth(self) -> float:
        return max(0.0, self.queued_until - time.time())


def run(config_path: Path, *, dry_run: bool) -> int:
    cfg = json.loads(config_path.read_text())

    receiver = UdpFaceReceiver(int(cfg["network"]["udp_port"]))
    
    klipper = KlipperDirect()
    
    if not dry_run:
        if not klipper.connect():
            print("[ERROR] Failed to connect to Klipper socket")
            return 2
        
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

    dx_dpan = float(dec.get("dx_dpan", 0.17))
    dx_dtilt = float(dec.get("dx_dtilt", 0.0))
    dy_dpan = float(dec.get("dy_dpan", 0.0))
    dy_dtilt = float(dec.get("dy_dtilt", 0.036))

    C_cam = np.array([[dx_dpan, dx_dtilt], [dy_dpan, dy_dtilt]])

    det = np.linalg.det(C_cam)
    if abs(det) < 1e-6:
        print("[ERROR] Calibration matrix near-singular")
        return 3

    # MPC parameters - use longer horizon to get trajectory
    dt = float(mpc_cfg.get("dt", 0.033))
    horizon = int(mpc_cfg.get("horizon", 20))  # Longer for trajectory
    
    q_error = float(mpc_cfg.get("q_error", 5000.0))
    r_accel = float(mpc_cfg.get("r_accel", 0.5))  # Slightly higher for smoother
    q_terminal = float(mpc_cfg.get("q_terminal", 10000.0))

    Q = np.diag([q_error, q_error])
    R = np.diag([r_accel, r_accel])
    Q_f = np.diag([q_terminal, q_terminal])

    pan_lim = (float(pan_cfg.get("min", -22.0)), float(pan_cfg.get("max", 22.0)))
    tilt_lim = (float(tilt_cfg.get("min", -3.0)), float(tilt_cfg.get("max", 5.0)))

    vel_max_pan = float(mpc_cfg.get("vel_max_pan", 70.0))
    vel_max_tilt = float(mpc_cfg.get("vel_max_tilt", 50.0))

    accel_max_pan = float(mpc_cfg.get("accel_max_pan", 400.0))
    accel_max_tilt = float(mpc_cfg.get("accel_max_tilt", 300.0))

    deadzone = float(mpc_cfg.get("deadzone", track_cfg.get("deadzone", 0.08)))
    min_conf = float(mpc_cfg.get("min_confidence", 0.40))
    meas_ema = float(mpc_cfg.get("meas_ema", 0.5))  # Less filtering
    return_to_center_delay = float(track_cfg.get("return_to_center_delay", 3.0))

    # Queue manager for smooth motion
    queue_mgr = MoveQueueManager(klipper, max_queue_time=0.25)

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

    # State
    pan_pos = float(pan_cfg.get("center", 0.0))
    tilt_pos = float(tilt_cfg.get("center", 0.0))
    pan_vel = 0.0
    tilt_vel = 0.0

    if not dry_run:
        klipper.move_stepper("stepper_0", pan_pos, speed=50)
        klipper.move_stepper("stepper_1", tilt_pos, speed=40)
        time.sleep(0.3)

    ex_f, ey_f = 0.0, 0.0
    filt_init = False

    last_face_time = time.time()
    last_print = 0.0
    last_solve = 0.0

    print("=" * 72)
    print("MPC HEAD TRACKER (QUEUED MOVES)")
    print(f"UDP port: {cfg['network']['udp_port']} | dry_run={dry_run}")
    print(f"dt={dt:.3f}s | horizon={horizon} | deadzone={deadzone}")
    print(f"Q_error={q_error} R_accel={r_accel}")
    print(f"vel_max: pan={vel_max_pan:.1f} tilt={vel_max_tilt:.1f} deg/s")
    print("=" * 72)

    # How many MPC trajectory steps to queue ahead
    steps_to_queue = 5

    try:
        while True:
            loop_start = time.time()

            pkt = receiver.get_latest()
            have_face = pkt is not None and pkt.detected and pkt.confidence >= min_conf

            if not have_face:
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
                        print("[CENTER] returning to center")
                    last_face_time = time.time()

                time.sleep(0.02)
                continue

            last_face_time = time.time()

            ex, ey = float(pkt.x), float(pkt.y)
            if not filt_init:
                ex_f, ey_f = ex, ey
                filt_init = True
            else:
                ex_f = meas_ema * ex_f + (1.0 - meas_ema) * ex
                ey_f = meas_ema * ey_f + (1.0 - meas_ema) * ey

            if abs(ex_f) < deadzone and abs(ey_f) < deadzone:
                pan_vel *= 0.9
                tilt_vel *= 0.9
                if time.time() - last_print > 1.0:
                    print(f"[HOLD] e=({ex_f:+.3f},{ey_f:+.3f}) pos=({pan_pos:+.2f},{tilt_pos:+.2f})")
                    last_print = time.time()
                time.sleep(dt)
                continue

            # Check queue depth - only solve if we need more moves
            queue_depth = queue_mgr.get_queue_depth()
            
            if queue_depth < 0.1 and time.time() - last_solve > 0.05:
                # Solve MPC for trajectory
                x0 = np.array([pan_pos, pan_vel, tilt_pos, tilt_vel])
                e_meas = np.array([ex_f, ey_f])

                t_solve_start = time.time()
                U_traj = mpc.solve(x0, e_meas)
                t_solve = time.time() - t_solve_start
                last_solve = time.time()

                # Queue multiple steps from trajectory
                p_pos, p_vel = pan_pos, pan_vel
                t_pos, t_vel = tilt_pos, tilt_vel

                queued = 0
                for k in range(min(steps_to_queue, horizon)):
                    u_pan = U_traj[k * 2]
                    u_tilt = U_traj[k * 2 + 1]

                    # Integrate
                    p_vel_new = np.clip(p_vel + u_pan * dt, -vel_max_pan, vel_max_pan)
                    t_vel_new = np.clip(t_vel + u_tilt * dt, -vel_max_tilt, vel_max_tilt)

                    p_pos_new = np.clip(p_pos + p_vel_new * dt, pan_lim[0], pan_lim[1])
                    t_pos_new = np.clip(t_pos + t_vel_new * dt, tilt_lim[0], tilt_lim[1])

                    step_pan = abs(p_pos_new - p_pos)
                    step_tilt = abs(t_pos_new - t_pos)

                    if step_pan < 0.01 and step_tilt < 0.01:
                        break

                    pan_speed = max(1.0, abs(p_vel_new) * 1.2)
                    tilt_speed = max(1.0, abs(t_vel_new) * 1.2)

                    move_time = max(step_pan / pan_speed, step_tilt / tilt_speed) if pan_speed > 0 else dt

                    if not dry_run:
                        if not queue_mgr.queue_move(p_pos_new, t_pos_new, pan_speed, tilt_speed, move_time):
                            break

                    p_pos, p_vel = p_pos_new, p_vel_new
                    t_pos, t_vel = t_pos_new, t_vel_new
                    queued += 1

                # Update state to end of queued trajectory
                pan_pos, pan_vel = p_pos, p_vel
                tilt_pos, tilt_vel = t_pos, t_vel

                now = time.time()
                if now - last_print > 0.5:
                    print(
                        f"[STAT] e=({ex_f:+.3f},{ey_f:+.3f}) v=({pan_vel:+.1f},{tilt_vel:+.1f}) "
                        f"pos=({pan_pos:+.2f},{tilt_pos:+.2f}) solve={t_solve*1000:.1f}ms q={queue_depth:.2f}s queued={queued}"
                    )
                    last_print = now

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[STOP] exiting")
        if not dry_run:
            klipper.emergency_stop()
            klipper.disconnect()
        return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="MPC with queued moves")
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
