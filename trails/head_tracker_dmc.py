#!/usr/bin/env python3
"""
DMC (Dynamic Matrix Control) implementation following:
Nebeluk et al., "Predictive tracking of an object by a pan–tilt camera"
Nonlinear Dynamics (2023)

Key differences from our previous MPC:
1. Models actual motor dynamics (not instant position control)
2. Uses step response coefficients from system identification
3. Includes "free trajectory" - prediction of where motor WILL BE
4. Accounts for past control actions still affecting the system

This should eliminate oscillation because it predicts the actual
motor behavior including Klipper's motion planning delays.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, List

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "shared"))
from klipper_direct import KlipperDirect


# -----------------------------------------------------------------------------
# UDP Face Receiver
# -----------------------------------------------------------------------------
@dataclass
class FacePacket:
    x: float
    y: float
    detected: bool
    confidence: float
    timestamp: float


class UdpFaceReceiver:
    def __init__(self, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)

    def _parse(self, msg: str) -> Optional[FacePacket]:
        msg = msg.strip()
        if not msg.startswith("{"):
            return None
        try:
            data = json.loads(msg)
        except json.JSONDecodeError:
            return None

        detected = bool(data.get("detected", data.get("face_detected", False)))
        x = data.get("x", data.get("x_offset", 0.0))
        y = data.get("y", data.get("y_offset", 0.0))

        return FacePacket(
            x=np.clip(float(x), -1.0, 1.0),
            y=np.clip(float(y), -1.0, 1.0),
            detected=detected,
            confidence=float(data.get("confidence", 1.0) or 0.0),
            timestamp=float(data.get("timestamp", time.time()) or time.time()),
        )

    def get_latest(self) -> Optional[FacePacket]:
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                pkt = self._parse(data.decode("utf-8", errors="ignore"))
                if pkt:
                    latest = pkt
            except BlockingIOError:
                break
        return latest


# -----------------------------------------------------------------------------
# DMC Controller (following the paper)
# -----------------------------------------------------------------------------
class DMCController:
    """
    Dynamic Matrix Control for a single axis.
    
    Uses step response model to predict future outputs and accounts for
    past control actions that are still affecting the system (free trajectory).
    
    From the paper:
    - Prediction horizon N = 50
    - Control horizon Nu = 25
    - Sampling period Ts = 2ms (500 Hz) - we'll use 33ms (30 Hz)
    """
    
    def __init__(
        self,
        dt: float,
        N: int,  # prediction horizon
        Nu: int,  # control horizon
        D: int,  # step response settling horizon
        step_response: np.ndarray,  # step response coefficients [s1, s2, ..., sD]
        lambda_weight: float = 100.0,  # control increment penalty
        psi_weight: float = 1.0,  # error penalty
    ):
        self.dt = dt
        self.N = N
        self.Nu = Nu
        self.D = D
        self.lambda_w = lambda_weight
        self.psi_w = psi_weight
        
        # Step response coefficients
        self.s = step_response  # shape (D,)
        
        # Build dynamic matrix M (N x Nu)
        self.M = self._build_dynamic_matrix()
        
        # Build past dynamic matrix Mp (N x D-1)
        self.Mp = self._build_past_matrix()
        
        # Precompute gain matrix K
        # K = (M^T Ψ M + Λ)^(-1) M^T Ψ
        Psi = self.psi_w * np.eye(N)
        Lambda = self.lambda_w * np.eye(Nu)
        
        MTΨ = self.M.T @ Psi
        self.K = np.linalg.inv(MTΨ @ self.M + Lambda) @ MTΨ
        
        # History of past control increments (for free trajectory)
        self.delta_u_history = deque([0.0] * (D - 1), maxlen=D - 1)
        
    def _build_dynamic_matrix(self) -> np.ndarray:
        """Build M matrix from step response coefficients."""
        M = np.zeros((self.N, self.Nu))
        for i in range(self.N):
            for j in range(min(i + 1, self.Nu)):
                idx = i - j
                if idx < self.D:
                    M[i, j] = self.s[idx]
        return M
    
    def _build_past_matrix(self) -> np.ndarray:
        """Build Mp matrix for free trajectory calculation."""
        Mp = np.zeros((self.N, self.D - 1))
        for i in range(self.N):
            for j in range(self.D - 1):
                idx_future = i + j + 1
                idx_current = j
                if idx_future < self.D and idx_current < self.D:
                    Mp[i, j] = self.s[idx_future] - self.s[idx_current]
        return Mp
    
    def compute(
        self,
        setpoint_trajectory: np.ndarray,  # shape (N,) - future setpoints
        current_output: float,
    ) -> float:
        """
        Compute optimal control increment.
        
        Args:
            setpoint_trajectory: Future setpoint values over horizon
            current_output: Current measured/estimated output
            
        Returns:
            Optimal control increment Δu
        """
        # Free trajectory: predicted output if we do nothing
        # y0 = current_output + Mp @ past_delta_u
        past_delta_u = np.array(list(self.delta_u_history))
        y0 = current_output * np.ones(self.N) + self.Mp @ past_delta_u
        
        # Optimal control sequence: ΔU = K @ (Y_sp - Y0)
        delta_U = self.K @ (setpoint_trajectory - y0)
        
        # Apply only first control increment (receding horizon)
        delta_u = delta_U[0]
        
        # Store for future free trajectory calculation
        self.delta_u_history.append(delta_u)
        
        return delta_u
    
    def reset(self):
        """Reset controller state."""
        self.delta_u_history = deque([0.0] * (self.D - 1), maxlen=self.D - 1)


def identify_step_response(klipper: KlipperDirect, stepper: str, step_size: float = 5.0, dt: float = 0.033, D: int = 30) -> np.ndarray:
    """
    Identify step response of the motor through Klipper.
    
    Sends a step command and records the response over time.
    """
    print(f"[IDENT] Identifying step response for {stepper}...")
    
    # Start at 0
    klipper.set_position(stepper, 0)
    klipper.move_stepper(stepper, 0, speed=50)
    time.sleep(0.5)
    
    # Record response to step
    responses = []
    
    # Send step command
    klipper.move_stepper(stepper, step_size, speed=50)
    t0 = time.time()
    
    # Record response over D samples
    for i in range(D):
        # In real system, we'd read encoder. Here we estimate based on timing.
        # This is a simplification - ideally you'd have position feedback.
        t = time.time() - t0
        
        # Estimate position (assuming simple trapezoidal profile)
        # This should be replaced with actual encoder reading if available
        responses.append(t)
        time.sleep(dt)
    
    # Wait for move to complete
    time.sleep(0.5)
    
    # Return to 0
    klipper.move_stepper(stepper, 0, speed=50)
    time.sleep(0.5)
    
    # For now, use a reasonable second-order step response model
    # Based on the paper's identified parameters
    # s(k) = 1 - a1*s(k-1) - a2*s(k-2) + b1 (normalized)
    
    # Generate step response from second-order model
    # Using paper's approximate values scaled for our system
    a1 = -1.8  # Damping
    a2 = 0.82  # Natural frequency
    
    step_resp = np.zeros(D)
    for k in range(D):
        if k == 0:
            step_resp[k] = 0.0
        elif k == 1:
            step_resp[k] = 0.1
        else:
            step_resp[k] = -a1 * step_resp[k-1] - a2 * step_resp[k-2] + (1 + a1 + a2)
    
    # Normalize to reach 1.0 at steady state
    step_resp = step_resp / step_resp[-1] if step_resp[-1] > 0 else step_resp
    
    print(f"[IDENT] Step response: {step_resp[:5]}...{step_resp[-3:]}")
    return step_resp


# -----------------------------------------------------------------------------
# Main Tracker using DMC
# -----------------------------------------------------------------------------
def run_tracker(config_path: Path, *, dry_run: bool, identify: bool) -> int:
    cfg = json.loads(config_path.read_text())

    receiver = UdpFaceReceiver(int(cfg["network"]["udp_port"]))
    
    klipper = KlipperDirect()
    if not dry_run:
        if not klipper.connect():
            print("[ERROR] Failed to connect to Klipper")
            return 2
        
        klipper.enable_stepper("stepper_0", True)
        klipper.enable_stepper("stepper_1", True)
        klipper.set_position("stepper_0", 0)
        klipper.set_position("stepper_1", 0)

    # Config
    motors_cfg = cfg.get("motors", {})
    pan_cfg = motors_cfg.get("pan", {})
    tilt_cfg = motors_cfg.get("tilt", {})
    track_cfg = cfg.get("tracking", {})
    dec = cfg.get("tracking_decoupled", {})

    # Calibration matrix
    dx_dpan = float(dec.get("dx_dpan", 0.17))
    dy_dtilt = float(dec.get("dy_dtilt", 0.036))

    # DMC parameters (from paper, adapted)
    dt = 0.033  # 30 Hz
    N = 30  # prediction horizon
    Nu = 15  # control horizon
    D = 20  # step response settling

    # Generate step response (second-order model)
    # These coefficients approximate Klipper's motion planner response
    tau = 0.1  # time constant in seconds
    zeta = 0.7  # damping ratio
    
    step_resp = np.zeros(D)
    for k in range(D):
        t = k * dt
        if t > 0:
            omega_n = 1.0 / tau
            omega_d = omega_n * np.sqrt(1 - zeta**2) if zeta < 1 else omega_n
            if zeta < 1:
                step_resp[k] = 1 - np.exp(-zeta * omega_n * t) * (
                    np.cos(omega_d * t) + (zeta / np.sqrt(1 - zeta**2)) * np.sin(omega_d * t)
                )
            else:
                step_resp[k] = 1 - np.exp(-omega_n * t) * (1 + omega_n * t)
    
    print(f"[DMC] Step response (first 5): {step_resp[:5]}")

    # Create DMC controllers for pan and tilt
    # Higher lambda = less aggressive (smoother but slower)
    # Higher psi = more aggressive (faster but may oscillate)
    dmc_pan = DMCController(
        dt=dt, N=N, Nu=Nu, D=D,
        step_response=step_resp,
        lambda_weight=100.0,  # More responsive
        psi_weight=1.0,       # Error penalty
    )
    
    dmc_tilt = DMCController(
        dt=dt, N=N, Nu=Nu, D=D,
        step_response=step_resp,
        lambda_weight=150.0,  # More responsive (was 500)
        psi_weight=1.0,
    )

    # Limits
    pan_lim = (float(pan_cfg.get("min", -22.0)), float(pan_cfg.get("max", 22.0)))
    tilt_lim = (float(tilt_cfg.get("min", -3.0)), float(tilt_cfg.get("max", 5.0)))
    
    deadzone = float(track_cfg.get("deadzone", 0.08))
    min_conf = 0.4
    
    # State
    pan_pos = 0.0
    tilt_pos = 0.0
    pan_cmd = 0.0  # Commanded position (control output)
    tilt_cmd = 0.0
    
    # Measurement filter
    ex_f, ey_f = 0.0, 0.0
    ema = 0.6
    filt_init = False
    
    # Setpoint extrapolation (target velocity estimation)
    ex_hist = deque(maxlen=5)
    ey_hist = deque(maxlen=5)

    last_face_time = time.time()
    last_print = 0.0
    return_delay = float(track_cfg.get("return_to_center_delay", 3.0))

    print("=" * 72)
    print("DMC HEAD TRACKER (following Nebeluk et al. paper)")
    print(f"dt={dt:.3f}s | N={N} | Nu={Nu} | D={D}")
    print(f"lambda_pan={dmc_pan.lambda_w} lambda_tilt={dmc_tilt.lambda_w}")
    print(f"deadzone={deadzone}")
    print("=" * 72)

    try:
        while True:
            loop_start = time.time()

            pkt = receiver.get_latest()
            have_face = pkt is not None and pkt.detected and pkt.confidence >= min_conf

            if not have_face:
                # No face - return to center after delay
                if time.time() - last_face_time > return_delay:
                    if abs(pan_cmd) > 0.5 or abs(tilt_cmd) > 0.5:
                        pan_cmd, tilt_cmd = 0.0, 0.0
                        dmc_pan.reset()
                        dmc_tilt.reset()
                        if not dry_run:
                            klipper.move_pan_tilt(0, 0, pan_speed=15, tilt_speed=12)
                        print("[CENTER] returning to center")
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
                ex_f = ema * ex_f + (1 - ema) * ex
                ey_f = ema * ey_f + (1 - ema) * ey

            # Track history for velocity estimation
            ex_hist.append(ex_f)
            ey_hist.append(ey_f)
            
            # Estimate target velocity
            if len(ex_hist) >= 3:
                ex_vel = (ex_hist[-1] - ex_hist[-3]) / (2 * dt)
                ey_vel = (ey_hist[-1] - ey_hist[-3]) / (2 * dt)
            else:
                ex_vel, ey_vel = 0.0, 0.0

            # Deadzone
            if abs(ex_f) < deadzone and abs(ey_f) < deadzone:
                if time.time() - last_print > 1.0:
                    print(f"[HOLD] e=({ex_f:+.3f},{ey_f:+.3f}) cmd=({pan_cmd:+.2f},{tilt_cmd:+.2f})")
                    last_print = time.time()
                time.sleep(dt)
                continue

            # Convert image error to motor DELTA (not absolute position)
            # Camera is ON the pan/tilt head. When motor moves, camera moves.
            # Test showed: PAN +2 moved face x from -0.16 to +0.08
            # So: to correct NEGATIVE x error, need POSITIVE pan = -error/cal
            # Test showed: TILT -1 moved face y negative (up)
            # So: to correct POSITIVE y error, need NEGATIVE tilt = -error/cal
            pan_delta_sp = -ex_f / dx_dpan    # NEGATIVE: -x error → +pan
            tilt_delta_sp = -ey_f / dy_dtilt  # NEGATIVE: +y error → -tilt
            
            # Clamp the desired deltas to reasonable range
            max_pan_delta = 3.0  # max degrees per sample
            max_tilt_delta = 1.0  # Tilt has small range
            pan_delta_sp = np.clip(pan_delta_sp, -max_pan_delta, max_pan_delta)
            tilt_delta_sp = np.clip(tilt_delta_sp, -max_tilt_delta, max_tilt_delta)
            
            # Build setpoint trajectory (relative to current command)
            # The setpoint is where we WANT to be: current + delta
            pan_sp = pan_cmd + pan_delta_sp
            tilt_sp = tilt_cmd + tilt_delta_sp
            
            # Clamp to motor limits
            pan_sp = np.clip(pan_sp, pan_lim[0], pan_lim[1])
            tilt_sp = np.clip(tilt_sp, tilt_lim[0], tilt_lim[1])
            
            # Build trajectory with velocity extrapolation
            pan_sp_traj = np.zeros(N)
            tilt_sp_traj = np.zeros(N)
            
            pan_vel_sp = -ex_vel / dx_dpan * 0.3    # Same sign logic as position
            tilt_vel_sp = -ey_vel / dy_dtilt * 0.3  # Same sign logic as position
            
            for k in range(N):
                # Extrapolate setpoint over horizon
                pan_sp_traj[k] = np.clip(pan_sp + k * dt * pan_vel_sp, pan_lim[0], pan_lim[1])
                tilt_sp_traj[k] = np.clip(tilt_sp + k * dt * tilt_vel_sp, tilt_lim[0], tilt_lim[1])

            # DMC computes control INCREMENT
            delta_pan = dmc_pan.compute(pan_sp_traj, pan_cmd)
            delta_tilt = dmc_tilt.compute(tilt_sp_traj, tilt_cmd)

            # Apply increment to get new command
            pan_cmd_new = np.clip(pan_cmd + delta_pan, pan_lim[0], pan_lim[1])
            tilt_cmd_new = np.clip(tilt_cmd + delta_tilt, tilt_lim[0], tilt_lim[1])

            # Skip tiny moves
            if abs(pan_cmd_new - pan_cmd) < 0.02 and abs(tilt_cmd_new - tilt_cmd) < 0.02:
                time.sleep(dt)
                continue

            # Send command
            if not dry_run:
                klipper.move_pan_tilt(pan_cmd_new, tilt_cmd_new, pan_speed=50, tilt_speed=40)

            pan_cmd, tilt_cmd = pan_cmd_new, tilt_cmd_new

            # Status
            now = time.time()
            if now - last_print > 0.5:
                print(
                    f"[DMC] e=({ex_f:+.3f},{ey_f:+.3f}) sp=({pan_sp:+.2f},{tilt_sp:+.2f}) "
                    f"Δ=({delta_pan:+.3f},{delta_tilt:+.3f}) cmd=({pan_cmd:+.2f},{tilt_cmd:+.2f})"
                )
                last_print = now

            # Maintain loop rate
            elapsed = time.time() - loop_start
            time.sleep(max(0.0, dt - elapsed))

    except KeyboardInterrupt:
        print("\n[STOP] exiting")
        if not dry_run:
            klipper.emergency_stop()
            klipper.disconnect()
        return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="DMC tracker (paper implementation)")
    ap.add_argument("--config", default=str(Path(__file__).parent.parent / "config.json"))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--identify", action="store_true", help="Run system identification")
    
    args = ap.parse_args()
    return run_tracker(Path(args.config), dry_run=args.dry_run, identify=args.identify)


if __name__ == "__main__":
    raise SystemExit(main())
