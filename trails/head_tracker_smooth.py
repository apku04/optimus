#!/usr/bin/env python3
"""Smooth Gimbal Tracker - Fluid motion through trajectory planning.

The Problem
-----------
Klipper's MANUAL_STEPPER executes each move as a separate trapezoidal profile:
accelerate → cruise → decelerate. If you send many small moves, you get
"micro-stepping" that looks choppy even if the controller is perfect.

The Solution
------------
Instead of sending frequent small corrections, we:
1. Predict where the face will be (using velocity + smoothing)
2. Compute a target position that centers the face
3. Send ONE smooth move toward that target at constant speed
4. Only update when the prediction changes significantly

This produces fluid, gimbal-like motion because Klipper can execute a
continuous sweep instead of stop-start micro-moves.

Key insight: The "smoothness" comes from the MOTOR execution, not the
controller update rate. A single 2-second sweep looks smoother than
100 tiny 20ms corrections.

Usage
-----
  python3 rpi/head_tracker_smooth.py
  python3 rpi/head_tracker_smooth.py --dry-run
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

sys.path.insert(0, str(Path(__file__).parent.parent / "shared"))
from motor_control import MotorController


@dataclass
class FacePacket:
    x: float
    y: float
    detected: bool
    confidence: float
    stability: float
    x_velocity: float
    y_velocity: float
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
                        x=float(parts[0]), y=float(parts[1]),
                        detected=True, confidence=1.0, stability=1.0,
                        x_velocity=0.0, y_velocity=0.0, timestamp=time.time(),
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
            x, y = float(x), float(y)
        except (TypeError, ValueError):
            return None

        return FacePacket(
            x=np.clip(x, -1.0, 1.0),
            y=np.clip(y, -1.0, 1.0),
            detected=detected,
            confidence=float(data.get("confidence", 1.0) or 0.0),
            stability=float(data.get("stability", 1.0) or 0.0),
            x_velocity=float(data.get("x_velocity", 0.0) or 0.0),
            y_velocity=float(data.get("y_velocity", 0.0) or 0.0),
            timestamp=float(data.get("timestamp", time.time()) or time.time()),
        )

    def get_latest(self) -> Optional[FacePacket]:
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except (BlockingIOError, OSError):
                break
            pkt = self._parse(data.decode("utf-8", errors="ignore"))
            if pkt is not None:
                latest = pkt
        return latest


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
    dec = cfg.get("tracking_decoupled", {})
    smooth_cfg = cfg.get("tracking_smooth", {})

    # Calibration: inverse of C_cam to convert image error to motor degrees
    dx_dpan = float(dec.get("dx_dpan", 0.17))
    dx_dtilt = float(dec.get("dx_dtilt", 0.0))
    dy_dpan = float(dec.get("dy_dpan", 0.0))
    dy_dtilt = float(dec.get("dy_dtilt", 0.036))

    # A = [[dx_dpan, dx_dtilt], [dy_dpan, dy_dtilt]]
    # inv(A) maps image error to motor correction
    det = dx_dpan * dy_dtilt - dx_dtilt * dy_dpan
    if abs(det) < 1e-6:
        print("[ERROR] Calibration matrix singular")
        return 3

    # inv(A) = 1/det * [[dy_dtilt, -dx_dtilt], [-dy_dpan, dx_dpan]]
    inv_A = np.array([
        [dy_dtilt / det, -dx_dtilt / det],
        [-dy_dpan / det, dx_dpan / det]
    ])

    # Parameters
    deadzone = float(smooth_cfg.get("deadzone", track_cfg.get("deadzone", 0.08)))
    min_conf = float(smooth_cfg.get("min_confidence", 0.40))

    # Smoothing: heavy filtering for stable target estimation
    meas_ema = float(smooth_cfg.get("meas_ema", 0.85))  # 0.85 = very smooth
    vel_ema = float(smooth_cfg.get("vel_ema", 0.90))    # velocity estimate smoothing

    # Prediction: look ahead to where face will be
    lookahead_s = float(smooth_cfg.get("lookahead_s", 0.15))

    # Motion: single smooth sweeps instead of micro-corrections
    pan_speed = float(smooth_cfg.get("pan_speed", 25.0))  # constant cruise speed
    tilt_speed = float(smooth_cfg.get("tilt_speed", 18.0))

    # Update threshold: only send new move if target changed significantly
    # This is KEY for smoothness - fewer moves = smoother execution
    update_threshold_deg = float(smooth_cfg.get("update_threshold_deg", 0.5))

    # Minimum time between commands (let moves execute)
    min_cmd_interval_s = float(smooth_cfg.get("min_cmd_interval_s", 0.15))

    # Gain: how aggressively to correct (< 1.0 = undershoot intentionally)
    gain = float(smooth_cfg.get("gain", 0.7))

    return_to_center_delay = float(track_cfg.get("return_to_center_delay", 3.0))

    # Limits
    pan_min, pan_max = float(pan_cfg.get("min", -22)), float(pan_cfg.get("max", 22))
    tilt_min, tilt_max = float(tilt_cfg.get("min", -3)), float(tilt_cfg.get("max", 5))

    # State
    pan_pos = float(pan_cfg.get("center", 0.0))
    tilt_pos = float(tilt_cfg.get("center", 0.0))

    if not dry_run:
        motors.move_pan(pan_pos, speed=pan_cfg.get("speed"), sync=False)
        motors.move_tilt(tilt_pos, speed=tilt_cfg.get("speed"), sync=False)

    # Filtered state
    ex_f, ey_f = 0.0, 0.0
    vx_f, vy_f = 0.0, 0.0
    filt_init = False

    # Command state
    last_target_pan = pan_pos
    last_target_tilt = tilt_pos
    last_cmd_time = 0.0
    last_face_time = time.time()
    last_print = 0.0

    print("=" * 72)
    print("SMOOTH GIMBAL TRACKER")
    print(f"UDP port: {cfg['network']['udp_port']} | dry_run={dry_run}")
    print(f"deadzone={deadzone} | gain={gain} | lookahead={lookahead_s}s")
    print(f"meas_ema={meas_ema} | update_threshold={update_threshold_deg}°")
    print(f"pan_speed={pan_speed} tilt_speed={tilt_speed} deg/s")
    print(f"inv(A)=[[{inv_A[0,0]:.3f},{inv_A[0,1]:.3f}],[{inv_A[1,0]:.3f},{inv_A[1,1]:.3f}]]")
    print("=" * 72)

    try:
        while True:
            now = time.time()
            pkt = receiver.get_latest()
            have_face = pkt is not None and pkt.detected and pkt.confidence >= min_conf

            if not have_face:
                # Lost face: let current move finish, then return to center
                if now - last_face_time > return_to_center_delay:
                    tgt_pan = float(pan_cfg.get("center", 0.0))
                    tgt_tilt = float(tilt_cfg.get("center", 0.0))
                    if abs(pan_pos - tgt_pan) > 1.0 or abs(tilt_pos - tgt_tilt) > 0.5:
                        pan_pos, tilt_pos = tgt_pan, tgt_tilt
                        last_target_pan, last_target_tilt = tgt_pan, tgt_tilt
                        if not dry_run:
                            motors.move_pan_tilt(pan_pos, tilt_pos,
                                                 pan_speed=pan_speed * 0.5,
                                                 tilt_speed=tilt_speed * 0.5,
                                                 sync=False)
                        print("[CENTER] no face -> returning to center")
                    last_face_time = now
                time.sleep(0.05)
                continue

            last_face_time = now

            # Get raw measurement
            ex_raw, ey_raw = float(pkt.x), float(pkt.y)
            vx_raw = float(pkt.x_velocity) if hasattr(pkt, 'x_velocity') else 0.0
            vy_raw = float(pkt.y_velocity) if hasattr(pkt, 'y_velocity') else 0.0

            # Heavy smoothing for stable estimate
            if not filt_init:
                ex_f, ey_f = ex_raw, ey_raw
                vx_f, vy_f = vx_raw, vy_raw
                filt_init = True
            else:
                ex_f = meas_ema * ex_f + (1 - meas_ema) * ex_raw
                ey_f = meas_ema * ey_f + (1 - meas_ema) * ey_raw
                vx_f = vel_ema * vx_f + (1 - vel_ema) * vx_raw
                vy_f = vel_ema * vy_f + (1 - vel_ema) * vy_raw

            # Predict where face will be after lookahead
            ex_pred = ex_f + vx_f * lookahead_s
            ey_pred = ey_f + vy_f * lookahead_s
            ex_pred = np.clip(ex_pred, -1.0, 1.0)
            ey_pred = np.clip(ey_pred, -1.0, 1.0)

            # Check if in deadzone
            if abs(ex_pred) < deadzone and abs(ey_pred) < deadzone:
                if now - last_print > 2.0:
                    print(f"[HOLD] e=({ex_f:+.3f},{ey_f:+.3f}) pos=({pan_pos:+.2f},{tilt_pos:+.2f})")
                    last_print = now
                time.sleep(0.03)
                continue

            # Compute motor correction: delta = -gain * inv(A) @ [ex, ey]
            e_vec = np.array([ex_pred, ey_pred])
            delta = -gain * (inv_A @ e_vec)
            delta_pan, delta_tilt = delta[0], delta[1]

            # Compute new target
            target_pan = np.clip(pan_pos + delta_pan, pan_min, pan_max)
            target_tilt = np.clip(tilt_pos + delta_tilt, tilt_min, tilt_max)

            # Check if target changed enough to warrant a new command
            delta_from_last = max(abs(target_pan - last_target_pan),
                                  abs(target_tilt - last_target_tilt))

            # Also check time since last command
            time_since_cmd = now - last_cmd_time

            # Only send if: significant change AND enough time passed
            if delta_from_last < update_threshold_deg and time_since_cmd < min_cmd_interval_s * 3:
                time.sleep(0.02)
                continue

            if time_since_cmd < min_cmd_interval_s:
                time.sleep(0.01)
                continue

            # Send the move
            if dry_run:
                print(f"[MOVE] e=({ex_f:+.3f},{ey_f:+.3f}) -> target=({target_pan:+.2f},{target_tilt:+.2f}) Δ={delta_from_last:.2f}°")
            else:
                motors.move_pan_tilt(target_pan, target_tilt,
                                     pan_speed=pan_speed,
                                     tilt_speed=tilt_speed,
                                     sync=False)

            # Update state
            pan_pos = target_pan
            tilt_pos = target_tilt
            last_target_pan = target_pan
            last_target_tilt = target_tilt
            last_cmd_time = now

            if now - last_print > 1.0:
                print(f"[STAT] e=({ex_f:+.3f},{ey_f:+.3f}) -> pos=({pan_pos:+.2f},{tilt_pos:+.2f})")
                last_print = now

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[STOP] exiting")
        return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Smooth gimbal head tracker")
    ap.add_argument("--config", default=str(Path(__file__).parent.parent / "config.json"))
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    return run(Path(args.config), dry_run=args.dry_run)


if __name__ == "__main__":
    raise SystemExit(main())
