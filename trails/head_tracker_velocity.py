#!/usr/bin/env python3
"""
MPC tracker using Klipper's GCODE_AXIS velocity control.

This uses the motion planner for smooth gimbal-like motion.
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
from collections import deque

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "shared"))
from klipper_velocity import KlipperVelocityControl


@dataclass
class FacePacket:
    x: float
    y: float
    detected: bool
    confidence: float


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
            x=float(x),
            y=float(y),
            detected=detected,
            confidence=float(data.get("confidence", 1.0) or 0.0),
        )

    def get_latest(self) -> Optional[FacePacket]:
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            pkt = self._parse(data.decode("utf-8", errors="ignore"))
            if pkt:
                latest = pkt
        return latest


def run(config_path: Path, *, dry_run: bool) -> int:
    cfg = json.loads(config_path.read_text())

    receiver = UdpFaceReceiver(int(cfg["network"]["udp_port"]))
    
    # Use velocity control
    motors = KlipperVelocityControl()
    
    if not dry_run:
        if not motors.connect():
            print("[ERROR] Failed to connect to Klipper")
            return 2

    dec = cfg.get("tracking_decoupled", {})
    track_cfg = cfg.get("tracking", {})
    pan_cfg = cfg.get("motors", {}).get("pan", {})
    tilt_cfg = cfg.get("motors", {}).get("tilt", {})

    # Calibration matrix
    dx_dpan = float(dec.get("dx_dpan", 0.17))
    dy_dtilt = float(dec.get("dy_dtilt", 0.036))

    # Simple proportional gain for velocity control
    # Velocity = gain * error
    kp_pan = float(dec.get("kp_pan", 25.0))  # deg/s per unit error
    kp_tilt = float(dec.get("kp_tilt", 15.0))
    
    # Derivative gain for damping
    kd_pan = float(dec.get("kd_pan", 5.0))
    kd_tilt = float(dec.get("kd_tilt", 3.0))

    deadzone = float(track_cfg.get("deadzone", 0.08))
    min_conf = 0.40
    
    # Position limits
    pan_min = float(pan_cfg.get("min", -22.0))
    pan_max = float(pan_cfg.get("max", 22.0))
    tilt_min = float(tilt_cfg.get("min", -3.0))
    tilt_max = float(tilt_cfg.get("max", 5.0))
    
    # Velocity limits
    vel_max_pan = 50.0
    vel_max_tilt = 35.0

    # EMA filter
    ema = 0.7
    ex_f, ey_f = 0.0, 0.0
    filt_init = False
    
    # For derivative
    ex_prev, ey_prev = 0.0, 0.0

    # Current position (tracked internally)
    pan_pos = 0.0
    tilt_pos = 0.0

    last_face_time = time.time()
    last_print = 0.0

    dt = 0.033  # ~30 Hz

    print("=" * 72)
    print("VELOCITY CONTROL TRACKER")
    print(f"UDP port: {cfg['network']['udp_port']} | dry_run={dry_run}")
    print(f"Kp: pan={kp_pan} tilt={kp_tilt} | Kd: pan={kd_pan} tilt={kd_tilt}")
    print(f"deadzone={deadzone}")
    print("=" * 72)

    try:
        while True:
            loop_start = time.time()

            pkt = receiver.get_latest()
            have_face = pkt is not None and pkt.detected and pkt.confidence >= min_conf

            if not have_face:
                # No face: slow down
                if not dry_run:
                    motors.move_velocity(0, 0, duration=dt)
                
                if time.time() - last_face_time > 3.0:
                    # Return to center
                    if abs(pan_pos) > 0.5 or abs(tilt_pos) > 0.5:
                        if not dry_run:
                            motors.move_to_position(0, 0, velocity=15)
                        pan_pos, tilt_pos = 0.0, 0.0
                        print("[CENTER] returning")
                    last_face_time = time.time()

                time.sleep(0.02)
                continue

            last_face_time = time.time()

            # Filter measurement
            ex, ey = float(pkt.x), float(pkt.y)
            if not filt_init:
                ex_f, ey_f = ex, ey
                ex_prev, ey_prev = ex, ey
                filt_init = True
            else:
                ex_f = ema * ex_f + (1.0 - ema) * ex
                ey_f = ema * ey_f + (1.0 - ema) * ey

            # Compute error derivative
            ex_dot = (ex_f - ex_prev) / dt
            ey_dot = (ey_f - ey_prev) / dt
            ex_prev, ey_prev = ex_f, ey_f

            # Deadzone
            if abs(ex_f) < deadzone and abs(ey_f) < deadzone:
                if not dry_run:
                    motors.move_velocity(0, 0, duration=dt)
                if time.time() - last_print > 1.0:
                    print(f"[HOLD] e=({ex_f:+.3f},{ey_f:+.3f}) pos=({pan_pos:+.2f},{tilt_pos:+.2f})")
                    last_print = time.time()
                time.sleep(dt)
                continue

            # PD control: velocity = Kp * error + Kd * error_derivative
            # Negative because positive error (face right) requires negative pan (move left)
            pan_vel = -kp_pan * ex_f / dx_dpan - kd_pan * ex_dot
            tilt_vel = -kp_tilt * ey_f / dy_dtilt - kd_tilt * ey_dot

            # Clamp velocities
            pan_vel = np.clip(pan_vel, -vel_max_pan, vel_max_pan)
            tilt_vel = np.clip(tilt_vel, -vel_max_tilt, vel_max_tilt)

            # Check position limits
            if pan_pos <= pan_min and pan_vel < 0:
                pan_vel = 0
            if pan_pos >= pan_max and pan_vel > 0:
                pan_vel = 0
            if tilt_pos <= tilt_min and tilt_vel < 0:
                tilt_vel = 0
            if tilt_pos >= tilt_max and tilt_vel > 0:
                tilt_vel = 0

            # Send velocity command
            if not dry_run:
                motors.move_velocity(pan_vel, tilt_vel, duration=dt)
            
            # Update position estimate
            pan_pos += pan_vel * dt
            tilt_pos += tilt_vel * dt

            # Status
            now = time.time()
            if now - last_print > 0.5:
                print(
                    f"[TRACK] e=({ex_f:+.3f},{ey_f:+.3f}) v=({pan_vel:+.1f},{tilt_vel:+.1f}) "
                    f"pos=({pan_pos:+.2f},{tilt_pos:+.2f})"
                )
                last_print = now

            # Maintain loop rate
            elapsed = time.time() - loop_start
            time.sleep(max(0.0, dt - elapsed))

    except KeyboardInterrupt:
        print("\n[STOP] exiting")
        if not dry_run:
            motors.disconnect()
        return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Velocity control tracker")
    ap.add_argument("--config", default=str(Path(__file__).parent.parent / "config.json"))
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    return run(Path(args.config), dry_run=args.dry_run)


if __name__ == "__main__":
    raise SystemExit(main())
