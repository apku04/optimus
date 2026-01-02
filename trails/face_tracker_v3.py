#!/usr/bin/env python3
"""
Face Tracker v3 - Using Working Tracking Logic
===============================================
Takes the WORKING tracking math from head_tracker_decoupled.py
and uses our smooth velocity-mode motor control.

Key: Uses calibrated A-matrix inverse for proper direction mapping.
"""

import json
import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# =============================================================================
# CONFIGURATION - Load from config.json
# =============================================================================

config_path = Path(__file__).parent.parent / "config.json"
with open(config_path) as f:
    config = json.load(f)

# Motor limits
pan_cfg = config["motors"]["pan"]
tilt_cfg = config["motors"]["tilt"]
PAN_MIN = float(pan_cfg["min"])
PAN_MAX = float(pan_cfg["max"])
TILT_MIN = float(tilt_cfg["min"])
TILT_MAX = float(tilt_cfg["max"])

# Calibrated A-matrix (from tracking_decoupled)
dec = config.get("tracking_decoupled", {})
dx_dpan = float(dec.get("dx_dpan", 0.17))
dx_dtilt = float(dec.get("dx_dtilt", 0.0))
dy_dpan = float(dec.get("dy_dpan", 0.0))
dy_dtilt = float(dec.get("dy_dtilt", 0.036))

# Determinant of A matrix
det = (dx_dpan * dy_dtilt) - (dx_dtilt * dy_dpan)
if abs(det) < 1e-9:
    print("[ERROR] A matrix is singular - run calibration!")
    det = 0.006  # Fallback

# Tracking params
alpha = float(dec.get("alpha", 0.8))
ema = float(dec.get("ema", 0.5))
deadzone = float(dec.get("deadzone_hysteresis", 0.02))
max_step_pan = float(dec.get("max_step_pan_deg", 0.8))
max_step_tilt = float(dec.get("max_step_tilt_deg", 0.32))
min_step_pan = float(dec.get("min_step_pan_deg", 0.04))
min_step_tilt = float(dec.get("min_step_tilt_deg", 0.03))

# Speed settings
pan_speed = float(dec.get("pan_speed", 80.0))
tilt_speed = float(dec.get("tilt_speed", 55.0))
pan_accel = float(pan_cfg.get("accel", 100.0))
tilt_accel = float(tilt_cfg.get("accel", 100.0))

# Inversion flags from config (if present)
invert_pan = bool(dec.get("invert_pan", False))
invert_tilt = bool(dec.get("invert_tilt", False))

# UDP settings
UDP_PORT = config["network"]["udp_port"]

# =============================================================================
# FACE PACKET
# =============================================================================

@dataclass
class FacePacket:
    x: float
    y: float
    detected: bool
    confidence: float
    timestamp: float


def parse_packet(msg: str) -> Optional[FacePacket]:
    """Parse UDP packet from Jetson."""
    msg = msg.strip()
    if not msg.startswith("{"):
        return None
    
    try:
        data = json.loads(msg)
    except json.JSONDecodeError:
        return None
    
    detected = bool(data.get("detected", data.get("face_detected", False)))
    
    # Accept various key formats
    x = data.get("x", data.get("x_offset", data.get("offset_x", 0.0)))
    y = data.get("y", data.get("y_offset", data.get("offset_y", 0.0)))
    
    try:
        x = float(x)
        y = float(y)
    except (TypeError, ValueError):
        return None
    
    conf = float(data.get("confidence", 1.0) or 1.0)
    ts = float(data.get("timestamp", time.time()) or time.time())
    
    return FacePacket(
        x=max(-1.0, min(1.0, x)),
        y=max(-1.0, min(1.0, y)),
        detected=detected,
        confidence=conf,
        timestamp=ts,
    )


# =============================================================================
# MOTOR CONTROL via Moonraker
# =============================================================================

import requests

MOONRAKER_URL = "http://localhost:7125"

def send_gcode(cmd: str):
    """Send G-code via Moonraker."""
    try:
        requests.post(
            f"{MOONRAKER_URL}/printer/gcode/script",
            json={"script": cmd},
            timeout=0.5
        )
    except Exception as e:
        print(f"[ERROR] {e}")


def move_motors(pan: float, tilt: float, pan_spd: float, tilt_spd: float):
    """Send smooth move commands to motors."""
    # Clamp to limits
    pan = max(PAN_MIN, min(PAN_MAX, pan))
    tilt = max(TILT_MIN, min(TILT_MAX, tilt))
    
    # Send SYNC=0 for non-blocking smooth motion
    cmd = f"MANUAL_STEPPER STEPPER=stepper_0 MOVE={pan:.3f} SPEED={pan_spd:.0f} ACCEL={pan_accel:.0f} SYNC=0"
    send_gcode(cmd)
    
    cmd = f"MANUAL_STEPPER STEPPER=stepper_1 MOVE={tilt:.3f} SPEED={tilt_spd:.0f} ACCEL={tilt_accel:.0f} SYNC=0"
    send_gcode(cmd)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# =============================================================================
# MAIN TRACKING LOOP
# =============================================================================

def main():
    print("=" * 60)
    print("FACE TRACKER v3 - Using Calibrated A-Matrix")
    print("=" * 60)
    print(f"A matrix: dx_dpan={dx_dpan:.4f} dy_dtilt={dy_dtilt:.4f}")
    print(f"          dx_dtilt={dx_dtilt:.4f} dy_dpan={dy_dpan:.4f}")
    print(f"          det={det:.6f}")
    print(f"alpha={alpha} ema={ema}")
    print(f"Pan: {PAN_MIN}° to {PAN_MAX}° | Tilt: {TILT_MIN}° to {TILT_MAX}°")
    print(f"invert_pan={invert_pan} invert_tilt={invert_tilt}")
    print("=" * 60)
    
    # Setup UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.setblocking(False)
    
    print(f"[UDP] Listening on port {UDP_PORT}")
    
    # Initialize motors at center
    pan = 0.0
    tilt = 0.0
    send_gcode("MANUAL_STEPPER STEPPER=stepper_0 ENABLE=1")
    send_gcode("MANUAL_STEPPER STEPPER=stepper_1 ENABLE=1")
    move_motors(0, 0, pan_speed, tilt_speed)
    print("[MOTORS] Enabled and centered")
    
    # State
    filt_x, filt_y = 0.0, 0.0
    filt_init = False
    last_cmd_time = 0.0
    cmd_count = 0
    
    print("\n[READY] Waiting for face data...")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            loop_start = time.time()
            
            # Drain UDP buffer, keep latest
            pkt = None
            while True:
                try:
                    data, _ = sock.recvfrom(4096)
                    p = parse_packet(data.decode("utf-8", errors="ignore"))
                    if p is not None:
                        pkt = p
                except BlockingIOError:
                    break
                except Exception:
                    break
            
            if pkt is None or not pkt.detected:
                time.sleep(0.02)
                continue
            
            # EMA filter
            if not filt_init:
                filt_x, filt_y = pkt.x, pkt.y
                filt_init = True
            else:
                filt_x = (ema * filt_x) + ((1.0 - ema) * pkt.x)
                filt_y = (ema * filt_y) + ((1.0 - ema) * pkt.y)
            
            # Check deadzone
            max_err = max(abs(filt_x), abs(filt_y))
            if max_err < deadzone:
                time.sleep(0.02)
                continue
            
            # =========================================================
            # THE WORKING FORMULA from head_tracker_decoupled.py
            # delta = -inv(A) * [x; y]
            # =========================================================
            delta_pan = -((dy_dtilt * filt_x) + (-dx_dtilt * filt_y)) / det
            delta_tilt = -((-dy_dpan * filt_x) + (dx_dpan * filt_y)) / det
            
            # Apply gain (alpha)
            delta_pan *= alpha
            delta_tilt *= alpha
            
            # Clamp step size
            delta_pan = _clamp(delta_pan, -max_step_pan, max_step_pan)
            delta_tilt = _clamp(delta_tilt, -max_step_tilt, max_step_tilt)
            
            # Apply inversion if configured
            if invert_pan:
                delta_pan *= -1.0
            if invert_tilt:
                delta_tilt *= -1.0
            
            # Skip tiny steps
            if abs(delta_pan) < min_step_pan:
                delta_pan = 0.0
            if abs(delta_tilt) < min_step_tilt:
                delta_tilt = 0.0
            
            if delta_pan == 0.0 and delta_tilt == 0.0:
                time.sleep(0.02)
                continue
            
            # Compute target position
            target_pan = _clamp(pan + delta_pan, PAN_MIN, PAN_MAX)
            target_tilt = _clamp(tilt + delta_tilt, TILT_MIN, TILT_MAX)
            
            # Send move
            move_motors(target_pan, target_tilt, pan_speed, tilt_speed)
            pan, tilt = target_pan, target_tilt
            cmd_count += 1
            last_cmd_time = time.time()
            
            # Status
            print(f"\r[TRACK] Pan:{pan:+6.2f}° Tilt:{tilt:+5.2f}° | "
                  f"Offset:({filt_x:+.2f},{filt_y:+.2f}) | "
                  f"Δ:({delta_pan:+.2f},{delta_tilt:+.2f}) | "
                  f"Cmds:{cmd_count}   ", end="", flush=True)
            
            # Small delay to avoid flooding
            elapsed = time.time() - loop_start
            if elapsed < 0.025:
                time.sleep(0.025 - elapsed)
    
    except KeyboardInterrupt:
        print("\n\n[STOP] Shutting down...")
    finally:
        send_gcode("MANUAL_STEPPER STEPPER=stepper_0 ENABLE=0")
        send_gcode("MANUAL_STEPPER STEPPER=stepper_1 ENABLE=0")
        sock.close()
        print("[DONE]")


if __name__ == "__main__":
    main()
