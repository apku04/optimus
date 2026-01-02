#!/usr/bin/env python3
"""
Face Tracker V6 - Smooth continuous motion.

Strategy:
- Continuously update target position (no waiting between moves)
- Use EMA filtering for smooth error signal
- Motor glides smoothly with SYNC=0
- Deadzone prevents jitter when centered
- Rate limiting prevents overshoot
"""

import json
import socket
import time
from pathlib import Path
import requests

# === Configuration ===
CONFIG_PATH = Path(__file__).parent.parent / "config.json"

cfg = json.loads(CONFIG_PATH.read_text())
net = cfg["network"]
motors_cfg = cfg["motors"]
dec = cfg.get("tracking_decoupled", {})

UDP_PORT = net["udp_port"]
MOONRAKER_URL = motors_cfg["moonraker_url"]

# Calibration matrix
dx_dpan = dec.get("dx_dpan", 0.17165)
dy_dpan = dec.get("dy_dpan", 0.002462)
dx_dtilt = dec.get("dx_dtilt", -0.015925)
dy_dtilt = dec.get("dy_dtilt", 0.036075)

det = (dx_dpan * dy_dtilt) - (dx_dtilt * dy_dpan)

# Motor limits
PAN_MIN = motors_cfg["pan"]["min"]
PAN_MAX = motors_cfg["pan"]["max"]
TILT_MIN = motors_cfg["tilt"]["min"]
TILT_MAX = motors_cfg["tilt"]["max"]
PAN_CENTER = motors_cfg["pan"].get("center", 0.0)
TILT_CENTER = motors_cfg["tilt"].get("center", 0.0)

# === Tuning parameters ===
DEADZONE = 0.06           # Don't move if error smaller than this
EMA_ALPHA = 0.75          # Measurement smoothing (higher = smoother but slower)
CMD_EMA = 0.85            # Command smoothing (higher = smoother motion)
MAX_PAN_RATE = 8.0        # Max degrees per second for pan
MAX_TILT_RATE = 4.0       # Max degrees per second for tilt
STEP_GAIN = 0.5           # How aggressive to chase error
MOVE_SPEED = 25           # Motor speed (deg/sec) - fast for responsiveness
MOVE_ACCEL = 120          # Motor acceleration (deg/sec^2)
MIN_CONFIDENCE = 0.5      # Ignore low confidence detections
LOOP_HZ = 30              # Control loop frequency
CMD_INTERVAL = 0.05       # Send command every 50ms (don't spam motor)

# State
pan_pos = PAN_CENTER
tilt_pos = TILT_CENTER
pan_target = PAN_CENTER
tilt_target = TILT_CENTER
ex_filt = 0.0
ey_filt = 0.0
filter_init = False

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def send_gcode(gcode: str) -> bool:
    try:
        url = f"{MOONRAKER_URL}/printer/gcode/script"
        resp = requests.post(url, json={"script": gcode}, timeout=1)
        return resp.status_code == 200
    except:
        return False

def move_smooth(pan: float, tilt: float):
    """Send smooth move command with SYNC=0."""
    gcode = (
        f"MANUAL_STEPPER STEPPER=stepper_0 MOVE={pan:.4f} SPEED={MOVE_SPEED} ACCEL={MOVE_ACCEL} SYNC=0\n"
        f"MANUAL_STEPPER STEPPER=stepper_1 MOVE={tilt:.4f} SPEED={MOVE_SPEED} ACCEL={MOVE_ACCEL} SYNC=0"
    )
    send_gcode(gcode)

def compute_step(ex: float, ey: float) -> tuple:
    """Compute motor step from image error using inverse A-matrix."""
    dpan = (dy_dtilt * ex - dx_dtilt * ey) / det
    dtilt = (-dy_dpan * ex + dx_dpan * ey) / det
    return -dpan, -dtilt  # Negate to reduce error

def main():
    global pan_pos, tilt_pos, pan_target, tilt_target, ex_filt, ey_filt, filter_init
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.setblocking(False)
    
    print("=" * 60)
    print("FACE TRACKER V6 - Smooth Continuous Motion")
    print(f"UDP port: {UDP_PORT} | Loop: {LOOP_HZ}Hz")
    print(f"Deadzone: {DEADZONE} | EMA: {EMA_ALPHA} | CMD_EMA: {CMD_EMA}")
    print(f"Max rate: pan={MAX_PAN_RATE}°/s tilt={MAX_TILT_RATE}°/s")
    print("=" * 60)
    
    # Initialize
    move_smooth(pan_pos, tilt_pos)
    time.sleep(0.3)
    
    loop_dt = 1.0 / LOOP_HZ
    last_cmd_time = 0.0
    last_print_time = 0.0
    last_loop_time = time.time()
    
    try:
        while True:
            now = time.time()
            dt = now - last_loop_time
            last_loop_time = now
            
            # Read latest UDP packet
            latest_data = None
            while True:
                try:
                    data, _ = sock.recvfrom(4096)
                    latest_data = data
                except BlockingIOError:
                    break
            
            # Parse and filter measurement
            if latest_data:
                try:
                    msg = json.loads(latest_data.decode())
                    if msg.get("detected") and msg.get("confidence", 0) >= MIN_CONFIDENCE:
                        ex_raw = msg.get("offset_x", 0.0)
                        ey_raw = msg.get("offset_y", 0.0)
                        
                        if not filter_init:
                            ex_filt = ex_raw
                            ey_filt = ey_raw
                            filter_init = True
                        else:
                            ex_filt = EMA_ALPHA * ex_filt + (1 - EMA_ALPHA) * ex_raw
                            ey_filt = EMA_ALPHA * ey_filt + (1 - EMA_ALPHA) * ey_raw
                except:
                    pass
            
            # Compute desired step
            error_mag = (ex_filt**2 + ey_filt**2) ** 0.5
            
            if error_mag < DEADZONE:
                # In deadzone - hold position
                dpan_desired = 0.0
                dtilt_desired = 0.0
            else:
                # Compute step from error
                dpan_desired, dtilt_desired = compute_step(ex_filt, ey_filt)
                dpan_desired *= STEP_GAIN
                dtilt_desired *= STEP_GAIN
            
            # Rate limit the step
            max_dpan = MAX_PAN_RATE * dt
            max_dtilt = MAX_TILT_RATE * dt
            dpan_limited = clamp(dpan_desired, -max_dpan, max_dpan)
            dtilt_limited = clamp(dtilt_desired, -max_dtilt, max_dtilt)
            
            # Smooth the target with EMA (prevents jerky updates)
            new_pan = clamp(pan_pos + dpan_limited, PAN_MIN, PAN_MAX)
            new_tilt = clamp(tilt_pos + dtilt_limited, TILT_MIN, TILT_MAX)
            
            pan_target = CMD_EMA * pan_target + (1 - CMD_EMA) * new_pan
            tilt_target = CMD_EMA * tilt_target + (1 - CMD_EMA) * new_tilt
            
            # Update internal position (for rate limiting reference)
            pan_pos = new_pan
            tilt_pos = new_tilt
            
            # Send command at controlled rate
            if (now - last_cmd_time) >= CMD_INTERVAL:
                move_smooth(pan_target, tilt_target)
                last_cmd_time = now
            
            # Print status
            if (now - last_print_time) > 0.5:
                status = "OK" if error_mag < DEADZONE else "TRACK"
                print(f"[{status}] e=({ex_filt:+.3f},{ey_filt:+.3f}) pos=({pan_target:+.2f},{tilt_target:+.2f})")
                last_print_time = now
            
            # Sleep to maintain loop rate
            elapsed = time.time() - now
            sleep_time = max(0, loop_dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        print("Returning to center...")
        move_smooth(PAN_CENTER, TILT_CENTER)
        sock.close()

if __name__ == "__main__":
    main()
