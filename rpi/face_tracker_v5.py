#!/usr/bin/env python3
"""
Face Tracker V5 - Simple step-and-settle approach.

Strategy:
1. Measure face error (with EMA filtering)
2. Compute small step toward face using calibrated A-matrix
3. Send move command
4. Wait for motor to settle
5. Repeat

No velocity control, no rate limiting - just small discrete steps with settling.
"""

import json
import socket
import time
import sys
from pathlib import Path

# Motor control via Moonraker
import requests

# === Configuration ===
CONFIG_PATH = Path(__file__).parent.parent / "config.json"

# Load config
cfg = json.loads(CONFIG_PATH.read_text())
net = cfg["network"]
motors_cfg = cfg["motors"]
dec = cfg.get("tracking_decoupled", {})

UDP_PORT = net["udp_port"]
MOONRAKER_URL = motors_cfg["moonraker_url"]

# Calibration matrix from config
dx_dpan = dec.get("dx_dpan", 0.17165)
dy_dpan = dec.get("dy_dpan", 0.002462)
dx_dtilt = dec.get("dx_dtilt", -0.015925)
dy_dtilt = dec.get("dy_dtilt", 0.036075)

# Compute inverse A-matrix
det = (dx_dpan * dy_dtilt) - (dx_dtilt * dy_dpan)
print(f"A-matrix det = {det:.6f}")

# Motor limits
PAN_MIN = motors_cfg["pan"]["min"]
PAN_MAX = motors_cfg["pan"]["max"]
TILT_MIN = motors_cfg["tilt"]["min"]
TILT_MAX = motors_cfg["tilt"]["max"]
PAN_CENTER = motors_cfg["pan"].get("center", 0.0)
TILT_CENTER = motors_cfg["tilt"].get("center", 0.0)

# === Tuning parameters ===

""" 
Key ones to tune:

Parameter	Effect
SETTLE_TIME	                    ↑ = less oscillation, slower response
FRESH_MEAS_WAIT	                ↑ = cleaner measurements, slower
MAX_PAN_STEP / MAX_TILT_STEP	↑ = faster catch-up, may overshoot
MOVE_SPEED	                    ↑ = faster motor movement
STEP_GAIN	                    ↑ = more aggressive, may oscillate
DEADZONE	              ↑ = less jitter when centered, less precise 

# === Tuning parameters ===
DEADZONE = 0.07           # Don't move if error smaller than this
EMA_ALPHA = 0.45          # Measurement smoothing (0=responsive, 0.9=smooth)
MAX_PAN_STEP = 0.25       # Max pan step per cycle (degrees)
MAX_TILT_STEP = 0.07      # Max tilt step per cycle (degrees)
STEP_GAIN = 0.28          # How aggressive to chase error
SETTLE_TIME = 0.12        # Wait after move before measuring (seconds)
FRESH_MEAS_WAIT = 0.07    # Wait for fresh measurement after settle
MOVE_SPEED = 15           # Motor speed (deg/sec)
MOVE_ACCEL = 50           # Motor acceleration (deg/sec^2)
MIN_CONFIDENCE = 0.5      # Ignore low confidence detections
LOOP_PERIOD = 0.015       # How often to poll for data

"""

DEADZONE = 0.1           # Don't move if error smaller than this
EMA_ALPHA = 0.45          # Slightly faster response
MAX_PAN_STEP = 0.75       # Bigger steps = faster
MAX_TILT_STEP = 0.17      # Bigger tilt steps
STEP_GAIN = 0.28          # Slightly more aggressive
SETTLE_TIME = 0.4         # A bit more settle to avoid oscillation
FRESH_MEAS_WAIT = 0.07    # Wait for fresh measurement
MOVE_SPEED = 200          # Faster motor
MOVE_ACCEL = 50           # Bit more accel
MIN_CONFIDENCE = 0.5      # Ignore low confidence detections
LOOP_PERIOD = 0.015       # Faster polling

# === Dynamic step scaling ===
# Steps scale from MIN_STEP_SCALE (near deadzone) to 1.0 (far away)
# This gives: fast catch-up when far, precision when close
MIN_STEP_SCALE = 0.3      # Min scale near deadzone (30% of max step)
FAR_THRESHOLD = 0.5       # Error above this = full step size
# Linear interpolation between DEADZONE and FAR_THRESHOLD

# === State ===
pan_pos = PAN_CENTER
tilt_pos = TILT_CENTER
ex_filt = 0.0
ey_filt = 0.0
filter_init = False

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def send_gcode(gcode: str) -> bool:
    """Send G-code via Moonraker."""
    try:
        url = f"{MOONRAKER_URL}/printer/gcode/script"
        resp = requests.post(url, json={"script": gcode}, timeout=2)
        return resp.status_code == 200
    except Exception as e:
        print(f"[WARN] Gcode failed: {e}")
        return False

def move_pan_tilt(pan: float, tilt: float, speed: float = MOVE_SPEED, accel: float = MOVE_ACCEL):
    """Move both motors with SYNC=0 (non-blocking)."""
    gcode = (
        f"MANUAL_STEPPER STEPPER=stepper_0 MOVE={pan:.4f} SPEED={speed} ACCEL={accel} SYNC=0\n"
        f"MANUAL_STEPPER STEPPER=stepper_1 MOVE={tilt:.4f} SPEED={speed} ACCEL={accel} SYNC=0"
    )
    send_gcode(gcode)

def compute_step(ex: float, ey: float) -> tuple:
    """
    Given image error (ex, ey), compute motor step (dpan, dtilt).
    Uses inverse of calibration matrix.
    
    The A-matrix maps motor->image: [dx, dy] = A * [dpan, dtilt]
    So we need: [dpan, dtilt] = inv(A) * [dx, dy]
    
    But we want to REDUCE error, so step = -inv(A) * [ex, ey]
    """
    # inv(A) = 1/det * [[dy_dtilt, -dx_dtilt], [-dy_dpan, dx_dpan]]
    dpan = (dy_dtilt * ex - dx_dtilt * ey) / det
    dtilt = (-dy_dpan * ex + dx_dpan * ey) / det
    
    # Negate to reduce error (if face is right of center, move camera right)
    dpan = -dpan
    dtilt = -dtilt
    
    return dpan, dtilt

def main():
    global pan_pos, tilt_pos, ex_filt, ey_filt, filter_init
    
    # Set up UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.setblocking(False)
    
    print("=" * 60)
    print("FACE TRACKER V5 - Step and Settle")
    print(f"UDP port: {UDP_PORT}")
    print(f"Deadzone: {DEADZONE} | EMA: {EMA_ALPHA}")
    print(f"Max step: pan={MAX_PAN_STEP}° tilt={MAX_TILT_STEP}°")
    print(f"Step gain: {STEP_GAIN} | Settle time: {SETTLE_TIME}s")
    print(f"A-matrix: dx_dpan={dx_dpan:.4f} dy_dtilt={dy_dtilt:.4f}")
    print("=" * 60)
    
    # Initialize motors to center
    move_pan_tilt(pan_pos, tilt_pos)
    time.sleep(0.5)
    
    last_move_time = 0.0
    last_print_time = 0.0
    waiting_for_fresh = False
    fresh_wait_start = 0.0
    
    try:
        while True:
            now = time.time()
            
            # Read latest UDP packet (non-blocking, get most recent)
            latest_data = None
            while True:
                try:
                    data, _ = sock.recvfrom(4096)
                    latest_data = data
                except BlockingIOError:
                    break
            
            # Parse packet
            if latest_data:
                try:
                    msg = json.loads(latest_data.decode())
                    detected = msg.get("detected", False)
                    confidence = msg.get("confidence", 0.0)
                    
                    if detected and confidence >= MIN_CONFIDENCE:
                        ex_raw = msg.get("offset_x", 0.0)
                        ey_raw = msg.get("offset_y", 0.0)
                        
                        # Apply EMA filter
                        if not filter_init:
                            ex_filt = ex_raw
                            ey_filt = ey_raw
                            filter_init = True
                        else:
                            ex_filt = EMA_ALPHA * ex_filt + (1 - EMA_ALPHA) * ex_raw
                            ey_filt = EMA_ALPHA * ey_filt + (1 - EMA_ALPHA) * ey_raw
                except (json.JSONDecodeError, KeyError):
                    pass
            
            # State machine: SETTLING -> WAITING_FOR_FRESH -> READY_TO_MOVE
            
            # Phase 1: Settling (motor moving, ignore measurements)
            if (now - last_move_time) < SETTLE_TIME:
                time.sleep(LOOP_PERIOD)
                continue
            
            # Phase 2: Transition from settling to waiting for fresh measurement
            if not waiting_for_fresh and last_move_time > 0:
                waiting_for_fresh = True
                fresh_wait_start = now
                # Reset filter to get fresh reading
                filter_init = False
                ex_filt = 0.0
                ey_filt = 0.0
                time.sleep(LOOP_PERIOD)
                continue
            
            # Phase 3: Waiting for fresh measurement after settle
            if waiting_for_fresh:
                if (now - fresh_wait_start) < FRESH_MEAS_WAIT:
                    time.sleep(LOOP_PERIOD)
                    continue
                # Done waiting, proceed to move decision
                waiting_for_fresh = False
            
            # Check if error is outside deadzone
            error_mag = (ex_filt**2 + ey_filt**2) ** 0.5
            
            if error_mag < DEADZONE:
                # Inside deadzone - no movement needed
                if (now - last_print_time) > 1.0:
                    print(f"[OK] e=({ex_filt:+.3f},{ey_filt:+.3f}) pos=({pan_pos:+.2f},{tilt_pos:+.2f}) - in deadzone")
                    last_print_time = now
                time.sleep(LOOP_PERIOD)
                continue
            
            # Compute step
            dpan, dtilt = compute_step(ex_filt, ey_filt)
            
            # Dynamic step scaling: bigger steps when far, smaller when close
            # Scale from MIN_STEP_SCALE at deadzone to 1.0 at FAR_THRESHOLD
            if error_mag >= FAR_THRESHOLD:
                step_scale = 1.0
            else:
                # Linear interpolation between DEADZONE and FAR_THRESHOLD
                t = (error_mag - DEADZONE) / (FAR_THRESHOLD - DEADZONE)
                step_scale = MIN_STEP_SCALE + t * (1.0 - MIN_STEP_SCALE)
            
            # Apply gain, scaling, and clamp
            dpan = clamp(dpan * STEP_GAIN * step_scale, -MAX_PAN_STEP, MAX_PAN_STEP)
            dtilt = clamp(dtilt * STEP_GAIN * step_scale, -MAX_TILT_STEP, MAX_TILT_STEP)
            
            # Update positions
            new_pan = clamp(pan_pos + dpan, PAN_MIN, PAN_MAX)
            new_tilt = clamp(tilt_pos + dtilt, TILT_MIN, TILT_MAX)
            
            # Only move if step is meaningful
            if abs(new_pan - pan_pos) > 0.01 or abs(new_tilt - tilt_pos) > 0.01:
                print(f"[MOVE] e=({ex_filt:+.3f},{ey_filt:+.3f}) |e|={error_mag:.2f} scale={step_scale:.2f} step=({dpan:+.3f},{dtilt:+.3f}) -> pos=({new_pan:+.2f},{new_tilt:+.2f})")
                
                pan_pos = new_pan
                tilt_pos = new_tilt
                move_pan_tilt(pan_pos, tilt_pos)
                last_move_time = now
                last_print_time = now
            
            time.sleep(LOOP_PERIOD)
    
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Return to center
        print("Returning to center...")
        move_pan_tilt(PAN_CENTER, TILT_CENTER)
        sock.close()

if __name__ == "__main__":
    main()
