#!/usr/bin/env python3
"""
Face Tracker - Smooth Velocity Mode (v2)
=========================================
Uses the winning motor tuning discovered on January 2, 2026.

Key principles (from MOTOR_TUNING_REPORT.md):
- Only send command when target changes > DEADZONE
- Let motor glide smoothly to target (don't spam positions)
- Speed proportional to error (bigger error = faster correction)
- Low acceleration (100°/s²) for smooth motion
"""

import json
import socket
import time
import threading
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).parent.parent / "shared"))

from klipper_direct import KlipperDirect

# =============================================================================
# CONFIGURATION
# =============================================================================

config_path = Path(__file__).parent.parent / "config.json"
with open(config_path) as f:
    config = json.load(f)

# Motor limits
PAN_MIN = config["motors"]["pan"]["min"]    # -22
PAN_MAX = config["motors"]["pan"]["max"]    # +22
TILT_MIN = config["motors"]["tilt"]["min"]  # -2.5
TILT_MAX = config["motors"]["tilt"]["max"]  # +5

# =============================================================================
# TUNING PARAMETERS - FROM MOTOR_TUNING_REPORT.md (Jan 2, 2026)
# =============================================================================

# Speed settings (degrees/sec)
MAX_SPEED = 30       # Maximum motor speed
MIN_SPEED = 10       # Minimum motor speed (for small corrections)
SPEED_GAIN = 3.0     # speed = error * SPEED_GAIN (capped at MAX_SPEED)

# Acceleration (degrees/sec²) - LOW for smooth motion
ACCEL = 100

# Deadzone - only send new command if target changed more than this
DEADZONE = 0.5  # degrees

# Face-to-motor mapping gains
# Jetson sends normalized offsets (-1 to +1), multiply by max angle
PAN_GAIN = 22.0      # Max pan angle (maps -1..+1 to -22..+22)
TILT_GAIN = 5.0      # Max tilt angle (asymmetric: -2.5 to +5)

# Center offset (if camera is not perfectly aligned)
PAN_CENTER_OFFSET = 0.0
TILT_CENTER_OFFSET = 0.0

# UDP settings
UDP_PORT = 5555

# =============================================================================
# GLOBALS
# =============================================================================

klipper = None
current_pan = 0.0
current_tilt = 0.0
target_pan = 0.0
target_tilt = 0.0
last_sent_pan = 0.0
last_sent_tilt = 0.0

stats = {
    "packets_received": 0,
    "commands_sent": 0,
    "last_face_x": 0,
    "last_face_y": 0,
    "fps": 0,
}

running = True

# =============================================================================
# KLIPPER CONNECTION
# =============================================================================

def connect_klipper():
    global klipper
    socket_path = "/home/acp/printer_data/comms/klippy.sock"
    
    try:
        klipper = KlipperDirect(socket_path)
        if klipper.connect():
            print(f"[KLIPPER] Connected to {socket_path}")
            
            # Enable motors
            klipper.gcode("MANUAL_STEPPER STEPPER=stepper_0 ENABLE=1")
            klipper.gcode("MANUAL_STEPPER STEPPER=stepper_1 ENABLE=1")
            
            # Set current position as 0
            klipper.gcode("MANUAL_STEPPER STEPPER=stepper_0 SET_POSITION=0")
            klipper.gcode("MANUAL_STEPPER STEPPER=stepper_1 SET_POSITION=0")
            
            print("[KLIPPER] Motors enabled and zeroed")
            return True
    except Exception as e:
        print(f"[KLIPPER] Connection error: {e}")
    
    return False

# =============================================================================
# MOTOR CONTROL - SMOOTH VELOCITY MODE
# =============================================================================

def send_motor_command(pan: float, tilt: float):
    """
    Send smooth move command to motors.
    Only sends if target changed significantly from last sent position.
    """
    global last_sent_pan, last_sent_tilt, stats
    
    # Clamp to limits
    pan = max(PAN_MIN, min(PAN_MAX, pan))
    tilt = max(TILT_MIN, min(TILT_MAX, tilt))
    
    # Check if change is significant
    pan_changed = abs(pan - last_sent_pan) > DEADZONE
    tilt_changed = abs(tilt - last_sent_tilt) > DEADZONE
    
    if not (pan_changed or tilt_changed):
        return  # No significant change, don't send
    
    # Calculate speed based on error (proportional)
    pan_error = abs(pan - last_sent_pan)
    tilt_error = abs(tilt - last_sent_tilt)
    
    pan_speed = min(MAX_SPEED, max(MIN_SPEED, pan_error * SPEED_GAIN))
    tilt_speed = min(MAX_SPEED, max(MIN_SPEED, tilt_error * SPEED_GAIN))
    
    # Send commands (SYNC=0 for non-blocking, smooth motion)
    if klipper:
        if pan_changed:
            cmd = f"MANUAL_STEPPER STEPPER=stepper_0 MOVE={pan:.2f} SPEED={pan_speed:.0f} ACCEL={ACCEL} SYNC=0"
            klipper.gcode(cmd, wait=False)
        
        if tilt_changed:
            cmd = f"MANUAL_STEPPER STEPPER=stepper_1 MOVE={tilt:.2f} SPEED={tilt_speed:.0f} ACCEL={ACCEL} SYNC=0"
            klipper.gcode(cmd, wait=False)
        
        last_sent_pan = pan
        last_sent_tilt = tilt
        stats["commands_sent"] += 1

# =============================================================================
# FACE POSITION PROCESSING
# =============================================================================

def process_face_position(offset_x: float, offset_y: float):
    """
    Convert face offset to motor angles.
    
    offset_x, offset_y: Normalized offset from center (-1 to +1)
      - Positive offset_x = face is RIGHT of center
      - Positive offset_y = face is BELOW center
    
    Returns: (pan_angle, tilt_angle) in degrees
    """
    global target_pan, target_tilt
    
    # Convert normalized offset to motor angles
    # Face offset tells us where face IS - we need to move gimbal OPPOSITE direction
    # to catch up and center the face!
    # PAN: face on RIGHT (X+) → gimbal needs to turn RIGHT → motor NEGATIVE
    # TILT: face on TOP (Y-) → gimbal needs to tilt UP → motor POSITIVE
    pan_delta = -offset_x * PAN_GAIN   # INVERTED - follow face, not run away!
    tilt_delta = -offset_y * TILT_GAIN  # INVERTED for correct direction
    
    # Apply center offsets
    target_pan = pan_delta + PAN_CENTER_OFFSET
    target_tilt = tilt_delta + TILT_CENTER_OFFSET
    
    # Update stats
    stats["last_face_x"] = offset_x
    stats["last_face_y"] = offset_y
    
    return target_pan, target_tilt

# =============================================================================
# UDP RECEIVER
# =============================================================================

def udp_receiver():
    """Receive face positions from Jetson via UDP"""
    global stats, running
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', UDP_PORT))
    sock.settimeout(1.0)
    
    print(f"[UDP] Listening on port {UDP_PORT}")
    
    last_fps_time = time.time()
    fps_count = 0
    
    while running:
        try:
            data, addr = sock.recvfrom(4096)
            msg = json.loads(data.decode('utf-8'))
            
            stats["packets_received"] += 1
            fps_count += 1
            
            # Calculate FPS
            now = time.time()
            if now - last_fps_time >= 1.0:
                stats["fps"] = fps_count
                fps_count = 0
                last_fps_time = now
            
            # Extract face position - Jetson sends normalized offsets
            face_detected = msg.get("face_detected") or msg.get("detected", False)
            
            if face_detected:
                # Use offset_x/offset_y (normalized -1 to +1 from center)
                offset_x = msg.get("offset_x", msg.get("x_offset", msg.get("x", 0)))
                offset_y = msg.get("offset_y", msg.get("y_offset", msg.get("y", 0)))
                
                # Convert to motor angles
                pan, tilt = process_face_position(offset_x, offset_y)
                
                # Send motor command (only if significant change)
                send_motor_command(pan, tilt)
            
        except socket.timeout:
            continue
        except json.JSONDecodeError:
            continue
        except Exception as e:
            print(f"[UDP] Error: {e}")
    
    sock.close()

# =============================================================================
# STATUS DISPLAY
# =============================================================================

def status_display():
    """Print status every second"""
    global running
    while running:
        print(f"\r[TRACK] Pan:{target_pan:+6.1f}° Tilt:{target_tilt:+6.1f}° | "
              f"Offset:({stats['last_face_x']:+.2f},{stats['last_face_y']:+.2f}) | "
              f"FPS:{stats['fps']} | Cmds:{stats['commands_sent']}   ", end="", flush=True)
        time.sleep(0.5)

# =============================================================================
# MAIN
# =============================================================================

def main():
    global running
    
    print("=" * 60)
    print("FACE TRACKER v2 - SMOOTH VELOCITY MODE")
    print("=" * 60)
    print(f"Pan range:  {PAN_MIN}° to {PAN_MAX}°")
    print(f"Tilt range: {TILT_MIN}° to {TILT_MAX}°")
    print(f"Max speed:  {MAX_SPEED}°/s")
    print(f"Accel:      {ACCEL}°/s²")
    print(f"Deadzone:   {DEADZONE}°")
    print(f"Pan gain:   {PAN_GAIN} °/pixel")
    print(f"Tilt gain:  {TILT_GAIN} °/pixel")
    print("=" * 60)
    
    # Connect to Klipper
    if not connect_klipper():
        print("[ERROR] Failed to connect to Klipper")
        return
    
    # Start UDP receiver thread
    udp_thread = threading.Thread(target=udp_receiver, daemon=True)
    udp_thread.start()
    
    # Start status display thread
    status_thread = threading.Thread(target=status_display, daemon=True)
    status_thread.start()
    
    print("\n[READY] Waiting for face data from Jetson...")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n[STOP] Shutting down...")
        running = False
        if klipper:
            klipper.gcode("STOP_ALL_MOTORS")
        print("[DONE]")


if __name__ == "__main__":
    main()
