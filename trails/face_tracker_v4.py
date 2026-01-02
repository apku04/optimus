#!/usr/bin/env python3
"""
Face Tracker v4 - Simple Direct Control
========================================
Based on visual calibration:
- Pan +motor → face moves RIGHT on screen (X increases)
- So to follow: pan = -X * gain

No complex A-matrix, just simple proportional control.
"""

import json
import socket
import time
import requests

# =============================================================================
# SIMPLE CONFIG
# =============================================================================

# Motor limits
PAN_MIN, PAN_MAX = -22.0, 22.0
TILT_MIN, TILT_MAX = -2.5, 5.0

# Gains - how many degrees per unit offset
PAN_GAIN = 15.0    # degrees per normalized offset
TILT_GAIN = 4.0

# Direction signs (from visual calibration)
PAN_SIGN = -1    # Pan+ moves face RIGHT, so invert to follow
TILT_SIGN = -1   # TBD - assuming same

# Control params
DEADZONE = 0.05  # Don't move if offset < this
SPEED = 30       # degrees/sec
ACCEL = 100      # degrees/sec^2

# Network
UDP_PORT = 5555
MOONRAKER = "http://localhost:7125"


def send_gcode(cmd):
    try:
        requests.post(f"{MOONRAKER}/printer/gcode/script", 
                      json={"script": cmd}, timeout=0.5)
    except:
        pass


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    print("=" * 50)
    print("FACE TRACKER v4 - Simple Direct Control")
    print("=" * 50)
    print(f"PAN_SIGN={PAN_SIGN} TILT_SIGN={TILT_SIGN}")
    print(f"PAN_GAIN={PAN_GAIN} TILT_GAIN={TILT_GAIN}")
    print("=" * 50)
    
    # UDP setup
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.setblocking(False)
    print(f"[UDP] Listening on port {UDP_PORT}")
    
    # Init motors
    send_gcode("MANUAL_STEPPER STEPPER=stepper_0 ENABLE=1")
    send_gcode("MANUAL_STEPPER STEPPER=stepper_1 ENABLE=1")
    send_gcode(f"MANUAL_STEPPER STEPPER=stepper_0 MOVE=0 SPEED={SPEED} ACCEL={ACCEL}")
    send_gcode(f"MANUAL_STEPPER STEPPER=stepper_1 MOVE=0 SPEED={SPEED} ACCEL={ACCEL}")
    print("[MOTORS] Centered")
    
    pan, tilt = 0.0, 0.0
    last_pan, last_tilt = 0.0, 0.0
    cmd_count = 0
    
    print("\n[READY] Tracking... Ctrl+C to stop\n")
    
    try:
        while True:
            # Drain UDP, keep latest
            pkt = None
            while True:
                try:
                    data, _ = sock.recvfrom(4096)
                    try:
                        msg = json.loads(data.decode())
                        if msg.get("detected", msg.get("face_detected", False)):
                            x = float(msg.get("x", msg.get("offset_x", 0)))
                            y = float(msg.get("y", msg.get("offset_y", 0)))
                            pkt = (x, y)
                    except:
                        pass
                except BlockingIOError:
                    break
            
            if pkt is None:
                time.sleep(0.02)
                continue
            
            x, y = pkt
            
            # Skip if in deadzone
            if abs(x) < DEADZONE and abs(y) < DEADZONE:
                time.sleep(0.02)
                continue
            
            # Simple proportional control
            # The offset IS the error - we want to drive it to zero
            # So target position = sign * offset * gain (absolute, not delta!)
            target_pan = clamp(PAN_SIGN * x * PAN_GAIN, PAN_MIN, PAN_MAX)
            target_tilt = clamp(TILT_SIGN * y * TILT_GAIN, TILT_MIN, TILT_MAX)
            
            # Only send if changed enough
            if abs(target_pan - last_pan) > 0.3 or abs(target_tilt - last_tilt) > 0.2:
                send_gcode(f"MANUAL_STEPPER STEPPER=stepper_0 MOVE={target_pan:.2f} SPEED={SPEED} ACCEL={ACCEL} SYNC=0")
                send_gcode(f"MANUAL_STEPPER STEPPER=stepper_1 MOVE={target_tilt:.2f} SPEED={SPEED} ACCEL={ACCEL} SYNC=0")
                
                pan, tilt = target_pan, target_tilt
                last_pan, last_tilt = target_pan, target_tilt
                cmd_count += 1
                
                print(f"\r[TRACK] Pan:{pan:+6.1f}° Tilt:{tilt:+5.1f}° | X:{x:+.2f} Y:{y:+.2f} | Cmds:{cmd_count}   ", end="", flush=True)
            
            time.sleep(0.03)
    
    except KeyboardInterrupt:
        print("\n\n[STOP]")
    finally:
        send_gcode("MANUAL_STEPPER STEPPER=stepper_0 ENABLE=0")
        send_gcode("MANUAL_STEPPER STEPPER=stepper_1 ENABLE=0")
        sock.close()


if __name__ == "__main__":
    main()
