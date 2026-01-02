#!/usr/bin/env python3
"""
FAST Direct Proportional Face Tracker

Simple, aggressive tracking - no fancy prediction, just fast reaction.
The key to smooth motion is FAST updates at HIGH SPEED, not complex algorithms.
"""

import json
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "shared"))
from klipper_direct import KlipperDirect

# =============================================================================
# CONFIGURATION - Tune these!
# =============================================================================

UDP_PORT = 5555
LOOP_HZ = 60  # Run as fast as possible

# Motor limits (degrees)
PAN_MIN, PAN_MAX = -22.0, 22.0
TILT_MIN, TILT_MAX = -2.5, 5.0

# Calibration: how much does face move (in image units) per degree of motor movement
# From testing: PAN +5° → face x +0.85, so dx_per_deg ≈ 0.17
# From testing: TILT +2° → face y +0.072, so dy_per_deg ≈ 0.036
DX_PER_PAN_DEG = 0.17
DY_PER_TILT_DEG = 0.036

# Gains - how aggressive to be
# Higher = more responsive but can overshoot
# Lower = smoother but slower to react
GAIN_PAN = 15.0   # degrees per unit error per second
GAIN_TILT = 40.0  # degrees per unit error per second (needs more due to smaller range)

# Deadzone - don't move if error is smaller than this
DEADZONE = 0.05

# Motor speeds (deg/sec) - Klipper will smooth these
SPEED_PAN = 60   # Fast pan
SPEED_TILT = 30  # Fast tilt

# =============================================================================

class FastTracker:
    def __init__(self):
        # UDP receiver
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', UDP_PORT))
        self.sock.setblocking(False)
        
        # Klipper connection
        self.klipper = KlipperDirect()
        self.klipper.connect()
        
        # Enable motors
        self.klipper.enable_stepper('stepper_0', True)
        self.klipper.enable_stepper('stepper_1', True)
        
        # Set current position as zero
        self.klipper.set_position('stepper_0', 0)
        self.klipper.set_position('stepper_1', 0)
        
        # Track current commanded position
        self.pan_pos = 0.0
        self.tilt_pos = 0.0
        
        # Face lost tracking
        self.face_lost_count = 0
        self.last_face_time = time.time()
        
    def get_face(self):
        """Get latest face position from UDP."""
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
                msg = data.decode('utf-8', errors='ignore').strip()
                if msg.startswith('{'):
                    d = json.loads(msg)
                    if d.get('face_detected') or d.get('detected'):
                        x = d.get('offset_x', d.get('x', 0))
                        y = d.get('offset_y', d.get('y', 0))
                        latest = (float(x), float(y))
            except BlockingIOError:
                break
            except:
                pass
        return latest
    
    def run(self):
        print("=" * 60)
        print("FAST FACE TRACKER")
        print(f"Gains: pan={GAIN_PAN}, tilt={GAIN_TILT}")
        print(f"Speeds: pan={SPEED_PAN}°/s, tilt={SPEED_TILT}°/s")
        print(f"Deadzone: {DEADZONE}")
        print("=" * 60)
        
        dt = 1.0 / LOOP_HZ
        
        try:
            while True:
                loop_start = time.time()
                
                # Get face position
                face = self.get_face()
                
                if face is None:
                    self.face_lost_count += 1
                    if self.face_lost_count > LOOP_HZ * 2:  # Lost for 2 seconds
                        # Return to center slowly
                        self.pan_pos *= 0.95
                        self.tilt_pos *= 0.95
                        self.klipper.move_stepper('stepper_0', self.pan_pos, speed=20)
                        self.klipper.move_stepper('stepper_1', self.tilt_pos, speed=10)
                        print(f"\r[LOST] returning to center... pan={self.pan_pos:+.1f} tilt={self.tilt_pos:+.1f}    ", end='', flush=True)
                else:
                    self.face_lost_count = 0
                    self.last_face_time = time.time()
                    
                    ex, ey = face  # Error in image space
                    
                    # Skip if in deadzone
                    if abs(ex) < DEADZONE and abs(ey) < DEADZONE:
                        print(f"\r[HOLD] e=({ex:+.2f},{ey:+.2f}) pos=({self.pan_pos:+.1f},{self.tilt_pos:+.1f})    ", end='', flush=True)
                    else:
                        # Simple proportional control:
                        # If face is to the RIGHT (ex > 0), we need to pan in direction that moves face LEFT
                        # Since camera is on head: PAN+ → face moves right, so we need PAN-
                        # Therefore: pan_delta = -ex * gain
                        
                        pan_delta = -ex * GAIN_PAN * dt
                        tilt_delta = -ey * GAIN_TILT * dt
                        
                        # Update position
                        self.pan_pos += pan_delta
                        self.tilt_pos += tilt_delta
                        
                        # Clamp to limits
                        self.pan_pos = max(PAN_MIN, min(PAN_MAX, self.pan_pos))
                        self.tilt_pos = max(TILT_MIN, min(TILT_MAX, self.tilt_pos))
                        
                        # Send to motors - use high speed so Klipper can move quickly
                        self.klipper.move_stepper('stepper_0', self.pan_pos, speed=SPEED_PAN)
                        self.klipper.move_stepper('stepper_1', self.tilt_pos, speed=SPEED_TILT)
                        
                        print(f"\r[TRACK] e=({ex:+.2f},{ey:+.2f}) Δ=({pan_delta:+.2f},{tilt_delta:+.2f}) pos=({self.pan_pos:+.1f},{self.tilt_pos:+.1f})    ", end='', flush=True)
                
                # Maintain loop rate
                elapsed = time.time() - loop_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                    
        except KeyboardInterrupt:
            print("\n[STOP] Returning to center...")
            self.klipper.move_stepper('stepper_0', 0, speed=30)
            self.klipper.move_stepper('stepper_1', 0, speed=15)
            time.sleep(1)
            self.klipper.disconnect()

if __name__ == '__main__':
    tracker = FastTracker()
    tracker.run()
