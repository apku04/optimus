#!/usr/bin/env python3
"""
Klipper velocity control using GCODE_AXIS registration.

This registers manual steppers as G-code axes (A, B), which enables
Klipper's motion planner to handle velocity commands properly.

Unlike MANUAL_STEPPER which does discrete moves, G-code axes use
the toolhead's motion planner for smooth velocity control.
"""

import time
from typing import Optional
from klipper_direct import KlipperDirect


class KlipperVelocityControl:
    """Velocity control for pan/tilt using Klipper's GCODE_AXIS feature."""
    
    def __init__(
        self,
        socket_path: str = "/home/acp/printer_data/comms/klippy.sock",
        pan_axis: str = "A",
        tilt_axis: str = "B",
    ):
        self.klipper = KlipperDirect(socket_path)
        self.pan_axis = pan_axis
        self.tilt_axis = tilt_axis
        self.registered = False
        
        self.pan_pos = 0.0
        self.tilt_pos = 0.0
        
    def connect(self) -> bool:
        """Connect and register axes for velocity control."""
        if not self.klipper.connect():
            return False
        
        # Register stepper_0 as axis A (pan)
        self.klipper.gcode(
            f"MANUAL_STEPPER STEPPER=stepper_0 GCODE_AXIS={self.pan_axis} "
            f"LIMIT_VELOCITY=70 LIMIT_ACCEL=400 INSTANTANEOUS_CORNER_VELOCITY=1"
        )
        
        # Register stepper_1 as axis B (tilt)
        self.klipper.gcode(
            f"MANUAL_STEPPER STEPPER=stepper_1 GCODE_AXIS={self.tilt_axis} "
            f"LIMIT_VELOCITY=50 LIMIT_ACCEL=300 INSTANTANEOUS_CORNER_VELOCITY=1"
        )
        
        time.sleep(0.1)
        
        # Set initial position to 0
        self.klipper.gcode(f"G92 {self.pan_axis}0 {self.tilt_axis}0")
        
        self.registered = True
        print(f"[VELOCITY] Registered axes: {self.pan_axis}=pan, {self.tilt_axis}=tilt")
        return True
    
    def disconnect(self):
        """Unregister axes and disconnect."""
        if self.registered:
            # Unregister axes
            self.klipper.gcode(f"MANUAL_STEPPER STEPPER=stepper_0 GCODE_AXIS=")
            self.klipper.gcode(f"MANUAL_STEPPER STEPPER=stepper_1 GCODE_AXIS=")
            self.registered = False
        self.klipper.disconnect()
    
    def move_velocity(
        self,
        pan_vel: float,
        tilt_vel: float,
        duration: float = 0.05,
    ) -> bool:
        """
        Move at specified velocities for a duration.
        
        Args:
            pan_vel: Pan velocity in deg/s
            tilt_vel: Tilt velocity in deg/s
            duration: How long to move at this velocity (seconds)
        
        This creates a smooth move that Klipper's planner can blend with
        subsequent moves, unlike MANUAL_STEPPER discrete moves.
        """
        if not self.registered:
            return False
        
        # Calculate target positions based on velocity * time
        pan_target = self.pan_pos + pan_vel * duration
        tilt_target = self.tilt_pos + tilt_vel * duration
        
        # Convert velocity to feedrate (deg/s to deg/min)
        pan_feed = abs(pan_vel) * 60
        tilt_feed = abs(tilt_vel) * 60
        
        # Use the slower feedrate for coordinated motion
        feed = max(pan_feed, tilt_feed)
        
        # G1 command with feedrate (velocity)
        # Klipper will plan a smooth path
        cmd = f"G1 {self.pan_axis}{pan_target:.4f} {self.tilt_axis}{tilt_target:.4f} F{feed:.1f}"
        self.klipper.gcode(cmd)
        
        # Update internal position tracking
        self.pan_pos = pan_target
        self.tilt_pos = tilt_target
        
        return True
    
    def move_to_position(
        self,
        pan_pos: float,
        tilt_pos: float,
        velocity: Optional[float] = None,
    ) -> bool:
        """Move to absolute position at given velocity."""
        if not self.registered:
            return False
        
        if velocity is None:
            # Fast move
            cmd = f"G0 {self.pan_axis}{pan_pos:.4f} {self.tilt_axis}{tilt_pos:.4f}"
        else:
            # Controlled move
            feed = velocity * 60  # Convert deg/s to deg/min
            cmd = f"G1 {self.pan_axis}{pan_pos:.4f} {self.tilt_axis}{tilt_pos:.4f} F{feed:.1f}"
        
        self.klipper.gcode(cmd)
        
        self.pan_pos = pan_pos
        self.tilt_pos = tilt_pos
        
        return True
    
    def stop(self):
        """Stop all motion."""
        if self.registered:
            # M410 stops all motion
            self.klipper.gcode("M410")


# Test
if __name__ == "__main__":
    print("Testing Klipper velocity control...")
    
    vel = KlipperVelocityControl()
    
    if not vel.connect():
        print("Failed to connect!")
        exit(1)
    
    print("Connected! Testing velocity commands...")
    
    # Test 1: Move at constant velocity
    print("\nTest 1: Move pan at 20 deg/s for 0.5s")
    for _ in range(10):
        vel.move_velocity(pan_vel=20, tilt_vel=0, duration=0.05)
        time.sleep(0.05)
    
    time.sleep(0.5)
    
    # Test 2: Return to center
    print("\nTest 2: Return to center")
    vel.move_to_position(0, 0, velocity=30)
    
    time.sleep(1)
    
    print("\nDone!")
    vel.disconnect()
