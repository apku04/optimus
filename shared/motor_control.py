#!/usr/bin/env python3
"""
Motor Control Module - Simple Moonraker API wrapper
No ROS required - just HTTP requests to Klipper/Moonraker
"""

import requests
import json
import time
from pathlib import Path


class MotorController:
    """Controls stepper motors via Moonraker HTTP API"""
    
    def __init__(self, config_path: str = None):
        # Load config
        if config_path is None:
            config_path = Path(__file__).parent.parent / "config.json"
        
        with open(config_path) as f:
            self.config = json.load(f)
        
        motors_config = self.config["motors"]
        self.moonraker_url = motors_config["moonraker_url"]
        
        # Support both old and new config format
        if "pan" in motors_config:
            # New format
            self.motors = {
                "pan": motors_config["pan"],
                "tilt": motors_config["tilt"]
            }
            # Add name field if using stepper field
            if "stepper" in self.motors["pan"]:
                self.motors["pan"]["name"] = self.motors["pan"]["stepper"]
            if "stepper" in self.motors["tilt"]:
                self.motors["tilt"]["name"] = self.motors["tilt"]["stepper"]
        else:
            # Old format
            self.motors = {
                "pan": motors_config["stepper_0"],
                "tilt": motors_config["stepper_1"]
            }
        
        # Current positions
        self.current_pan = self.motors["pan"]["center"]
        self.current_tilt = self.motors["tilt"]["center"]
    
    def _send_gcode(self, gcode: str) -> bool:
        """Send G-code command to Klipper via Moonraker"""
        try:
            url = f"{self.moonraker_url}/printer/gcode/script"
            # Moonraker accepts both JSON body and query param; JSON is the most
            # widely supported across setups and proxies.
            response = requests.post(url, json={"script": gcode}, timeout=2)
            if response.status_code == 200:
                return True

            # Fallback for older configs that only accept query param.
            response = requests.post(url, params={"script": gcode}, timeout=2)
            return response.status_code == 200
        except Exception as e:
            print(f"Motor error: {e}")
            return False
    
    def move_motor(self, motor_key: str, position: float, speed: float = None, sync: bool = False) -> bool:
        """Move a motor to absolute position (degrees)
        
        Args:
            sync: If False, command returns immediately (non-blocking)
                  If True, waits for move to complete
        """
        motor = self.motors.get(motor_key)
        if motor is None:
            print(f"Unknown motor: {motor_key}")
            return False
        
        # Clamp to limits
        position = max(motor["min"], min(motor["max"], position))
        speed = float(speed if speed is not None else motor.get("speed", 40))
        motor_name = motor.get("name") or motor.get("stepper", f"stepper_{motor_key}")
        
        # SYNC=0 allows non-blocking moves for smoother motion
        sync_val = 1 if sync else 0
        # Keep motor enabled for holding torque unless user disables in config.
        enable = motor.get("enable", True)
        enable_gcode = f"MANUAL_STEPPER STEPPER={motor_name} ENABLE=1\n" if enable else ""

        accel = motor.get("accel")
        accel_part = f" ACCEL={float(accel)}" if accel is not None else ""

        gcode = (
            f"{enable_gcode}"
            f"MANUAL_STEPPER STEPPER={motor_name} MOVE={position} SPEED={speed}{accel_part} SYNC={sync_val}"
        )
        
        success = self._send_gcode(gcode)
        if success:
            if motor_key == "pan":
                self.current_pan = position
            elif motor_key == "tilt":
                self.current_tilt = position
        
        return success
    
    def move_pan(self, position: float, speed: float = None, sync: bool = False) -> bool:
        """Move pan motor (horizontal)"""
        return self.move_motor("pan", position, speed, sync)
    
    def move_tilt(self, position: float, speed: float = None, sync: bool = False) -> bool:
        """Move tilt motor (vertical)"""
        return self.move_motor("tilt", position, speed, sync)

    def move_pan_tilt(
        self,
        pan_position: float,
        tilt_position: float,
        pan_speed: float = None,
        tilt_speed: float = None,
        sync: bool = False,
    ) -> bool:
        """Move pan and tilt together in a single Moonraker request.

        This reduces sequential command jitter ("tick-tick" feel).

        If sync=True, waits for completion by setting SYNC=1 on the second move.
        """
        pan_motor = self.motors.get("pan")
        tilt_motor = self.motors.get("tilt")
        if pan_motor is None or tilt_motor is None:
            print("Unknown motors: pan/tilt")
            return False

        pan_position = max(pan_motor["min"], min(pan_motor["max"], pan_position))
        tilt_position = max(tilt_motor["min"], min(tilt_motor["max"], tilt_position))

        pan_speed = float(pan_speed if pan_speed is not None else pan_motor.get("speed", 40))
        tilt_speed = float(tilt_speed if tilt_speed is not None else tilt_motor.get("speed", 40))

        pan_name = pan_motor.get("name") or pan_motor.get("stepper", "stepper_0")
        tilt_name = tilt_motor.get("name") or tilt_motor.get("stepper", "stepper_1")

        pan_enable = pan_motor.get("enable", True)
        tilt_enable = tilt_motor.get("enable", True)

        pan_accel = pan_motor.get("accel")
        tilt_accel = tilt_motor.get("accel")
        pan_accel_part = f" ACCEL={float(pan_accel)}" if pan_accel is not None else ""
        tilt_accel_part = f" ACCEL={float(tilt_accel)}" if tilt_accel is not None else ""

        # SYNC behavior: if sync=True, set SYNC=1 on the last command to wait.
        pan_sync = 0
        tilt_sync = 1 if sync else 0

        gcode = ""
        if pan_enable:
            gcode += f"MANUAL_STEPPER STEPPER={pan_name} ENABLE=1\n"
        if tilt_enable:
            gcode += f"MANUAL_STEPPER STEPPER={tilt_name} ENABLE=1\n"

        gcode += f"MANUAL_STEPPER STEPPER={pan_name} MOVE={pan_position} SPEED={pan_speed}{pan_accel_part} SYNC={pan_sync}\n"
        gcode += f"MANUAL_STEPPER STEPPER={tilt_name} MOVE={tilt_position} SPEED={tilt_speed}{tilt_accel_part} SYNC={tilt_sync}"

        success = self._send_gcode(gcode)
        if success:
            self.current_pan = pan_position
            self.current_tilt = tilt_position
        return success
    
    def center(self) -> bool:
        """Move both motors to center position"""
        pan_ok = self.move_pan(self.motors["pan"]["center"])
        tilt_ok = self.move_tilt(self.motors["tilt"]["center"])
        return pan_ok and tilt_ok
    
    def get_status(self) -> dict:
        """Get Klipper/Moonraker status"""
        try:
            response = requests.get(f"{self.moonraker_url}/printer/info", timeout=2)
            return response.json() if response.status_code == 200 else None
        except:
            return None
    
    def is_ready(self) -> bool:
        """Check if Moonraker/Klipper is ready"""
        status = self.get_status()
        if status and "result" in status:
            return status["result"].get("state") == "ready"
        return False


# Test if run directly
if __name__ == "__main__":
    print("Testing Motor Controller...")
    mc = MotorController()
    
    if mc.is_ready():
        print("✓ Moonraker connected")
        print("Moving to center...")
        mc.center()
        time.sleep(1)
        print("Pan left...")
        mc.move_pan(-10)
        time.sleep(1)
        print("Pan right...")
        mc.move_pan(10)
        time.sleep(1)
        print("Back to center...")
        mc.center()
        print("✓ Test complete")
    else:
        print("✗ Moonraker not ready")
