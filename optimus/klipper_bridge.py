#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import requests
import json
import time

class KlipperBridgeNode(Node):
    def __init__(self):
        super().__init__('klipper_bridge_node')
        
        self.declare_parameter('moonraker_url', 'http://localhost:7125')
        self.moonraker_url = self.get_parameter('moonraker_url').value
        
        self.status_pub = self.create_publisher(String, 'printer/status', 10)
        
        # Fan control subscribers
        self.create_subscription(Float32, 'fans/fan0/set_speed', self.fan0_cb, 10)
        self.create_subscription(Float32, 'fans/fan1/set_speed', self.fan1_cb, 10)
        
        # Motor position subscribers
        self.create_subscription(Float32, 'motor/stepper_0/position', self.motor0_cb, 10)
        self.create_subscription(Float32, 'motor/stepper_1/position', self.motor1_cb, 10)
        self.create_subscription(Float32, 'motor/stepper_2/position', self.motor2_cb, 10)
        self.create_subscription(Float32, 'motor/stepper_3/position', self.motor3_cb, 10)
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info(f"Klipper Bridge Node started, connecting to {self.moonraker_url}")

    def fan0_cb(self, msg):
        self.set_fan_speed('fan0', msg.data)

    def fan1_cb(self, msg):
        self.set_fan_speed('fan1', msg.data)

    def motor0_cb(self, msg):
        self.move_motor('stepper_0', msg.data)
    
    def motor1_cb(self, msg):
        self.move_motor('stepper_1', msg.data)
    
    def motor2_cb(self, msg):
        self.move_motor('stepper_2', msg.data)
    
    def motor3_cb(self, msg):
        self.move_motor('stepper_3', msg.data)
    
    def move_motor(self, stepper_name, angle):
        """Move a stepper motor to a specific angle"""
        try:
            # Move motor and wait for completion, then release immediately
            # Using very slow speed for gentle calibration (1.5 mm/s)
            # SYNC=1 waits for completion, then ENABLE=0 releases the motor
            gcode = f"MANUAL_STEPPER STEPPER={stepper_name} ENABLE=1\nMANUAL_STEPPER STEPPER={stepper_name} MOVE={angle} SPEED=1.5 SYNC=1\nMANUAL_STEPPER STEPPER={stepper_name} ENABLE=0"
            response = requests.post(
                f"{self.moonraker_url}/printer/gcode/script",
                params={"script": gcode},
                timeout=30.0
            )
            if response.status_code == 200:
                self.get_logger().debug(f"Moved {stepper_name} to {angle}° and released")
            else:
                self.get_logger().error(f"Failed to move motor: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error moving motor: {e}")

    def set_fan_speed(self, fan_name, speed):
        try:
            speed = max(0.0, min(1.0, speed))
            gcode = f"SET_FAN_SPEED FAN={fan_name} SPEED={speed:.2f}"
            response = requests.post(
                f"{self.moonraker_url}/printer/gcode/script",
                params={"script": gcode},
                timeout=5.0
            )
            if response.status_code != 200:
                self.get_logger().error(f"Failed to set fan speed: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error setting fan speed: {e}")

    def timer_callback(self):
        try:
            # Get printer objects (heaters, fans, print status)
            response = requests.get(
                f"{self.moonraker_url}/printer/objects/query?heater_bed&extruder&print_stats&fan_generic fan0&fan_generic fan1",
                timeout=1.0
            )
            
            if response.status_code == 200:
                data = response.json()
                status = data.get('result', {}).get('status', {})
                
                # Publish status as JSON string for now
                msg = String()
                msg.data = json.dumps(status)
                self.status_pub.publish(msg)
            else:
                self.get_logger().warn(f"Moonraker returned {response.status_code}")
                
        except Exception as e:
            self.get_logger().warn(f"Failed to connect to Moonraker: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = KlipperBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
