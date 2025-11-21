#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Temperature
from std_msgs.msg import String, Float32
import json
import os
import time
import subprocess
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from .sensor_reader import SensorReader

try:
    import smbus2 as smbus
except ImportError:
    import smbus

try:
    from luma.core.interface.serial import i2c
    from luma.core.render import canvas
    from luma.oled.device import sh1106
    LUMA_AVAILABLE = True
except ImportError:
    LUMA_AVAILABLE = False

class OptimusHardwareNode(Node):
    def __init__(self):
        super().__init__('optimus_hardware_node')
        
        # --- Config Loading ---
        try:
            share_dir = get_package_share_directory('optimus')
            self.config_path = os.path.join(share_dir, 'config.json')
            with open(self.config_path, 'r') as f:
                self.config = json.load(f)
        except FileNotFoundError:
            self.get_logger().error(f"Config file not found at {self.config_path}")
            raise
            
        # --- I2C / Sensor Setup ---
        self.i2c_bus_num = self.config['settings'].get('i2c_bus', 1)
        self.mux_address = int(self.config['settings'].get('mux_address', '0x70'), 16)
        
        self.sensor_reader = SensorReader(
            i2c_bus=self.i2c_bus_num,
            mux_address=self.mux_address
        )
        
        if self.sensor_reader.connect():
            self.get_logger().info("Connected to I2C bus")
        else:
            self.get_logger().error("Failed to connect to I2C bus")
            
        # --- Publishers (Sensors & Fans) ---
        self.publishers_ = {}
        self.fan_publishers_ = {}
        
        for sensor in self.config['sensors']:
            topic_name = f"sensors/{sensor['name']}/temperature"
            self.publishers_[sensor['name']] = self.create_publisher(Temperature, topic_name, 10)
            
            # Create fan publishers if mapping exists
            if 'fan_mapping' in sensor:
                for fan_name in sensor['fan_mapping']:
                    if fan_name not in self.fan_publishers_:
                        fan_topic = f"fans/{fan_name}/set_speed"
                        self.fan_publishers_[fan_name] = self.create_publisher(Float32, fan_topic, 10)

        # --- Subscribers (OLED Data) ---
        self.robot_status = {}
        self.sensor_data = {} # Local cache for display
        self.vision_status = "Wait"
        self.vision_last_seen = time.time()
        self.vision_timeout = 5.0  # seconds
        self.diag_status = "OK"
        self.diag_failures = []
        self.create_subscription(String, 'printer/status', self.robot_status_cb, 10)
        self.create_subscription(String, '/vision_status', self.vision_status_cb, 10)
        
        # Create latched publisher for diagnostics (transient local)
        diag_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.diag_pub = self.create_publisher(String, '/diagnostics/result', diag_qos)

        # Run Diagnostics on Boot
        self.run_system_diagnostics()
        
                # --- OLED Setup ---
        self.oled_device = None
        # OLED is on the main bus (0x3D), no mux needed usually, but we ensure mux is not blocking
        
        if LUMA_AVAILABLE:
            try:
                # Ensure mux is disabled/idle before init
                self.sensor_reader.disable_mux_channels()
                time.sleep(0.05)
                
                serial = i2c(port=self.i2c_bus_num, address=0x3D)
                # FIX: Explicitly set width/height to 128x128 for SH1107 compatibility
                self.oled_device = sh1106(serial, width=128, height=128, rotate=0)
                self.get_logger().info("OLED initialized (SH1106 128x128)")
                
                # Clear screen initially
                with canvas(self.oled_device) as draw:
                    draw.rectangle(self.oled_device.bounding_box, outline="black", fill="black")
                    draw.text((10, 50), "Optimus Init...", fill="white")
                    
            except Exception as e:
                self.get_logger().error(f"Failed to init OLED: {e}")
                self.oled_device = None
        else:
            self.get_logger().warn("luma.oled not installed")

        # --- Main Loop ---
        # Run at 1Hz (or whatever the poll interval is, but OLED needs frequent updates if we want animations)
        # Let's split: Sensor polling at configured interval, OLED at 1Hz.
        # But to avoid I2C conflict, we should probably just run one loop or use a lock.
        # Simple approach: Run loop at 1Hz. Check if it's time to read sensors. Always update OLED.
        
        self.timer = self.create_timer(1.0, self.control_loop)
        self.last_sensor_read = 0
        self.sensor_interval = self.config['settings'].get('poll_interval', 5.0)
        self.blink_state = False
        
        self.get_logger().info("Optimus Hardware Node started")

    def run_system_diagnostics(self):
        try:
            self.get_logger().info("Running system diagnostics...")
            result = subprocess.run(
                ['/home/acp/rpi-robot-diagnostics/run_diagnostics.py'],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            msg = String()
            msg.data = result.stdout
            self.diag_pub.publish(msg)

            if result.returncode == 0:
                self.diag_status = "OK"
                self.diag_failures = []
                self.get_logger().info("System Diagnostics Passed")
            else:
                self.diag_status = "Fail"
                self.diag_failures = self.parse_diagnostic_failures(result.stdout)
                self.get_logger().error(f"System Diagnostics Failed:\n{result.stdout}")
                
        except Exception as e:
            self.diag_status = "Err"
            self.diag_failures = ["Diag script error"]
            self.get_logger().error(f"Failed to run diagnostics: {e}")

    def parse_diagnostic_failures(self, output):
        failures = []
        lines = output.split('\n')
        for line in lines:
            # Look for lines in the summary that start with checkmark and contain test names
            if 'USB Microphone:' in line or 'Temperature Sensors:' in line or 'OLED' in line or 'Motor Controller' in line or 'I2C Multiplexer' in line:
                if 'FAIL' in line or 'Cannot test' in line or 'missing' in line or line.strip().startswith('✗'):
                    # Extract just the test name and clean simple reason
                    if 'USB Microphone' in line:
                        failures.append("Mic: missing deps")
                    elif 'Temperature Sensors' in line:
                        failures.append("Temp sensors fail")
                    elif 'OLED' in line and 'FAIL' in line:
                        failures.append("OLED fail")
                    elif 'Motor Controller' in line or 'Klipper' in line:
                        failures.append("Motor fail")
        return failures if failures else ["Check diag log"]

    def robot_status_cb(self, msg):
        try:
            self.robot_status = json.loads(msg.data)
        except:
            pass

    def vision_status_cb(self, msg):
        self.vision_last_seen = time.time()
        if "Active" in msg.data:
            self.vision_status = "OK"
        else:
            self.vision_status = "Err"

    def control_loop(self):
        now = time.time()
        
        # 0. Toggle blink state for animations
        self.blink_state = not self.blink_state
        
        # 1. Check vision timeout
        if now - self.vision_last_seen > self.vision_timeout:
            if self.vision_status != "Wait":
                self.vision_status = "Lost"
        
        # 2. Read Sensors (if interval passed)
        if now - self.last_sensor_read >= self.sensor_interval:
            self.read_sensors()
            self.last_sensor_read = now
            
        # 3. Update OLED (every loop)
        self.update_display()
        
        # 4. Ensure Mux is disabled when idle (optional, but good for safety)
        self.sensor_reader.disable_mux_channels()

    def read_sensors(self):
        try:
            for sensor_config in self.config['sensors']:
                # This handles Mux switching internally in SensorReader
                reading = self.sensor_reader.read_sensor(sensor_config)
                
                if reading and reading.get('success'):
                    msg = Temperature()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = sensor_config['name']
                    msg.temperature = float(reading['temperature'])
                    msg.variance = 0.0
                    
                    self.publishers_[sensor_config['name']].publish(msg)
                    
                    # Update local cache for OLED
                    # Map sensor name to display name or short code
                    short_name = sensor_config['name']
                    if 'rpi' in short_name: short_name = 'RPi'
                    elif 'power' in short_name: short_name = 'PSU'
                    elif 'stepper' in short_name: short_name = 'Drv'
                    
                    self.sensor_data[short_name] = reading['temperature']
                    
                    # --- Fan Control Logic ---
                    if 'fan_mapping' in sensor_config:
                        temp = float(reading['temperature'])
                        start_temp = sensor_config.get('fan_start_temp', 40)
                        max_temp = sensor_config.get('fan_max_temp', 60)
                        
                        speed = 0.0
                        if temp >= max_temp:
                            speed = 1.0
                        elif temp >= start_temp:
                            # Linear interpolation
                            speed = (temp - start_temp) / (max_temp - start_temp)
                            # Ensure minimum speed to start fan if needed (e.g. 0.2)
                            speed = max(0.2, speed)
                        
                        # Publish speed to mapped fans
                        for fan_name in sensor_config['fan_mapping']:
                            if fan_name in self.fan_publishers_:
                                msg = Float32()
                                msg.data = float(speed)
                                self.fan_publishers_[fan_name].publish(msg)
                                self.get_logger().debug(f"Fan {fan_name} set to {speed:.2f} (T={temp}C)")

                    self.get_logger().debug(f"Published {sensor_config['name']}: {reading['temperature']}C")
                else:
                    self.get_logger().warn(f"Failed to read {sensor_config['name']}")
        finally:
            # Always disable mux after reading sensors to free bus for OLED (if it was shared)
            self.sensor_reader.disable_mux_channels()

    def get_status_symbol(self, status):
        """Return visual symbol for status"""
        if status == "OK":
            return "[OK]"
        elif status in ["Err", "Lost"]:
            return "[!!]" if self.blink_state else "[XX]"
        elif status == "Wait":
            return "[..]"
        else:
            return "[??]"

    def update_display(self):
        if not self.oled_device:
            return

        try:
            # OLED is on main bus, no mux switch needed
            # self.sensor_reader.select_mux_channel(self.oled_channel)
            
            # Get Motor Controller Status String
            motor_state = "Unknown"
            if self.robot_status:
                stats = self.robot_status.get('print_stats', {})
                motor_state = stats.get('state', 'Unknown')
                # Map printer states to robot terms
                if motor_state == 'ready':
                    motor_state = "Ready"
                elif motor_state == 'standby':
                    motor_state = "Standby"
                elif motor_state == 'printing':
                    motor_state = "Moving"

            with canvas(self.oled_device) as draw:
                # Explicitly clear the screen
                draw.rectangle(self.oled_device.bounding_box, outline="black", fill="black")

                # --- HEADER: Title and Time (Line 0-10) ---
                draw.text((0, 0), "OPTIMUS", fill="white")
                draw.text((85, 0), datetime.now().strftime('%H:%M'), fill="white")
                draw.line((0, 10, 127, 10), fill="white")
                
                # --- STATUS ROW 1: Vision (Line 12-22) ---
                vision_symbol = self.get_status_symbol(self.vision_status)
                if self.vision_status in ["Err", "Lost"]:
                    # Blink error
                    if self.blink_state:
                        draw.rectangle((0, 12, 60, 22), outline="white", fill="white")
                        draw.text((2, 13), f"VIS {vision_symbol}", fill="black")
                    else:
                        draw.text((0, 13), f"VIS {vision_symbol}", fill="white")
                else:
                    draw.text((0, 13), f"VIS {vision_symbol}", fill="white")
                
                # --- STATUS ROW 2: Motion (Line 24-34) ---
                motion_short = motor_state[:7] if len(motor_state) > 7 else motor_state
                draw.text((0, 25), f"MOT: {motion_short}", fill="white")
                
                # --- DIAG STATUS (Right side, line 13-34) ---
                if self.diag_status != "OK":
                    if self.blink_state:
                        draw.rectangle((95, 12, 127, 34), outline="white", fill="white")
                        draw.text((100, 18), "DIAG", fill="black")
                        draw.text((100, 26), "ERR!", fill="black")
                    else:
                        draw.rectangle((95, 12, 127, 34), outline="white", fill="black")
                        draw.text((100, 18), "DIAG", fill="white")
                        draw.text((100, 26), "ERR!", fill="white")
                else:
                    draw.rectangle((95, 12, 127, 34), outline="white", fill="black")
                    draw.text((102, 20), "OK", fill="white")
                
                draw.line((0, 36, 127, 36), fill="white")
                
                # --- TEMPERATURE SECTION (Line 38-90) ---
                y = 38
                for name, temp in self.sensor_data.items():
                    # Determine warning level
                    warn_level = ""
                    if temp > 65:
                        warn_level = "!" if self.blink_state else "X"
                    elif temp > 55:
                        warn_level = "!"
                    elif temp > 45:
                        warn_level = "~"
                    
                    # Format with aligned columns
                    name_padded = f"{name:3s}"
                    temp_str = f"{temp:5.1f}"
                    draw.text((2, y), f"{warn_level:1s}{name_padded} {temp_str}C", fill="white")
                    y += 11
                
                # --- MOTOR DRIVER TEMPS (if available) ---
                if self.robot_status:
                    driver_temps = self.robot_status.get('temperature_sensor', {})
                    # Check for common motor driver temp sensors
                    temp1 = None
                    temp2 = None
                    
                    # Try to get heater_bed/extruder as generic temp inputs
                    if 'heater_bed' in self.robot_status:
                        temp1 = self.robot_status['heater_bed'].get('temperature', 0)
                    if 'extruder' in self.robot_status:
                        temp2 = self.robot_status['extruder'].get('temperature', 0)
                    
                    if temp1 is not None and temp2 is not None:
                        draw.line((0, y, 127, y), fill="white")
                        y += 2
                        draw.text((2, y), f"M1  {temp1:5.1f}C", fill="white")
                        draw.text((68, y), f"M2  {temp2:5.1f}C", fill="white")
                        y += 11
                
                # --- DIAGNOSTIC ERROR BOX (Bottom, if needed) ---
                if self.diag_status != "OK" and self.diag_failures:
                    # Position at bottom
                    box_top = 115
                    draw.line((0, box_top-2, 127, box_top-2), fill="white")
                    if self.blink_state:
                        draw.rectangle((0, box_top, 127, 127), outline="white", fill="white")
                        draw.text((2, box_top+1), f"{self.diag_failures[0][:20]}", fill="black")
                    else:
                        draw.text((2, box_top+1), f"!{self.diag_failures[0][:19]}", fill="white")
                        
        except Exception as e:
            self.get_logger().warn(f"OLED update failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OptimusHardwareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
