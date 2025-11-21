#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import String, Float32
import json
import os
import time
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
        self.printer_status = {}
        self.sensor_data = {} # Local cache for display
        self.create_subscription(String, 'printer/status', self.printer_status_cb, 10)
        
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
        
        self.get_logger().info("Optimus Hardware Node started")

    def printer_status_cb(self, msg):
        try:
            self.printer_status = json.loads(msg.data)
        except:
            pass

    def control_loop(self):
        now = time.time()
        
        # 1. Read Sensors (if interval passed)
        if now - self.last_sensor_read >= self.sensor_interval:
            self.read_sensors()
            self.last_sensor_read = now
            
        # 2. Update OLED (every loop)
        self.update_display()
        
        # 3. Ensure Mux is disabled when idle (optional, but good for safety)
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

    def update_display(self):
        if not self.oled_device:
            return

        try:
            # OLED is on main bus, no mux switch needed
            # self.sensor_reader.select_mux_channel(self.oled_channel)
            
            # Get Printer Status String
            printer_state = "Unknown"
            if self.printer_status:
                stats = self.printer_status.get('print_stats', {})
                printer_state = stats.get('state', 'Unknown')

            with canvas(self.oled_device) as draw:
                # Explicitly clear the screen with a black rectangle
                draw.rectangle(self.oled_device.bounding_box, outline="black", fill="black")

                # --- Header Section (User Preferred Layout) ---
                draw.text((0, 0), "=== OPTIMUS ===", fill="white")
                draw.text((0, 12), f"Status: {printer_state}", fill="white")
                draw.text((0, 24), f"Time: {datetime.now().strftime('%H:%M:%S')}", fill="white")
                draw.line((0, 36, 127, 36), fill="white")
                
                y = 40
                # --- Sensor Data Section ---
                for name, temp in self.sensor_data.items():
                    draw.text((0, y), f"{name}: {temp:.1f}C", fill="white")
                    y += 10
                
                # --- Printer Temps (Bed/Extruder) ---
                if self.printer_status:
                    bed = self.printer_status.get('heater_bed', {})
                    ext = self.printer_status.get('extruder', {})
                    if bed and ext:
                        y += 10
                        draw.text((0, y), f"Bed:{bed.get('temperature',0):.0f}C Ext:{ext.get('temperature',0):.0f}C", fill="white")
                        
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
