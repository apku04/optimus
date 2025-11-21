#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import String
import json
import os
import time
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
            
        # --- Publishers (Sensors) ---
        self.publishers_ = {}
        for sensor in self.config['sensors']:
            topic_name = f"sensors/{sensor['name']}/temperature"
            self.publishers_[sensor['name']] = self.create_publisher(Temperature, topic_name, 10)

        # --- Subscribers (OLED Data) ---
        self.printer_status = {}
        self.sensor_data = {} # Local cache for display
        self.create_subscription(String, 'printer/status', self.printer_status_cb, 10)
        
        # --- OLED Setup ---
        self.oled_device = None
        self.oled_channel = 2 # Hardcoded for now based on diagnostics
        
        if LUMA_AVAILABLE:
            try:
                # We need to enable the mux channel for OLED init
                self.sensor_reader.select_mux_channel(self.oled_channel)
                time.sleep(0.05)
                
                serial = i2c(port=self.i2c_bus_num, address=0x3D)
                # FIX: Explicitly set width/height to 128x128 for SH1107 compatibility
                self.oled_device = sh1106(serial, width=128, height=128, rotate=0)
                self.get_logger().info("OLED initialized (SH1106 128x128)")
                
                # Clear screen initially
                with canvas(self.oled_device) as draw:
                    draw.rectangle(self.oled_device.bounding_box, outline="white", fill="black")
                    draw.text((10, 50), "Optimus Init...", fill="white")
                    
            except Exception as e:
                self.get_logger().error(f"Failed to init OLED: {e}")
                self.oled_device = None
            finally:
                self.sensor_reader.disable_mux_channels()
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
                
                self.get_logger().debug(f"Published {sensor_config['name']}: {reading['temperature']}C")
            else:
                self.get_logger().warn(f"Failed to read {sensor_config['name']}")

    def update_display(self):
        if not self.oled_device:
            return

        try:
            # Explicitly enable OLED channel
            self.sensor_reader.select_mux_channel(self.oled_channel)
            
            with canvas(self.oled_device) as draw:
                # Draw header
                draw.text((0, 0), "Optimus Robot", fill="white")
                draw.line((0, 12, 127, 12), fill="white")
                
                y = 14
                # Draw Sensors
                for name, temp in self.sensor_data.items():
                    draw.text((0, y), f"{name}: {temp:.1f}C", fill="white")
                    y += 10
                
                # Draw Printer Status
                if self.printer_status:
                    stats = self.printer_status.get('print_stats', {})
                    state = stats.get('state', 'Unknown')
                    draw.text((0, y), f"Printer: {state}", fill="white")
                    
                    # Bed/Extruder
                    bed = self.printer_status.get('heater_bed', {})
                    ext = self.printer_status.get('extruder', {})
                    if bed and ext:
                        y += 10
                        draw.text((0, y), f"B:{bed.get('temperature',0):.0f} E:{ext.get('temperature',0):.0f}", fill="white")
                        
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
