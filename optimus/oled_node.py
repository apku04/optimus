#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Temperature
import json
import time
from PIL import Image, ImageDraw, ImageFont

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

MUX_ADDR = 0x70
OLED_CHANNEL = 2

class OledDisplayNode(Node):
    def __init__(self):
        super().__init__('oled_display_node')
        
        self.printer_status = {}
        self.sensor_data = {}
        self.vision_status = "Wait"
        self.bus = None
        
        # Initialize I2C Bus for Mux Control
        try:
            self.bus = smbus.SMBus(1)
        except Exception as e:
            self.get_logger().error(f"Failed to open I2C bus: {e}")

        if LUMA_AVAILABLE:
            try:
                self.enable_mux()
                serial = i2c(port=1, address=0x3D)
                self.device = sh1106(serial)
                self.get_logger().info("OLED initialized at 0x3D (SH1106) on Mux Ch 2")
            except Exception as e:
                self.get_logger().error(f"Failed to init OLED: {e}")
                self.device = None
            finally:
                self.disable_mux()
        else:
            self.get_logger().warn("luma.oled not installed")
            self.device = None

        # Subscribers
        self.create_subscription(String, 'printer/status', self.printer_status_cb, 10)
        self.create_subscription(String, '/vision_status', self.vision_status_cb, 10)
        
        # Subscribe to known sensors
        self.create_subscription(Temperature, 'sensors/rpi_case/temperature', 
                               lambda msg: self.sensor_cb('RPi', msg), 10)
        self.create_subscription(Temperature, 'sensors/power_supply/temperature', 
                               lambda msg: self.sensor_cb('PSU', msg), 10)
        self.create_subscription(Temperature, 'sensors/stepper_drivers/temperature', 
                               lambda msg: self.sensor_cb('Drv', msg), 10)

        self.timer = self.create_timer(1.0, self.update_display)
        self.get_logger().info("OLED Display Node started")

    def enable_mux(self):
        if self.bus:
            try:
                self.bus.write_byte(MUX_ADDR, 1 << OLED_CHANNEL)
                time.sleep(0.02)
            except Exception as e:
                self.get_logger().warn(f"Mux enable failed: {e}")

    def disable_mux(self):
        if self.bus:
            try:
                self.bus.write_byte(MUX_ADDR, 0x00)
                time.sleep(0.02)
            except Exception as e:
                self.get_logger().warn(f"Mux disable failed: {e}")

    def vision_status_cb(self, msg):
        if "Active" in msg.data:
            self.vision_status = "OK"
        else:
            self.vision_status = "Err"

    def printer_status_cb(self, msg):
        try:
            self.printer_status = json.loads(msg.data)
        except:
            pass

    def sensor_cb(self, name, msg):
        self.sensor_data[name] = msg.temperature

    def update_display(self):
        if not self.device:
            return

        try:
            self.enable_mux()
            with canvas(self.device) as draw:
                # Draw header
                draw.text((0, 0), f"Optimus | Vis:{self.vision_status}", fill="white")
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
                    draw.text((0, y), f"Status: {state}", fill="white")
                    
                    # Bed/Extruder
                    bed = self.printer_status.get('heater_bed', {})
                    ext = self.printer_status.get('extruder', {})
                    if bed and ext:
                        y += 10
                        draw.text((0, y), f"B:{bed.get('temperature',0):.0f} E:{ext.get('temperature',0):.0f}", fill="white")
        except Exception as e:
            self.get_logger().warn(f"OLED update failed: {e}")
        finally:
            self.disable_mux()

def main(args=None):
    rclpy.init(args=args)
    node = OledDisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
