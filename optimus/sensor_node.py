#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import String
import json
import os
from ament_index_python.packages import get_package_share_directory
from .sensor_reader import SensorReader

class OptimusSensorNode(Node):
    def __init__(self):
        super().__init__('optimus_sensor_node')
        
        # Load config
        # Use package share directory to find config.json
        try:
            share_dir = get_package_share_directory('optimus')
            self.config_path = os.path.join(share_dir, 'config.json')
            with open(self.config_path, 'r') as f:
                self.config = json.load(f)
        except FileNotFoundError:
            self.get_logger().error(f"Config file not found at {self.config_path}")
            raise
            
        self.sensor_reader = SensorReader(
            i2c_bus=self.config['settings'].get('i2c_bus', 1),
            mux_address=int(self.config['settings'].get('mux_address', '0x70'), 16)
        )
        
        if self.sensor_reader.connect():
            self.get_logger().info("Connected to I2C bus")
        else:
            self.get_logger().error("Failed to connect to I2C bus")
            
        self.publishers_ = {}
        
        # Create publishers for each sensor
        for sensor in self.config['sensors']:
            topic_name = f"sensors/{sensor['name']}/temperature"
            self.publishers_[sensor['name']] = self.create_publisher(Temperature, topic_name, 10)
            
        self.timer = self.create_timer(self.config['settings'].get('poll_interval', 5.0), self.timer_callback)
        self.get_logger().info("Optimus Sensor Node started")

    def timer_callback(self):
        for sensor_config in self.config['sensors']:
            reading = self.sensor_reader.read_sensor(sensor_config)
            
            if reading and reading.get('success'):
                msg = Temperature()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = sensor_config['name']
                msg.temperature = float(reading['temperature'])
                msg.variance = 0.0
                
                self.publishers_[sensor_config['name']].publish(msg)
                self.get_logger().debug(f"Published {sensor_config['name']}: {reading['temperature']}C")
            else:
                self.get_logger().warn(f"Failed to read {sensor_config['name']}")
        
        self.sensor_reader.disable_mux_channels()

def main(args=None):
    rclpy.init(args=args)
    node = OptimusSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
