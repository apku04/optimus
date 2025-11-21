# Optimus - ROS 2 Robot Control System

A ROS 2 Jazzy package for controlling and monitoring a 3D printer robot with integrated sensor monitoring, OLED display, and Klipper integration.

## Features

- **Hardware Monitoring**: Real-time temperature monitoring via I2C sensors (SHT3x, BME280) through PCA9548A multiplexer
- **OLED Display**: Live status display on SH1106/SH1107 128x128 OLED showing system status, temperatures, and printer state
- **Klipper Integration**: Direct communication with Klipper/Moonraker API for printer status and fan control
- **Automatic Fan Control**: Temperature-based fan speed control with configurable thresholds
- **Systemd Integration**: Automatic startup on boot

## Hardware Requirements

- Raspberry Pi (tested on Raspberry Pi 5)
- PCA9548A I2C Multiplexer (0x70)
- Temperature Sensors:
  - SHT3x (I2C address 0x44) - Power Supply monitoring
  - BME280 (I2C address 0x76) - RPi Case and Stepper Driver monitoring
- SH1106/SH1107 OLED Display (I2C address 0x3D)
- Klipper-based 3D Printer with Moonraker API

## Installation

### Prerequisites

1. ROS 2 Jazzy installed on Ubuntu 24.04
2. Python dependencies:
```bash
pip3 install smbus2 luma.oled pillow requests
```

### Building the Package

```bash
cd ~/ros2_ws/src
git clone git@github.com:apku04/optimus.git
cd ~/ros2_ws
colcon build --packages-select optimus
source install/setup.bash
```

## Configuration

Edit `optimus/config.json` to customize:
- Sensor I2C channels and addresses
- Temperature thresholds (warning, critical, shutdown)
- Fan mappings and temperature triggers
- Polling intervals

Example sensor configuration:
```json
{
  "name": "power_supply",
  "type": "sht3x",
  "channel": 0,
  "address": "0x44",
  "thresholds": {
    "warning": 50,
    "critical": 60,
    "shutdown": 70
  },
  "fan_mapping": ["fan0"],
  "fan_start_temp": 45,
  "fan_max_temp": 60
}
```

## Usage

### Manual Launch

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch optimus optimus.launch.py
```

### Systemd Service (Auto-start on Boot)

The package includes a systemd service configuration:

```bash
# Check status
systemctl status optimus-ros

# View logs
journalctl -u optimus-ros -f

# Restart service
sudo systemctl restart optimus-ros
```

Service location: `/etc/systemd/system/optimus-ros.service`

## Architecture

### Nodes

#### `hardware_node`
- Manages all I2C hardware (sensors via multiplexer, OLED on main bus)
- Reads temperature sensors at configured intervals
- Publishes sensor data to ROS topics
- Controls fan speeds based on temperature thresholds
- Updates OLED display with system status

#### `klipper_bridge`
- Connects to Moonraker API (default: `http://localhost:7125`)
- Publishes printer status to `/printer/status`
- Subscribes to fan control commands (`/fans/*/set_speed`)
- Sends fan speed commands to Klipper

### Topics

**Published by hardware_node:**
- `/sensors/{sensor_name}/temperature` - Temperature readings (sensor_msgs/Temperature)
- `/fans/{fan_name}/set_speed` - Fan speed commands (std_msgs/Float32, range 0.0-1.0)

**Published by klipper_bridge:**
- `/printer/status` - Klipper printer status (std_msgs/String, JSON formatted)

## OLED Display Layout

```
=== OPTIMUS ===
Status: [Printer State]
Time: HH:MM:SS
-------------------
RPi: XX.X°C
PSU: XX.X°C
Drv: XX.X°C

Bed:XX°C Ext:XX°C
```

## Fan Control Logic

Fans are automatically controlled based on sensor temperatures:
- Below `fan_start_temp`: Fan off (speed = 0.0)
- Between `fan_start_temp` and `fan_max_temp`: Linear interpolation (minimum 0.2)
- Above `fan_max_temp`: Full speed (speed = 1.0)

Multiple sensors can map to the same fan (highest temperature wins).

## Troubleshooting

### I2C Issues

Check I2C devices:
```bash
sudo i2cdetect -y 1
```

Verify multiplexer channels:
```bash
python3 test_mux.py  # In ros2_ws directory
```

### OLED Not Displaying

1. Verify I2C address: `sudo i2cdetect -y 1` (should show 0x3D)
2. Check service logs: `journalctl -u optimus-ros -n 50`
3. Ensure no other processes are using the OLED

### Sensor Read Errors

- `[Errno 121] Remote I/O error`: Check sensor wiring and I2C connections
- Verify sensor is on correct multiplexer channel
- Ensure no I2C bus conflicts

## Development

### Running Tests

```bash
# Test I2C multiplexer
python3 ~/ros2_ws/test_mux.py

# Test sensors manually
python3 ~/rpi-robot-diagnostics/test_temp_sensors.py

# Test OLED display
python3 ~/rpi-robot-diagnostics/test_oled_displays.py
```

### Debugging

Enable debug logging by modifying the node:
```python
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

View all ROS topics:
```bash
ros2 topic list
ros2 topic echo /printer/status
```

## License

TODO: Add license

## Contributors

- acp@todo.todo

## Version History

- **v0.0.0** - Initial release
  - Basic sensor monitoring
  - OLED display integration
  - Klipper bridge
  - Automatic fan control
