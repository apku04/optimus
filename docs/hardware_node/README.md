# Hardware Node Documentation

## Overview

The `hardware_node` is the core hardware management node for the Optimus robot system. It handles all I2C-based hardware including temperature sensors, I2C multiplexer management, OLED display control, and automatic fan speed regulation based on temperature readings.

## Node Name

`optimus_hardware_node`

## Purpose

- Read temperature data from multiple I2C sensors via PCA9548A multiplexer
- Control OLED display showing system status and temperatures
- Calculate and publish fan speed commands based on temperature thresholds
- Run system diagnostics on startup
- Monitor printer/robot status from Klipper via subscriptions

## Configuration

The node reads its configuration from `config.json` located in the package share directory:

```json
{
  "sensors": [...],
  "fans": [...],
  "settings": {
    "i2c_bus": 1,
    "mux_address": "0x70",
    "poll_interval": 5,
    "moonraker_url": "http://localhost:7125"
  }
}
```

### Sensor Configuration

Each sensor entry defines:
- `name`: Unique identifier
- `type`: `"sht3x"`, `"bme280"`, or `"cpu"` (RPi CPU temp)
- `channel`: I2C multiplexer channel number (0-7)
- `address`: I2C address (e.g., `"0x44"`)
- `fan_mapping`: Array of fan names this sensor controls
- `fan_start_temp`: Temperature (°C) where fan starts
- `fan_max_temp`: Temperature (°C) where fan reaches max speed
- `thresholds`: `warning`, `critical`, `shutdown` temperatures

### Fan Configuration

Each fan entry defines:
- `name`: Unique identifier (e.g., `"fan0"`, `"fan1"`)
- `klipper_name`: Corresponding fan name in Klipper config
- `min_speed`: Minimum speed (0.0-1.0) when fan turns on
- `max_speed`: Maximum speed (0.0-1.0)

## Hardware Requirements

### I2C Bus

- **Bus**: I2C bus 1 (`/dev/i2c-1`)
- **Multiplexer**: PCA9548A at address `0x70`
- **OLED**: SH1106/SH1107 128x128 at address `0x3D` (on main bus)

### Supported Temperature Sensors

1. **SHT3x** (Temperature + Humidity)
   - I2C address: `0x44`
   - Typical use: Power supply monitoring

2. **BME280/BMP280** (Temperature + Pressure)
   - I2C address: `0x76`
   - Typical use: Case temperature, stepper driver area

3. **CPU Temperature** (virtual sensor)
   - Reads from `/sys/class/thermal/thermal_zone0/temp`
   - Raspberry Pi CPU temperature

## ROS 2 Interface

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/sensors/{sensor_name}/temperature` | `sensor_msgs/Temperature` | Temperature readings from each configured sensor |
| `/fans/{fan_name}/set_speed` | `std_msgs/Float32` | Fan speed commands (0.0-1.0) sent to klipper_bridge |
| `/diagnostics/result` | `std_msgs/String` | System diagnostic results (latched QoS) |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/printer/status` | `std_msgs/String` | JSON-formatted printer status from klipper_bridge |
| `/vision_status` | `std_msgs/String` | Vision system status (for OLED display) |

### Message Formats

**Temperature Message:**
```python
header:
  stamp: {sec: ..., nanosec: ...}
  frame_id: "sensor_name"
temperature: 45.2  # Celsius
variance: 0.0
```

**Fan Speed Message:**
```python
data: 0.45  # Speed from 0.0 (off) to 1.0 (full speed)
```

## Architecture

### Main Control Loop

The node runs a 1 Hz timer callback (`control_loop`) that:

1. **Toggles blink state** for visual error indicators on OLED
2. **Checks vision timeout** (marks vision as "Lost" if no updates)
3. **Reads sensors** at configured `poll_interval` (default 5 seconds)
4. **Updates OLED display** every loop iteration
5. **Disables multiplexer** to free I2C bus for OLED

### Sensor Reading Flow

```
control_loop()
  └─> read_sensors()
       ├─> for each sensor in config:
       │    ├─> sensor_reader.read_sensor(sensor_config)
       │    │    └─> Handles mux channel selection
       │    ├─> Publish to /sensors/{name}/temperature
       │    └─> Cache temperature in self.sensor_data
       │
       └─> Fan Control Logic:
            ├─> For each fan:
            │    ├─> Find all sensors controlling this fan
            │    ├─> Calculate speed for each sensor's temp
            │    │    └─> Linear interpolation: min_speed + ratio*(max_speed - min_speed)
            │    └─> Use MAX speed from all sensors
            └─> Publish to /fans/{name}/set_speed
```

### Fan Speed Calculation

For each sensor controlling a fan:

```python
if temp >= fan_max_temp:
    speed = max_fan_speed
elif temp >= fan_start_temp:
    ratio = (temp - fan_start_temp) / (fan_max_temp - fan_start_temp)
    speed = min_fan_speed + ratio * (max_fan_speed - min_fan_speed)
else:
    speed = 0.0
```

If multiple sensors control one fan, the **maximum** calculated speed is used.

### I2C Multiplexer Management

The `SensorReader` class handles multiplexer channel switching:

1. Before reading a sensor, switch to its channel
2. After reading all sensors, disable all channels to free the bus
3. OLED is on the main bus (no mux channel needed)

### OLED Display Layout

```
OPTIMUS                    HH:MM
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
VIS [OK]                   DIAG
MOT: Ready                   OK
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 CPU   49.0C
 Case  34.2C
 PSU   29.4C
 Drv   29.5C
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
M1  0.0C        M2  0.0C
```

**Status Indicators:**
- `[OK]` - Normal operation
- `[..]` - Waiting for data
- `[!!]` / `[XX]` - Error (blinking)

**Warning Symbols** (temperature):
- `~` - Over 50°C
- `!` - Over 60°C
- `!`/`X` - Over 70°C (blinking)

## System Diagnostics

On startup, the node runs `/home/acp/rpi-robot-diagnostics/run_diagnostics.py` to check:
- I2C multiplexer connectivity
- Temperature sensor functionality
- OLED display
- Motor controller (Klipper) connection
- USB microphone (if voice node dependencies installed)

Results are published to `/diagnostics/result` with transient local QoS (latched) and displayed on OLED if failures occur.

## Dependencies

### Python Packages
```bash
pip install smbus2 luma.oled pillow requests
```

### ROS 2 Packages
- `rclpy`
- `sensor_msgs`
- `std_msgs`
- `ament_index_python`

## Error Handling

### I2C Communication Errors
- Logged as warnings
- Sensor reading marked as failed
- Fan control continues with last known temperatures
- OLED continues to display cached data

### OLED Initialization Failure
- Node continues without display
- All sensor and fan functionality remains operational

### Diagnostics Script Failure
- Status set to "Err"
- Error displayed on OLED
- Node continues normal operation

## Troubleshooting

### No Temperature Readings

1. Check I2C bus: `sudo i2cdetect -y 1`
2. Verify multiplexer at `0x70`
3. Check sensor wiring and multiplexer channels
4. Review logs: `ros2 topic echo /sensors/*/temperature`

### Fan Not Responding

1. Verify fan speed is being published: `ros2 topic echo /fans/fan0/set_speed`
2. Check `klipper_bridge` node is running: `ros2 node list`
3. Verify Moonraker connection
4. Check Klipper fan configuration (`printer.cfg`)

### OLED Not Displaying

1. Verify OLED at `0x3D`: `sudo i2cdetect -y 1`
2. Check luma.oled installation: `pip list | grep luma`
3. Ensure no other process is using the OLED
4. Review initialization logs

### High Fan Speed

- Fan controlled by multiple sensors uses the **highest** calculated speed
- Check all sensors mapped to the fan in `config.json`
- Adjust `fan_start_temp` and `fan_max_temp` thresholds

## Code Structure

```
hardware_node.py
├─ OptimusHardwareNode (class)
│   ├─ __init__()
│   │   ├─ Load config.json
│   │   ├─ Initialize I2C and SensorReader
│   │   ├─ Create publishers/subscribers
│   │   ├─ Initialize OLED display
│   │   ├─ Run system diagnostics
│   │   └─ Start control loop timer (1 Hz)
│   │
│   ├─ control_loop()
│   │   ├─ Toggle blink state
│   │   ├─ Check vision timeout
│   │   ├─ read_sensors() (at poll_interval)
│   │   ├─ update_display()
│   │   └─ Disable multiplexer
│   │
│   ├─ read_sensors()
│   │   ├─ For each sensor: read and publish
│   │   └─ Fan control logic
│   │
│   ├─ update_display()
│   │   └─ Render OLED with status/temps
│   │
│   ├─ run_system_diagnostics()
│   └─ Helper callbacks (robot_status_cb, vision_status_cb)
│
└─ main()
```

## See Also

- [Sensor Reader Documentation](../sensor_reader/README.md)
- [Klipper Bridge Documentation](../klipper_bridge/README.md)
- [Configuration Guide](../configuration.md)
