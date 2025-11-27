# Klipper Bridge Node Documentation

## Overview

The `klipper_bridge` node provides bidirectional communication between ROS 2 and Klipper/Moonraker. It translates ROS 2 messages (fan speeds, motor positions) into Klipper G-code commands and publishes Klipper's status back to ROS 2.

## Node Name

`klipper_bridge_node`

## Purpose

- Receive fan speed commands from ROS and send to Klipper
- Receive motor position commands and control manual steppers in Klipper
- Query Klipper status (temperatures, fans, print stats) and publish to ROS
- Act as the communication bridge between the Optimus hardware control system and Klipper firmware

## Configuration

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `moonraker_url` | string | `http://localhost:7125` | Moonraker API endpoint URL |

### Launch Example

```python
Node(
    package='optimus',
    executable='klipper_bridge',
    name='klipper_bridge',
    parameters=[{'moonraker_url': 'http://localhost:7125'}]
)
```

## ROS 2 Interface

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/printer/status` | `std_msgs/String` | 1 Hz | JSON-formatted Klipper status including temperatures, fans, and print state |

**Status Message Format:**
```json
{
  "heater_bed": {
    "temperature": 0.0,
    "target": 0.0,
    "power": 0.0
  },
  "extruder": {
    "temperature": 0.0,
    "target": 0.0,
    "power": 0.0
  },
  "print_stats": {
    "state": "ready",
    "filename": "",
    "total_duration": 0.0
  },
  "fan_generic fan0": {
    "speed": 0.45
  },
  "fan_generic fan1": {
    "speed": 0.44
  }
}
```

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/fans/fan0/set_speed` | `std_msgs/Float32` | Set speed for fan0 (0.0-1.0) |
| `/fans/fan1/set_speed` | `std_msgs/Float32` | Set speed for fan1 (0.0-1.0) |
| `/motor/stepper_0/position` | `std_msgs/Float32` | Move stepper_0 to angle (degrees or mm) |
| `/motor/stepper_1/position` | `std_msgs/Float32` | Move stepper_1 to angle (degrees or mm) |
| `/motor/stepper_2/position` | `std_msgs/Float32` | Move stepper_2 to angle (degrees or mm) |
| `/motor/stepper_3/position` | `std_msgs/Float32` | Move stepper_3 to angle (degrees or mm) |

## Architecture

### Communication Flow

```
ROS 2 Topic                Klipper Bridge              Moonraker/Klipper
─────────────────         ──────────────────          ─────────────────

/fans/fan0/set_speed  ──>  fan0_cb()           ──>    POST /printer/gcode/script
                                                       {"script": "SET_FAN_SPEED FAN=fan0 SPEED=0.45"}

/motor/stepper_0/pos  ──>  motor0_cb()         ──>    POST /printer/gcode/script
                                                       {"script": "MANUAL_STEPPER ..."}

                           timer_callback() (1Hz) ──>  GET /printer/objects/query
                                                       
                      <──  publish status        <──   {"result": {"status": {...}}}
/printer/status
```

### Status Polling Loop

The node polls Klipper status at 1 Hz via timer callback:

1. Send GET request to Moonraker `/printer/objects/query` endpoint
2. Request specific objects: `heater_bed`, `extruder`, `print_stats`, `fan_generic fan0`, `fan_generic fan1`
3. Parse JSON response
4. Publish to `/printer/status` as JSON string

## Fan Control

### Input Message

```python
# /fans/fan0/set_speed
data: 0.45  # Speed from 0.0 (off) to 1.0 (full)
```

### Klipper G-code Command

```gcode
SET_FAN_SPEED FAN=fan0 SPEED=0.45
```

### Implementation

```python
def set_fan_speed(self, fan_name, speed):
    # Clamp speed to valid range
    speed = max(0.0, min(1.0, speed))
    
    # Generate G-code
    gcode = f"SET_FAN_SPEED FAN={fan_name} SPEED={speed:.2f}"
    
    # Send to Moonraker
    requests.post(
        f"{self.moonraker_url}/printer/gcode/script",
        params={"script": gcode},
        timeout=5.0
    )
```

### Error Handling

- Invalid speeds are clamped to 0.0-1.0 range
- HTTP errors are logged but do not crash the node
- Connection failures are logged as warnings

## Motor Control

### Input Message

```python
# /motor/stepper_0/position
data: 90.0  # Target position in degrees or mm
```

### Klipper G-code Sequence

```gcode
MANUAL_STEPPER STEPPER=stepper_0 ENABLE=1
MANUAL_STEPPER STEPPER=stepper_0 MOVE=90.0 SPEED=1.5 SYNC=1
MANUAL_STEPPER STEPPER=stepper_0 ENABLE=0
```

### Command Breakdown

1. **ENABLE=1**: Power on the stepper motor
2. **MOVE={angle} SPEED=1.5 SYNC=1**: 
   - Move to target position
   - Speed: 1.5 mm/s (gentle movement for calibration)
   - SYNC=1: Wait for move to complete before next command
3. **ENABLE=0**: Release/disable motor (conserves power, reduces heat)

### Implementation

```python
def move_motor(self, stepper_name, angle):
    gcode = (
        f"MANUAL_STEPPER STEPPER={stepper_name} ENABLE=1\n"
        f"MANUAL_STEPPER STEPPER={stepper_name} MOVE={angle} SPEED=1.5 SYNC=1\n"
        f"MANUAL_STEPPER STEPPER={stepper_name} ENABLE=0"
    )
    
    requests.post(
        f"{self.moonraker_url}/printer/gcode/script",
        params={"script": gcode},
        timeout=30.0  # Longer timeout for motor moves
    )
```

### Motor Configuration Requirements

Klipper `printer.cfg` must define manual steppers:

```ini
[manual_stepper stepper_0]
step_pin: PG0
dir_pin: PG1
enable_pin: !PF15
microsteps: 16
rotation_distance: 40
endstop_pin: ^PG6
```

## Moonraker API Integration

### Endpoints Used

#### 1. Execute G-code Script
```
POST /printer/gcode/script
Params: {script: "G-code commands"}
```

**Use Cases:**
- Fan speed control
- Motor movements
- Any Klipper G-code command

#### 2. Query Printer Objects
```
GET /printer/objects/query?heater_bed&extruder&print_stats&fan_generic+fan0&fan_generic+fan1
```

**Response Format:**
```json
{
  "result": {
    "status": {
      "heater_bed": {...},
      "extruder": {...},
      "print_stats": {...},
      "fan_generic fan0": {...},
      "fan_generic fan1": {...}
    }
  }
}
```

## Error Handling

### Connection Failures

```python
except Exception as e:
    self.get_logger().warn(f"Failed to connect to Moonraker: {e}")
```

- Logged as warnings (not errors)
- Node continues attempting to reconnect on next timer cycle
- Does not crash the node

### HTTP Errors

```python
if response.status_code != 200:
    self.get_logger().error(f"Failed to set fan speed: {response.status_code}")
```

- Non-200 status codes logged
- Request failure does not propagate errors to hardware_node
- Retry on next fan speed update

### Timeout Configuration

| Operation | Timeout | Reason |
|-----------|---------|--------|
| Status query | 1.0 s | Fast polling, non-critical |
| Fan speed | 5.0 s | Quick command, reasonable wait |
| Motor move | 30.0 s | Long moves may take time |

## Dependencies

### Python Packages
```bash
pip install requests
```

### ROS 2 Packages
- `rclpy`
- `std_msgs`

### External Services
- **Moonraker** running on specified port (default 7125)
- **Klipper** firmware connected to Moonraker
- Klipper configuration with `[fan_generic]` and `[manual_stepper]` sections

## Integration with Hardware Node

```
hardware_node               klipper_bridge              Klipper
─────────────              ──────────────              ────────

read_sensors()
  └─> calculate_fan_speed()
       └─> publish to /fans/fan1/set_speed  ──>  fan1_cb()  ──>  SET_FAN_SPEED
       

                                             ──>  timer_callback()  ──>  Query Status
                                             <──  Receive Status
           subscribe /printer/status   <──  publish status
           
update_display()
  └─> show motor state from printer status
```

## Troubleshooting

### Fans Not Responding

1. **Check Moonraker connection:**
   ```bash
   curl http://localhost:7125/printer/info
   ```

2. **Verify fan topics are publishing:**
   ```bash
   ros2 topic echo /fans/fan0/set_speed
   ```

3. **Check Klipper config:**
   ```ini
   [fan_generic fan0]
   pin: PA8
   max_power: 1.0
   ```

4. **Review bridge logs:**
   ```bash
   ros2 node info /klipper_bridge_node
   ```

### Motors Not Moving

1. **Verify manual_stepper configuration in printer.cfg**
2. **Check motor position topics:**
   ```bash
   ros2 topic pub /motor/stepper_0/position std_msgs/Float32 "{data: 10.0}"
   ```
3. **Monitor Klipper console for errors**
4. **Verify motors are not disabled by emergency stop**

### Status Not Publishing

1. **Check Moonraker is running:**
   ```bash
   systemctl status moonraker
   ```

2. **Verify query endpoint:**
   ```bash
   curl "http://localhost:7125/printer/objects/query?print_stats"
   ```

3. **Check ROS topic:**
   ```bash
   ros2 topic hz /printer/status  # Should be ~1 Hz
   ```

## Performance Considerations

### Status Polling Rate

- Default: 1 Hz (configurable via timer period)
- Higher rates may increase CPU usage on Raspberry Pi
- Moonraker caches most status data, queries are fast

### Fan Speed Updates

- No rate limiting implemented
- Rapid updates from hardware_node (every 5s) are acceptable
- Klipper handles command queuing internally

### Motor Movements

- Movements are **synchronous** (SYNC=1)
- HTTP request blocks until move completes
- Maximum timeout: 30 seconds
- Suitable for calibration, not real-time control

## Code Structure

```
klipper_bridge.py
├─ KlipperBridgeNode (class)
│   ├─ __init__()
│   │   ├─ Declare parameters (moonraker_url)
│   │   ├─ Create status publisher
│   │   ├─ Subscribe to fan control topics
│   │   ├─ Subscribe to motor position topics
│   │   └─ Start 1 Hz timer for status polling
│   │
│   ├─ fan0_cb() / fan1_cb()
│   │   └─> set_fan_speed()
│   │
│   ├─ motor0_cb() / motor1_cb() / motor2_cb() / motor3_cb()
│   │   └─> move_motor()
│   │
│   ├─ timer_callback()
│   │   ├─> GET request to Moonraker
│   │   └─> Publish status to /printer/status
│   │
│   ├─ set_fan_speed(fan_name, speed)
│   │   └─> POST G-code: SET_FAN_SPEED
│   │
│   └─ move_motor(stepper_name, angle)
│       └─> POST G-code: MANUAL_STEPPER sequence
│
└─ main()
```

## See Also

- [Hardware Node Documentation](../hardware_node/README.md)
- [Klipper G-code Reference](https://www.klipper3d.org/G-Codes.html)
- [Moonraker API Documentation](https://moonraker.readthedocs.io/en/latest/web_api/)
- [Motor Calibration Server Documentation](../motor_calibration_server/README.md)
