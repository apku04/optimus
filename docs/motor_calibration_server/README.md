# Motor Calibration Server Documentation

## Overview

The `motor_calibration_server` provides a web-based interface for calibrating motor positions and movement limits. It runs a Flask web server accessible via browser, allowing interactive motor control and recording of center, minimum, and maximum positions for each stepper motor.

## Node Name

`motor_calibration_server`

## Purpose

- Provide user-friendly web interface for motor calibration
- Allow real-time motor position control via slider
- Record calibration data (center, min, max positions)
- Save calibration to JSON file for use by other nodes
- Publish motor position commands to ROS topics

## Architecture

### Dual-Thread Design

```
Main Thread (ROS)          Background Thread (Flask)
─────────────────          ──────────────────────────
rclpy.spin()               app.run()
├─ motor_publishers        ├─ Web routes
│   ├─ /motor/stepper_0    │   ├─ GET  /
│   ├─ /motor/stepper_1    │   ├─ GET  /state
│   ├─ /motor/stepper_2    │   ├─ POST /move
│   └─ /motor/stepper_3    │   ├─ POST /record
│                          │   ├─ POST /save
└─ Shared: ros_node        └─ POST /move_center
   global variable
```

## Web Interface

### URL

```
http://{raspberry_pi_ip}:5000
```

**Default:** `http://localhost:5000` (when accessing from the Pi itself)

### Interface Layout

```
┌──────────────────────────────────────────┐
│     🤖 Motor Calibration Tool            │
├──────────────────────────────────────────┤
│  [Motor 0] [Motor 1] [Motor 2] [Motor 3] │  ← Motor Selection
├──────────────────────────────────────────┤
│  Instructions: Move to center position   │  ← Step-by-step guide
├──────────────────────────────────────────┤
│          Position: 45.0°                 │  ← Real-time position
│    [--------●------------]               │  ← Slider (-180 to 180)
├──────────────────────────────────────────┤
│  [Start] [Record] [Center] [Reset]       │  ← Control buttons
├──────────────────────────────────────────┤
│  Calibration Data:                       │
│   Motor 0: ✓ Complete                    │
│     Center: 0.0°  Min: -90°  Max: 90°    │
│   Motor 1: ○ Incomplete                  │
│     Center: ---  Min: ---  Max: ---      │
├──────────────────────────────────────────┤
│     [Save All Calibration Data]          │
└──────────────────────────────────────────┘
```

## Calibration Workflow

### Step-by-Step Process

1. **Select Motor**
   - Click motor button (Motor 0-3)
   - Instructions update to guide next step

2. **Start Calibration**
   - Click "Start Calibration" button
   - Instruction: "Move to CENTER position"

3. **Find Center Position**
   - Use slider to move motor to neutral/center position
   - Click "Record Position" when centered
   - Center position saved

4. **Find Minimum Position**
   - Instruction: "Move to MINIMUM (counter-clockwise)"
   - Use slider to move to physical limit
   - Click "Record Position"
   - Minimum position saved

5. **Return to Center** (optional)
   - Click "Move to Center" to safely return
   - Prevents stress on motor/mechanism

6. **Find Maximum Position**
   - Instruction: "Move to MAXIMUM (clockwise)"
   - Use slider to move to opposite limit
   - Click "Record Position"
   - Maximum position saved

7. **Completion**
   - Status shows "✓ Complete"
   - Displays: `Center=0.0°, Range=[-90° to 90°]`

8. **Save Calibration**
   - Click "Save All Calibration Data"
   - Writes to `/home/acp/motor_calibration.json`

### Calibration State Machine

```
┌────────┐
│  IDLE  │ <─── Select Motor
└───┬────┘
    │ Start Calibration
    ▼
┌─────────────┐
│ FIND_CENTER │
└───┬─────────┘
    │ Record Position
    ▼
┌──────────┐    Move to Center
│ FIND_MIN │ ◄─────────────┐
└───┬──────┘                │
    │ Record Position       │
    ▼                       │
┌──────────┐                │
│ FIND_MAX │ ───────────────┘
└───┬──────┘
    │ Record Position
    ▼
┌──────────┐
│ COMPLETE │
└──────────┘
```

## ROS 2 Interface

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/motor/stepper_0/position` | `std_msgs/Float32` | Target position for motor 0 (degrees or mm) |
| `/motor/stepper_1/position` | `std_msgs/Float32` | Target position for motor 1 |
| `/motor/stepper_2/position` | `std_msgs/Float32` | Target position for motor 2 |
| `/motor/stepper_3/position` | `std_msgs/Float32` | Target position for motor 3 |

### Message Format

```python
# When slider moves or button clicked
data: 45.5  # Target position in degrees/mm
```

**Note:** These topics are subscribed to by `klipper_bridge`, which translates them into Klipper `MANUAL_STEPPER` commands.

## API Endpoints

### GET /

Serves the main HTML/JavaScript web interface.

**Response:** HTML page

### GET /state

Returns current calibration state as JSON.

**Response:**
```json
{
  "current_motor": "stepper_0",
  "step": "find_center",
  "instructions": "Move to center position...",
  "motors": {
    "stepper_0": {
      "position": 0.0,
      "center": null,
      "min": null,
      "max": null
    },
    ...
  }
}
```

### POST /select_motor

Select which motor to calibrate.

**Request:**
```json
{
  "motor": "stepper_1"
}
```

**Response:**
```json
{
  "success": true
}
```

### POST /move

Move motor to specific position.

**Request:**
```json
{
  "motor": "stepper_0",
  "position": 45.5
}
```

**Effect:** Publishes `Float32(45.5)` to `/motor/stepper_0/position`

**Response:**
```json
{
  "success": true
}
```

### POST /start_calibration

Begin calibration process for current motor.

**Effect:** Sets state to `find_center`, updates instructions

**Response:**
```json
{
  "success": true
}
```

### POST /record

Record current position (center, min, or max depending on calibration step).

**Request:**
```json
{
  "motor": "stepper_0",
  "position": 0.0
}
```

**Effect:** 
- Saves position to `calibration_state['motors'][motor][step]`
- Advances to next calibration step

**Response:**
```json
{
  "success": true
}
```

### POST /move_center

Move motor back to its recorded center position.

**Request:**
```json
{
  "motor": "stepper_0"
}
```

**Effect:** Publishes center position to motor topic

**Response:**
```json
{
  "success": true,
  "position": 0.0
}
```

### POST /reset

Reset calibration data for a motor.

**Request:**
```json
{
  "motor": "stepper_0"
}
```

**Effect:** Clears center/min/max values, resets to idle

**Response:**
```json
{
  "success": true
}
```

### POST /save

Save all calibration data to JSON file.

**Effect:** Writes to `/home/acp/motor_calibration.json`

**Response:**
```json
{
  "success": true,
  "file": "/home/acp/motor_calibration.json"
}
```

## Calibration Data Format

### Output File

`/home/acp/motor_calibration.json`

```json
{
  "stepper_0": {
    "position": 0.0,
    "center": 0.0,
    "min": -90.0,
    "max": 90.0
  },
  "stepper_1": {
    "position": 0.0,
    "center": 0.0,
    "min": -45.0,
    "max": 45.0
  },
  "stepper_2": {
    "position": 0.0,
    "center": 0.0,
    "min": -60.0,
    "max": 60.0
  },
  "stepper_3": {
    "position": 0.0,
    "center": 0.0,
    "min": -30.0,
    "max": 30.0
  }
}
```

### Usage in Other Nodes

```python
import json

with open('/home/acp/motor_calibration.json', 'r') as f:
    calibration = json.load(f)

# Get motor 0 limits
motor0_center = calibration['stepper_0']['center']
motor0_min = calibration['stepper_0']['min']
motor0_max = calibration['stepper_0']['max']

# Clamp target position to limits
target = max(motor0_min, min(motor0_max, target))
```

## Integration with Klipper Bridge

```
Web Interface          motor_calibration_server       klipper_bridge          Klipper
─────────────          ────────────────────────       ──────────────          ───────

Slider moved ──>       POST /move                ──>  Publish to topic  ──>   Subscribes
                       {motor, position}              /motor/stepper_0/pos     motor0_cb()
                                                                          ──>  MANUAL_STEPPER
                                                                               commands
```

## Usage

### Starting the Server

```bash
# From terminal
ros2 run optimus motor_calibration_server

# Output:
# ============================================================
# Motor Calibration Web Interface Ready!
# Open in browser: http://localhost:5000
# ============================================================
```

### Accessing from Another Computer

```bash
# Find Raspberry Pi IP
hostname -I

# On another computer, open browser:
http://192.168.1.xxx:5000
```

### Calibrating Multiple Motors

1. Calibrate Motor 0 completely
2. Click "Save All Calibration Data" (saves progress)
3. Select Motor 1
4. Repeat process
5. Save again after each motor

## Safety Considerations

### Physical Limits

- **Move slowly:** Use slider carefully near limits
- **Listen for sounds:** Stop if motor sounds strained
- **Visual inspection:** Ensure mechanism isn't binding
- **Emergency stop:** Keep Klipper emergency stop accessible

### Soft Limits

Once calibrated, implement software limits:

```python
def clamp_position(motor_name, target):
    cal = calibration[motor_name]
    return max(cal['min'], min(cal['max'], target))
```

### Motor Holding Current

- Motors stay energized after move (ENABLE=1)
- Can cause heating if held at position for extended periods
- Klipper bridge sends ENABLE=0 after moves to release

## Troubleshooting

### Web Interface Not Loading

1. **Check server is running:**
   ```bash
   ros2 node list | grep calibration
   ```

2. **Verify port 5000 is accessible:**
   ```bash
   sudo netstat -tulpn | grep 5000
   ```

3. **Check firewall:**
   ```bash
   sudo ufw status
   sudo ufw allow 5000/tcp
   ```

### Motors Not Moving

1. **Check topic publishing:**
   ```bash
   ros2 topic echo /motor/stepper_0/position
   ```

2. **Verify klipper_bridge is running:**
   ```bash
   ros2 node list | grep klipper
   ```

3. **Check Klipper connection:**
   - Open Moonraker web interface
   - Check manual stepper configuration

### Calibration Data Not Saving

1. **Check file permissions:**
   ```bash
   ls -la /home/acp/motor_calibration.json
   ```

2. **Verify write access:**
   ```bash
   touch /home/acp/motor_calibration.json
   ```

3. **Check disk space:**
   ```bash
   df -h
   ```

### Slider Not Responsive

- **Browser compatibility:** Use modern browser (Chrome, Firefox, Edge)
- **JavaScript errors:** Open browser console (F12) and check for errors
- **Network latency:** Check connection quality if accessing remotely

## Dependencies

### Python Packages

```bash
pip install flask rclpy
```

### ROS 2 Packages

- `rclpy`
- `std_msgs`

### External Dependencies

- **Klipper bridge node** must be running
- **Moonraker** must be accessible
- Manual steppers configured in `printer.cfg`

## Performance Considerations

### Network Latency

- Web interface polls state every 2 seconds
- Slider updates send immediate commands
- Remote access may have slight delay

### Motor Response Time

- Typical move time: 1-5 seconds (depends on distance and speed)
- Speed set in klipper_bridge: 1.5 mm/s (gentle for calibration)

### Concurrent Users

- Flask runs in single-threaded mode
- Only one user should calibrate at a time
- Multiple browsers will share the same state

## Code Structure

```
motor_calibration_server.py
├─ Flask Application (app)
│   ├─ HTML_TEMPLATE (embedded HTML/CSS/JS)
│   ├─ Routes:
│   │   ├─ GET  /
│   │   ├─ GET  /state
│   │   ├─ POST /select_motor
│   │   ├─ POST /move
│   │   ├─ POST /start_calibration
│   │   ├─ POST /record
│   │   ├─ POST /move_center
│   │   ├─ POST /reset
│   │   └─ POST /save
│   └─ Global: calibration_state (shared dict)
│
├─ MotorCalibrationNode (ROS 2)
│   └─ motor_publishers (dict of 4 publishers)
│
├─ run_flask() (thread function)
└─ main()
    ├─> Initialize ROS node
    ├─> Start Flask in background thread
    └─> rclpy.spin()
```

## Best Practices

### Before Calibration

1. Ensure motors are mechanically connected and secure
2. Verify no obstructions in movement range
3. Check Klipper configuration has correct pin assignments
4. Test emergency stop functionality

### During Calibration

1. Move slowly and carefully near limits
2. Listen for unusual sounds
3. Watch for binding or resistance
4. Save frequently (after each motor)

### After Calibration

1. Verify calibration file was created
2. Test motors move within recorded ranges
3. Implement software limits in motion planning code
4. Document any unusual positions or quirks

## See Also

- [Klipper Bridge Documentation](../klipper_bridge/README.md)
- [Klipper Manual Stepper Documentation](https://www.klipper3d.org/G-Codes.html#manual-stepper)
- [Flask Documentation](https://flask.palletsprojects.com/)
