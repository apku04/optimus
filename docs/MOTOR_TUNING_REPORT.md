# Motor Tuning Report - Gimbal Control

**Date:** January 2, 2026  
**Status:** ✅ WORKING - Smooth velocity mode achieved

---

## Hardware Setup

| Component | Model | Connection |
|-----------|-------|------------|
| **Controller** | BTT Octopus V1.1 | USB to RPi |
| **Drivers** | TMC2209 | UART mode |
| **Firmware** | Klipper | via Moonraker API |
| **Pan Motor** | stepper_0 | Range: -22° to +22° |
| **Tilt Motor** | stepper_1 | Range: -2.5° to +5° |

---

## ❌ What DIDN'T Work

### 1. Position Streaming (Choppy)
```
Problem: Sending continuous position updates creates "chop-chop-chop" motion
Cause: Each MANUAL_STEPPER MOVE command = motor start/stop cycle
Result: Table shaking, aggressive jerky motion
```

### 2. SYNC Mode with High Frequency
```
Problem: Even with SYNC=1, frequent small moves cause vibration
Cause: Motor constantly starting/stopping for tiny adjustments
Result: Buzzing, vibration, not smooth
```

### 3. High Speed + High Acceleration
```
Problem: Fast movements are aggressive and jerky
Cause: TMC2209 + stepper physics = harsh starts/stops at high accel
Result: Scary fast, not gimbal-like
```

### 4. Heavy Smoothing + Large Deadzone
```
Problem: Too much smoothing = no response
Cause: Targets never exceed deadzone threshold
Result: Motors don't move at all
```

---

## ✅ WINNING CONFIGURATION

### Key Insight
**Don't send frequent position updates. Send ONE move command and let motor glide smoothly to target.**

### Working Parameters

```python
# VELOCITY MODE - SMOOTH MOTION
MAX_SPEED = 30        # degrees/sec - SLOW for smooth motion
ACCEL = 100           # degrees/sec² - Gentle acceleration
MIN_CHANGE = 0.5      # degrees - Only send new command if target changed > 0.5°

# Speed scales with distance:
# - Short distance (< 3°): speed = 10°/s (very gentle)
# - Medium distance: speed = distance * 3 (proportional)
# - Long distance: capped at MAX_SPEED (30°/s)
```

### Command Pattern
```python
# Send ONE smooth move, don't spam intermediate positions
cmd = f"MANUAL_STEPPER STEPPER=stepper_0 MOVE={target:.2f} SPEED={speed:.0f} ACCEL={ACCEL} SYNC=0"
```

### Input Throttling
- Web UI: Max 10 updates/second (100ms minimum between sends)
- Backend: Only sends new command if target changed > 0.5°
- Result: ~2-5 actual motor commands per second during drag

---

## File Reference

| File | Purpose |
|------|---------|
| `/home/acp/optimus/rpi/motor_test_velocity.py` | ✅ WORKING smooth control |
| `/home/acp/optimus/rpi/motor_test_sync.py` | ❌ Choppy, not recommended |
| `/home/acp/optimus/rpi/motor_test_web.py` | ❌ Original, queue issues |

---

## Klipper Connection

```python
# Direct socket connection (fastest, ~1-5ms latency)
SOCKET_PATH = "/home/acp/printer_data/comms/klippy.sock"

from klipper_direct import KlipperDirect
klipper = KlipperDirect(SOCKET_PATH)
klipper.connect()

# Enable motors
klipper.gcode("MANUAL_STEPPER STEPPER=stepper_0 ENABLE=1")
klipper.gcode("MANUAL_STEPPER STEPPER=stepper_1 ENABLE=1")

# Smooth move (non-blocking)
klipper.gcode(f"MANUAL_STEPPER STEPPER=stepper_0 MOVE={pan} SPEED={speed} ACCEL=100 SYNC=0", wait=False)
```

---

## For Face Tracking Implementation

### Recommended Approach
1. **Receive face position from Jetson** (UDP port 5555)
2. **Convert to motor angles** (map pixel offset to degrees)
3. **Only send motor command if change > 0.5°**
4. **Use proportional speed** (bigger error = faster correction)
5. **Keep acceleration low (100)** for smooth motion

### Suggested Face Tracker Parameters
```python
# Face tracking tuning
FACE_DEADZONE = 0.5      # degrees - ignore tiny face movements
MAX_TRACK_SPEED = 30     # degrees/sec - smooth following
MIN_TRACK_SPEED = 10     # degrees/sec - minimum speed
TRACK_ACCEL = 100        # degrees/sec² - gentle ramps
SPEED_GAIN = 3.0         # speed = error * SPEED_GAIN (capped at MAX)

# Example speed calculation:
# error = 5° → speed = min(30, max(10, 5 * 3)) = 15°/s
# error = 15° → speed = min(30, max(10, 15 * 3)) = 30°/s
```

---

## Network Architecture

```
┌─────────────────┐        UDP:5555         ┌─────────────────┐
│     JETSON      │ ───────────────────────▶│       RPI       │
│  192.168.1.133  │    face_x, face_y       │  192.168.1.136  │
│                 │      timestamp          │                 │
│  Face Detection │                         │  Motor Control  │
│  (GPU CUDA)     │                         │  (Klipper)      │
└─────────────────┘                         └────────┬────────┘
                                                     │
                                              Unix Socket
                                                     │
                                            ┌────────▼────────┐
                                            │    OCTOPUS      │
                                            │  BTT V1.1       │
                                            │  TMC2209        │
                                            └─────────────────┘
```

---

## Quick Test Commands

```bash
# Run smooth motor test
cd /home/acp/optimus && python3 rpi/motor_test_velocity.py
# Open: http://192.168.1.136:8083/

# Stop all motors
curl -X POST "http://localhost:7125/printer/gcode/script" -d '{"script": "STOP_ALL_MOTORS"}'

# Test UDP from Jetson
timeout 5 python3 -c "
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 5555))
data, addr = sock.recvfrom(4096)
print(data.decode())
"
```

---

## Summary

| Aspect | Bad | Good |
|--------|-----|------|
| Command frequency | 60+ Hz | 2-5 Hz |
| Speed | 100-200°/s | 10-30°/s |
| Acceleration | 400-2000 | 100 |
| Approach | Stream positions | Set target, let motor glide |
| SYNC mode | SYNC=1 (blocking) | SYNC=0 (non-blocking) |

**The key is: LET THE MOTOR DO THE INTERPOLATION, not the software.**
