# Face Tracker Tuning Guide

## Overview

The face tracker uses a **step-and-settle** approach:
1. Measure face position error
2. Compute motor step using calibrated A-matrix
3. Move motors
4. Wait for settle time
5. Take fresh measurement
6. Repeat

This works well with camera-on-gimbal setups where the camera moves with the motors.

## Key Parameters

All tuning parameters are in `rpi/face_tracker_v5.py` (lines 53-72):

### Basic Control

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DEADZONE` | 0.1 | Don't move if error < this (normalized units) |
| `STEP_GAIN` | 0.28 | How aggressive each step is |
| `EMA_ALPHA` | 0.45 | Measurement smoothing (0=no filter, 1=max filter) |

### Step Limits

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAX_PAN_STEP` | 0.25° | Maximum pan step per cycle |
| `MAX_TILT_STEP` | 0.07° | Maximum tilt step per cycle |

### Dynamic Step Scaling

Steps scale based on distance from target:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MIN_STEP_SCALE` | 0.3 | Scale factor when close (30% of max) |
| `FAR_THRESHOLD` | 0.5 | Error above this = full speed |

**How it works:**
- Error near deadzone → small precise steps (30% of max)
- Error > FAR_THRESHOLD → full-size steps (100% of max)
- Linear interpolation in between

### Timing

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SETTLE_TIME` | 0.2s | Wait after move for motor to settle |
| `FRESH_MEAS_WAIT` | 0.07s | Wait for fresh camera measurement |
| `LOOP_PERIOD` | 0.015s | Main loop sleep time |

### Motor Settings

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MOVE_SPEED` | 200°/s | Motor movement speed |
| `MOVE_ACCEL` | 50°/s² | Motor acceleration |

## Tuning Tips

### Problem: Oscillation / Jitter
- ↑ Increase `SETTLE_TIME` (try 0.25-0.3)
- ↑ Increase `DEADZONE` (try 0.12-0.15)
- ↓ Decrease `STEP_GAIN` (try 0.2-0.25)

### Problem: Too Slow
- ↓ Decrease `SETTLE_TIME` (try 0.15)
- ↑ Increase `MAX_PAN_STEP` / `MAX_TILT_STEP`
- ↑ Increase `FAR_THRESHOLD` (larger area uses full speed)

### Problem: Overshoots Target
- ↓ Decrease `STEP_GAIN`
- ↓ Decrease `MIN_STEP_SCALE` (smaller steps when close)
- ↑ Increase `FRESH_MEAS_WAIT`

### Problem: Doesn't Track Fast Movement
- ↓ Decrease `EMA_ALPHA` (less filtering = faster response)
- ↑ Increase `MAX_PAN_STEP` / `MAX_TILT_STEP`
- ↑ Increase `FAR_THRESHOLD`

## Calibration Matrix

The A-matrix maps motor degrees to image offsets. Default values:

```
dx_dpan = 0.17165    # Pan 1° → face moves 0.17 in X
dy_dtilt = 0.036     # Tilt 1° → face moves 0.036 in Y
dx_dtilt = -0.016    # Cross-coupling (usually small)
dy_dpan = 0.002      # Cross-coupling (usually small)
```

If tracking is backwards, flip the sign of the corresponding value.

## Testing Changes

After editing parameters:

```bash
pkill -9 -f face_tracker; python3 rpi/face_tracker_v5.py
```

Watch the console output - it shows:
- `[OK]` - In deadzone, no movement
- `[MOVE]` - Making a step (shows error, scale, step size, new position)
