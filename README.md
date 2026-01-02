# Face Tracking Gimbal

A face-tracking gimbal system using Jetson Nano for vision and Raspberry Pi + Klipper for motor control.

## Features

- Real-time face detection using OpenCV Haar cascades
- Smooth pan/tilt motor control via Klipper firmware
- Dynamic step scaling (fast when far, precise when close)
- Web streaming for debugging
- UDP-based communication between devices

## Project Structure

```
optimus/
├── jetson/
│   └── face_tracker.py      # Face detection + UDP sender
├── rpi/
│   └── face_tracker_v5.py   # Motor control + UDP receiver
├── shared/
│   ├── motor_control.py     # Moonraker API wrapper
│   └── klipper_direct.py    # Direct Klipper socket control
├── docs/
│   ├── SETUP.md             # Installation guide
│   └── TUNING.md            # Tuning parameters
├── config.json              # Configuration file
└── config.annotated.jsonc   # Annotated config with comments
```

## Quick Start

1. **Jetson**: `python3 jetson/face_tracker.py`
2. **RPi**: `python3 rpi/face_tracker_v5.py`

See [docs/SETUP.md](docs/SETUP.md) for full installation instructions.

## Configuration

Edit `config.json` to adjust network settings, motor limits, and tracking parameters.

See [docs/TUNING.md](docs/TUNING.md) for tuning guide.

## Hardware

- Jetson Nano (face detection)
- Raspberry Pi 4 (motor control)
- BTT Octopus V1.1 + TMC2209 drivers
- Pan/tilt gimbal mechanism

## Architecture

```
┌─────────────┐     UDP:5555      ┌─────────────┐    Moonraker    ┌──────────┐
│   Jetson    │ ───────────────► │     RPi     │ ─────────────► │  Klipper │
│ (OpenCV)    │   face offset    │ (controller)│    G-code      │ (motors) │
└─────────────┘                  └─────────────┘                 └──────────┘
```

## License

MIT
