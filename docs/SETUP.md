# Face Tracking Gimbal - Setup Guide

## Overview

This project implements a face-tracking gimbal using:
- **Jetson Nano**: Runs face detection via OpenCV and sends position over UDP
- **Raspberry Pi**: Controls pan/tilt motors via Klipper firmware

## Hardware

- BTT Octopus V1.1 mainboard with TMC2209 drivers
- Pan motor (stepper_0): ±22° range
- Tilt motor (stepper_1): -2.5° to +5° range
- Camera mounted on gimbal (camera-on-head configuration)

## Network

| Device | IP Address | Role |
|--------|------------|------|
| Jetson | 192.168.1.133 | Face detection, UDP sender |
| RPi | 192.168.1.136 | Motor control, UDP receiver |
| UDP Port | 5555 | Face position data |
| Moonraker | localhost:7125 | Motor API (on RPi) |

## Installation

### Jetson (Face Detection)

```bash
# Install dependencies
pip3 install opencv-python flask

# Run face tracker
cd jetson
python3 face_tracker.py
```

The Jetson also serves a web stream at `http://192.168.1.133:8080` for debugging.

### Raspberry Pi (Motor Control)

```bash
# Install dependencies  
pip3 install requests

# Run motor controller
cd rpi
python3 face_tracker_v5.py
```

## Configuration

Edit `config.json` to adjust:
- Network IPs and ports
- Motor limits and speeds
- Tracking parameters (deadzone, gains)
- Calibration matrix

See [TUNING.md](TUNING.md) for tuning guide.

## Quick Start

1. Start Klipper on RPi (should auto-start via systemd)
2. Start face tracker on Jetson: `python3 jetson/face_tracker.py`
3. Start motor controller on RPi: `python3 rpi/face_tracker_v5.py`
4. Face the camera - gimbal should track your face!
