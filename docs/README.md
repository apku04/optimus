# Optimus Robot - ROS 2 System Documentation

**ROS 2 Version:** Jazzy  
**Platform:** Raspberry Pi (Ubuntu 24.04)  
**Last Updated:** November 27, 2025

## Table of Contents

1. [System Overview](#system-overview)
2. [Node Architecture](#node-architecture)
3. [ROS 2 Integration](#ros-2-integration)
4. [Development Workflow](#development-workflow)
5. [Debugging Guide](#debugging-guide)
6. [Creating New Nodes](#creating-new-nodes)
7. [Modifying Existing Nodes](#modifying-existing-nodes)
8. [System Management](#system-management)

---

## System Overview

The Optimus robot system is a ROS 2-based platform combining voice interaction, environmental sensing, motor control, and hardware management into an integrated robotics solution.

### Core Components

```
┌─────────────────────────────────────────────────────────────┐
│                    Optimus Robot System                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐      ┌──────────────┐                   │
│  │ voice_node   │◄────►│ hardware_node│                   │
│  │ (STT/TTS)    │      │ (Sensors/Fan)│                   │
│  └──────────────┘      └───────┬──────┘                   │
│         │                      │                           │
│         │                      ▼                           │
│         │              ┌───────────────┐                   │
│         │              │ sensor_reader │                   │
│         │              │ (I2C Mux)     │                   │
│         │              └───────────────┘                   │
│         │                                                  │
│         ▼                      ▼                           │
│  ┌─────────────────────────────────────┐                  │
│  │        klipper_bridge               │                  │
│  │  (Motor Control via Moonraker)      │                  │
│  └─────────────┬───────────────────────┘                  │
│                │                                           │
│                ▼                                           │
│         ┌─────────────┐                                    │
│         │   Klipper   │                                    │
│         │  (Steppers) │                                    │
│         └─────────────┘                                    │
│                                                             │
│  ┌────────────────────────────────────┐                   │
│  │ motor_calibration_server           │                   │
│  │ (Web UI - Manual Calibration)      │                   │
│  └────────────────────────────────────┘                   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Key Features

- **Voice Interaction:** Wake-word detection, speech-to-text, LLM integration, text-to-speech
- **Environmental Monitoring:** Temperature, humidity, pressure sensors via I2C
- **Intelligent Cooling:** Temperature-based fan control through Klipper/Moonraker
- **Motor Control:** 4 stepper motors with calibration and position control
- **Visual Display:** Dual OLED displays for system status
- **Web Interface:** Browser-based motor calibration tool

---

## Node Architecture

### 1. voice_node

**Purpose:** Voice interaction and user interface

**Key Responsibilities:**
- Wake-word detection ("Hey Optimus")
- Speech-to-text conversion (Vosk)
- LLM query processing (Ollama)
- Text-to-speech playback (Piper)
- Optional vision integration (camera + LLM)

**ROS Topics:**
- Subscribes: `/sensor/combined` (sensor data for context)
- Publishes: None (standalone operation)

**External Dependencies:**
- Vosk server (STT)
- Ollama server at 192.168.1.184:11434 (LLM)
- Piper TTS (local)
- ALSA audio devices

**Documentation:** [voice_node/README.md](voice_node/README.md)

---

### 2. hardware_node

**Purpose:** Core hardware management and coordination

**Key Responsibilities:**
- I2C sensor initialization and validation
- OLED display updates (system status)
- Fan control based on temperature thresholds
- Sensor data aggregation and publishing

**ROS Topics:**
- Subscribes: `/sensor/data` (from sensor_reader)
- Publishes: `/sensor/combined` (aggregated data)

**External Dependencies:**
- PCA9548A I2C multiplexer at 0x70
- SSD1306 OLED displays (128x64)
- Klipper/Moonraker for fan control
- sensor_reader node

**Configuration:** `config.json` (sensors, fans, thresholds)

**Documentation:** [hardware_node/README.md](hardware_node/README.md)

---

### 3. sensor_reader

**Purpose:** Low-level I2C sensor reading

**Key Responsibilities:**
- Read temperature/humidity from SHT3x sensors
- Read temperature/pressure from BME280/BMP280 sensors
- Handle I2C multiplexer channel switching
- Publish raw sensor data at 1Hz

**ROS Topics:**
- Publishes: `/sensor/data` (JSON string with all sensor readings)

**Hardware:**
- PCA9548A multiplexer (0x70)
- Channel 0: RPi case sensor (SHT31)
- Channel 1: CPU sensor (SHT31)
- Channel 2: External sensor (BME280/BMP280)

**Documentation:** [sensor_reader/README.md](sensor_reader/README.md)

---

### 4. klipper_bridge

**Purpose:** Interface between ROS 2 and Klipper firmware

**Key Responsibilities:**
- Subscribe to motor position commands
- Translate ROS messages to Klipper G-code
- Send commands via Moonraker HTTP API
- Control stepper motors (MANUAL_STEPPER)
- Manage fan speeds

**ROS Topics:**
- Subscribes: `/motor/stepper_0/position` through `/motor/stepper_3/position`

**External Dependencies:**
- Moonraker API at localhost:7125
- Klipper firmware with configured manual steppers

**Documentation:** [klipper_bridge/README.md](klipper_bridge/README.md)

---

### 5. motor_calibration_server

**Purpose:** Web-based motor calibration interface

**Key Responsibilities:**
- Provide Flask web UI on port 5000
- Allow interactive motor position control
- Record center, min, max positions
- Save calibration to JSON file
- Publish position commands to ROS topics

**ROS Topics:**
- Publishes: `/motor/stepper_0/position` through `/motor/stepper_3/position`

**Output File:** `/home/acp/motor_calibration.json`

**Documentation:** [motor_calibration_server/README.md](motor_calibration_server/README.md)

---

## ROS 2 Integration

### Topic Data Flow

```
sensor_reader ──┬──> /sensor/data ──┬──> hardware_node ──> /sensor/combined ──> voice_node
                │                    │                                           (context)
                │                    └──> fan_control_logic
                │                          (internal to hardware_node)
                │
                └──> I2C Hardware
                     (SHT3x, BME280, etc.)


motor_calibration_server ──┬──> /motor/stepper_0/position ──┬──> klipper_bridge
voice_node (future)         │                                │         │
other control nodes ────────┴──> /motor/stepper_1/position ──┤         │
                                 /motor/stepper_2/position ──┤         │
                                 /motor/stepper_3/position ──┘         │
                                                                        ▼
                                                                  Moonraker API
                                                                        │
                                                                        ▼
                                                                     Klipper
                                                                  (Hardware)
```

### Message Types

| Topic | Type | Rate | Purpose |
|-------|------|------|---------|
| `/sensor/data` | `std_msgs/String` | 1 Hz | Raw sensor readings (JSON) |
| `/sensor/combined` | `std_msgs/String` | 1 Hz | Aggregated sensor data (JSON) |
| `/motor/stepper_N/position` | `std_msgs/Float32` | On-demand | Target motor position (degrees/mm) |

### Launch System

The system uses a single launch file to start all nodes:

**File:** `ros2_ws/src/optimus/launch/optimus.launch.py`

```python
# Launches:
# - sensor_reader
# - hardware_node
# - klipper_bridge
# - voice_node
# - motor_calibration_server (optional)
```

**Start all nodes:**
```bash
ros2 launch optimus optimus.launch.py
```

### Package Structure

```
ros2_ws/src/optimus/
├── package.xml              # ROS package metadata
├── setup.py                 # Python package configuration
├── setup.cfg                # Package install configuration
├── optimus/                 # Python package
│   ├── __init__.py
│   ├── hardware_node.py
│   ├── voice_node.py
│   ├── sensor_reader.py
│   ├── klipper_bridge.py
│   └── motor_calibration_server.py
├── launch/
│   └── optimus.launch.py   # Launch file
├── docs/                    # Documentation (this folder)
└── config.json              # Configuration file
```

---

## Development Workflow

### Prerequisites

```bash
# Install ROS 2 Jazzy
# Install required Python packages
pip install smbus2 adafruit-circuitpython-ssd1306 requests flask vosk pyaudio

# Install Piper TTS
# Install Ollama (on LLM server)
```

### Building the Package

```bash
cd /home/acp/ros2_ws

# Build all packages
colcon build

# Build only optimus package
colcon build --packages-select optimus

# Build with verbose output (for debugging)
colcon build --event-handlers console_direct+ --packages-select optimus
```

### Testing Changes

```bash
# After building, source the workspace
source install/setup.bash

# Test individual node
ros2 run optimus hardware_node

# Test with launch file
ros2 launch optimus optimus.launch.py
```

---

## Debugging Guide

### 1. Check Running Nodes

```bash
# List all active ROS 2 nodes
ros2 node list

# Expected output:
# /hardware_node
# /voice_node
# /sensor_reader
# /klipper_bridge
# /motor_calibration_server
```

### 2. Inspect Node Details

```bash
# Get info about a specific node
ros2 node info /hardware_node

# Shows:
# - Subscribed topics
# - Published topics
# - Services
# - Actions
```

### 3. Monitor Topics

```bash
# List all topics
ros2 topic list

# Echo topic data (real-time)
ros2 topic echo /sensor/data

# Check topic rate
ros2 topic hz /sensor/data

# Show topic info
ros2 topic info /sensor/data
```

### 4. Test Topic Publishing

```bash
# Manually publish to motor topic
ros2 topic pub /motor/stepper_0/position std_msgs/Float32 "data: 45.0"

# Publish once
ros2 topic pub --once /motor/stepper_0/position std_msgs/Float32 "data: 90.0"
```

### 5. Check Logs

```bash
# ROS 2 logs
ros2 run optimus hardware_node --ros-args --log-level debug

# System service logs (if running as service)
journalctl -u optimus -f

# Klipper logs
tail -f /home/acp/printer_data/logs/klippy.log

# Moonraker logs
tail -f /home/acp/printer_data/logs/moonraker.log
```

### 6. Common Issues

#### Node Not Starting

```bash
# Check Python import errors
python3 -c "from optimus import hardware_node"

# Verify package is installed
ros2 pkg list | grep optimus

# Rebuild if necessary
cd /home/acp/ros2_ws
colcon build --packages-select optimus --symlink-install
source install/setup.bash
```

#### No Data on Topics

```bash
# Check if publisher is running
ros2 topic info /sensor/data

# Check for errors in node
ros2 run optimus sensor_reader

# Verify hardware connections
i2cdetect -y 1
```

#### I2C Errors

```bash
# Scan I2C bus
i2cdetect -y 1

# Expected devices:
# 0x3c - OLED display
# 0x3d - OLED display
# 0x44 - SHT3x sensor
# 0x70 - PCA9548A multiplexer
# 0x76/0x77 - BME280/BMP280

# Check permissions
sudo usermod -a -G i2c $USER
```

#### Klipper Bridge Not Connecting

```bash
# Test Moonraker API
curl http://localhost:7125/printer/info

# Check Klipper status
curl http://localhost:7125/printer/objects/query?print_stats

# Verify manual steppers configured
grep "manual_stepper" /home/acp/printer_data/config/printer.cfg
```

#### Audio Issues (voice_node)

```bash
# List audio devices
aplay -l

# Test playback
aplay /usr/share/sounds/alsa/Front_Center.wav

# Check ALSA volume
amixer get PCM

# Set volume
amixer set PCM 90%
```

---

## Creating New Nodes

### Step 1: Create Node File

```bash
cd /home/acp/ros2_ws/src/optimus/optimus
touch my_new_node.py
chmod +x my_new_node.py
```

### Step 2: Write Node Code

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNewNode(Node):
    def __init__(self):
        super().__init__('my_new_node')
        
        # Create publisher
        self.publisher = self.create_publisher(String, '/my/topic', 10)
        
        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            '/sensor/combined',
            self.listener_callback,
            10
        )
        
        # Create timer (runs every 1 second)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('MyNewNode initialized')
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from MyNewNode'
        self.publisher.publish(msg)
        self.get_logger().debug('Published message')
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MyNewNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Register Node in setup.py

Edit `/home/acp/ros2_ws/src/optimus/setup.py`:

```python
entry_points={
    'console_scripts': [
        'hardware_node = optimus.hardware_node:main',
        'voice_node = optimus.voice_node:main',
        'sensor_reader = optimus.sensor_reader:main',
        'klipper_bridge = optimus.klipper_bridge:main',
        'motor_calibration_server = optimus.motor_calibration_server:main',
        'my_new_node = optimus.my_new_node:main',  # ADD THIS LINE
    ],
},
```

### Step 4: Build and Test

```bash
cd /home/acp/ros2_ws
colcon build --packages-select optimus
source install/setup.bash

# Test node
ros2 run optimus my_new_node
```

### Step 5: Add to Launch File (Optional)

Edit `/home/acp/ros2_ws/src/optimus/launch/optimus.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ... existing nodes ...
        
        Node(
            package='optimus',
            executable='my_new_node',
            name='my_new_node',
            output='screen'
        ),
    ])
```

---

## Modifying Existing Nodes

### Step 1: Edit Node File

```bash
# Open node in editor
nano /home/acp/ros2_ws/src/optimus/optimus/hardware_node.py

# Or use VS Code
code /home/acp/ros2_ws/src/optimus/optimus/hardware_node.py
```

### Step 2: Make Changes

Example: Change sensor reading rate in `sensor_reader.py`:

```python
# Find timer creation
self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz

# Change to 2 Hz
self.timer = self.create_timer(0.5, self.timer_callback)  # 2 Hz
```

### Step 3: Rebuild (if necessary)

```bash
cd /home/acp/ros2_ws

# For Python changes, rebuild with symlink (faster)
colcon build --packages-select optimus --symlink-install
source install/setup.bash
```

**Note:** With `--symlink-install`, Python changes don't require rebuild. Just restart the node.

### Step 4: Restart Node

```bash
# If running manually
Ctrl+C  # Stop node
ros2 run optimus sensor_reader  # Restart

# If running as systemd service
sudo systemctl restart optimus
```

---

## System Management

### Running as Systemd Service

The Optimus system can run automatically at boot using systemd.

**Service File:** `/etc/systemd/system/optimus.service`

```ini
[Unit]
Description=Optimus Robot ROS2 System
After=network.target

[Service]
Type=simple
User=acp
WorkingDirectory=/home/acp
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /home/acp/ros2_ws/install/setup.bash && ros2 launch optimus optimus.launch.py"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### Service Management Commands

```bash
# Enable autostart
sudo systemctl enable optimus

# Start service
sudo systemctl start optimus

# Stop service
sudo systemctl stop optimus

# Restart service
sudo systemctl restart optimus

# Check status
sudo systemctl status optimus

# View logs
journalctl -u optimus -f

# Disable autostart
sudo systemctl disable optimus
```

### Manual Launch

```bash
cd /home/acp/ros2_ws
source install/setup.bash

# Launch all nodes
ros2 launch optimus optimus.launch.py

# Launch individual nodes
ros2 run optimus hardware_node
ros2 run optimus voice_node
ros2 run optimus sensor_reader
ros2 run optimus klipper_bridge
ros2 run optimus motor_calibration_server
```

### Stopping Nodes

```bash
# If launched manually
Ctrl+C

# If running as service
sudo systemctl stop optimus

# Kill specific node
ros2 node list
# Find node name, then:
pkill -f "ros2 run optimus hardware_node"
```

---

## Configuration Management

### Main Configuration File

**Location:** `/home/acp/ros2_ws/src/optimus/config.json`

```json
{
  "sensors": [
    {
      "name": "rpi_case",
      "type": "SHT31",
      "address": "0x44",
      "channel": 0
    },
    ...
  ],
  "fans": [
    {
      "name": "fan1",
      "min_speed": 0.3,
      "max_speed": 1.0,
      "sensors": [
        {
          "name": "rpi_case",
          "temp_start": 25,
          "temp_max": 70,
          "weight": 0.4
        },
        ...
      ]
    }
  ]
}
```

### Modifying Configuration

```bash
# Edit config
nano /home/acp/ros2_ws/src/optimus/config.json

# Restart nodes to apply changes
sudo systemctl restart optimus
```

### Motor Calibration File

**Location:** `/home/acp/motor_calibration.json`

Generated by motor_calibration_server. Contains center, min, max positions for each stepper.

---

## Best Practices

### Code Style

- Follow PEP 8 for Python code
- Use descriptive variable names
- Add docstrings to functions
- Use `self.get_logger()` for logging

### Logging Levels

```python
self.get_logger().debug('Detailed diagnostic info')
self.get_logger().info('Normal operation info')
self.get_logger().warning('Warning condition')
self.get_logger().error('Error occurred')
self.get_logger().fatal('Fatal error')
```

### Error Handling

```python
try:
    # Risky operation
    result = some_function()
except Exception as e:
    self.get_logger().error(f'Operation failed: {e}')
    # Handle gracefully
```

### Node Lifecycle

```python
def __init__(self):
    super().__init__('node_name')
    # Initialize resources

def destroy_node(self):
    # Cleanup resources
    super().destroy_node()
```

---

## Performance Monitoring

### CPU Usage

```bash
# Monitor ROS nodes
top

# Filter by process
ps aux | grep ros2
```

### Memory Usage

```bash
# System memory
free -h

# Per-process memory
ps aux --sort=-%mem | head
```

### Network Bandwidth

```bash
# Monitor ROS 2 traffic
ros2 topic bw /sensor/data
```

### I2C Bus Load

```bash
# Monitor I2C bus (requires i2c-tools)
i2cdetect -y 1

# Check bus errors
dmesg | grep i2c
```

---

## Troubleshooting Checklist

### Node Won't Start

- [ ] Check `colcon build` completed without errors
- [ ] Sourced workspace: `source install/setup.bash`
- [ ] Node registered in `setup.py` entry_points
- [ ] Python file has execute permissions
- [ ] No Python import errors

### No Data on Topics

- [ ] Publisher node is running: `ros2 node list`
- [ ] Topic exists: `ros2 topic list`
- [ ] Check topic rate: `ros2 topic hz /topic/name`
- [ ] Verify hardware connections (I2C, sensors)
- [ ] Check node logs for errors

### System Performance Issues

- [ ] Check CPU usage: `top`
- [ ] Check memory: `free -h`
- [ ] Check disk space: `df -h`
- [ ] Monitor I2C errors: `dmesg | grep i2c`
- [ ] Reduce logging verbosity

### Hardware Not Responding

- [ ] I2C devices detected: `i2cdetect -y 1`
- [ ] Klipper connected: `curl http://localhost:7125/printer/info`
- [ ] Check power supply
- [ ] Verify wiring connections
- [ ] Check device permissions

---

## Additional Resources

### ROS 2 Documentation

- [ROS 2 Jazzy Docs](https://docs.ros.org/en/jazzy/)
- [ROS 2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [rclpy API](https://docs.ros2.org/latest/api/rclpy/)

### Hardware Documentation

- [Klipper Documentation](https://www.klipper3d.org/)
- [Moonraker API](https://moonraker.readthedocs.io/)
- [SHT3x Datasheet](https://www.sensirion.com/en/environmental-sensors/humidity-sensors/digital-humidity-sensors-for-various-applications/)
- [BME280 Datasheet](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/)

### Tools

- [VS Code ROS Extension](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)
- [rqt Tools](https://docs.ros.org/en/jazzy/Concepts/About-RQt.html)
- [Plotjuggler](https://github.com/facontidavide/PlotJuggler) (for topic visualization)

---

## Support and Contributions

### Reporting Issues

When reporting issues, include:
1. Node name and version
2. ROS 2 distro (Jazzy)
3. Error messages and logs
4. Steps to reproduce
5. System info: `uname -a`, `ros2 --version`

### Code Contributions

1. Fork repository
2. Create feature branch
3. Make changes
4. Test thoroughly
5. Submit pull request with detailed description

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | Nov 2025 | Initial documentation |

---

## License

[Specify your license here]

---

**For node-specific documentation, see individual README files:**

- [hardware_node](hardware_node/README.md)
- [klipper_bridge](klipper_bridge/README.md)
- [voice_node](voice_node/README.md)
- [sensor_reader](sensor_reader/README.md)
- [motor_calibration_server](motor_calibration_server/README.md)
