# UUV Position Stabilization Control System

Modular robot control system - Pixhawk integrated QR code tracking and stabilization system.

## Folder Structure

```
uuv_control/
├── __init__.py                 # Package initialization
├── pixhawk_connection.py       # Pixhawk connection and basic commands
├── pid_controller.py           # PID controller class
├── image_processor.py          # Image processing and QR code detection
├── forward_controller.py       # Forward/Backward control (Channel 5)
├── yaw_controller.py           # Yaw control (Channel 4)
├── lateral_controller.py       # Lateral control (Channel 6)
├── throttle_controller.py      # Throttle control (Channel 3)
├── main.py                     # Main control script
└── README.md                   # This file
```

## Features

### 1. Pixhawk Connection and Control
- Connect to Pixhawk (UDP, TCP, Serial)
- ARM/DISARM commands
- RC channel override support

### 2. Image Processing
- ArUco marker detection
- Marker information extraction (area, edge lengths, center)

### 3. Control Modules

#### Forward Controller (Channel 5 - X Axis)
- **Target**: Bring QR code area to 20000px
- **Logic**: 
  - Area < 20000px → Move forward (approach)
  - Area > 20000px → Move backward (retreat)
- **PID Coefficients**: Kp=0.02, Ki=0.0005, Kd=0.01, Deadband=200px

#### Yaw Controller (Channel 4)
- **Target**: Equalize left and right edge lengths
- **Logic**: 
  - Right edge > Left edge → Turn right
  - Left edge > Right edge → Turn left
- **PID Coefficients**: Kp=5.0, Ki=0.025, Kd=1.0, Deadband=2px

#### Lateral Controller (Channel 6 - Y Axis)
- **Target**: Center marker on screen X axis
- **Logic**: 
  - Marker on right → Move right
  - Marker on left → Move left
- **PID Coefficients**: Kp=2.0, Ki=0.02, Kd=0.4, Deadband=15px

#### Throttle Controller (Channel 3 - Z Axis)
- **Target**: Center marker on screen Y axis
- **Logic**: 
  - Marker above → Move up
  - Marker below → Move down
- **PID Coefficients**: Kp=2.0, Ki=0.02, Kd=0.4, Deadband=15px

## Usage

### Basic Usage

```python
from uuv_control.main import UUVControlSystem

# Initialize control system
control_system = UUVControlSystem(
    connection_string='udp:127.0.0.1:14551',  # Pixhawk connection
    camera_index=0,                            # Camera index
    frame_width=640,                          # Image width
    frame_height=480                           # Image height
)

# Run
control_system.run()
```

### Command Line Execution

#### With Pixhawk

```bash
python -m uuv_control.main
```

or

```bash
python run_uuv_control.py
```

#### Without Pixhawk (Simulation Mode)

For testing without Pixhawk hardware:

```bash
python run_uuv_control_sim.py
```

**Simulation Mode:**
- Pixhawk commands are commented out (original code preserved)
- All Pixhawk operations are simulated with print statements
- PWM values are printed to console instead of being sent
- Perfect for testing image processing and control logic without hardware
- Log file: `uuv_control_sim_YYYYMMDD_HHMMSS.log`

### Connection String Examples

```python
# SITL Simulation (UDP)
connection_string = 'udp:127.0.0.1:14551'

# TCP Connection
connection_string = 'tcp:192.168.1.100:5760'

# USB Serial Connection (Linux)
connection_string = '/dev/ttyUSB0'

# USB Serial Connection (Windows)
connection_string = 'COM3'
```

## Channel Assignments

| Channel | Name | Axis | Description |
|-------|------|-------|----------|
| Ch3 | Throttle | Z | Up/Down (Dive/Surface) |
| Ch4 | Yaw | - | Left/Right Rotation |
| Ch5 | Forward | X | Forward/Backward |
| Ch6 | Lateral | Y | Right/Left |

## PID Coefficients

### Forward Controller
- **Kp**: 0.02
- **Ki**: 0.0005
- **Kd**: 0.01
- **Deadband**: 200px

### Yaw Controller
- **Kp**: 5.0
- **Ki**: 0.025
- **Kd**: 1.0
- **Deadband**: 2px

### Lateral Controller
- **Kp**: 2.0
- **Ki**: 0.02
- **Kd**: 0.4
- **Deadband**: 15px

### Throttle Controller
- **Kp**: 2.0
- **Ki**: 0.02
- **Kd**: 0.4
- **Deadband**: 15px

## Requirements

```txt
pymavlink
opencv-python
numpy
```

## Installation

```bash
pip install pymavlink opencv-python numpy
```

## Logging

The system automatically creates log files:
- File name: `uuv_control_YYYYMMDD_HHMMSS.log`
- Location: Execution directory
- Format: Timestamp, log level, message

## Safety

- When the system shuts down, it automatically:
  - Resets RC override
  - Disarms Pixhawk
  - Closes connection

## Notes

- PWM values: Range 1100-1900 (1500 = neutral)
- When marker is not detected, all channels return to neutral position (1500)
- Within deadband, PID output is reset and integral is reset
