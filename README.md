# UUV Position Stabilization Control System

UUV system with Pixhawk integration. Provides QR code detection and 4-axis stabilization.

## Project Structure

```
UUV_Position_Stabilization/
├── uuv_control/              # Main control modules
│   ├── __init__.py
│   ├── pixhawk_connection.py # Pixhawk connection and commands
│   ├── pid_controller.py    # PID controller class
│   ├── image_processor.py   # Image processing and QR code detection
│   ├── forward_controller.py # Forward/Backward control (Ch5)
│   ├── yaw_controller.py    # Yaw control (Ch4)
│   ├── lateral_controller.py # Lateral control (Ch6)
│   ├── throttle_controller.py # Throttle control (Ch3)
│   ├── main.py              # Main control script
│   └── README.md           # Detailed documentation
├── run_uuv_control.py      # Easy execution script
├── examples/
│   └── yaw90.py                # Original yaw control example
│   └── yaw.py                  # Original yaw image processing
│   └──fwd_bwd.py              # Original forward/backward control
│   └──center.py               # Original center control
└── README.md               # This file
```

## Quick Start

### 1. Install Requirements

```bash
pip install pymavlink opencv-python numpy
```

### 2. Run the System

```bash
python run_uuv_control.py
```

or

```bash
python -m uuv_control.main
```

## Features

### Control Axes

1. **Forward/Backward (Channel 5 - X Axis)**
   - QR code area < 20000px → Move forward (approach)
   - QR code area > 20000px → Move backward (retreat)

2. **Yaw (Channel 4)**
   - Right edge > Left edge → Turn right
   - Left edge > Right edge → Turn left

3. **Lateral (Channel 6 - Y Axis)**
   - Marker on right → Move right
   - Marker on left → Move left

4. **Throttle (Channel 3 - Z Axis)**
   - Marker above → Move up
   - Marker below → Move down

### Pixhawk Features

- Connection management (UDP, TCP, Serial)
- ARM/DISARM commands
- RC channel override
- Automatic safety shutdown

## Configuration

### Pixhawk Connection

Change the `connection_string` in `uuv_control/main.py`:

```python
# SITL Simulation
connection_string = 'udp:127.0.0.1:14551'

# TCP Connection
connection_string = 'tcp:192.168.1.100:5760'

# USB Serial (Linux)
connection_string = '/dev/ttyUSB0'

# USB Serial (Windows)
connection_string = 'COM3'
```

### PID Coefficients

PID coefficients for each controller can be adjusted in the respective module file:

- **Forward**: `uuv_control/forward_controller.py`
- **Yaw**: `uuv_control/yaw_controller.py`
- **Lateral**: `uuv_control/lateral_controller.py`
- **Throttle**: `uuv_control/throttle_controller.py`

## PID Coefficients (Default)

| Controller | Kp | Ki | Kd | Deadband |
|-----------|----|----|----|----------|
| Forward | 0.02 | 0.0005 | 0.01 | 200px |
| Yaw | 5.0 | 0.025 | 1.0 | 2px |
| Lateral | 2.0 | 0.02 | 0.4 | 15px |
| Throttle | 2.0 | 0.02 | 0.4 | 15px |

## Usage Example

```python
from uuv_control.main import UUVControlSystem

# Initialize control system
control_system = UUVControlSystem(
    connection_string='udp:127.0.0.1:14551',
    camera_index=0,
    frame_width=640,
    frame_height=480
)

# Run
control_system.run()
```

## Modular Structure

The system is designed modularly. Each controller can be used independently:

```python
from uuv_control.forward_controller import ForwardController
from uuv_control.yaw_controller import YawController

forward_ctrl = ForwardController(target_area=20000)
yaw_ctrl = YawController()

# Control with marker info
marker_info = [...]  # Info from image_processor
forward_pwm = forward_ctrl.calculate_control(marker_info)
yaw_pwm = yaw_ctrl.calculate_control(marker_info)
```

## Logging

The system automatically creates log files:
- **File**: `uuv_control_YYYYMMDD_HHMMSS.log`
- **Location**: Execution directory
- **Content**: All control operations and errors

## Safety

- When the system shuts down, it automatically:
  - Resets RC override
  - Disarms Pixhawk
  - Closes connection
- When marker is not detected, all channels return to neutral position (1500)

## Detailed Documentation

For detailed documentation, see: `uuv_control/README.md`

## Original Codes

Files in the project root directory are original test codes:
- `yaw90.py`: Pixhawk yaw control example
- `yaw.py`: Yaw image processing
- `fwd_bwd.py`: Forward/backward control
- `center.py`: Center control

## License

This project is for educational and research purposes.


