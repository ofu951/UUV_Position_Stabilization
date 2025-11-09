# UUV Position Stabilization Control System

Modular robot control system with Pixhawk integration. Provides QR code detection and 4-axis stabilization.

## Table of Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [Features](#features)
- [Configuration](#configuration)
- [Usage](#usage)
- [Testing Without Pixhawk](#testing-without-pixhawk)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [Documentation](#documentation)
- [License](#license)

## Installation

### Prerequisites

- Python 3.7 or higher
- Git
- Camera (for image processing)
- Pixhawk (optional - simulation mode available)

### Step 1: Clone the Repository

#### Option A: Using Setup Scripts (Recommended)

**Windows:**
```bash
# Navigate to your preferred directory
cd C:\Users\YourUsername\Documents

# Run the setup script
setup_personal_repo.bat
```

**Linux/Mac:**
```bash
# Navigate to your preferred directory
cd ~/Documents

# Make script executable and run
chmod +x setup_personal_repo.sh
./setup_personal_repo.sh
```

The setup scripts will:
- Clone the repository
- Configure Git settings for this repository (you'll be prompted for your name and email)

#### Option B: Manual Clone

**Using HTTPS:**
```bash
# Navigate to where you want to clone
cd C:\Users\YourUsername\Documents  # Windows
# or
cd ~/Documents  # Linux/Mac

# Clone the repository
git clone https://github.com/ofu951/UUV_Position_Stabilization.git

# Navigate into the repository
cd UUV_Position_Stabilization

# Configure Git for this repository (local config only)
git config user.name "Your Name"
git config user.email "your-email@example.com"
```

**Using SSH (if configured):**
```bash
git clone git@github.com-personal:ofu951/UUV_Position_Stabilization.git
cd UUV_Position_Stabilization
git config user.name "Your Name"
git config user.email "your-email@example.com"
```

> **Note:** For detailed Git setup instructions (multiple GitHub accounts, SSH keys, etc.), see [GIT_SETUP_GUIDE.md](GIT_SETUP_GUIDE.md)

### Step 2: Install Dependencies

```bash
# Navigate to the project directory
cd UUV_Position_Stabilization

# Install all required packages
pip install -r requirements.txt
```

Or install manually:
```bash
pip install pymavlink opencv-python numpy
```

**Required packages:**
- `pymavlink>=2.4.0` - Pixhawk communication
- `opencv-python>=4.5.0` - Image processing and ArUco marker detection
- `numpy>=1.19.0` - Numerical operations

### Step 3: Verify Installation

```bash
# Test Python imports
python -c "import cv2, numpy, pymavlink; print('All packages installed successfully!')"
```

## Quick Start

### With Pixhawk (Real Hardware)

```bash
python run_uuv_control.py
```

or

```bash
python -m uuv_control.main
```

### Without Pixhawk (Simulation Mode - For Testing)

If you don't have a Pixhawk available, you can test the system in simulation mode:

```bash
python run_uuv_control_sim.py
```

**Simulation Mode Features:**
- All Pixhawk commands are commented out (original code preserved)
- Pixhawk connection and arm operations are simulated with print statements
- PWM values are printed to console every second instead of being sent to Pixhawk
- All control functions work normally (image processing, PID controllers, etc.)
- Perfect for testing and debugging without hardware

**Simulation Mode Output:**
- Console prints show what PWM values would be sent to Pixhawk
- All channels and their directions are displayed
- Safe shutdown is simulated
- Log file: `uuv_control_sim_YYYYMMDD_HHMMSS.log`

For more details, see [QUICK_START.md](QUICK_START.md)

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

### Camera Settings

Camera index and resolution can be adjusted in the `main()` function:

```python
control_system = UUVControlSystem(
    connection_string='udp:127.0.0.1:14551',
    camera_index=0,        # Camera index (0, 1, 2, ...)
    frame_width=640,       # Image width
    frame_height=480       # Image height
)
```

### PID Coefficients

PID coefficients for each controller can be adjusted in the respective module file:

- **Forward**: `uuv_control/forward_controller.py`
- **Yaw**: `uuv_control/yaw_controller.py`
- **Lateral**: `uuv_control/lateral_controller.py`
- **Throttle**: `uuv_control/throttle_controller.py`

### PID Coefficients (Default)

| Controller | Kp | Ki | Kd | Deadband |
|-----------|----|----|----|----------|
| Forward | 0.02 | 0.0005 | 0.01 | 200px |
| Yaw | 5.0 | 0.025 | 1.0 | 2px |
| Lateral | 2.0 | 0.02 | 0.4 | 15px |
| Throttle | 2.0 | 0.02 | 0.4 | 15px |

## Usage

### Basic Usage Example

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

### Modular Structure

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

## Testing Without Pixhawk

The system can be tested without Pixhawk hardware using simulation mode:

```bash
python run_uuv_control_sim.py
```

**What works in simulation mode:**
- Camera and image processing
- ArUco marker detection
- All PID controllers (Forward, Yaw, Lateral, Throttle)
- PWM value calculations
- Visualization and control display

**What is simulated:**
- Pixhawk connection (prints success message)
- Pixhawk arm operation (prints success message)
- PWM signal transmission (prints values to console every second)
- Pixhawk disarm and disconnect (prints messages)

**Console output example:**
```
[SIM] ========================================
[SIM] PIXHAWK CONNECTION SIMULATION
[SIM] ========================================
[SIM] Pixhawk connection: SUCCESS (SIMULATED)
[SIM] Pixhawk arm: SUCCESS (SIMULATED)

============================================================
[SIM] PWM VALUES (Would be sent to Pixhawk):
[SIM]   Ch3 (Throttle): 1500 (CENTER)
[SIM]   Ch4 (Yaw):      1500 (STRAIGHT)
[SIM]   Ch5 (Forward):  1500 (NEUTRAL)
[SIM]   Ch6 (Lateral):  1500 (CENTER)
============================================================
```

This allows you to:
- Test image processing and marker detection
- Verify PID controller behavior
- Debug control logic
- Develop and test without hardware

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
├── examples/                # Example scripts
│   ├── yaw90.py            # Original yaw control example
│   ├── yaw.py              # Original yaw image processing
│   ├── fwd_bwd.py          # Original forward/backward control
│   └── center.py           # Original center control
├── run_uuv_control.py      # Easy execution script (with Pixhawk)
├── run_uuv_control_sim.py  # Simulation mode (no Pixhawk required)
├── setup_personal_repo.bat # Windows setup script
├── setup_personal_repo.sh  # Linux/Mac setup script
├── GIT_SETUP_GUIDE.md     # Git configuration guide
├── QUICK_START.md         # Quick start guide
├── requirements.txt        # Python dependencies
└── README.md              # This file
```

## Logging

The system automatically creates log files:
- **With Pixhawk**: `uuv_control_YYYYMMDD_HHMMSS.log`
- **Simulation Mode**: `uuv_control_sim_YYYYMMDD_HHMMSS.log`
- **Location**: Execution directory
- **Content**: All control operations and errors

## Safety

- When the system shuts down, it automatically:
  - Resets RC override
  - Disarms Pixhawk
  - Closes connection
- When marker is not detected, all channels return to neutral position (1500)
- In simulation mode, shutdown operations are simulated with print statements

## Contributing

### Repository Access

**Important:** This repository is private and access is controlled by the repository owner. 

- **To contribute:** You must be invited as a collaborator by the repository owner
- **To request access:** Contact the repository owner to receive a collaborator invitation
- **After receiving invitation:** Accept the invitation via the email link or GitHub notification
- **File editing permissions:** Only invited collaborators with write access can push changes to the repository

### Git Configuration

If you're working with multiple GitHub accounts, please refer to [GIT_SETUP_GUIDE.md](GIT_SETUP_GUIDE.md) for proper configuration.

**Important notes:**
- Always use **local Git config** (not global) for this repository
- Set your name and email using: `git config user.name "Your Name"` and `git config user.email "your-email@example.com"`
- Never commit sensitive information (tokens, passwords, API keys)

## Documentation

- **Quick Start Guide**: See [QUICK_START.md](QUICK_START.md) for detailed quick start instructions
- **Git Setup Guide**: See [GIT_SETUP_GUIDE.md](GIT_SETUP_GUIDE.md) for Git configuration with multiple GitHub accounts
- **Detailed Module Documentation**: See `uuv_control/README.md` for detailed module documentation

## Original Codes

Files in the `examples/` directory are original test codes:
- `yaw90.py`: Pixhawk yaw control example
- `yaw.py`: Yaw image processing
- `fwd_bwd.py`: Forward/backward control
- `center.py`: Center control

## License

This project is for educational and research purposes.
