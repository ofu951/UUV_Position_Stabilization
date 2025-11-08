# Quick Start Guide

## 1. Install Requirements

```bash
pip install -r requirements.txt
```

or manually:

```bash
pip install pymavlink opencv-python numpy
```

## 2. Run the Code

### Method 1: Easy Execution Script (Recommended)

```bash
python run_uuv_control.py
```

### Method 2: Run as Module

```bash
python -m uuv_control.main
```

## 3. Configuration

### Pixhawk Connection

Change the `connection_string` value in `uuv_control/main.py`:

```python
# Around line 24
connection_string = 'udp:127.0.0.1:14551'  # For SITL
# or
connection_string = 'tcp:192.168.1.100:5760'  # For TCP
# or
connection_string = 'COM3'  # For Windows USB
# or
connection_string = '/dev/ttyUSB0'  # For Linux USB
```

### Camera Settings

Camera index and resolution settings can be changed in the `main()` function:

```python
control_system = UUVControlSystem(
    connection_string='udp:127.0.0.1:14551',
    camera_index=0,        # Camera index (0, 1, 2, ...)
    frame_width=640,       # Image width
    frame_height=480      # Image height
)
```

## 4. Pre-Run Checks

- Is Pixhawk connection ready?
- Is camera working?
- Are required Python packages installed?
- Are ArUco markers ready?

## 5. Execution

1. Start Pixhawk (or SITL simulation)
2. Connect camera
3. Run the script:
   ```bash
   python run_uuv_control.py
   ```

## 6. Exit

- Press 'q' key to exit
- Or stop with Ctrl+C
- System will automatically shut down safely

## Troubleshooting

### Import Error

If you get an import error:

```bash
# Run as module
python -m uuv_control.main
```

### Camera Not Found

Change camera index or ensure camera is connected.

### Pixhawk Connection Error

- Check connection string
- Ensure Pixhawk is running
- Check firewall settings (for UDP/TCP)

## Notes

- Log file is created on first run: `uuv_control_YYYYMMDD_HHMMSS.log`
- Robot stays in neutral position when marker is not detected
- All channels are at neutral position (1500 PWM value)
