"""
UUV Control System - Execution Script
This script runs the main control system
"""

import sys
import os

# Add project root directory to path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

# Import uuv_control module
try:
    from uuv_control.main import main
except ImportError as e:
    print(f"Import error: {e}")
    print("\nPlease run the following command:")
    print("python -m uuv_control.main")
    sys.exit(1)

if __name__ == "__main__":
    main()
