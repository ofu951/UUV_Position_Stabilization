#!/bin/bash
# Run script that automatically uses virtual environment

set -e

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "Virtual environment not found!"
    echo "Please run setup first:"
    echo "  ./setup_venv.sh"
    exit 1
fi

# Activate virtual environment and run
source venv/bin/activate
python3 run_uuv_control.py "$@"

