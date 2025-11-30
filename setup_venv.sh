#!/bin/bash
# Virtual Environment Setup Script for UUV Position Stabilization

set -e

echo "=========================================="
echo "UUV Position Stabilization - Setup"
echo "=========================================="
echo ""

# Check if virtual environment already exists
if [ -d "venv" ]; then
    echo "Virtual environment already exists."
    read -p "Do you want to recreate it? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Removing existing virtual environment..."
        rm -rf venv
    else
        echo "Using existing virtual environment."
        echo ""
        echo "To activate the virtual environment, run:"
        echo "  source venv/bin/activate"
        echo ""
        echo "To install/update packages, run:"
        echo "  pip install -r requirements.txt"
        exit 0
    fi
fi

# Check if python3-venv is installed
if ! python3 -m venv --help > /dev/null 2>&1; then
    echo "ERROR: python3-venv is not installed."
    echo "Please install it with:"
    echo "  sudo apt install python3-venv python3-full"
    exit 1
fi

# Create virtual environment
echo "Creating virtual environment..."
python3 -m venv venv

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install requirements
echo "Installing required packages..."
pip install -r requirements.txt

echo ""
echo "=========================================="
echo "Setup completed successfully!"
echo "=========================================="
echo ""
echo "To use the virtual environment:"
echo "  1. Activate it: source venv/bin/activate"
echo "  2. Run the program: python3 run_uuv_control.py"
echo "  3. Deactivate when done: deactivate"
echo ""
echo "Or use the run script directly:"
echo "  ./run_with_venv.sh"
echo ""

