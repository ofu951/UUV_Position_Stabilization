#!/bin/bash
# Setup script for personal GitHub repository
# UUV Position Stabilization - Personal Account Setup

echo "========================================"
echo "UUV Position Stabilization - Git Setup"
echo "Personal Account (ofu951)"
echo "========================================"
echo ""

# Check if git is installed
if ! command -v git &> /dev/null; then
    echo "ERROR: Git is not installed!"
    echo "Please install Git from https://git-scm.com/"
    exit 1
fi

echo "Step 1: Cloning repository..."
echo ""

# Change to home directory (or specify your preferred location)
cd ~/Documents 2>/dev/null || cd ~

# Clone the repository
git clone https://github.com/ofu951/UUV_Position_Stabilization.git

if [ $? -ne 0 ]; then
    echo ""
    echo "ERROR: Clone failed!"
    echo ""
    echo "Possible reasons:"
    echo "- Repository URL is incorrect"
    echo "- You need to authenticate (use Personal Access Token)"
    echo "- Network connection issue"
    echo ""
    echo "For HTTPS authentication, you'll need a Personal Access Token."
    echo "Get it from: https://github.com/settings/tokens"
    echo ""
    exit 1
fi

echo ""
echo "Step 2: Configuring git for this repository..."
echo ""

cd UUV_Position_Stabilization

# Set local git config (only for this repository)
read -p "Please enter your personal name: " PERSONAL_NAME
git config user.name "$PERSONAL_NAME"

echo ""
read -p "Please enter your personal email: " PERSONAL_EMAIL
git config user.email "$PERSONAL_EMAIL"

echo ""
echo "Step 3: Verifying configuration..."
echo ""
git config --list --local

echo ""
echo "========================================"
echo "Setup Complete!"
echo "========================================"
echo ""
echo "Repository location: $(pwd)"
echo ""
echo "Next steps:"
echo "1. Add your files: git add ."
echo "2. Commit: git commit -m 'Initial commit'"
echo "3. Push: git push origin main"
echo ""

