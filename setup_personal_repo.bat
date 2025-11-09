@echo off
REM Setup script for personal GitHub repository
REM UUV Position Stabilization - Personal Account Setup

echo ========================================
echo UUV Position Stabilization - Git Setup
echo Personal Account (ofu951)
echo ========================================
echo.

REM Check if git is installed
git --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Git is not installed!
    echo Please install Git from https://git-scm.com/
    pause
    exit /b 1
)

echo Step 1: Cloning repository...
echo.

REM Change to Documents folder (or specify your preferred location)
cd /d %USERPROFILE%\Documents

REM Clone the repository
git clone https://github.com/ofu951/UUV_Position_Stabilization.git

if errorlevel 1 (
    echo.
    echo ERROR: Clone failed!
    echo.
    echo Possible reasons:
    echo - Repository URL is incorrect
    echo - You need to authenticate (use Personal Access Token)
    echo - Network connection issue
    echo.
    echo For HTTPS authentication, you'll need a Personal Access Token.
    echo Get it from: https://github.com/settings/tokens
    echo.
    pause
    exit /b 1
)

echo.
echo Step 2: Configuring git for this repository...
echo.

cd UUV_Position_Stabilization

REM Set local git config (only for this repository)
echo Please enter your personal name:
set /p PERSONAL_NAME="Name: "
git config user.name "%PERSONAL_NAME%"

echo.
echo Please enter your personal email:
set /p PERSONAL_EMAIL="Email: "
git config user.email "%PERSONAL_EMAIL%"

echo.
echo Step 3: Verifying configuration...
echo.
git config --list --local

echo.
echo ========================================
echo Setup Complete!
echo ========================================
echo.
echo Repository location: %CD%
echo.
echo Next steps:
echo 1. Add your files: git add .
echo 2. Commit: git commit -m "Initial commit"
echo 3. Push: git push origin main
echo.
pause

