@echo off
REM Quick Setup Script for ATmega128 Educational Framework
REM Run this script as Administrator for best results

echo ========================================
echo ATmega128 Educational Setup
echo ========================================
echo.

REM Check if Python is installed
echo [1/4] Checking Python installation...
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python from https://python.org/downloads/
    echo Make sure to check "Add Python to PATH" during installation
    pause
    exit /b 1
) else (
    python --version
    echo Python is installed correctly
)
echo.

REM Check if pip is available
echo [2/4] Checking pip installation...
pip --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: pip is not available
    echo Please reinstall Python with pip included
    pause
    exit /b 1
) else (
    pip --version
    echo pip is available
)
echo.

REM Install Python requirements
echo [3/4] Installing Python packages...
echo This may take a few minutes...
pip install -r requirements.txt --user
if %errorlevel% neq 0 (
    echo ERROR: Failed to install Python packages
    echo Please check your internet connection and try again
    pause
    exit /b 1
) else (
    echo Python packages installed successfully
)
echo.

REM Check if Microchip Studio tools are available
echo [4/4] Checking Microchip Studio installation...
set "STUDIO_PATH=C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin"
if exist "%STUDIO_PATH%\avr-gcc.exe" (
    echo Microchip Studio tools found
    "%STUDIO_PATH%\avr-gcc.exe" --version | findstr "avr-gcc"
) else (
    echo WARNING: Microchip Studio not found at expected location
    echo Please install Microchip Studio from:
    echo https://www.microchip.com/en-us/tools-resources/develop/microchip-studio
    echo.
    echo Current search path: %STUDIO_PATH%
)
echo.

REM Test VS Code availability
echo Checking VS Code installation...
code --version >nul 2>&1
if %errorlevel% neq 0 (
    echo WARNING: VS Code not found in PATH
    echo Please install VS Code from https://code.visualstudio.com/
    echo Make sure to check "Add to PATH" during installation
) else (
    echo VS Code is available
    code --version | findstr -v "^$"
)
echo.

echo ========================================
echo Setup Summary:
echo ========================================
echo Python:          OK
echo pip:             OK  
echo Python packages: OK
if exist "%STUDIO_PATH%\avr-gcc.exe" (
    echo Microchip Studio: OK
) else (
    echo Microchip Studio: MISSING - Please install
)

code --version >nul 2>&1
if %errorlevel% neq 0 (
    echo VS Code:         MISSING - Please install
) else (
    echo VS Code:         OK
)
echo.

echo ========================================
echo Next Steps:
echo ========================================
echo 1. Install missing software (if any shown above)
echo 2. Download AVRDUDESS from: https://github.com/ZakKemble/AVRDUDESS/releases
echo 3. Connect your ATmega128 development board
echo 4. Open VS Code in this folder: code .
echo 5. Start with project: 01_Port
echo.
echo For detailed setup instructions, see: STUDENT_SETUP_GUIDE.md
echo.

pause