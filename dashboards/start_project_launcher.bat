@echo off
REM Quick Start Script for Project Launcher Dashboard
REM © 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)

echo ========================================
echo  ATmega128 Project Launcher Dashboard
echo  Inha University in Tashkent
echo ========================================
echo.

REM Check Python installation
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python is not installed or not in PATH!
    echo Please install Python 3.8+ from python.org
    pause
    exit /b 1
)

echo [OK] Python found
echo.

REM Check required packages
echo Checking required packages...
python -c "import flask" >nul 2>&1
if errorlevel 1 (
    echo [INSTALLING] Flask not found, installing...
    pip install flask flask-socketio pyserial
) else (
    echo [OK] Flask installed
)

python -c "import flask_socketio" >nul 2>&1
if errorlevel 1 (
    echo [INSTALLING] Flask-SocketIO not found, installing...
    pip install flask-socketio
) else (
    echo [OK] Flask-SocketIO installed
)

python -c "import serial" >nul 2>&1
if errorlevel 1 (
    echo [INSTALLING] PySerial not found, installing...
    pip install pyserial
) else (
    echo [OK] PySerial installed
)

echo.
echo ========================================
echo Starting Project Launcher Dashboard...
echo ========================================
echo.
echo Dashboard will open on: http://localhost:5001
echo Press Ctrl+C to stop the server
echo.

REM Start the dashboard
cd /d "%~dp0"
python project_launcher_dashboard.py

pause
