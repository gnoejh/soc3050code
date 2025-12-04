@echo off
title ATmega128 Classroom Dashboard
cd /d "%~dp0"

echo ========================================
echo  ATmega128 CLASSROOM DASHBOARD
echo ========================================
echo.
echo Starting multi-user web interface...
echo Students can access via web browser
echo.

REM Check if Python is available
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Python not found!
    echo Please install Python 3.8+ from https://python.org
    pause
    exit /b 1
)

REM Install required packages if missing
echo Checking Python packages...
pip show flask >nul 2>&1
if %errorlevel% neq 0 (
    echo Installing Flask...
    pip install flask flask-socketio eventlet
)

REM Get network IP for student access
echo.
echo Detecting network configuration...
for /f "tokens=2 delims=:" %%a in ('ipconfig ^| findstr /i "IPv4"') do (
    set "ip=%%a"
    set "ip=!ip: =!"
    if not "!ip!"=="127.0.0.1" (
        echo Student Access URL: http://!ip!:5001
        goto :start_server
    )
)

:start_server
echo.
echo ========================================
echo  DASHBOARD STARTING...
echo ========================================
echo.
echo ^> Student Login: http://localhost:5001
echo ^> Teacher Panel: http://localhost:5001/teacher
echo.
echo Press Ctrl+C to stop the server
echo ========================================
echo.

REM Start the dashboard
python dashboards\multi_user_dashboard.py

echo.
echo Dashboard stopped.
pause