@echo off
REM Quick launcher for Serial_Communications Python project
REM © 2025 Prof. Hong Jeong, IUT

echo ====================================================================
echo Serial Communications - Python Interface
echo SOC 3050 - Embedded Systems and Applications
echo ====================================================================
echo.

REM Check if Python is available
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo Error: Python not found!
    echo Please install Python 3.8 or later
    pause
    exit /b 1
)

REM Check if virtual environment exists
if not exist venv (
    echo Creating virtual environment...
    python -m venv venv
    echo.
)

REM Activate virtual environment
echo Activating virtual environment...
call venv\Scripts\activate.bat

REM Install requirements
if exist requirements.txt (
    echo Installing requirements...
    pip install -q -r requirements.txt
    echo.
)

REM Run main program
echo Starting Serial Communications Python Interface...
echo.
python main.py

REM Deactivate virtual environment
deactivate

echo.
pause

