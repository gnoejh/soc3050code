@echo off
REM Quick launcher for ADC_Basic Python project
echo ====================================================================
echo ADC Basic - Real-time Data Visualization
echo SOC 3050 - Embedded Systems and Applications
echo ====================================================================
echo.

python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo Error: Python not found!
    pause
    exit /b 1
)

if not exist venv (
    echo Creating virtual environment...
    python -m venv venv
)

call venv\Scripts\activate.bat

if exist requirements.txt (
    echo Installing requirements...
    pip install -q -r requirements.txt
)

echo Starting ADC Basic Python Interface...
echo.
python main.py

deactivate
pause

