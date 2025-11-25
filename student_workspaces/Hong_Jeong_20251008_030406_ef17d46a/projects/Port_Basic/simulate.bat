@echo off
REM SimulIDE Quick Launch for ATmega128 Projects
REM Double-click this file to build and simulate current project

echo.
echo ========================================
echo   SimulIDE Quick Launcher
echo ========================================
echo.

REM Check if in project directory
if not exist "Main.c" (
    echo ERROR: Main.c not found!
    echo.
    echo This script must be run from a project directory.
    echo Example: projects\Port_Basic\
    echo.
    pause
    exit /b 1
)

REM Get current directory
set "PROJECT_DIR=%CD%"
echo Project: %PROJECT_DIR%
echo.

REM Build project
echo [1/3] Building project...
powershell -ExecutionPolicy Bypass -File "..\..\cli-build-project.ps1" -ProjectDir "%PROJECT_DIR%"
if errorlevel 1 (
    echo.
    echo Build failed! Check errors above.
    pause
    exit /b 1
)

REM Check HEX file exists
if not exist "Main.hex" (
    echo.
    echo ERROR: Main.hex not found after build!
    pause
    exit /b 1
)

echo.
echo [2/3] Launching SimulIDE...

REM Launch SimulIDE
powershell -ExecutionPolicy Bypass -File "..\..\cli-simulide.ps1" -ProjectDir "%PROJECT_DIR%" -BuildFirst $false

echo.
echo [3/3] SimulIDE launched!
echo.
echo Tips:
echo   - Click Play button (triangle) to start simulation
echo   - Open Serial Terminal for UART output
echo   - Right-click MCU to reload firmware
echo.

exit /b 0
