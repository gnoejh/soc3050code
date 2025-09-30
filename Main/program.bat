@echo off
REM Programming script for ATmega128 using AVRDUDE
echo [PROGRAM] Programming ATmega128 with AVRDUDE...

if not exist "Main.hex" (
    echo [ERROR] Main.hex not found! Run build.bat first.
    pause
    exit /b 1
)

REM Set AVRDUDE path and default settings
set AVRDUDE="C:\Program Files (x86)\AVRDUDESS\avrdude.exe"
set PROGRAMMER=arduino
set MCU=m128
set COMPORT=COM3
set HEXFILE=Main.hex

echo [INFO] HEX file: %HEXFILE%
echo [INFO] Programmer: %PROGRAMMER%
echo [INFO] MCU: %MCU%
echo [INFO] COM Port: %COMPORT%
echo.

REM Check if AVRDUDE exists
if not exist %AVRDUDE% (
    echo [ERROR] AVRDUDE not found at: %AVRDUDE%
    echo [INFO] Please install AVRDUDESS or update the path in this script.
    pause
    exit /b 1
)

echo [PROGRAM] Programming ATmega128...
%AVRDUDE% -c %PROGRAMMER% -p %MCU% -P %COMPORT% -U flash:w:"%HEXFILE%":a

if %ERRORLEVEL% == 0 (
    echo.
    echo [SUCCESS] Programming completed successfully!
    echo [INFO] ATmega128 is now running: %HEXFILE%
) else (
    echo.
    echo [ERROR] Programming failed!
    echo [HELP] Check:
    echo   - COM port is correct (%COMPORT%)
    echo   - Programmer is connected
    echo   - ATmega128 is powered
    echo   - Correct programmer type (%PROGRAMMER%)
)

echo.
pause