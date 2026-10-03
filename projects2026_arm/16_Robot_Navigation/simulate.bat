@echo off
REM ============================================================================
REM  16_Robot_Navigation - build, then simulate
REM
REM  ALWAYS rebuilds first.  The AVR edition shipped a simulate.bat that only
REM  built when Main.hex was missing, so every fix after the first build was
REM  invisible and people debugged firmware they had already replaced.
REM  See CLAUDE.md section 9e.  Do not "optimise" this back.
REM
REM  No Wokwi at all:  host\run.bat runs the same navigation code on the PC.
REM ============================================================================
call "%~dp0build.bat"
if errorlevel 1 exit /b 1

echo.
echo ============================================================
echo  Simulate in the browser - free, no account, no licence
echo ============================================================
echo.
echo   1. A Wokwi tab is opening on the Nucleo-C031C6 board, and
echo      Notepad is opening this folder's diagram.json.
echo   2. In the Wokwi tab open its "diagram.json" tab, select all,
echo      and paste in the Notepad text: the app board - OLED (I2C,
echo      PB8/PB9), joystick (PA0/PA1, SEL PB3), buttons A (PB4) and
echo      B (PB5), knob (PA4), buzzer (PA6).
echo   3. Click into the code editor and press F1.
echo   4. Choose "Upload Firmware and Start Simulation..."
echo   5. Pick  Main.elf  from this folder:
echo.
echo        %CD%\Main.elf
echo.
echo   6. The OLED shows level 1 as the robot knows it: almost nothing.
echo      Press A (key a) to start.  B (key b) is the next level, the
echo      joystick moves the goal flag before the start, the knob sets
echo      the speed, SEL (space) outlines the walls the robot has not
echo      found yet.  Type  help  in the serial monitor.
echo.
start "" notepad "%~dp0diagram.json"
start "" "https://wokwi.com/projects/new/st-nucleo-c031c6"
exit /b 0
