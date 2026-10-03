@echo off
REM ============================================================================
REM  15_Robot_Line_Follower - build, then simulate
REM
REM  ALWAYS rebuilds first.  The AVR edition shipped a simulate.bat that only
REM  built when Main.hex was missing, so every fix after the first build was
REM  invisible and people debugged firmware they had already replaced.
REM  See CLAUDE.md section 9e.  Do not "optimise" this back.
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
echo      B (PB5), knob (PA4), buzzer (PA6).  This lesson needs all.
echo   3. Click into the code editor and press F1.
echo   4. Choose "Upload Firmware and Start Simulation..."
echo   5. Pick  Main.elf  from this folder:
echo.
echo        %CD%\Main.elf
echo.
echo   6. Joystick down/up picks a track, A starts it, A again stops.
echo      B = next track, SEL (space) = manual/auto, knob = speed.
echo      In the serial monitor type  gains  or  stats.
echo.
echo   The same races, measured, with no simulator:  host\run.bat
echo.
start "" notepad "%~dp0diagram.json"
start "" "https://wokwi.com/projects/new/st-nucleo-c031c6"
exit /b 0
