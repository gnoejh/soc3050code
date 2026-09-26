@echo off
REM ============================================================================
REM  06_Time_And_Timers - build, then simulate
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
echo      and paste in the Notepad text.  A servo (PA6), an HC-SR04
echo      ranger (TRIG PA8, ECHO PA7) and the LED bar appear.
echo      This lesson needs them.
echo   3. Click into the code editor and press F1.
echo   4. Choose "Upload Firmware and Start Simulation..."
echo   5. Pick  Main.elf  from this folder:
echo.
echo        %CD%\Main.elf
echo.
echo   6. Click the HC-SR04 while it runs: a distance slider appears.
echo      Drag it and watch the servo, the bar and the serial monitor.
echo.
echo   If you have the Wokwi VS Code extension installed, you can
echo   instead press F1 in VS Code and run "Wokwi: Start Simulator"
echo   from this folder - wokwi.toml and diagram.json are read directly.
echo.
start "" notepad "%~dp0diagram.json"
start "" "https://wokwi.com/projects/new/st-nucleo-c031c6"
exit /b 0
