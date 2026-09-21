@echo off
REM ============================================================================
REM  04_Startup_And_LinkerScript - build, then simulate
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
echo      and paste in the Notepad text.  Eight LEDs appear on PB0-PB7.
echo      (Skip this and the program still runs - just no LED bar.)
echo   3. Click into the code editor and press F1.
echo   4. Choose "Upload Firmware and Start Simulation..."
echo   5. Pick  Main.elf  from this folder:
echo.
echo        %CD%\Main.elf
echo.
echo   6. Watch the serial monitor for the memory report, the LED bar
echo      for your patterns, and LD4 for the per-frame heartbeat.
echo.
echo   If you have the Wokwi VS Code extension installed, you can
echo   instead press F1 in VS Code and run "Wokwi: Start Simulator"
echo   from this folder - wokwi.toml and diagram.json are read directly.
echo.
start "" notepad "%~dp0diagram.json"
start "" "https://wokwi.com/projects/new/st-nucleo-c031c6"
exit /b 0
