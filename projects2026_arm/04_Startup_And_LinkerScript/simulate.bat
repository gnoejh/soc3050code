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
echo   1. A Wokwi tab is opening on the Nucleo-C031C6 board.
echo   2. Click into the editor and press F1.
echo   3. Choose "Upload Firmware and Start Simulation..."
echo   4. Pick  Main.elf  from this folder:
echo.
echo        %CD%\Main.elf
echo.
echo   5. Watch the serial monitor for the memory report, and the
echo      green LED on the board for the blink.
echo.
echo   If you have the Wokwi VS Code extension installed, you can
echo   instead press F1 in VS Code and run "Wokwi: Start Simulator"
echo   from this folder - wokwi.toml and diagram.json are already here.
echo.
start "" "https://wokwi.com/projects/new/st-nucleo-c031c6"
exit /b 0
