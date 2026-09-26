@echo off
REM ============================================================================
REM  05_GPIO_And_Interrupts - build, then simulate
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
echo      and paste in the Notepad text.  The LED bar appears on
echo      PB0-PB7 and two buttons on PA0 (A) and PA1 (B).
echo      This lesson needs them - without the diagram there is
echo      nothing to press.
echo   3. Click into the code editor and press F1.
echo   4. Choose "Upload Firmware and Start Simulation..."
echo   5. Pick  Main.elf  from this folder:
echo.
echo        %CD%\Main.elf
echo.
echo   6. Read the banner on the serial monitor, then press the
echo      buttons - click them, or click the diagram once and hold
echo      the A or B key.  Every press prints what each method saw.
echo.
echo   If you have the Wokwi VS Code extension installed, you can
echo   instead press F1 in VS Code and run "Wokwi: Start Simulator"
echo   from this folder - wokwi.toml and diagram.json are read directly.
echo.
start "" notepad "%~dp0diagram.json"
start "" "https://wokwi.com/projects/new/st-nucleo-c031c6"
exit /b 0
