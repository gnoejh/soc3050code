@echo off
REM ============================================================================
REM  10_Fixed_Point_And_DMA - build, then simulate
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
echo      and paste in the Notepad text: the app board - joystick
echo      (PA0/PA1), knob (PA4), buttons A/B (PB4/PB5) and the OLED
echo      (I2C, PB8/PB9).
echo   3. Click into the code editor and press F1.
echo   4. Choose "Upload Firmware and Start Simulation..."
echo   5. Pick  Main.elf  from this folder:
echo.
echo        %CD%\Main.elf
echo.
echo   6. Read the arena's leaderboard in the serial monitor, then
echo      the DMA section: it says whether the DMA moved or whether
echo      it fell back to polling.  Press A (key a) to race again,
echo      B (key b) to switch DMA / polled inputs.
echo.
echo   The same cycle counts, from a Cortex-M0+ model on this PC:
echo        host\run.bat
echo.
start "" notepad "%~dp0diagram.json"
start "" "https://wokwi.com/projects/new/st-nucleo-c031c6"
exit /b 0
