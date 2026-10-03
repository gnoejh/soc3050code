@echo off
REM ============================================================================
REM  14_Drone_Missions - build, then simulate
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
echo      and paste in the Notepad text: the app board - OLED
echo      (I2C, PB8/PB9), joystick (PA0/PA1/PB3), buttons A (PB4)
echo      and B (PB5), knob (PA4), buzzer (PA6).
echo   3. Click into the code editor and press F1.
echo   4. Choose "Upload Firmware and Start Simulation..."
echo   5. Pick  Main.elf  from this folder:
echo.
echo        %CD%\Main.elf
echo.
echo   6. Wait one second (GPS lock), click the diagram, press  a :
echo      the drone takes off and flies the built-in mission on the
echo      OLED map.  b = return home.  The knob is the wind.
echo.
echo   Python, browser route:  python host\mission.py host\default.mission
echo   prints frames - type or paste them into the serial monitor one
echo   line at a time.  Copy the monitor's text to capture.txt, then
echo        python host\mission.py --score capture.txt --race
echo   Live route (Wokwi VS Code extension; needs pyserial):
echo        python host\mission.py host\default.mission --port rfc2217://localhost:4000 --log flight.log
echo.
start "" notepad "%~dp0diagram.json"
start "" "https://wokwi.com/projects/new/st-nucleo-c031c6"
exit /b 0
