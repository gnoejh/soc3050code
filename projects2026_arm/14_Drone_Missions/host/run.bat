@echo off
REM ============================================================================
REM  14_Drone_Missions\host - build and run the host tests (Windows)
REM
REM  Needs gcc on PATH (MinGW-w64) and Python 3.  Same steps as run.sh:
REM  the flight tests, mission.py scoring the wind flight, and the
REM  mission.py -> C parser -> mission.py round trip.
REM ============================================================================
setlocal
cd /d "%~dp0"
if not exist out mkdir out

echo == build ==
gcc -O2 -std=c11 -Wall -Wextra -I.. -I..\..\_lib -o out\sitl.exe sitl.c ..\world.c ..\flight.c ..\mavlite.c ..\ui.c ..\fm.c ..\..\_lib\proto.c ..\..\_lib\oled.c -lm
if errorlevel 1 exit /b 1

out\sitl.exe
if errorlevel 1 exit /b 1

echo.
echo == mission.py --score on the wind flight, checked against the C numbers ==
python mission.py --score out\wind.log --race --expect out\wind.expect
if errorlevel 1 exit /b 1

echo.
echo == round trip: mission.py -^> C parser -^> mission.py ==
python mission.py default.mission > out\frames.txt
out\sitl.exe --parse out\frames.txt out\loaded.txt
if errorlevel 1 exit /b 1
python mission.py --verify-mis default.mission out\loaded.txt
if errorlevel 1 exit /b 1

echo.
echo ALL HOST TESTS PASSED
exit /b 0
