@echo off
REM ============================================================================
REM  16_Robot_Navigation\host\run.bat - the robot, the world and the planner,
REM  compiled for the PC and run on every level.  No board, no Wokwi.
REM
REM  Needs gcc on PATH (MinGW-w64) and python.  Exit code 0 = everything passed.
REM    host\run.bat         summary tables
REM    host\run.bat -v      one line per robot run
REM ============================================================================
setlocal
cd /d "%~dp0"
gcc -O2 -std=c11 -Wall -Wextra -I.. -I..\..\_lib -o sitl.exe sitl.c ..\world.c ..\nav.c ..\astar.c ..\levels.c ..\view.c ..\mathx.c ..\..\_lib\oled.c -lm
if errorlevel 1 exit /b 1
sitl.exe %*
if errorlevel 1 exit /b 1
python planner.py --compare paths.txt
