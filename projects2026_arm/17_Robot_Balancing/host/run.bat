@echo off
REM ============================================================================
REM  17_Robot_Balancing - host verification (cmd)
REM
REM  1. lqr.py designs K from ..\params.h, checks it stabilises the linear
REM     model, and checks control.c carries the same K (--check).
REM  2. sitl.c runs the firmware's own world.c, control.c and view.c on the PC.
REM  Needs gcc (MinGW) and python on PATH.  Exit status non-zero on failure.
REM ============================================================================
setlocal
cd /d "%~dp0"
echo === lqr.py ===
python lqr.py --check ..\control.c
if errorlevel 1 exit /b 1
echo.
echo === sitl ===
gcc -O2 -std=c11 -Wall -Wextra -I.. -I..\..\_lib sitl.c ..\world.c ..\control.c ..\view.c ..\..\_lib\oled.c -lm -o sitl.exe
if errorlevel 1 exit /b 1
sitl.exe %*
exit /b %errorlevel%
