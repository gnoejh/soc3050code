@echo off
REM ============================================================================
REM  13_Drone_Attitude\host - build and run the SITL test with the PC's gcc
REM
REM  Links the SAME ..\world.c and ..\control.c the firmware links.  Exit code
REM  is the test's: 0 = every check passed.  Needs gcc on PATH (MinGW-w64).
REM
REM  The cycle budget is a separate, slower check (about 35 s): build the
REM  firmware first, then   python host\cycles.py
REM ============================================================================
pushd "%~dp0"
gcc -std=c11 -O2 -Wall -Wextra -I.. -I..\..\_lib -o sitl.exe sitl.c ..\world.c ..\control.c -lm
if errorlevel 1 (popd & exit /b 1)
sitl.exe
set RC=%ERRORLEVEL%
popd
exit /b %RC%
