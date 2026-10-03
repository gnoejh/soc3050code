@echo off
REM ============================================================================
REM  15_Robot_Line_Follower\host - build and run the Software-In-The-Loop test
REM
REM  Needs a PC C compiler called gcc on PATH (MinGW-w64; the course machine
REM  has 15.2).  The SAME world.c, track.c, control.c and view.c the firmware
REM  uses, plus _lib\oled.c with the I2C call stubbed out.
REM
REM    run.bat                    all the tests; exit code 1 on any failure
REM    run.bat race 3 600         one race: track 3 (HAIRPINS), knob 600
REM    run.bat race 2 700 20 0    ... with kp 20, kd 0 (then ki, dfilt, noise)
REM    run.bat ascii 4            the OLED frame for track 4, as text
REM ============================================================================
setlocal
set H=%~dp0
set L=%~dp0..
gcc -O2 -std=c11 -Wall -Wextra -I"%L%" -I"%L%\..\_lib" -o "%H%sitl.exe" "%H%sitl.c" "%L%\world.c" "%L%\track.c" "%L%\control.c" "%L%\view.c" "%L%\..\_lib\oled.c" -lm
if errorlevel 1 exit /b 1
"%H%sitl.exe" %*
exit /b %errorlevel%
