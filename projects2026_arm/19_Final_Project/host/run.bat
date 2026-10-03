@echo off
REM ============================================================================
REM  19_Final_Project\host - compile the pure-C half of the project with the
REM  PC's gcc and run the tests.  Exit code 0 = every check passed.
REM
REM  The SAME app.c and health.c the board runs, plus _lib\oled.c's drawing
REM  code.  No stm32c031xx.h anywhere: if one of those files ever includes it,
REM  this build fails - which is the point.
REM ============================================================================
setlocal
cd /d "%~dp0"
gcc -std=c11 -O2 -Wall -Wextra -I.. -I..\..\_lib -o test_app.exe test_app.c ..\app.c ..\health.c ..\..\_lib\oled.c
if errorlevel 1 exit /b 1
"%~dp0test_app.exe"
exit /b %errorlevel%
