@echo off
REM ============================================================================
REM  run.bat [strategy.c ...] [-- league options]  -  the sumo league on the PC
REM
REM    host\run.bat                                    built-ins + student.c
REM    host\run.bat strategies\alice.c strategies\bob.c  ... plus these files
REM    host\run.bat -- -s 77 --tour student            options for league.exe
REM
REM  Each extra file is compiled ON ITS OWN with -DSTRATEGY_PREFIX=<its name>,
REM  so its strategy_step becomes <name>_step, and register.h is force-included
REM  to hand it to the league.  Its object file is then checked with nm: any
REM  variable in .data or .bss (nm types B b D d C) is state outside the
REM  64-byte strategy memory, and the file is rejected.
REM
REM  Needs gcc on PATH (MinGW-w64).  No gcc?  Use the firmware's tournament:
REM  pick TOURNAMENT on the board, or type  tour  in the serial monitor.
REM ============================================================================
setlocal enabledelayedexpansion
set H=%~dp0
set L=%H%..
set LIB=%H%..\..\_lib
set OUT=%H%build
if not exist "%OUT%" mkdir "%OUT%"
set CFLAGS=-std=c11 -Wall -Wextra -ffp-contract=off -I"%L%" -I"%LIB%"

set OBJS=
set OPTS=
:args
if "%~1"=="" goto build
if "%~1"=="--" goto opts
set NAME=%~n1
gcc %CFLAGS% -O2 -c -DSTRATEGY_PREFIX=!NAME! -include "%H%register.h" "%~1" -o "%OUT%\!NAME!.o"
if errorlevel 1 ( echo REJECT %~1: does not compile & exit /b 1 )
nm "%OUT%\!NAME!.o" | findstr /R /C:" [BbDdCc] [^.]"
if not errorlevel 1 ( echo REJECT %~1: the symbols above are variables outside strategy_mem_t & exit /b 1 )
set OBJS=!OBJS! "%OUT%\!NAME!.o"
shift
goto args
:opts
shift
:optloop
if "%~1"=="" goto build
set OPTS=!OPTS! %1
shift
goto optloop

:build
set SRC="%H%league.c" "%L%\world.c" "%L%\referee.c" "%L%\bots.c" "%L%\student.c" "%L%\scene.c" "%LIB%\oled.c"
gcc %CFLAGS% -O2 %SRC% %OBJS% -o "%OUT%\league.exe" || exit /b 1
REM The same program at -O0 and -Os: if optimisation changed a single result,
REM the physics depends on the compiler, and the chip's -Os build could disagree.
gcc %CFLAGS% -O0 %SRC% %OBJS% -o "%OUT%\league_O0.exe" || exit /b 1
gcc %CFLAGS% -Os %SRC% %OBJS% -o "%OUT%\league_Os.exe" || exit /b 1

"%OUT%\league.exe" %OPTS%
if errorlevel 1 exit /b 1

echo.
echo 6. Compiler independence: -O2 against -O0 and -Os (the chip's level)
"%OUT%\league.exe" %OPTS% --hash > "%OUT%\h_O2.txt"
"%OUT%\league_O0.exe" %OPTS% --hash > "%OUT%\h_O0.txt"
"%OUT%\league_Os.exe" %OPTS% --hash > "%OUT%\h_Os.txt"
type "%OUT%\h_O2.txt"
fc /b "%OUT%\h_O2.txt" "%OUT%\h_O0.txt" > nul || ( echo   [FAIL] -O0 changed the results & exit /b 1 )
fc /b "%OUT%\h_O2.txt" "%OUT%\h_Os.txt" > nul || ( echo   [FAIL] -Os changed the results & exit /b 1 )
echo   [PASS] identical league and tournament hashes
exit /b 0
