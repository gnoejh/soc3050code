@echo off
REM ============================================================================
REM  run.bat - lesson 10 host tests.   host\run.bat [path\to\Main.elf]
REM
REM    1. test_fix   fix.c + bench.c compiled with the PC's gcc, vs double
REM    2. m0sim.py   the arena's workloads run from Main.elf on a Cortex-M0+
REM                  instruction model, cycles counted (needs build.bat first)
REM    3. flashcost  what each number format costs in flash, from nm
REM
REM  Needs a host gcc on PATH (MinGW) and Python 3 - standard library only.
REM ============================================================================
setlocal
set "HERE=%~dp0"
set "L=%HERE%.."
set "ELF=%~1"
if "%ELF%"=="" set "ELF=%L%\Main.elf"
set "OUT=%TEMP%"

gcc -O2 -std=c11 -Wall -Wextra -ffp-contract=off -I"%L%" -I"%L%\..\_lib" "%L%\fix.c" "%L%\bench.c" "%HERE%test_fix.c" -lm -o "%OUT%\test_fix.exe"
if errorlevel 1 exit /b 1
"%OUT%\test_fix.exe" > "%OUT%\test_fix.txt"
set "RC=%ERRORLEVEL%"
type "%OUT%\test_fix.txt"
if not "%RC%"=="0" exit /b 1

if not exist "%ELF%" (
    echo.
    echo No %ELF% yet - run build.bat first for the cycle model and the flash costs.
    exit /b 0
)
python "%HERE%m0sim.py" "%ELF%" --expect "%OUT%\test_fix.txt"
if errorlevel 1 exit /b 1
python "%HERE%flashcost.py" "%ELF%"
if errorlevel 1 exit /b 1
exit /b 0
