@echo off
REM  host\run.bat - lesson 11 host test, plain gcc (MinGW) on Windows.
REM  Build the firmware first (build.bat) so the objdump cross-check has a
REM  Main.elf to read.  Exit code 0 only if every check passes.
setlocal
cd /d "%~dp0"
set "OBJDUMP=%~dp0..\..\..\tools\arm-toolchain\bin\arm-none-eabi-objdump.exe"
if exist objdump.txt del objdump.txt
if exist "..\Main.elf" "%OBJDUMP%" -d "..\Main.elf" > objdump.txt
gcc -std=c11 -O2 -Wall -Wextra -o test.exe test.c ..\diag.c
if errorlevel 1 exit /b 1
test.exe objdump.txt
exit /b %errorlevel%
