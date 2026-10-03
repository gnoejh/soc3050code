@echo off
REM ============================================================================
REM  fpu_compare.bat - compile host\kernel.c for the Cortex-M0+ and the
REM  Cortex-M4F, then disassemble both.  SOC3050 lesson 10.
REM
REM  Compile only: nothing is linked or run.  Uses the vendored toolchain, so
REM  it works on a fresh clone.  Run from anywhere:   host\fpu_compare.bat
REM ============================================================================
setlocal
set "HERE=%~dp0"
set "BIN=%HERE%..\..\..\tools\arm-toolchain\bin"
set "OUT=%TEMP%\soc3050_fpu"
if not exist "%OUT%" mkdir "%OUT%"

"%BIN%\arm-none-eabi-gcc.exe" -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -Os -c "%HERE%kernel.c" -o "%OUT%\kernel_m0plus.o"
if errorlevel 1 exit /b 1
"%BIN%\arm-none-eabi-gcc.exe" -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Os -c "%HERE%kernel.c" -o "%OUT%\kernel_m4f.o"
if errorlevel 1 exit /b 1

echo ================ Cortex-M0+ (no FPU): soft float ================
"%BIN%\arm-none-eabi-objdump.exe" -d -r "%OUT%\kernel_m0plus.o"
echo.
echo ================ Cortex-M4F (FPv4-SP): hard float ===============
"%BIN%\arm-none-eabi-objdump.exe" -d "%OUT%\kernel_m4f.o"
echo.
echo ---- code size (bytes of .text) ----
"%BIN%\arm-none-eabi-size.exe" "%OUT%\kernel_m0plus.o" "%OUT%\kernel_m4f.o"
exit /b 0
