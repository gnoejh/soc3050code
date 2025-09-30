@echo off
REM Simple batch build script for ATmega128 project
echo [BUILD] Starting ATmega128 Project Build...

set AVR_GCC="C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe"
set AVR_OBJCOPY="C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe"
set AVR_SIZE="C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe"

echo [COMPILE] Compiling source files...
%AVR_GCC% -mmcu=atmega128 -DF_CPU=16000000UL -Os -Wall -std=gnu99 -I. Main.c main_blink.c -o Main.elf

if %ERRORLEVEL% == 0 (
    echo [SUCCESS] Compilation successful!
    echo [HEX] Generating HEX file...
    %AVR_OBJCOPY% -O ihex Main.elf Main.hex
    echo [MEMORY] Memory usage:
    %AVR_SIZE% Main.elf
    echo.
    echo [DONE] Build completed successfully!
    echo        - ELF file: Main.elf
    echo        - HEX file: Main.hex
) else (
    echo [ERROR] Compilation failed!
)

pause