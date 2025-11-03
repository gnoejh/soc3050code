@echo off
REM Build script for Timer1 CTC Precision project

echo Building Timer1 CTC Precision...

set AVR_GCC=w:\soc3050code\tools\avr-toolchain\bin\avr-gcc.exe
set AVR_OBJCOPY=w:\soc3050code\tools\avr-toolchain\bin\avr-objcopy.exe

"%AVR_GCC%" ^
    -mmcu=atmega128 ^
    -DF_CPU=16000000UL ^
    -DBAUD=9600 ^
    -Os ^
    -Wall ^
    -Wextra ^
    -I. ^
    -I../../shared_libs ^
    Main.c ^
    ../../shared_libs/_uart.c ^
    ../../shared_libs/_init.c ^
    ../../shared_libs/_port.c ^
    ../../shared_libs/_glcd.c ^
    -o Main.elf

if %errorlevel% neq 0 (
    echo Build failed!
    exit /b %errorlevel%
)

echo Build successful! Generating HEX file...

"%AVR_OBJCOPY%" ^
    -O ihex ^
    -R .eeprom ^
    Main.elf ^
    Main.hex

if %errorlevel% neq 0 (
    echo HEX generation failed!
    exit /b %errorlevel%
)

echo Files created: Main.elf, Main.hex
echo.
"%AVR_GCC:avr-gcc.exe=avr-size.exe%" Main.elf
echo.
echo Build complete!
