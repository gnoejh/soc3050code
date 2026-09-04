@echo off
REM Build script for Stepper Motor Control project
REM Uses shared_libs and tools folders

echo Building Stepper Motor Control...

set AVR_GCC=../../tools/avr-toolchain/bin/avr-gcc.exe
set AVR_OBJCOPY=../../tools/avr-toolchain/bin/avr-objcopy.exe
set AVR_SIZE=../../tools/avr-toolchain/bin/avr-size.exe

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
  -o Main.elf

if %ERRORLEVEL% EQU 0 (
    echo Build successful!
    echo Generating HEX file...
    "%AVR_OBJCOPY%" ^
      -O ihex ^
      -R .eeprom ^
      Main.elf ^
      Main.hex
    echo Files created: Main.elf, Main.hex
    echo.
    "%AVR_SIZE%" Main.elf
    echo.
    echo Build complete!
) else (
    echo Build failed!
    exit /b 1
)
