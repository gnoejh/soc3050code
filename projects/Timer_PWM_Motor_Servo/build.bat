@echo off
REM Build script for PWM Servo Motor Control project (REFINED)
REM Now uses _pwm.h library for professional servo control

echo Building PWM Servo Motor Control (Refined with _pwm.h)...

set AVR_GCC=w:\soc3050code\tools\avr-toolchain\bin\avr-gcc.exe
set AVR_OBJCOPY=w:\soc3050code\tools\avr-toolchain\bin\avr-objcopy.exe

"%AVR_GCC%" ^
  -mmcu=atmega128 ^
  -DF_CPU=7372800UL ^
  -DBAUD=9600 ^
  -Os ^
  -Wall ^
  -Wextra ^
  -I. ^
  -I../../shared_libs ^
  Main.c ^
  ../../shared_libs/_port.c ^
  ../../shared_libs/_init.c ^
  ../../shared_libs/_adc.c ^
  ../../shared_libs/_uart.c ^
  ../../shared_libs/_timer.c ^
  ../../shared_libs/_pwm.c ^
  ../../shared_libs/_glcd.c ^
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
    "%AVR_GCC:avr-gcc.exe=avr-size.exe%" Main.elf
    echo.
    echo Build complete!
) else (
    echo Build failed!
    exit /b 1
)
