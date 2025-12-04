@echo off
REM Build script for SPI EEPROM Memory project

echo Building SPI EEPROM Memory...

"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe" ^
  -mmcu=atmega128 ^
  -DF_CPU=7372800UL ^
  -DBAUD=9600 ^
  -Os ^
  -Wall ^
  -Wextra ^
  -I. ^
  -I../../shared_libs ^
  Main.c ^
  ../../shared_libs/_uart.c ^
  -o Main.elf

if %ERRORLEVEL% EQU 0 (
    echo Build successful!
    echo Generating HEX file...
    "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" ^
      -O ihex ^
      -R .eeprom ^
      Main.elf ^
      Main.hex
    echo Files created: Main.elf, Main.hex
) else (
    echo Build failed!
    exit /b 1
)
