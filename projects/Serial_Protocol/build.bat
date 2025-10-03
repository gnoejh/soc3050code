@echo off
REM Build Serial_Protocol without _uart.c library

"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe" -mmcu=atmega128 -DF_CPU=16000000UL -DBAUD=9600 -Os -Wall -I. -I../../shared_libs Main.c -o Main.elf

if %ERRORLEVEL% == 0 (
    "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom Main.elf Main.hex
    echo Build completed!
    echo Files: Main.elf, Main.hex
) else (
    echo Build failed!
)
