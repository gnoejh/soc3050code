@echo off
REM Custom build script for Serial_interrupt project (without UART library)
echo Building Serial_interrupt project for ATmega128 with custom startup...

"..\..\tools\avr-toolchain\bin\avr-gcc.exe" -mmcu=atmega128 -DF_CPU=16000000UL -DBAUD=9600 -O1 -Wall -I. -I../../shared_libs -nostartfiles minimal_startup.c Main.c ../../shared_libs/_port.c -lm -lgcc -o Main.elf

if %ERRORLEVEL% EQU 0 (
    echo Build successful! Creating HEX file...
    "..\..\tools\avr-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom Main.elf Main.hex
    echo Creating clean HEX file for SimulIDE...
    "..\..\tools\avr-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom Main.hex Main_clean.hex
    echo Build completed! Files: Main.elf, Main.hex, Main_clean.hex
    echo Use Main_clean.hex for SimulIDE to avoid ELPM format issues
) else (
    echo Build failed!
)

pause