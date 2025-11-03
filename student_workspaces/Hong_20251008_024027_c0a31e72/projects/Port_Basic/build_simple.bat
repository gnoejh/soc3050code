@echo off
REM Simple build script for Port_Basic project - no external libraries
echo Building Port_Basic project (basic port operations only)...

"..\..\tools\avr-toolchain\bin\avr-gcc.exe" -mmcu=atmega128 -DF_CPU=16000000UL -Os -Wall -I. Main.c -o Main.elf

if %ERRORLEVEL% EQU 0 (
    echo Build successful! Creating HEX file...
    "..\..\tools\avr-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom Main.elf Main.hex
    echo Build completed! Files: Main.elf, Main.hex
) else (
    echo Build failed!
)

pause