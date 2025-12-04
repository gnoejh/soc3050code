@echo off
REM Build script for Serial Communications Lab.c
REM This builds the interrupt-based Q&A system with LCD display
echo Building Serial Communications Lab for ATmega128...

"..\..\tools\avr-toolchain\bin\avr-gcc.exe" -mmcu=atmega128 -DF_CPU=16000000UL -DBAUD=9600 -O3 -Wall -I. -I../../shared_libs -funsigned-char -funsigned-bitfields -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax Lab.c ../../shared_libs/_glcd.c ../../shared_libs/_port.c -lm -o Lab.elf

if %ERRORLEVEL% EQU 0 (
    echo Build successful! Creating HEX file...
    "..\..\tools\avr-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom Lab.elf Lab.hex
    echo Build completed! Files: Lab.elf, Lab.hex
    echo.
    echo Programming instructions:
    echo   1. For hardware: avrdude -c arduino -p m128 -P COM3 -U flash:w:Lab.hex:i
    echo   2. For SimulIDE: Load Lab.hex directly into AVR component
    echo.
) else (
    echo Build failed!
)

pause
