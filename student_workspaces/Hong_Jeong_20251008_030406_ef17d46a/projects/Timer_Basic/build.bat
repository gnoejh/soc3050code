@echo off
echo Building 13_Timer_Basic Project...

"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe" ^
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
    -o Main.elf

if %errorlevel% neq 0 (
    echo Build failed!
    exit /b %errorlevel%
)

echo Build successful! Generating HEX file...

"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" ^
    -O ihex ^
    -R .eeprom ^
    Main.elf ^
    Main.hex

if %errorlevel% neq 0 (
    echo HEX generation failed!
    exit /b %errorlevel%
)

echo Files created: Main.elf, Main.hex
