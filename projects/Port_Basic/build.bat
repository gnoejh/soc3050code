@echo off
echo Building 01_Port_Basic Project...

"..\..\tools\avr-toolchain\bin\avr-gcc.exe" ^
    -mmcu=atmega128 ^
    -DF_CPU=7372800UL ^
    -DBAUD=9600 ^
    -Os ^
    -Wall ^
    -Wextra ^
    -I. ^
    -I../../shared_libs ^
    Main.c ^
    -o Main.elf

if %errorlevel% neq 0 (
    echo Build failed!
    exit /b %errorlevel%
)

echo Build successful! Generating HEX file...

"..\..\tools\avr-toolchain\bin\avr-objcopy.exe" ^
    -O ihex ^
    -R .eeprom ^
    Main.elf ^
    Main.hex

if %errorlevel% neq 0 (
    echo HEX generation failed!
    exit /b %errorlevel%
)

echo Files created: Main.elf, Main.hex
