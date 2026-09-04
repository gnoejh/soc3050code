@echo off
REM Build script for Timer0 CTC Precision project

echo Building Timer0 CTC Precision...

set AVR_GCC=w:\soc3050code\tools\avr-toolchain\bin\avr-gcc.exe
set AVR_OBJCOPY=w:\soc3050code\tools\avr-toolchain\bin\avr-objcopy.exe
set AVR_SIZE=w:\soc3050code\tools\avr-toolchain\bin\avr-size.exe

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
    -o Main.elf

if %ERRORLEVEL% NEQ 0 (
    echo Compilation failed!
    exit /b 1
)

"%AVR_OBJCOPY%" -O ihex -R .eeprom Main.elf Main.hex

if %ERRORLEVEL% NEQ 0 (
    echo Hex file generation failed!
    exit /b 1
)

echo.
echo Build successful!
echo.
"%AVR_SIZE%" --format=berkeley Main.elf
echo.
echo Output files:
echo   Main.elf - Executable with debug info
echo   Main.hex - Flash programming file
echo.
