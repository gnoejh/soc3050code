@echo off
REM Build script for Timer Stopwatch project
echo Building Timer Stopwatch...

REM Define paths
set TOOLCHAIN_PATH=..\..\tools\avr-toolchain
set AVR_GCC=%TOOLCHAIN_PATH%\bin\avr-gcc.exe
set AVR_OBJCOPY=%TOOLCHAIN_PATH%\bin\avr-objcopy.exe
set AVR_SIZE=%TOOLCHAIN_PATH%\bin\avr-size.exe

REM Compiler flags
set MCU=atmega128
set CFLAGS=-mmcu=%MCU% -DF_CPU=16000000UL -Os -Wall -Wextra

REM Source files
set SOURCES=Main.c ..\..\shared_libs\_uart.c ..\..\shared_libs\_init.c ..\..\shared_libs\_port.c ..\..\shared_libs\_glcd.c

REM Compile
%AVR_GCC% %CFLAGS% %SOURCES% -o Main.elf

if %errorlevel% neq 0 (
    echo Build failed!
    exit /b %errorlevel%
)

echo Build successful! Generating HEX file...

REM Generate HEX file
%AVR_OBJCOPY% -O ihex -R .eeprom Main.elf Main.hex

echo Files created: Main.elf, Main.hex
echo.

REM Show size
%AVR_SIZE% --format=berkeley Main.elf

echo.
echo Build complete!
