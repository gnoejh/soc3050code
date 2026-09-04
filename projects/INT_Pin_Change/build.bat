@echo off
echo Building INT_Pin_Change...
set TOOLCHAIN_PATH=..\..\tools\avr-toolchain
set AVR_GCC=%TOOLCHAIN_PATH%\bin\avr-gcc.exe
set AVR_OBJCOPY=%TOOLCHAIN_PATH%\bin\avr-objcopy.exe
set AVR_SIZE=%TOOLCHAIN_PATH%\bin\avr-size.exe
set MCU=atmega128
set CFLAGS=-mmcu=%MCU% -DF_CPU=16000000UL -Os -Wall -Wextra
set SOURCES=Main.c ..\..\shared_libs\_uart.c ..\..\shared_libs\_init.c ..\..\shared_libs\_port.c ..\..\shared_libs\_glcd.c ..\..\shared_libs\_adc.c
%AVR_GCC% %CFLAGS% %SOURCES% -o Main.elf
if %errorlevel% neq 0 (echo Build failed! & exit /b %errorlevel%)
%AVR_OBJCOPY% -O ihex -R .eeprom Main.elf Main.hex
%AVR_SIZE% --format=berkeley Main.elf
echo Build complete!
