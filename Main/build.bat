@echo off
REM Simple batch build script for ATmega128 project
echo [BUILD] Starting ATmega128 Project Build...

set AVR_GCC="C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe"
set AVR_OBJCOPY="C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe"
set AVR_SIZE="C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe"

REM Build like Microchip Studio - include all needed modules
set SOURCE_FILES=Main.c main_serial.c main_blink.c main_game_hangman.c main_game_obstacle.c main_game_puzzle.c main_game_pong_uart_control.c main_game_word_puzzle.c main_graphics.c main_sound.c main_sound_atari.c main_sound_twingkle.c main_motors.c main_timer.c main_interrupt.c main_joystick.c main_memory.c main_cds.c main_iot.c main_inline.c main_accelerometer.c main_adc.c _uart.c _init.c _interrupt.c _timer2.c _adc.c _glcd.c _port.c variables.c

echo [CONFIG] Building with all modules (like Microchip Studio)
echo [COMPILE] Compiling source files: %SOURCE_FILES%
%AVR_GCC% -mmcu=atmega128 -DF_CPU=16000000UL -Os -Wall -std=gnu99 -I. %SOURCE_FILES% -o Main.elf

if %ERRORLEVEL% == 0 (
    echo [SUCCESS] Compilation successful!
    echo [HEX] Generating HEX file...
    %AVR_OBJCOPY% -O ihex Main.elf Main.hex
    echo [MEMORY] Memory usage:
    %AVR_SIZE% Main.elf
    echo.
    echo [DONE] Build completed successfully!
    echo        - ELF file: Main.elf
    echo        - HEX file: Main.hex
) else (
    echo [ERROR] Compilation failed!
)

pause