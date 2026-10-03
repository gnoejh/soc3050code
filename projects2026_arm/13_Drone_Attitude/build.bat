@echo off
REM ============================================================================
REM  13_Drone_Attitude - build
REM
REM  world.c (the simulated drone) and control.c (the flight controller) are
REM  compiled because they sit in this folder - they are the lesson.  From
REM  _lib\: the kernel, lesson 08's interrupt UART and frame checksums, I2C,
REM  the OLED, the app board's controls (pad + adc) and the buzzer.
REM  host\run.bat links the SAME world.c and control.c into a PC test.
REM ============================================================================
set TARGET=c031c6
set LIBS=retarget os uart proto i2c oled adc pad beep
call "%~dp0..\_build\build-lesson.bat"
