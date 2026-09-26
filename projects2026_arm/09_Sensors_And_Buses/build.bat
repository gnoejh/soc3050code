@echo off
REM ============================================================================
REM  09_Sensors_And_Buses - build
REM
REM  adc.c, i2c.c and spi.c are compiled because they sit in this folder - they
REM  are the subject of the lesson.  From _lib\: the kernel (os), lesson 08's
REM  interrupt-driven UART (uart) and frame format (proto), and retarget's
REM  C library stubs.
REM ============================================================================
set TARGET=c031c6
set LIBS=retarget os uart proto
call "%~dp0..\_build\build-lesson.bat"
