@echo off
REM ============================================================================
REM  10_Fixed_Point_And_DMA - build
REM
REM  fix.c, bench.c and dma.c are compiled because they sit in this folder -
REM  they are the subject of the lesson.  From _lib\: printf to the serial
REM  monitor (retarget), the polled ADC for the fallback path (adc), and the
REM  OLED with its I2C driver (oled, i2c).  No os: this lesson owns SysTick.
REM ============================================================================
set TARGET=c031c6
set LIBS=retarget adc i2c oled
call "%~dp0..\_build\build-lesson.bat"
