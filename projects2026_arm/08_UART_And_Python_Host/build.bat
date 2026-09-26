@echo off
REM ============================================================================
REM  08_UART_And_Python_Host - build
REM
REM  LIBS: os is lesson 07's kernel, from _lib\.  retarget supplies the C
REM  library stubs; its weak _write() is replaced by the strong, interrupt-
REM  driven one in uart.c - the same weak/strong trick as the vector table.
REM ============================================================================
set TARGET=c031c6
set LIBS=retarget os
call "%~dp0..\_build\build-lesson.bat"
