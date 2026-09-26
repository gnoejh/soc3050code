@echo off
REM ============================================================================
REM  06_Time_And_Timers - build
REM
REM  startup.c and link.ld come from _startup\c031c6\.  gpio.h is header-only
REM  and needs no LIBS entry; retarget is the printf-to-serial glue.
REM ============================================================================
set TARGET=c031c6
set LIBS=retarget
call "%~dp0..\_build\build-lesson.bat"
