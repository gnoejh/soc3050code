@echo off
REM ============================================================================
REM  07_RTOS - build
REM
REM  os.c is compiled because it sits in this folder - it is the subject of the
REM  lesson, the way startup.c was in lesson 04.  startup.c and link.ld come
REM  from _startup\c031c6\; retarget from _lib\.
REM ============================================================================
set TARGET=c031c6
set LIBS=retarget
call "%~dp0..\_build\build-lesson.bat"
