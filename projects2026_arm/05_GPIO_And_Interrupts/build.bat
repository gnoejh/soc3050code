@echo off
REM ============================================================================
REM  05_GPIO_And_Interrupts - build
REM
REM  From this lesson on, startup.c and link.ld are infrastructure: this folder
REM  carries neither, so the engine links the target's defaults from
REM  _startup\c031c6\.  Lesson 04 is where those two files were the subject.
REM
REM  LIBS names modules from _lib\ without the .c extension.  retarget is the
REM  printf-to-serial glue lesson 04 carried in its own folder.
REM ============================================================================
set TARGET=c031c6
set LIBS=retarget
call "%~dp0..\_build\build-lesson.bat"
