@echo off
REM ============================================================================
REM  04_Startup_And_LinkerScript - build
REM
REM  This lesson is self-sufficient: it carries its own startup.c and link.ld,
REM  which are the subject of the lesson rather than shared infrastructure.
REM  The engine compiles every .c in this folder and links with link.ld.
REM
REM  LIBS names modules from _lib\ without the .c extension.  Lesson 04 uses
REM  none - everything it needs is in this folder, on purpose.
REM ============================================================================
set TARGET=c031c6
set LIBS=
call "%~dp0..\_build\build-lesson.bat"
