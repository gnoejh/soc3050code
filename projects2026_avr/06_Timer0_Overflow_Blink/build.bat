@echo off
REM 06_Timer0_Overflow_Blink - build
REM LIBS names the shared_libs modules this lesson links against, without the
REM .c extension.  Regenerate with: python ..\_build\resolve-libs.py --all
set LIBS=
call "%~dp0..\_build\build-lesson.bat"
