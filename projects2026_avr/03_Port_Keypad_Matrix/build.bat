@echo off
REM 03_Port_Keypad_Matrix - build
REM LIBS names the shared_libs modules this lesson links against, without the
REM .c extension.  Regenerate with: python ..\_build\resolve-libs.py --all
set LIBS=_uart
call "%~dp0..\_build\build-lesson.bat"
