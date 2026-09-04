@echo off
REM 08_Timer1_Input_Capture - build
REM LIBS names the shared_libs modules this lesson links against, without the
REM .c extension.  Regenerate with: python ..\_build\resolve-libs.py --all
set LIBS=
call "%~dp0..\_build\build-lesson.bat"
