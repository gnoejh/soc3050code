@echo off
REM 22_RTOS_Scheduler - build
REM LIBS names the shared_libs modules this lesson links against, without the
REM .c extension.  Regenerate with: python ..\_build\resolve-libs.py --all
set LIBS=_glcd
call "%~dp0..\_build\build-lesson.bat"
