@echo off
REM 16_I2C_RTC_DS1307 - build
REM LIBS names the shared_libs modules this lesson links against, without the
REM .c extension.  Regenerate with: python ..\_build\resolve-libs.py --all
set LIBS=_uart
call "%~dp0..\_build\build-lesson.bat"
