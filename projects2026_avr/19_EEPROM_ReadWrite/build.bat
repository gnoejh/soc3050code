@echo off
REM 19_EEPROM_ReadWrite - build
REM LIBS names the shared_libs modules this lesson links against, without the
REM .c extension.  Regenerate with: python ..\_build\resolve-libs.py --all
set LIBS=_init _port _glcd
call "%~dp0..\_build\build-lesson.bat"
