@echo off
REM 23_Game_Engine_GLCD - build
REM LIBS names the shared_libs modules this lesson links against, without the
REM .c extension.  Regenerate with: python ..\_build\resolve-libs.py --all
REM
REM _game defines the TIMER1_COMPA ISR (__vector_12), which _timer.c also
REM defines, so _timer must stay off this list.  _glcd is absent on purpose:
REM _game drives the panel itself and keeps its font in flash, where _glcd
REM spends 476 bytes of SRAM on one.
set LIBS=_game
call "%~dp0..\_build\build-lesson.bat"
