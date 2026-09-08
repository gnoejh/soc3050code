@echo off
REM 24_Game_Arcade - build
REM LIBS names the shared_libs modules this lesson links against, without the
REM .c extension.  Regenerate with: python ..\_build\resolve-libs.py --all
REM
REM _game defines the TIMER1_COMPA and TIMER2_COMP ISRs, which _timer.c also
REM defines, so _timer must stay off this list.  _buzzer is absent too: its
REM Sound() busy-waits for the whole note, and _game does the same job from an
REM interrupt without stopping the frame.
set LIBS=_game
call "%~dp0..\_build\build-lesson.bat"
