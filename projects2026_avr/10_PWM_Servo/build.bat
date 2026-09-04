@echo off
REM 10_PWM_Servo - build
REM LIBS names the shared_libs modules this lesson links against, without the
REM .c extension.  Regenerate with: python ..\_build\resolve-libs.py --all
set LIBS=_adc _pwm _timer _uart
call "%~dp0..\_build\build-lesson.bat"
