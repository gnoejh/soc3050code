@echo off
REM ============================================================================
REM  11_Faults_Watchdog_Power - build
REM
REM  Main.c, crash.c, diag.c and wdog.c are compiled because they sit in this
REM  folder - they are the subject of the lesson.  So is link.ld: this lesson
REM  carries its own, adding a .noinit section and _stack_floor (see the file).
REM  startup.c is the shared one.  From _lib\: printf to USART2 (retarget),
REM  the app board's buttons and ADC (pad, adc), and the OLED (oled, i2c).
REM ============================================================================
set TARGET=c031c6
set LIBS=retarget pad adc i2c oled
call "%~dp0..\_build\build-lesson.bat"
