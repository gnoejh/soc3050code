@echo off
REM ============================================================================
REM  Target: ST Nucleo-C031C6  -  STM32C031C6, Cortex-M0+, 48 MHz
REM
REM  Called by _build\build-lesson.bat.  Sets everything that is per-chip.
REM  Chosen as Target A because it is the STM32 part wokwi.com actually hosts
REM  and because all four Part 0 decks quote its reference manual, RM0490.
REM ============================================================================
set "MCUFLAGS=-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft"
set "DEVDEF=-DSTM32C031xx"
set "DEVINC=Device\ST\STM32C0xx\Include"
set "FLASHKB=32"
set "RAMKB=12"
set "WOKWIBOARD=st-nucleo-c031c6"
