@echo off
REM ============================================================================
REM  Shared build engine - SOC3050 2026 AVR edition
REM
REM  Not run directly.  Each lesson's build.bat sets LIBS then calls this:
REM      set LIBS=_init _port _uart
REM      call "%~dp0..\_build\build-lesson.bat"
REM
REM  LIBS lists shared_libs module names without the .c extension.  Leave it
REM  empty for a self-contained lesson.  The list is per-lesson on purpose:
REM  some lessons define their own ISR or UART routine that a shared library
REM  also defines, so linking every library would be a duplicate symbol.
REM  Regenerate the lists with _build\resolve-libs.py.
REM ============================================================================
setlocal enabledelayedexpansion

set "ROOT=%~dp0..\.."
set "GCC=%ROOT%\tools\avr-toolchain\bin\avr-gcc.exe"
set "OBJCOPY=%ROOT%\tools\avr-toolchain\bin\avr-objcopy.exe"
set "SIZE=%ROOT%\tools\avr-toolchain\bin\avr-size.exe"

if not exist "%GCC%" (
    echo ERROR: AVR toolchain not found at "%GCC%"
    exit /b 1
)
if not exist "Main.c" (
    echo ERROR: Main.c not found. Run build.bat from inside a lesson folder.
    exit /b 1
)

for %%I in ("%CD%") do set "LESSON=%%~nxI"
echo Building %LESSON% ...

REM ---- assemble the source list ------------------------------------------
set "SOURCES=Main.c"
for %%L in (%LIBS%) do (
    if not exist "%ROOT%\shared_libs\%%L.c" (
        echo ERROR: shared library "%%L.c" does not exist.
        exit /b 1
    )
    set "SOURCES=!SOURCES! "%ROOT%\shared_libs\%%L.c""
)

REM ---- compile + link ------------------------------------------------------
REM  F_CPU is fixed at 16 MHz for the whole 2026 edition so that it always
REM  matches Frequency="16 MHz" on the MCU in the shared SimulIDE board.
"%GCC%" ^
    -mmcu=atmega128 ^
    -DF_CPU=16000000UL ^
    -DBAUD=9600 ^
    -Os ^
    -Wall ^
    -Wextra ^
    -ffunction-sections ^
    -fdata-sections ^
    -I. ^
    -I"%ROOT%\shared_libs" ^
    !SOURCES! ^
    -Wl,--gc-sections ^
    -o Main.elf
if errorlevel 1 (
    echo.
    echo Build FAILED.
    exit /b 1
)

"%OBJCOPY%" -O ihex -R .eeprom Main.elf Main.hex
if errorlevel 1 (
    echo HEX generation FAILED.
    exit /b 1
)

REM ---- report size ---------------------------------------------------------
REM  This toolchain's avr-size has no --format=avr, so use the default Berkeley
REM  output.  A size failure must never fail an otherwise good build.
echo.
"%SIZE%" Main.elf
echo.
echo Build OK: Main.hex created
exit /b 0
