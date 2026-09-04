@echo off
REM Enhanced RTOS Build Script - Standalone Implementation
REM Follows cli-build-project.ps1 pattern to avoid ELPM errors in SimulIDE

echo ========================================
echo Building Enhanced RTOS v2.0
echo ========================================
echo.

set PROJECT_ROOT=w:\soc3050code
set AVR_GCC=%PROJECT_ROOT%\tools\avr-toolchain\bin\avr-gcc.exe
set AVR_OBJCOPY=%PROJECT_ROOT%\tools\avr-toolchain\bin\avr-objcopy.exe
set AVR_SIZE=%PROJECT_ROOT%\tools\avr-toolchain\bin\avr-size.exe

set MCU=atmega128
set F_CPU=16000000UL
set BAUD=9600

REM Verify tools exist
if not exist "%AVR_GCC%" (
    echo [ERROR] AVR-GCC not found at: %AVR_GCC%
    pause
    exit /b 1
)

if not exist "%AVR_OBJCOPY%" (
    echo [ERROR] AVR-OBJCOPY not found at: %AVR_OBJCOPY%
    echo [ERROR] avr-objcopy.exe is REQUIRED to avoid ELPM errors in SimulIDE
    pause
    exit /b 1
)

echo Compiling Main.c...
echo [INFO] Using -mshort-calls to avoid ELPM errors in SimulIDE
echo.

REM Compile with -mshort-calls flag (forces LPM instead of ELPM)
REM This avoids "ELPM with no RAMPZ" errors in SimulIDE
"%AVR_GCC%" ^
    -mmcu=%MCU% ^
    -DF_CPU=%F_CPU% ^
    -DBAUD=%BAUD% ^
    -mshort-calls ^
    -Os ^
    -Wall ^
    -Wextra ^
    -Wno-unused-parameter ^
    -std=gnu99 ^
    -I. ^
    -I%PROJECT_ROOT%\shared_libs ^
    Main.c ^
    %PROJECT_ROOT%\shared_libs\_glcd.c ^
    -o Main.elf

if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Compilation failed!
    pause
    exit /b %errorlevel%
)

echo.
echo [SUCCESS] Compilation complete!
echo.
echo Generating HEX file with avr-objcopy.exe...
echo [INFO] This step is REQUIRED to avoid ELPM errors in SimulIDE

REM Use avr-objcopy to convert ELF to HEX (required for SimulIDE compatibility)
"%AVR_OBJCOPY%" ^
    -O ihex ^
    -R .eeprom ^
    Main.elf ^
    Main.hex

if %errorlevel% neq 0 (
    echo [ERROR] HEX generation failed!
    echo [ERROR] avr-objcopy.exe is required to generate SimulIDE-compatible HEX files
    pause
    exit /b %errorlevel%
)

echo [SUCCESS] HEX file created!
echo.
echo ========================================
echo Build Summary
echo ========================================

"%AVR_SIZE%" Main.elf

echo.
echo Files Generated:
echo   - Main.elf (ELF executable with debug info)
echo   - Main.hex (Intel HEX for programming)
echo.
echo ========================================
echo Ready for SimulIDE!
echo ========================================
echo.
echo Next Steps:
echo 1. Open SimulIDE
echo 2. Load: ..\..\tools\simulide\Simulator110.simu
echo 3. Right-click ATmega128 ^> Load Firmware
echo 4. Select Main.hex
echo 5. Connect Serial Monitor (9600 baud)
echo 6. Start simulation (F5)
echo.

pause
