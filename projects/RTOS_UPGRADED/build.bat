@echo off
REM Enhanced RTOS Build Script - Standalone Implementation

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

echo Compiling source files...
echo.

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
    -c Main.c ^
    -o Main.o

if %errorlevel% neq 0 (
    echo [ERROR] Main.c compilation failed!
    pause
    exit /b %errorlevel%
)

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
    -c TASK_TEMPLATE.c ^
    -o TASK_TEMPLATE.o

if %errorlevel% neq 0 (
    echo [ERROR] TASK_TEMPLATE.c compilation failed!
    pause
    exit /b %errorlevel%
)

echo Linking...
"%AVR_GCC%" ^
    -mmcu=%MCU% ^
    -mshort-calls ^
    Main.o ^
    TASK_TEMPLATE.o ^
    %PROJECT_ROOT%\shared_libs\_glcd.c ^
    -o Main.elf

if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Build failed!
    pause
    exit /b %errorlevel%
)

echo.
echo [SUCCESS] Compilation complete!
echo.
echo Generating HEX file...

"%AVR_OBJCOPY%" ^
    -O ihex ^
    -R .eeprom ^
    Main.elf ^
    Main.hex

if %errorlevel% neq 0 (
    echo [ERROR] HEX generation failed!
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
