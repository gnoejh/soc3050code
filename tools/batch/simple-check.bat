@echo off
echo.
echo ============================================
echo   SOC3050 Simple Environment Check
echo ============================================
echo.

echo Checking bundled tools...
echo.

REM Check AVR-GCC
if exist "tools\avr-toolchain\bin\avr-gcc.exe" (
    echo ✅ AVR-GCC Compiler found
) else (
    echo ❌ AVR-GCC Compiler missing
    set ERROR_FOUND=1
)

REM Check AVR-Objcopy
if exist "tools\avr-toolchain\bin\avr-objcopy.exe" (
    echo ✅ AVR-Objcopy found
) else (
    echo ❌ AVR-Objcopy missing
    set ERROR_FOUND=1
)

REM Check SimulIDE
if exist "tools\simulide\SimulIDE_1.1.0-SR1_Win64\simulide.exe" (
    echo ✅ SimulIDE Simulator found
) else (
    echo ❌ SimulIDE Simulator missing
    set ERROR_FOUND=1
)

REM Check ATmega DFP
if exist "tools\packs\atmel\ATmega_DFP\1.7.374\include\avr\iom128.h" (
    echo ✅ ATmega128 Device Pack found
) else (
    echo ❌ ATmega128 Device Pack missing
    set ERROR_FOUND=1
)

REM Check VS Code configuration
if exist ".vscode\tasks.json" (
    echo ✅ VS Code tasks configured
) else (
    echo ❌ VS Code tasks missing
    set ERROR_FOUND=1
)

REM Check sample projects
if exist "projects\Port_Basic\Main.c" (
    echo ✅ Sample projects found
) else (
    echo ❌ Sample projects missing
    set ERROR_FOUND=1
)

echo.
echo ============================================

if defined ERROR_FOUND (
    echo ❌ SETUP ISSUES DETECTED
    echo.
    echo Some tools are missing. This may indicate:
    echo - Incomplete repository clone
    echo - Missing Git LFS files
    echo - Repository corruption
    echo.
    echo Please re-clone the repository or contact instructor.
) else (
    echo ✅ ENVIRONMENT READY!
    echo.
    echo All required tools are present.
    echo.
    echo Next steps:
    echo 1. Open VS Code: code .
    echo 2. Navigate to projects/ folder
    echo 3. Start with Port_Basic project
    echo 4. Use Ctrl+Shift+P to run tasks
)

echo.
echo ============================================
pause