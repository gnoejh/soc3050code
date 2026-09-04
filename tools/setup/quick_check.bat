@echo off
:: Quick Environment Check for Students
:: Verifies that essential tools are installed correctly

echo =====================================
echo   ATmega128 Environment Quick Check
echo =====================================
echo.

set "errors=0"

:: Check Python
echo [1/4] Checking Python installation...
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo   ❌ ERROR: Python not found. Install from https://python.org/downloads/
    set /a errors+=1
) else (
    for /f "tokens=2" %%a in ('python --version 2^>^&1') do set python_version=%%a
    echo   ✅ Python found: %python_version%
)

:: Check pip
echo [2/4] Checking pip package manager...
pip --version >nul 2>&1
if %errorlevel% neq 0 (
    echo   ❌ ERROR: pip not found. Reinstall Python with "Add to PATH" checked
    set /a errors+=1
) else (
    echo   ✅ pip package manager available
)

:: Check VS Code
echo [3/4] Checking VS Code installation...
code --version >nul 2>&1
if %errorlevel% neq 0 (
    echo   ⚠️  WARNING: VS Code not in PATH. Install from https://code.visualstudio.com/
    echo      (You can still use Microchip Studio)
) else (
    echo   ✅ VS Code found and accessible
)

:: Check Microchip Studio
echo [4/4] Checking Microchip Studio...
if exist "C:\Program Files (x86)\Atmel\Studio" (
    echo   ✅ Microchip Studio installation detected
) else if exist "C:\Program Files\Microchip\MPLABX" (
    echo   ⚠️  MPLAB X detected instead of Microchip Studio
    echo      Download Microchip Studio: https://microchip.com/studio
) else (
    echo   ⚠️  Microchip Studio not found at standard location
    echo      Download from: https://microchip.com/studio
)

echo.
echo =====================================
if %errors% equ 0 (
    echo   🎉 READY TO GO! Your environment looks good.
    echo   
    echo   Next steps:
    echo   1. Open this folder in VS Code: code .
    echo   2. Navigate to 01_Port\Main.c
    echo   3. Press Ctrl+Shift+P and select "Build Current Project"
) else (
    echo   ❌ SETUP NEEDED: %errors% issue(s) found above
    echo   
    echo   Please install missing tools and run this check again.
    echo   See STUDENT_SETUP_GUIDE.md for detailed instructions.
)
echo =====================================
echo.
pause