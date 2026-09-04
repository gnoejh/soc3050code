@echo off
echo.
echo ============================================
echo   SOC3050 Self-Sufficient Environment
echo ============================================
echo.
echo Verifying that all tools are ready...
echo.

REM Run the PowerShell verification script
powershell.exe -ExecutionPolicy Bypass -File "%~dp0verify-environment.ps1"

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ============================================
    echo   ✅ ENVIRONMENT READY!
    echo ============================================
    echo.
    echo Next steps:
    echo   1. Open VS Code: code .
    echo   2. Navigate to projects/ folder
    echo   3. Start with Port_Basic project
    echo   4. Use Ctrl+Shift+P to run tasks
    echo.
) else (
    echo.
    echo ============================================
    echo   ❌ SETUP ISSUES DETECTED
    echo ============================================
    echo.
    echo Please check the output above for details.
    echo Contact instructor if problems persist.
    echo.
)

pause