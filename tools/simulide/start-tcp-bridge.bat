@echo off
REM Start TCP to Serial Bridge for SimulIDE
REM This bridges VS Code Serial Monitor (COM4) to SimulIDE (TCP localhost:1234)

echo Starting TCP to Serial Bridge...
echo.
echo Make sure:
echo   1. SimulIDE is running with TCP port enabled
echo   2. VS Code Serial Monitor is closed initially
echo.
pause

REM Try PowerShell version first
powershell.exe -ExecutionPolicy Bypass -File "%~dp0tcp-serial-bridge.ps1"
