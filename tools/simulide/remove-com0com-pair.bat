@echo off
REM Remove com0com Port Pair - Must run as Administrator

echo.
echo This will remove the COM4-COM5 virtual port pair from com0com
echo so that COM4 can be used for the TCP bridge.
echo.
echo Checking for Administrator privileges...
echo.

net session >nul 2>&1
if %errorLevel% == 0 (
    echo Running as Administrator - OK
    echo.
    powershell.exe -ExecutionPolicy Bypass -File "%~dp0remove-com0com-pair.ps1"
) else (
    echo ERROR: Not running as Administrator!
    echo.
    echo Please right-click this file and select "Run as Administrator"
    echo.
    pause
)
