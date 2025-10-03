@echo off
REM ========================================
REM  Quick SimulIDE Launcher for %PROJECT_NAME%
REM ========================================
REM
REM This batch file provides one-click simulation
REM Double-click to build and simulate this project
REM
REM Usage:
REM   1. Double-click this file
REM   2. Wait for build to complete
REM   3. SimulIDE will launch automatically
REM   4. Click Play button in SimulIDE
REM

setlocal EnableDelayedExpansion

REM Get the directory where this batch file is located
set "PROJECT_DIR=%~dp0"
set "PROJECT_NAME=%~n0"

REM Remove trailing backslash
if "%PROJECT_DIR:~-1%"=="\" set "PROJECT_DIR=%PROJECT_DIR:~0,-1%"

REM Find workspace root (go up until we find cli-simulide.ps1)
set "WORKSPACE_ROOT=%PROJECT_DIR%"
:find_root
if exist "%WORKSPACE_ROOT%\cli-simulide.ps1" goto :found_root
pushd "%WORKSPACE_ROOT%\.."
set "WORKSPACE_ROOT=%CD%"
popd
if "%WORKSPACE_ROOT%"=="%WORKSPACE_ROOT:~0,3%" (
    echo ERROR: Could not find workspace root!
    echo Please ensure cli-simulide.ps1 exists in workspace.
    pause
    exit /b 1
)
goto :find_root

:found_root
echo.
echo ========================================
echo   SimulIDE Quick Launch
echo ========================================
echo.
echo Project: %PROJECT_NAME%
echo Location: %PROJECT_DIR%
echo Workspace: %WORKSPACE_ROOT%
echo.

REM Check if PowerShell is available
where pwsh >nul 2>&1
if %ERRORLEVEL% == 0 (
    set "PS_EXEC=pwsh"
) else (
    where powershell >nul 2>&1
    if %ERRORLEVEL% == 0 (
        set "PS_EXEC=powershell"
    ) else (
        echo ERROR: PowerShell not found!
        echo Please install PowerShell or PowerShell Core
        pause
        exit /b 1
    )
)

echo Using PowerShell: !PS_EXEC!
echo.
echo Building and launching SimulIDE...
echo.

REM Launch SimulIDE with this project
!PS_EXEC! -ExecutionPolicy Bypass -File "%WORKSPACE_ROOT%\cli-simulide.ps1" -ProjectDir "%PROJECT_DIR%"

if %ERRORLEVEL% neq 0 (
    echo.
    echo ERROR: SimulIDE launch failed!
    echo Check the error messages above.
    echo.
    pause
    exit /b 1
)

echo.
echo ========================================
echo   SimulIDE Launched Successfully!
echo ========================================
echo.
echo Next steps:
echo   1. Click Play button in SimulIDE
echo   2. Interact with the simulation
echo   3. Check serial terminal for output
echo.
echo Press any key to close this window...
pause >nul

endlocal
