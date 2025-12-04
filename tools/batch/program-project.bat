@echo off
REM Program Current Project - Batch Wrapper
REM Usage: program-project.bat [project-directory] [programmer] [port]

if "%1"=="" (
    set PROJECT_DIR=%CD%
) else (
    set PROJECT_DIR=%1
)

if "%2"=="" (
    set PROGRAMMER=arduino
) else (
    set PROGRAMMER=%2
)

if "%3"=="" (
    set PORT=COM3
) else (
    set PORT=%3
)

echo Programming project: %PROJECT_DIR%
echo Using programmer: %PROGRAMMER% on port: %PORT%
powershell.exe -ExecutionPolicy Bypass -File "%~dp0tools\cli\cli-program-project.ps1" -ProjectDir "%PROJECT_DIR%" -Programmer "%PROGRAMMER%" -Port "%PORT%"