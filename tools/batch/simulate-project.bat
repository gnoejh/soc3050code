@echo off
REM Build and Simulate Project - Batch Wrapper
REM Usage: simulate-project.bat [project-directory]

if "%1"=="" (
    set PROJECT_DIR=%CD%
) else (
    set PROJECT_DIR=%1
)

echo Building and simulating project: %PROJECT_DIR%
powershell.exe -ExecutionPolicy Bypass -File "%~dp0tools\cli\cli-build-simulide.ps1" -ProjectDir "%PROJECT_DIR%"