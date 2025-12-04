@echo off
REM Build Project Wrapper
REM This allows students to build without PowerShell execution policy issues

REM Get the absolute path to the repository root
set "REPO_ROOT=%~dp0"

if "%1"=="" (
    echo Building current project...
    powershell -ExecutionPolicy Bypass -File "%REPO_ROOT%tools\cli\cli-build-project.ps1" -ProjectDir "%CD%"
) else (
    echo Building project: %1
    powershell -ExecutionPolicy Bypass -File "%REPO_ROOT%tools\cli\cli-build-project.ps1" -ProjectDir "%1"
)
