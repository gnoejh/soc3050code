@echo off
REM Create New Project - Batch Wrapper
REM Usage: new-project.bat

echo Creating new project...
powershell.exe -ExecutionPolicy Bypass -File "%~dp0tools\cli\cli-new-project.ps1"