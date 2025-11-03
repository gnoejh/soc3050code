@echo off
REM Build All Projects - Batch Wrapper
REM Usage: build-all.bat

echo Building all projects...
powershell.exe -ExecutionPolicy Bypass -File "%~dp0tools\cli\cli-build-all.ps1" -TestAll