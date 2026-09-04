@echo off
REM Quick Build Wrapper - Calls tools/batch/build-project.bat
REM Usage: build.bat [ProjectDir]

call "%~dp0tools\batch\build-project.bat" %*
