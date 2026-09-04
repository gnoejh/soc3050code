@echo off
REM ============================================================================
REM  Shared simulator launcher - SOC3050 2026 AVR edition (SimulIDE 1.1.0-SR2)
REM
REM  Called by each lesson's simulate.bat.  Builds the lesson if needed, then
REM  drops a copy of the shared master board into the lesson folder and opens
REM  it in SimulIDE 1.1.0-SR2.
REM
REM  Why a copy: the board is maintained in exactly one place
REM  (projects2026_avr\Simulator.simu).  SimulIDE resolves the MCU's
REM  Program="Main.hex" relative to the circuit file, so a copy sitting next to
REM  the lesson's own Main.hex loads that lesson's firmware and nothing else.
REM  The copy is generated output and is git-ignored.
REM ============================================================================
setlocal

set "HERE=%~dp0"
set "ROOT=%HERE%..\.."
set "MASTER=%HERE%..\Simulator.simu"
set "SIMULIDE=%ROOT%\tools\simulide110sr2\SimulIDE_1.1.0-SR2_Win64\simulide.exe"

if not exist "%SIMULIDE%" (
    echo ERROR: SimulIDE 1.1.0-SR2 not found at
    echo        "%SIMULIDE%"
    exit /b 1
)
if not exist "%MASTER%" (
    echo ERROR: shared master board not found at "%MASTER%"
    exit /b 1
)

if not exist "Main.hex" (
    echo Main.hex missing - building first...
    call build.bat || exit /b 1
)

copy /Y "%MASTER%" "Board.simu" >nul
if errorlevel 1 (
    echo ERROR: could not place the board file in this folder.
    exit /b 1
)

echo Launching SimulIDE 1.1.0-SR2 with Board.simu ...
echo   Press the Play button to start, then Power to run the MCU.
start "" "%SIMULIDE%" "%CD%\Board.simu"
exit /b 0
