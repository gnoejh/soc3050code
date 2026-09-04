# Quick Test Script for Bluetooth/ESP-01 Project
# Automates the testing workflow

param(
    [string]$Mode = "emulator",  # emulator, loopback, or hardware
    [string]$Port = "COM10",
    [int]$Baud = 115200
)

Write-Host "======================================" -ForegroundColor Cyan
Write-Host "  ESP-01 Testing Helper" -ForegroundColor Cyan
Write-Host "======================================" -ForegroundColor Cyan
Write-Host ""

function Test-PythonInstalled {
    try {
        $pythonVersion = & python --version 2>&1
        return $true
    }
    catch {
        return $false
    }
}

function Install-PySerial {
    Write-Host "Installing pyserial..." -ForegroundColor Yellow
    & python -m pip install pyserial
}

switch ($Mode) {
    "emulator" {
        Write-Host "Mode: Python AT Emulator" -ForegroundColor Green
        Write-Host ""
        Write-Host "This will:" -ForegroundColor White
        Write-Host "  1. Check Python installation"
        Write-Host "  2. Start AT command emulator on $Port"
        Write-Host "  3. Respond to commands from ATmega128"
        Write-Host ""
        
        if (-not (Test-PythonInstalled)) {
            Write-Host "ERROR: Python not found!" -ForegroundColor Red
            Write-Host "Please install Python from python.org" -ForegroundColor Yellow
            exit 1
        }
        
        Write-Host "✓ Python found" -ForegroundColor Green
        
        # Check if pyserial is installed
        $pyserialCheck = & python -c "import serial; print('OK')" 2>&1
        if ($pyserialCheck -notmatch "OK") {
            Write-Host "pyserial not installed, installing now..." -ForegroundColor Yellow
            Install-PySerial
        }
        else {
            Write-Host "✓ pyserial installed" -ForegroundColor Green
        }
        
        Write-Host ""
        Write-Host "Starting emulator..." -ForegroundColor Cyan
        Write-Host "Press Ctrl+C to stop" -ForegroundColor Yellow
        Write-Host ""
        
        & python "$PSScriptRoot\test_esp_emulator.py" --port $Port --baud $Baud
    }
    
    "loopback" {
        Write-Host "Mode: Loopback Test" -ForegroundColor Green
        Write-Host ""
        Write-Host "Instructions:" -ForegroundColor White
        Write-Host "  1. Build project: Press Ctrl+Shift+B in VS Code"
        Write-Host "  2. Run simulator: .\tools\simulide\cli-simulide.ps1 -ProjectDir 'projects\Bluethooth'"
        Write-Host "  3. Open serial monitor on $Port @ 9600 baud"
        Write-Host "  4. Type commands - they should echo back"
        Write-Host ""
        Write-Host "This tests UART logic without ESP-01 hardware"
        Write-Host ""
    }
    
    "hardware" {
        Write-Host "Mode: Hardware Test" -ForegroundColor Green
        Write-Host ""
        Write-Host "Checklist:" -ForegroundColor White
        Write-Host "  [ ] ESP-01 powered with 3.3V (≥500mA)"
        Write-Host "  [ ] TX → PE0 (direct)"
        Write-Host "  [ ] RX ← PE1 (with voltage divider!)"
        Write-Host "  [ ] CH_PD pulled HIGH"
        Write-Host "  [ ] Common ground"
        Write-Host ""
        Write-Host "Build and upload:" -ForegroundColor Cyan
        Write-Host "  .\tools\cli\cli-build-project.ps1 -ProjectDir 'projects\Bluethooth' -SourceFile 'Main.c'"
        Write-Host "  .\tools\cli\cli-program-project.ps1 -ProjectDir 'projects\Bluethooth' -Programmer arduino -Port COM3"
        Write-Host ""
        Write-Host "Monitor debug output on $Port @ 9600 baud"
        Write-Host ""
    }
    
    default {
        Write-Host "Unknown mode: $Mode" -ForegroundColor Red
        Write-Host ""
        Write-Host "Usage:" -ForegroundColor White
        Write-Host "  .\test-bluetooth.ps1 -Mode emulator   # Python AT emulator (recommended)"
        Write-Host "  .\test-bluetooth.ps1 -Mode loopback   # Internal loopback test"
        Write-Host "  .\test-bluetooth.ps1 -Mode hardware   # Real ESP-01 hardware"
        Write-Host ""
        Write-Host "Options:" -ForegroundColor White
        Write-Host "  -Port COM10    # Serial port (default: COM10)"
        Write-Host "  -Baud 115200   # Baud rate (default: 115200)"
        Write-Host ""
    }
}
