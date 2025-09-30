# Advanced AVRDUDE Programming Script for ATmega128
# Usage: .\program.ps1 [-ComPort COM3] [-Programmer arduino] [-HexFile Main.hex]

param(
    [string]$ComPort = "",
    [string]$Programmer = "arduino",
    [string]$HexFile = "Main.hex",
    [switch]$ListPorts,
    [switch]$Help
)

if ($Help) {
    Write-Host "🔧 ATmega128 AVRDUDE Programming Script" -ForegroundColor Green
    Write-Host "=====================================" -ForegroundColor Green
    Write-Host ""
    Write-Host "Usage:" -ForegroundColor Yellow
    Write-Host "  .\program.ps1                    # Use defaults (arduino, auto-detect port)"
    Write-Host "  .\program.ps1 -ComPort COM5      # Specify COM port"
    Write-Host "  .\program.ps1 -Programmer usbasp # Use different programmer"
    Write-Host "  .\program.ps1 -ListPorts         # List available COM ports"
    Write-Host ""
    Write-Host "Common Programmers:" -ForegroundColor Cyan
    Write-Host "  arduino    - Arduino as ISP"
    Write-Host "  usbasp     - USBasp programmer"
    Write-Host "  avrisp     - AVRISP mkII"
    Write-Host "  stk500v2   - STK500 v2"
    Write-Host "  dragon_isp - AVR Dragon"
    exit 0
}

$AVRDUDE = "C:\Program Files (x86)\AVRDUDESS\avrdude.exe"
$MCU = "m128"

Write-Host "🔧 ATmega128 AVRDUDE Programming" -ForegroundColor Green
Write-Host "================================" -ForegroundColor Green

# Check if AVRDUDE exists
if (-not (Test-Path $AVRDUDE)) {
    Write-Host "❌ AVRDUDE not found at: $AVRDUDE" -ForegroundColor Red
    Write-Host "💡 Please install AVRDUDESS or update the path" -ForegroundColor Yellow
    exit 1
}

# List ports if requested
if ($ListPorts) {
    Write-Host "📡 Available COM ports:" -ForegroundColor Yellow
    $ports = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($ports.Count -eq 0) {
        Write-Host "  No COM ports found" -ForegroundColor Gray
    } else {
        $ports | ForEach-Object { Write-Host "  $_" -ForegroundColor White }
    }
    exit 0
}

# Check if HEX file exists
if (-not (Test-Path $HexFile)) {
    Write-Host "❌ HEX file not found: $HexFile" -ForegroundColor Red
    Write-Host "💡 Run build.bat first to generate the HEX file" -ForegroundColor Yellow
    exit 1
}

# Auto-detect COM port if not specified
if ($ComPort -eq "") {
    Write-Host "🔍 Auto-detecting COM ports..." -ForegroundColor Cyan
    $availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
    
    if ($availablePorts.Count -eq 0) {
        Write-Host "❌ No COM ports found!" -ForegroundColor Red
        Write-Host "💡 Connect your programmer and try again" -ForegroundColor Yellow
        exit 1
    }
    
    if ($availablePorts.Count -eq 1) {
        $ComPort = $availablePorts[0]
        Write-Host "✅ Using COM port: $ComPort" -ForegroundColor Green
    } else {
        Write-Host "📡 Multiple COM ports found:" -ForegroundColor Yellow
        for ($i = 0; $i -lt $availablePorts.Count; $i++) {
            Write-Host "  [$($i+1)] $($availablePorts[$i])" -ForegroundColor White
        }
        
        $selection = Read-Host "Select COM port (1-$($availablePorts.Count)) or press Enter for $($availablePorts[0])"
        
        if ($selection -eq "") {
            $ComPort = $availablePorts[0]
        } elseif ($selection -match '^\d+$' -and [int]$selection -le $availablePorts.Count -and [int]$selection -gt 0) {
            $ComPort = $availablePorts[[int]$selection - 1]
        } else {
            Write-Host "❌ Invalid selection" -ForegroundColor Red
            exit 1
        }
    }
}

# Display programming info
Write-Host ""
Write-Host "📋 Programming Configuration:" -ForegroundColor Cyan
Write-Host "  HEX File:    $HexFile" -ForegroundColor White
Write-Host "  MCU:         $MCU" -ForegroundColor White  
Write-Host "  Programmer:  $Programmer" -ForegroundColor White
Write-Host "  COM Port:    $ComPort" -ForegroundColor White
Write-Host ""

# Build AVRDUDE command
$avrdudeArgs = @(
    "-c", $Programmer,
    "-p", $MCU,
    "-P", $ComPort,
    "-U", "flash:w:`"$HexFile`":a"
)

Write-Host "🚀 Starting programming..." -ForegroundColor Yellow
Write-Host "Command: avrdude.exe $($avrdudeArgs -join ' ')" -ForegroundColor Gray
Write-Host ""

# Execute AVRDUDE
try {
    $result = & $AVRDUDE @avrdudeArgs
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host ""
        Write-Host "✅ Programming completed successfully!" -ForegroundColor Green
        Write-Host "🎯 ATmega128 is now running your program" -ForegroundColor Cyan
        
        # Get file size info
        $hexSize = (Get-Item $HexFile).Length
        Write-Host "📊 HEX file size: $hexSize bytes" -ForegroundColor White
    } else {
        Write-Host ""
        Write-Host "❌ Programming failed!" -ForegroundColor Red
        Write-Host ""
        Write-Host "🔧 Troubleshooting tips:" -ForegroundColor Yellow
        Write-Host "  • Check COM port: $ComPort" -ForegroundColor Gray
        Write-Host "  • Verify programmer connection" -ForegroundColor Gray
        Write-Host "  • Ensure ATmega128 is powered" -ForegroundColor Gray
        Write-Host "  • Try different programmer: -Programmer usbasp" -ForegroundColor Gray
        Write-Host "  • List ports: .\program.ps1 -ListPorts" -ForegroundColor Gray
    }
} catch {
    Write-Host "❌ Error executing AVRDUDE: $($_.Exception.Message)" -ForegroundColor Red
}