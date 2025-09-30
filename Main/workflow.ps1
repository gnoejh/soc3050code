# Complete Build, Program, and Monitor Workflow for ATmega128
# This script builds, programs, and opens serial monitor automatically

param(
    [string]$ComPort = "",
    [switch]$SkipBuild,
    [switch]$SkipProgram
)

Write-Host "🚀 ATmega128 Complete Workflow" -ForegroundColor Green
Write-Host "==============================" -ForegroundColor Green

# Step 1: Build (unless skipped)
if (-not $SkipBuild) {
    Write-Host "🔧 Step 1: Building project..." -ForegroundColor Yellow
    .\build.bat
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "❌ Build failed! Cannot continue." -ForegroundColor Red
        exit 1
    }
    Write-Host "✅ Build successful!" -ForegroundColor Green
    Write-Host ""
}

# Step 2: Program (unless skipped)  
if (-not $SkipProgram) {
    Write-Host "📡 Step 2: Programming ATmega128..." -ForegroundColor Yellow
    Write-Host "⚡ Using AVRDUDE direct programming..." -ForegroundColor Cyan
    
    # Use the AVRDUDE programming script
    .\program.ps1 -ComPort $ComPort
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✅ Programming complete!" -ForegroundColor Green
    } else {
        Write-Host "❌ Programming failed!" -ForegroundColor Red
        $skipSerial = Read-Host "Continue to serial monitor anyway? (y/N)"
        if ($skipSerial -ne "y" -and $skipSerial -ne "Y") {
            exit 1
        }
    }
    Write-Host ""
}

# Step 3: Serial Monitor
Write-Host "📺 Step 3: Starting Serial Monitor..." -ForegroundColor Yellow

if ($ComPort -eq "") {
    Write-Host "🔍 Auto-detecting COM ports..." -ForegroundColor Cyan
    $availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
    
    if ($availablePorts.Count -eq 0) {
        Write-Host "❌ No COM ports found!" -ForegroundColor Red
        exit 1
    }
    
    Write-Host "Available COM ports:" -ForegroundColor White
    for ($i = 0; $i -lt $availablePorts.Count; $i++) {
        Write-Host "  [$($i+1)] $($availablePorts[$i])" -ForegroundColor White
    }
    
    $selection = Read-Host "Select COM port (1-$($availablePorts.Count)) or press Enter for $($availablePorts[0])"
    
    if ($selection -eq "") {
        $ComPort = $availablePorts[0]
    } elseif ($selection -match '^\d+$' -and [int]$selection -le $availablePorts.Count -and [int]$selection -gt 0) {
        $ComPort = $availablePorts[[int]$selection - 1]
    } else {
        $ComPort = $selection.ToUpper()
    }
}

Write-Host "🔗 Starting serial monitor on $ComPort at 9600 baud..." -ForegroundColor Green
Write-Host "📝 Type 'exit' to quit the serial monitor" -ForegroundColor Yellow
Write-Host "========================================" -ForegroundColor Gray

# Start the serial monitor
.\serial.ps1 -Port $ComPort -BaudRate 9600