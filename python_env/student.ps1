#!/usr/bin/env pwsh
# ATmega128 Student Development Environment
# Quick access to all framework features

param(
    [Parameter(Position = 0)]
    [string]$Command = "help",
    [Parameter(Position = 1)]
    [string]$SecondArg = "",
    [string]$Port = "COM3",
    [int]$Duration = 60,
    [string]$File = ""
)

function Show-StudentMenu {
    Write-Host "🎓 ATmega128 Educational Framework" -ForegroundColor Green
    Write-Host "Assembly → C → Python → IoT Learning" -ForegroundColor Cyan
    Write-Host "=" * 40 -ForegroundColor Green
    Write-Host ""
    Write-Host "📋 Available Commands:" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "🎮 Educational Demonstrations:" -ForegroundColor Green
    Write-Host "   .\student.ps1 demo                    # Complete learning progression"
    Write-Host "   .\student.ps1 monitor -Duration 120   # Real-time sensor monitoring"
    Write-Host "   .\student.ps1 visualize data.csv      # Data analysis & visualization"
    Write-Host ""
    Write-Host "🔧 Development Tools:" -ForegroundColor Green  
    Write-Host "   .\student.ps1 check                   # System status"
    Write-Host "   .\student.ps1 sample                  # Create sample data & plots"
    Write-Host "   .\student.ps1 build                   # Build C code (from Main/)"
    Write-Host "   .\student.ps1 program                 # Program ATmega128"
    Write-Host ""
    Write-Host "📚 Learning Path:" -ForegroundColor Cyan
    Write-Host "   1. Connect ATmega128 via USB" -ForegroundColor White
    Write-Host "   2. Run: .\student.ps1 build" -ForegroundColor White  
    Write-Host "   3. Run: .\student.ps1 program" -ForegroundColor White
    Write-Host "   4. Run: .\student.ps1 demo" -ForegroundColor White
    Write-Host "   5. Explore: monitor, visualize, etc." -ForegroundColor White
    Write-Host ""
    Write-Host "💡 Need help? Check README.md or ask your instructor!" -ForegroundColor Yellow
}

# Navigate to python_env if not already there
if (-not (Test-Path "main.py")) {
    if (Test-Path "../python_env/main.py") {
        Push-Location "../python_env"
    }
    elseif (Test-Path "python_env/main.py") {
        Push-Location "python_env"
    }
    else {
        Write-Host "❌ Cannot find python_env directory" -ForegroundColor Red
        exit 1
    }
}

switch ($Command.ToLower()) {
    "help" { Show-StudentMenu }
    "demo" { python main.py demo --port $Port }
    "monitor" { python main.py monitor --port $Port --duration $Duration }
    "visualize" { 
        # Use either -File parameter or second positional argument
        $targetFile = if ($File) { $File } else { $SecondArg }
        
        if (-not $targetFile) {
            Write-Host "❌ CSV file required. Usage: .\student.ps1 visualize data.csv" -ForegroundColor Red
            exit 1
        }
        
        if (-not (Test-Path $targetFile)) {
            Write-Host "❌ File not found: $targetFile" -ForegroundColor Red
            exit 1
        }
        
        python main.py visualize $targetFile 
    }
    "check" { python main.py check }
    "sample" { 
        python main.py visualize data.csv
    }
    "build" {
        if (Test-Path "../Main/build.ps1") {
            Push-Location "../Main"
            .\build.ps1
            Pop-Location
        }
        elseif (Test-Path "Main/build.ps1") {
            Push-Location "Main"  
            .\build.ps1
            Pop-Location
        }
        else {
            Write-Host "❌ Cannot find Main/build.ps1" -ForegroundColor Red
        }
    }
    "program" {
        if (Test-Path "../Main/program.ps1") {
            Push-Location "../Main"
            .\program.ps1 $Port
            Pop-Location  
        }
        elseif (Test-Path "Main/program.ps1") {
            Push-Location "Main"
            .\program.ps1 $Port
            Pop-Location
        }
        else {
            Write-Host "❌ Cannot find Main/program.ps1" -ForegroundColor Red
        }
    }
    default {
        Write-Host "❌ Unknown command: $Command" -ForegroundColor Red
        Write-Host ""
        Show-StudentMenu
    }
}
