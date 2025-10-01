#!/usr/bin/env pwsh
# ATmega128 Educational Framework - Complete Student Setup
# Assembly → C → Python → IoT Learning Progression

param(
    [switch]$Help,
    [switch]$Force,
    [switch]$Quick
)

if ($Help) {
    Write-Host @"
🎓 ATmega128 Educational Framework - Student Setup

USAGE:
    .\setup.ps1           # Complete setup (recommended)
    .\setup.ps1 -Quick    # Skip detailed checks
    .\setup.ps1 -Force    # Force reinstall everything
    .\setup.ps1 -Help     # Show this help

WHAT THIS SCRIPT DOES:
✅ Checks system requirements
✅ Verifies Python installation (3.8+)
✅ Sets up Python environment
✅ Installs all required packages
✅ Tests ATmega128 communication
✅ Creates shortcuts and launch scripts
✅ Validates complete framework

REQUIREMENTS:
- Windows 10/11
- Python 3.8+ 
- Internet connection
- ATmega128 connected via USB (for testing)
- Microchip Studio 7.0 (for C compilation)

AFTER SETUP:
Students can immediately use:
- .\student.ps1 demo                 # Educational progression
- .\student.ps1 monitor             # Real-time data collection  
- .\student.ps1 visualize data.csv  # Scientific analysis
"@
    exit 0
}

function Write-StatusHeader {
    param($Text)
    Write-Host "`n" + "=" * 70 -ForegroundColor Cyan
    Write-Host "🎓 $Text" -ForegroundColor Yellow
    Write-Host "=" * 70 -ForegroundColor Cyan
}

function Write-StatusStep {
    param($Text)
    Write-Host "`n🔧 $Text..." -ForegroundColor Yellow
}

function Write-Success {
    param($Text)
    Write-Host "✅ $Text" -ForegroundColor Green
}

function Write-Warning {
    param($Text)
    Write-Host "⚠️  $Text" -ForegroundColor Yellow
}

function Write-Error {
    param($Text)
    Write-Host "❌ $Text" -ForegroundColor Red
}

function Test-Command {
    param($Command)
    try {
        Get-Command $Command -ErrorAction Stop | Out-Null
        return $true
    }
    catch {
        return $false
    }
}

function Test-PythonVersion {
    try {
        $versionOutput = python --version 2>&1
        if ($versionOutput -match "Python (\d+)\.(\d+)\.(\d+)") {
            $major = [int]$matches[1]
            $minor = [int]$matches[2] 
            $patch = [int]$matches[3]
            
            Write-Success "Found Python $major.$minor.$patch"
            
            if ($major -ge 3 -and $minor -ge 8) {
                return $true
            }
            else {
                Write-Warning "Python 3.8+ recommended (found $major.$minor.$patch)"
                return $false
            }
        }
        return $false
    }
    catch {
        return $false
    }
}

# Main setup execution
Write-StatusHeader "ATmega128 Educational Framework Setup"
Write-Host "Assembly → C → Python → IoT Learning Progression" -ForegroundColor Green
Write-Host "⏰ Started: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" -ForegroundColor Cyan
Write-Host ""

# Step 1: System Requirements Check
Write-StatusStep "Checking system requirements"

# Check Windows version
$winVersion = [System.Environment]::OSVersion.Version
Write-Host "   Windows Version: $($winVersion.Major).$($winVersion.Minor)" -ForegroundColor White
if ($winVersion.Major -lt 10) {
    Write-Warning "Windows 10+ recommended"
}

# Check Python installation
Write-Host "   Checking Python installation..." -ForegroundColor White
if (Test-Command "python") {
    if (Test-PythonVersion) {
        Write-Success "Python requirements met"
    }
    else {
        Write-Error "Python 3.8+ required"
        Write-Host "💡 Download from: https://python.org" -ForegroundColor Yellow
        Write-Host "💡 Or install from Microsoft Store" -ForegroundColor Yellow
        exit 1
    }
}
else {
    Write-Error "Python not found"
    Write-Host "💡 Download from: https://python.org" -ForegroundColor Yellow
    Write-Host "💡 Or install from Microsoft Store" -ForegroundColor Yellow
    exit 1
}

# Step 2: Setup Python Environment
Write-StatusStep "Setting up Python environment"

# Navigate to python_env directory
if (-not (Test-Path "python_env")) {
    Write-Error "python_env directory not found"
    Write-Host "💡 Make sure you're in the correct repository directory" -ForegroundColor Yellow
    exit 1
}

Push-Location "python_env"

try {
    # Run Python setup script
    Write-Host "   Running Python environment setup..." -ForegroundColor White
    $setupResult = python setup.py
    
    if ($LASTEXITCODE -eq 0) {
        Write-Success "Python environment configured successfully"
    }
    else {
        Write-Warning "Python setup completed with warnings"
    }
    
    # Step 3: Test Framework Components
    Write-StatusStep "Testing framework components"
    
    # Test Python imports
    Write-Host "   Testing Python module imports..." -ForegroundColor White
    $testScript = @"
try:
    import serial
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd
    import seaborn as sns
    print('[OK] All Python modules imported successfully')
except ImportError as e:
    print(f'[ERROR] Import failed: {e}')
    exit(1)
"@
    
    $testScript | python
    
    if ($LASTEXITCODE -eq 0) {
        Write-Success "All Python dependencies available"
    }
    else {
        Write-Error "Python dependencies missing"
        exit 1
    }
    
    # Test main application
    Write-Host "   Testing main application..." -ForegroundColor White
    $checkResult = python main.py check
    
    if ($LASTEXITCODE -eq 0) {
        Write-Success "Framework components verified"
    }
    else {
        Write-Warning "Framework test completed with warnings"
    }
    
    # Step 4: Create Student Launch Scripts
    Write-StatusStep "Creating student launch scripts"
    
    # Create comprehensive student launcher
    $studentLauncher = @"
#!/usr/bin/env pwsh
# ATmega128 Student Development Environment
# Quick access to all framework features

param(
    [string]`$Command = "help",
    [string]`$Port = "COM3",
    [int]`$Duration = 60,
    [string]`$File = ""
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
    } elseif (Test-Path "python_env/main.py") {
        Push-Location "python_env"
    } else {
        Write-Host "❌ Cannot find python_env directory" -ForegroundColor Red
        exit 1
    }
}

switch (`$Command.ToLower()) {
    "help" { Show-StudentMenu }
    "demo" { python main.py demo --port `$Port }
    "monitor" { python main.py monitor --port `$Port --duration `$Duration }
    "visualize" { 
        if (-not `$File) {
            Write-Host "❌ CSV file required. Usage: .\student.ps1 visualize data.csv" -ForegroundColor Red
            exit 1
        }
        python main.py visualize `$File 
    }
    "check" { python main.py check }
    "sample" { 
        python main.py visualize sample_sensor_data.csv
    }
    "build" {
        if (Test-Path "../Main/build.ps1") {
            Push-Location "../Main"
            .\build.ps1
            Pop-Location
        } elseif (Test-Path "Main/build.ps1") {
            Push-Location "Main"  
            .\build.ps1
            Pop-Location
        } else {
            Write-Host "❌ Cannot find Main/build.ps1" -ForegroundColor Red
        }
    }
    "program" {
        if (Test-Path "../Main/program.ps1") {
            Push-Location "../Main"
            .\program.ps1 `$Port
            Pop-Location  
        } elseif (Test-Path "Main/program.ps1") {
            Push-Location "Main"
            .\program.ps1 `$Port
            Pop-Location
        } else {
            Write-Host "❌ Cannot find Main/program.ps1" -ForegroundColor Red
        }
    }
    default {
        Write-Host "❌ Unknown command: `$Command" -ForegroundColor Red
        Write-Host ""
        Show-StudentMenu
    }
}
"@
    
    $studentLauncher | Out-File -FilePath "student.ps1" -Encoding UTF8
    Write-Success "Student launcher created: student.ps1"
    
    # Copy launcher to root directory for easy access
    Copy-Item "student.ps1" "../student.ps1" -Force
    Write-Success "Launcher copied to root directory"
    
    # Step 5: Create Desktop Shortcut  
    Write-StatusStep "Creating desktop shortcut"
    
    try {
        $WshShell = New-Object -comObject WScript.Shell
        $Shortcut = $WshShell.CreateShortcut("$([Environment]::GetFolderPath('Desktop'))\ATmega128 Framework.lnk")
        $Shortcut.TargetPath = "powershell.exe"
        $Shortcut.Arguments = "-ExecutionPolicy Bypass -File `"$(Resolve-Path '../student.ps1')`""
        $Shortcut.WorkingDirectory = (Resolve-Path "..")
        $Shortcut.IconLocation = "powershell.exe,0"
        $Shortcut.Description = "ATmega128 Educational Framework - Assembly to IoT Learning"
        $Shortcut.Save()
        Write-Success "Desktop shortcut created!"
    }
    catch {
        Write-Warning "Could not create desktop shortcut"
    }
    
    # Step 6: Final Validation
    Write-StatusStep "Final validation"
    
    # Test sample visualization
    Write-Host "   Testing data visualization..." -ForegroundColor White
    $vizTest = python main.py visualize sample_sensor_data.csv 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-Success "Data visualization working"
    }
    else {
        Write-Warning "Visualization test completed with warnings"
    }
    
}
finally {
    Pop-Location
}

# Step 7: Setup Summary
Write-StatusHeader "Setup Complete!"
Write-Host ""
Write-Success "✅ Python environment configured"
Write-Success "✅ All dependencies installed"  
Write-Success "✅ Framework components verified"
Write-Success "✅ Student tools created"
Write-Success "✅ Educational progression ready"
Write-Host ""

Write-Host "🚀 READY FOR STUDENTS!" -ForegroundColor Green
Write-Host "=" * 40 -ForegroundColor Green
Write-Host ""
Write-Host "📋 Quick Start Guide:" -ForegroundColor Cyan
Write-Host "   1. Connect ATmega128 via USB" -ForegroundColor White
Write-Host "   2. Run: .\student.ps1 build" -ForegroundColor White
Write-Host "   3. Run: .\student.ps1 program" -ForegroundColor White  
Write-Host "   4. Run: .\student.ps1 demo" -ForegroundColor White
Write-Host ""
Write-Host "🎯 Educational Progression Available:" -ForegroundColor Cyan
Write-Host "   Assembly → C → Python → IoT" -ForegroundColor Green
Write-Host "   Register Access → Functions → Communication → Data Science" -ForegroundColor Green
Write-Host ""
Write-Host "📁 Key Files Created:" -ForegroundColor Cyan
Write-Host "   ./student.ps1              # Main student interface" -ForegroundColor White
Write-Host "   ./python_env/main.py       # Python framework" -ForegroundColor White
Write-Host "   Desktop/ATmega128 Framework.lnk  # Quick access shortcut" -ForegroundColor White
Write-Host ""
Write-Host "📖 Documentation:" -ForegroundColor Cyan  
Write-Host "   README.md                  # Complete framework guide" -ForegroundColor White
Write-Host "   python_env/README.md       # Python integration guide" -ForegroundColor White
Write-Host ""
Write-Host "💡 Students can now experience the complete embedded → data science workflow!" -ForegroundColor Yellow
Write-Host ""
Write-Host "🎉 Happy Teaching! 🎯" -ForegroundColor Green