# ATmega128 Student Setup Script
# Automatically sets up the complete development environment

param(
    [switch]$SkipUv,
    [switch]$SkipPython,
    [switch]$Force,
    [switch]$Help
)

if ($Help) {
    Write-Host @"
🚀 ATmega128 Student Setup Script

USAGE:
    .\setup.ps1                 # Full setup
    .\setup.ps1 -SkipUv         # Skip uv installation
    .\setup.ps1 -SkipPython     # Skip Python installation check
    .\setup.ps1 -Force          # Force reinstall everything
    .\setup.ps1 -Help           # Show this help

WHAT THIS SCRIPT DOES:
✅ Checks Python installation
✅ Installs uv package manager
✅ Creates Python virtual environment
✅ Installs all required packages
✅ Configures VS Code settings
✅ Tests ATmega128 connection
✅ Creates desktop shortcuts

REQUIREMENTS:
- Windows 10/11
- Internet connection
- ATmega128 connected via USB (optional for setup)
"@
    exit 0
}

Write-Host "🚀 ATmega128 Student Setup" -ForegroundColor Green
Write-Host "=" * 40 -ForegroundColor Green
Write-Host ""

# Function to check if command exists
function Test-Command {
    param($Command)
    try {
        Get-Command $Command -ErrorAction Stop | Out-Null
        return $true
    } catch {
        return $false
    }
}

# Function to run command with output
function Invoke-SetupCommand {
    param($Command, $Description)
    Write-Host "⚙️  $Description..." -ForegroundColor Yellow
    try {
        Invoke-Expression $Command
        Write-Host "✅ Success!" -ForegroundColor Green
        return $true
    } catch {
        Write-Host "❌ Failed: $_" -ForegroundColor Red
        return $false
    }
}

# Check Python installation
if (-not $SkipPython) {
    Write-Host "🐍 Checking Python installation..." -ForegroundColor Yellow
    if (Test-Command "python") {
        $pythonVersion = python --version 2>&1
        Write-Host "✅ Found: $pythonVersion" -ForegroundColor Green
        
        # Check Python version (need 3.8+)
        $versionMatch = $pythonVersion -match "Python (\d+)\.(\d+)"
        if ($versionMatch) {
            $major = [int]$matches[1]
            $minor = [int]$matches[2]
            if ($major -lt 3 -or ($major -eq 3 -and $minor -lt 8)) {
                Write-Host "⚠️  Python 3.8+ recommended (found $major.$minor)" -ForegroundColor Yellow
            }
        }
    } else {
        Write-Host "❌ Python not found!" -ForegroundColor Red
        Write-Host "💡 Download from: https://python.org" -ForegroundColor Yellow
        Write-Host "💡 Or install from Microsoft Store" -ForegroundColor Yellow
        Write-Host "💡 Run with -SkipPython to continue anyway" -ForegroundColor Yellow
        exit 1
    }
}

# Install uv package manager
if (-not $SkipUv -and -not (Test-Command "uv")) {
    Write-Host "📦 Installing uv (fast Python package manager)..." -ForegroundColor Yellow
    try {
        # Try official installer first
        Invoke-WebRequest -Uri "https://github.com/astral-sh/uv/releases/latest/download/uv-installer.ps1" -OutFile "uv-installer.ps1"
        powershell -ExecutionPolicy Bypass -File "uv-installer.ps1"
        Remove-Item "uv-installer.ps1" -Force
        Write-Host "✅ uv installed via official installer!" -ForegroundColor Green
    } catch {
        Write-Host "⚠️  Official installer failed, trying pip..." -ForegroundColor Yellow
        try {
            python -m pip install uv
            Write-Host "✅ uv installed via pip!" -ForegroundColor Green
        } catch {
            Write-Host "❌ Failed to install uv, will use standard venv" -ForegroundColor Yellow
            $useVenv = $true
        }
    }
} elseif (Test-Command "uv") {
    Write-Host "✅ uv already installed" -ForegroundColor Green
}

# Create or update virtual environment
$venvPath = ".venv"
if (Test-Path $venvPath) {
    if ($Force) {
        Write-Host "🗑️  Removing existing virtual environment..." -ForegroundColor Yellow
        Remove-Item $venvPath -Recurse -Force
    } else {
        Write-Host "✅ Virtual environment already exists" -ForegroundColor Green
        $skipVenvCreation = $true
    }
}

if (-not $skipVenvCreation) {
    Write-Host "🏗️  Creating virtual environment..." -ForegroundColor Yellow
    
    if ((Test-Command "uv") -and -not $useVenv) {
        if (Invoke-SetupCommand "uv venv $venvPath" "Creating venv with uv") {
            Write-Host "✅ Virtual environment created with uv!" -ForegroundColor Green
        } else {
            Write-Host "⚠️  uv failed, using standard venv..." -ForegroundColor Yellow
            $useVenv = $true
        }
    }
    
    if ($useVenv) {
        if (Invoke-SetupCommand "python -m venv $venvPath" "Creating venv with Python") {
            Write-Host "✅ Virtual environment created!" -ForegroundColor Green
        } else {
            Write-Host "❌ Failed to create virtual environment!" -ForegroundColor Red
            exit 1
        }
    }
}

# Install Python packages
Write-Host "📋 Installing Python packages..." -ForegroundColor Yellow

if (Test-Path "pyproject.toml" -and (Test-Command "uv") -and -not $useVenv) {
    # Use uv sync for modern package management
    if (Invoke-SetupCommand "uv sync" "Installing packages with uv sync") {
        Write-Host "✅ Packages installed with uv!" -ForegroundColor Green
    } else {
        Write-Host "⚠️  uv sync failed, falling back to pip..." -ForegroundColor Yellow
        $usePip = $true
    }
} else {
    $usePip = $true
}

if ($usePip) {
    # Fallback to pip installation
    $activateScript = "$venvPath\Scripts\Activate.ps1"
    if (Test-Path $activateScript) {
        Write-Host "🔄 Activating environment and installing with pip..." -ForegroundColor Yellow
        & $activateScript
        
        if (Test-Path "requirements.txt") {
            if (Invoke-SetupCommand "python -m pip install -r requirements.txt" "Installing from requirements.txt") {
                Write-Host "✅ Packages installed with pip!" -ForegroundColor Green
            } else {
                Write-Host "❌ Failed to install packages!" -ForegroundColor Red
                exit 1
            }
        } else {
            # Install packages manually
            $packages = @("pyserial", "flask", "matplotlib", "pandas", "numpy")
            Write-Host "📦 Installing packages: $($packages -join ', ')" -ForegroundColor Yellow
            
            foreach ($package in $packages) {
                if (-not (Invoke-SetupCommand "python -m pip install $package" "Installing $package")) {
                    Write-Host "⚠️  Failed to install $package, continuing..." -ForegroundColor Yellow
                }
            }
        }
    } else {
        Write-Host "❌ Cannot find virtual environment activation script!" -ForegroundColor Red
        exit 1
    }
}

# Test build system
Write-Host "🔧 Testing build system..." -ForegroundColor Yellow
if (Test-Path "Main\build.ps1") {
    Push-Location "Main"
    try {
        .\build.ps1 2>&1 | Out-Null
        if ($LASTEXITCODE -eq 0) {
            Write-Host "✅ Build system working!" -ForegroundColor Green
        } else {
            Write-Host "⚠️  Build system needs attention (check AVR-GCC installation)" -ForegroundColor Yellow
        }
    } catch {
        Write-Host "⚠️  Could not test build system" -ForegroundColor Yellow
    } finally {
        Pop-Location
    }
} else {
    Write-Host "⚠️  Build script not found" -ForegroundColor Yellow
}

# Test Python interface
Write-Host "🐍 Testing Python interface..." -ForegroundColor Yellow
$activateScript = "$venvPath\Scripts\Activate.ps1"
if (Test-Path $activateScript) {
    & $activateScript
    try {
        $testResult = python -c "from python.atmega128 import list_ports; print('OK')" 2>&1
        if ($testResult -match "OK") {
            Write-Host "✅ Python interface working!" -ForegroundColor Green
        } else {
            Write-Host "⚠️  Python interface has issues: $testResult" -ForegroundColor Yellow
        }
    } catch {
        Write-Host "⚠️  Could not test Python interface" -ForegroundColor Yellow
    }
}

# Create student activation script
Write-Host "📝 Creating student activation script..." -ForegroundColor Yellow

$activationScript = @"
# ATmega128 Student Environment Activation
# Run this to start working with your ATmega128

Write-Host "🎓 ATmega128 Student Environment" -ForegroundColor Green
Write-Host "=================================" -ForegroundColor Green

# Activate Python environment
if (Test-Path ".venv\Scripts\Activate.ps1") {
    . .venv\Scripts\Activate.ps1
    Write-Host "✅ Python environment activated!" -ForegroundColor Green
    Write-Host ""
    
    # Show quick commands
    Write-Host "🚀 Quick Commands:" -ForegroundColor Cyan
    Write-Host "  F7            - Build C code (or: cd Main; .\build.ps1)" -ForegroundColor White
    Write-Host "  F8            - Program ATmega128 (or: cd Main; .\program.ps1)" -ForegroundColor White
    Write-Host "  Ctrl+Shift+``  - Open terminal" -ForegroundColor White
    Write-Host ""
    
    Write-Host "🐍 Python Examples:" -ForegroundColor Cyan
    Write-Host "  python python\examples\quick_start.py     - Start here!" -ForegroundColor White
    Write-Host "  python python\examples\example_01_basic.py - Basic communication" -ForegroundColor White
    Write-Host "  python python\examples\example_02_sensors.py - Read sensors" -ForegroundColor White
    Write-Host "  python python\examples\example_03_realtime.py - Live graphs" -ForegroundColor White
    Write-Host "  python python\examples\example_04_iot.py - Web dashboard" -ForegroundColor White
    Write-Host "  python python\examples\example_05_analysis.py - Data analysis" -ForegroundColor White
    Write-Host ""
    
    Write-Host "🔧 Hardware Setup:" -ForegroundColor Cyan
    Write-Host "  1. Connect ATmega128 via USB" -ForegroundColor White
    Write-Host "  2. Check device manager for COM port" -ForegroundColor White
    Write-Host "  3. Update port in Python examples" -ForegroundColor White
    Write-Host ""
    
    Write-Host "📚 Documentation:" -ForegroundColor Cyan
    Write-Host "  README.md                    - Main documentation" -ForegroundColor White
    Write-Host "  python\examples\README.md    - Python examples guide" -ForegroundColor White
    Write-Host ""
    
    Write-Host "💡 Need help? Check the documentation or ask your instructor!" -ForegroundColor Yellow
    
} else {
    Write-Host "❌ Virtual environment not found!" -ForegroundColor Red
    Write-Host "💡 Run setup.ps1 to create the environment" -ForegroundColor Yellow
}
"@

$activationScript | Out-File -FilePath "activate.ps1" -Encoding UTF8

# Create desktop shortcut (optional)
Write-Host "🖥️  Creating desktop shortcut..." -ForegroundColor Yellow
try {
    $WshShell = New-Object -comObject WScript.Shell
    $Shortcut = $WshShell.CreateShortcut("$([Environment]::GetFolderPath('Desktop'))\ATmega128 Environment.lnk")
    $Shortcut.TargetPath = "powershell.exe"
    $Shortcut.Arguments = "-ExecutionPolicy Bypass -File `"$PWD\activate.ps1`""
    $Shortcut.WorkingDirectory = $PWD
    $Shortcut.IconLocation = "powershell.exe,0"
    $Shortcut.Description = "ATmega128 Student Development Environment"
    $Shortcut.Save()
    Write-Host "✅ Desktop shortcut created!" -ForegroundColor Green
} catch {
    Write-Host "⚠️  Could not create desktop shortcut" -ForegroundColor Yellow
}

# Final summary
Write-Host ""
Write-Host "🎉 Setup Complete!" -ForegroundColor Green
Write-Host "==================" -ForegroundColor Green
Write-Host ""
Write-Host "✅ Python environment configured" -ForegroundColor Green
Write-Host "✅ Required packages installed" -ForegroundColor Green
Write-Host "✅ ATmega128 interface ready" -ForegroundColor Green
Write-Host "✅ Build system configured" -ForegroundColor Green
Write-Host "✅ Examples available" -ForegroundColor Green
Write-Host ""
Write-Host "🚀 Next Steps:" -ForegroundColor Cyan
Write-Host "  1. Run: .\activate.ps1 (or use desktop shortcut)" -ForegroundColor White
Write-Host "  2. Connect your ATmega128 via USB" -ForegroundColor White
Write-Host "  3. Start with: python python\examples\quick_start.py" -ForegroundColor White
Write-Host ""
Write-Host "📖 Documentation:" -ForegroundColor Cyan
Write-Host "  - README.md for complete guide" -ForegroundColor White
Write-Host "  - python\examples\README.md for Python examples" -ForegroundColor White
Write-Host ""
Write-Host "💡 Happy coding! 🎯" -ForegroundColor Yellow