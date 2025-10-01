#!/usr/bin/env pwsh
# ATmega128 Educational Framework - Python Integration Launcher
# Complete setup and demonstration script

param(
    [string]$Command = "help",
    [string]$Port = "COM3",
    [int]$Duration = 60,
    [string]$File = ""
)

function Show-Banner {
    Write-Host "=" * 70 -ForegroundColor Cyan
    Write-Host "🎓 ATmega128 EDUCATIONAL FRAMEWORK - Python Integration" -ForegroundColor Yellow
    Write-Host "🔄 Assembly → C → Python → IoT Learning Progression" -ForegroundColor Green
    Write-Host "=" * 70 -ForegroundColor Cyan
    Write-Host ""
}

function Show-Help {
    Write-Host "📋 Available Commands:" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "🚀 Quick Start:" -ForegroundColor Green
    Write-Host "   .\run.ps1 setup                    # Install Python dependencies"
    Write-Host "   .\run.ps1 demo                     # Run complete educational demo"
    Write-Host ""
    Write-Host "🔧 Setup & Installation:" -ForegroundColor Green
    Write-Host "   .\run.ps1 setup                    # Install all Python dependencies"
    Write-Host "   .\run.ps1 check                    # Check system status"
    Write-Host ""
    Write-Host "🎮 Educational Demonstrations:" -ForegroundColor Green
    Write-Host "   .\run.ps1 demo [-Port COM3]        # Interactive educational demo"
    Write-Host "   .\run.ps1 monitor [-Duration 60]   # Real-time sensor monitoring"
    Write-Host ""
    Write-Host "📈 Data Analysis:" -ForegroundColor Green
    Write-Host "   .\run.ps1 visualize -File data.csv # Visualize sensor data"
    Write-Host "   .\run.ps1 sample                   # Create sample data & visualize"
    Write-Host ""
    Write-Host "🎯 Prerequisites:" -ForegroundColor Yellow
    Write-Host "   1. ATmega128 programmed with EDUCATIONAL_DEMO"
    Write-Host "   2. Python 3.7+ installed"
    Write-Host "   3. Serial connection on specified port"
    Write-Host ""
}

function Test-PythonInstalled {
    try {
        $pythonVersion = python --version 2>&1
        if ($pythonVersion -match "Python 3\.") {
            Write-Host "✅ Python found: $pythonVersion" -ForegroundColor Green
            return $true
        }
        else {
            Write-Host "❌ Python 3.x not found" -ForegroundColor Red
            return $false
        }
    }
    catch {
        Write-Host "❌ Python not installed or not in PATH" -ForegroundColor Red
        return $false
    }
}

function Install-Dependencies {
    Write-Host "🔧 Installing Python dependencies..." -ForegroundColor Yellow
    Write-Host ""
    
    if (-not (Test-PythonInstalled)) {
        Write-Host "❌ Python installation required first" -ForegroundColor Red
        return $false
    }
    
    try {
        Write-Host "📦 Running setup script..." -ForegroundColor Cyan
        python setup.py
        
        if ($LASTEXITCODE -eq 0) {
            Write-Host ""
            Write-Host "✅ Dependencies installed successfully!" -ForegroundColor Green
            return $true
        }
        else {
            Write-Host "❌ Setup failed" -ForegroundColor Red
            return $false
        }
    }
    catch {
        Write-Host "❌ Error running setup: $_" -ForegroundColor Red
        return $false
    }
}

function Test-System {
    Write-Host "🔍 Checking system status..." -ForegroundColor Yellow
    Write-Host ""
    
    if (-not (Test-PythonInstalled)) {
        return $false
    }
    
    try {
        python main.py check
        return ($LASTEXITCODE -eq 0)
    }
    catch {
        Write-Host "❌ System check failed: $_" -ForegroundColor Red
        return $false
    }
}

function Start-Demo {
    Write-Host "🎮 Starting educational demonstration..." -ForegroundColor Yellow
    Write-Host ""
    Write-Host "⚠️  PREREQUISITES:" -ForegroundColor Red
    Write-Host "   1. ATmega128 connected to $Port" -ForegroundColor White
    Write-Host "   2. EDUCATIONAL_DEMO compiled and programmed" -ForegroundColor White
    Write-Host "   3. Serial terminal applications closed" -ForegroundColor White
    Write-Host ""
    
    $ready = Read-Host "Are you ready to proceed? (y/n)"
    if ($ready -notmatch "^[Yy]") {
        Write-Host "Demo cancelled" -ForegroundColor Yellow
        return
    }
    
    try {
        python main.py demo --port $Port
    }
    catch {
        Write-Host "❌ Demo failed: $_" -ForegroundColor Red
    }
}

function Start-Monitoring {
    Write-Host "🔍 Starting sensor monitoring..." -ForegroundColor Yellow
    Write-Host ""
    Write-Host "📡 Port: $Port" -ForegroundColor Cyan
    Write-Host "⏱️  Duration: $Duration seconds" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "⚠️  PREREQUISITES:" -ForegroundColor Red
    Write-Host "   1. ATmega128 connected to $Port" -ForegroundColor White
    Write-Host "   2. EDUCATIONAL_DEMO compiled and programmed" -ForegroundColor White
    Write-Host "   3. Serial terminal applications closed" -ForegroundColor White
    Write-Host ""
    
    $ready = Read-Host "Start monitoring? (y/n)"
    if ($ready -notmatch "^[Yy]") {
        Write-Host "Monitoring cancelled" -ForegroundColor Yellow
        return
    }
    
    try {
        python main.py monitor --port $Port --duration $Duration
    }
    catch {
        Write-Host "❌ Monitoring failed: $_" -ForegroundColor Red
    }
}

function Start-Visualization {
    if (-not $File) {
        Write-Host "❌ CSV file required for visualization" -ForegroundColor Red
        Write-Host "Usage: .\run.ps1 visualize -File 'sensor_data.csv'" -ForegroundColor Yellow
        return
    }
    
    if (-not (Test-Path $File)) {
        Write-Host "❌ File not found: $File" -ForegroundColor Red
        return
    }
    
    Write-Host "📈 Visualizing data from: $File" -ForegroundColor Yellow
    Write-Host ""
    
    try {
        python main.py visualize $File
    }
    catch {
        Write-Host "❌ Visualization failed: $_" -ForegroundColor Red
    }
}

function Create-SampleVisualization {
    Write-Host "📊 Creating sample data and visualization..." -ForegroundColor Yellow
    Write-Host ""
    
    try {
        # Check if sample file exists
        if (Test-Path "sample_sensor_data.csv") {
            Write-Host "✅ Sample data file found" -ForegroundColor Green
        }
        else {
            Write-Host "📄 Creating sample data..." -ForegroundColor Cyan
            python setup.py  # This creates sample data
        }
        
        Write-Host "📈 Opening visualization..." -ForegroundColor Cyan
        python main.py visualize sample_sensor_data.csv
        
    }
    catch {
        Write-Host "❌ Sample visualization failed: $_" -ForegroundColor Red
    }
}

# Main script execution
Show-Banner

switch ($Command.ToLower()) {
    "help" { 
        Show-Help 
    }
    "setup" { 
        if (Install-Dependencies) {
            Write-Host ""
            Write-Host "🚀 Setup complete! Try: .\run.ps1 demo" -ForegroundColor Green
        }
    }
    "check" { 
        Test-System 
    }
    "demo" { 
        Start-Demo 
    }
    "monitor" { 
        Start-Monitoring 
    }
    "visualize" { 
        Start-Visualization 
    }
    "sample" { 
        Create-SampleVisualization 
    }
    default {
        Write-Host "❌ Unknown command: $Command" -ForegroundColor Red
        Write-Host ""
        Show-Help
    }
}

Write-Host ""
Write-Host "👋 Script complete!" -ForegroundColor Green