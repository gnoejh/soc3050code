#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Open SimulIDE with the official circuit file
    
.DESCRIPTION
    Opens SimulIDE with the official Simulator110.simu circuit file from the tools/simulide directory.
    This ensures students always use the correct circuit file, not random files from user directories.
#>

param(
    [string]$WorkspaceRoot = $PWD.Path
)

Write-Host "🎯 Opening SimulIDE with OFFICIAL circuit file..." -ForegroundColor Cyan

# Path to the official circuit file
$CircuitPath = Join-Path $WorkspaceRoot "tools\simulide\Simulator110.simu"

# Verify circuit file exists
if (-not (Test-Path $CircuitPath)) {
    Write-Host "❌ Official circuit not found: $CircuitPath" -ForegroundColor Red
    Write-Host "   Please ensure tools\simulide\Simulator110.simu exists in the workspace" -ForegroundColor Yellow
    exit 1
}

Write-Host "✅ Circuit file found: $CircuitPath" -ForegroundColor Green

# Try workspace SimulIDE first
$SimulIDEPath = Join-Path $WorkspaceRoot "tools\simulide\SimulIDE_1.1.0-SR1_Win64\simulide.exe"

if (Test-Path $SimulIDEPath) {
    Write-Host "✅ Using workspace SimulIDE: $SimulIDEPath" -ForegroundColor Green
    Write-Host "✅ Loading circuit: $CircuitPath" -ForegroundColor Green
    Start-Process $SimulIDEPath -ArgumentList "`"$CircuitPath`""
    exit 0
}

# Try system installations
Write-Host "⚠️  Workspace SimulIDE not found, trying system installation..." -ForegroundColor Yellow
$SystemPaths = @(
    "C:\Program Files\SimulIDE\simulide.exe",
    "C:\Program Files (x86)\SimulIDE\simulide.exe"
)

foreach ($Path in $SystemPaths) {
    if (Test-Path $Path) {
        Write-Host "✅ Using system SimulIDE: $Path" -ForegroundColor Green
        Write-Host "✅ Loading circuit: $CircuitPath" -ForegroundColor Green
        Start-Process $Path -ArgumentList "`"$CircuitPath`""
        exit 0
    }
}

Write-Host "❌ SimulIDE not found! Install to workspace or Program Files" -ForegroundColor Red
exit 1