#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Launch SimulIDE with TCP-configured circuit
    
.DESCRIPTION
    Launches SimulIDE with the shared circuit file (tools/simulide/Simulator110.simu)
    that has been configured for TCP socket communication on localhost:9002.
    
.PARAMETER CircuitFile
    Path to .simu circuit file (default: tools/simulide/Simulator110.simu)
    
.PARAMETER SimulIDEPath
    Path to SimulIDE executable (auto-detected if not specified)
    
.EXAMPLE
    .\cli-launch-simulide-tcp.ps1
    Launch SimulIDE with TCP-configured circuit
#>

param(
    [string]$CircuitFile = "",
    [string]$SimulIDEPath = ""
)

# Compute repository root (this script is in tools/cli, so go up two levels)
$RepoRoot = (Get-Item $PSScriptRoot).Parent.Parent.FullName

# Default circuit file
if ([string]::IsNullOrWhiteSpace($CircuitFile)) {
    $CircuitFile = Join-Path $RepoRoot "tools\simulide\Simulator110.simu"
}

Write-Host "`n[START] Launching SimulIDE with TCP-configured circuit..." -ForegroundColor Cyan
Write-Host "   Circuit: $CircuitFile" -ForegroundColor Gray

# Auto-detect SimulIDE if path not provided
if ([string]::IsNullOrWhiteSpace($SimulIDEPath)) {
    $SimulIDEPath = Join-Path $RepoRoot "tools\simulide\SimulIDE_1.1.0-SR1_Win64\bin\simulide.exe"
    
    if (-not (Test-Path $SimulIDEPath)) {
        # Try older version
        $SimulIDEPath = Join-Path $RepoRoot "tools\simulide\SimulIDE_0.4.15-SR10_Win64\bin\simulide.exe"
    }
    
    if (-not (Test-Path $SimulIDEPath)) {
        Write-Host "[ERROR] Error: SimulIDE executable not found" -ForegroundColor Red
        Write-Host "   Searched: tools\simulide\SimulIDE_*\bin\simulide.exe" -ForegroundColor Gray
        exit 1
    }
}

# Verify circuit file exists
if (-not (Test-Path $CircuitFile)) {
    Write-Host "[ERROR] Error: Circuit file not found: $CircuitFile" -ForegroundColor Red
    exit 1
}

# Launch SimulIDE
Write-Host "   SimulIDE: $SimulIDEPath" -ForegroundColor Gray
Write-Host "`n[INFO] SimulIDE will connect to localhost:9002 (bridged to localhost:9001)" -ForegroundColor Yellow
Write-Host "`n" -ForegroundColor Gray

try {
    & $SimulIDEPath $CircuitFile
    Write-Host "`n[OK] SimulIDE closed" -ForegroundColor Green
}
catch {
    Write-Host "`n[ERROR] Error launching SimulIDE: $_" -ForegroundColor Red
    exit 1
}
