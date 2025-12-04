#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Launch SimulIDE with fresh reload (closes existing instance first)
    
.DESCRIPTION
    Closes any running SimulIDE instances, builds project, and launches SimulIDE fresh.
    This ensures the new firmware is loaded automatically.
    
.PARAMETER ProjectDir
    Path to project directory containing Main.c (default: current directory)
    
.EXAMPLE
    .\cli-simulide-fresh.ps1
    Close SimulIDE, build, and launch with fresh firmware
    
.EXAMPLE
    .\cli-simulide-fresh.ps1 -ProjectDir "projects/Timer_Programming"
    Fresh launch for specific project
#>

param(
    [string]$ProjectDir = $PWD.Path
)

$ErrorColor = "Red"
$SuccessColor = "Green"
$InfoColor = "Cyan"
$WarningColor = "Yellow"

Write-Host "`n========================================" -ForegroundColor $InfoColor
Write-Host "  SimulIDE Fresh Launcher" -ForegroundColor $InfoColor
Write-Host "========================================`n" -ForegroundColor $InfoColor

# Close any running SimulIDE instances
Write-Host "[CLEANUP] Checking for running SimulIDE instances..." -ForegroundColor $InfoColor
$RunningSimulIDE = Get-Process -Name "simulide" -ErrorAction SilentlyContinue

if ($RunningSimulIDE) {
    Write-Host "   Found $($RunningSimulIDE.Count) running instance(s)" -ForegroundColor $WarningColor
    Write-Host "   Closing SimulIDE..." -ForegroundColor $InfoColor
    
    $RunningSimulIDE | Stop-Process -Force -ErrorAction SilentlyContinue
    Start-Sleep -Milliseconds 500
    
    Write-Host "   [OK] SimulIDE closed" -ForegroundColor $SuccessColor
}
else {
    Write-Host "   No running instances found" -ForegroundColor $InfoColor
}

Write-Host ""

# Get workspace root
$WorkspaceRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent

# Call the main SimulIDE script
$MainScript = Join-Path $PSScriptRoot "cli-simulide.ps1"

if (Test-Path $MainScript) {
    Write-Host "[LAUNCH] Starting SimulIDE with fresh firmware..." -ForegroundColor $InfoColor
    Write-Host ""
    
    # Call main script with parameters
    & $MainScript -ProjectDir $ProjectDir -BuildFirst $true
}
else {
    Write-Host "[ERROR] Main SimulIDE script not found: $MainScript" -ForegroundColor $ErrorColor
    exit 1
}
