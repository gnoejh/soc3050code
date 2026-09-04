#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Intelligent SimulIDE launcher that automatically chooses the best version for each project
    
.DESCRIPTION
    This script automatically detects the project type and launches the most compatible SimulIDE version:
    - Graphics_Display: Prefers SimulIDE 0.4.15 (known working)
    - Other projects: Uses SimulIDE 1.1.0 (latest features)
    
    Based on real-world testing showing Graphics_Display GLCD library compatibility issues with SimulIDE 1.1.0
    
.PARAMETER ProjectDir
    Path to project directory (default: current directory)
    
.PARAMETER ForceVersion
    Force a specific SimulIDE version: "0.4.15" or "1.1.0"
    
.EXAMPLE
    .\auto-simulide-launcher.ps1
    Auto-detect project and use best SimulIDE version
    
.EXAMPLE
    .\auto-simulide-launcher.ps1 -ProjectDir "projects/Graphics_Display"
    Launch Graphics_Display with SimulIDE 0.4.15 (recommended)
    
.EXAMPLE
    .\auto-simulide-launcher.ps1 -ForceVersion "1.1.0"
    Force using SimulIDE 1.1.0 regardless of project type
#>

param(
    [string]$ProjectDir = $PWD.Path,
    [ValidateSet("0.4.15", "1.1.0", "auto")]
    [string]$ForceVersion = "auto"
)

# Colors
$InfoColor = "Cyan"
$SuccessColor = "Green"
$WarningColor = "Yellow"
$ErrorColor = "Red"

# Get workspace root
$WorkspaceRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent

Write-Host "`n========================================" -ForegroundColor $InfoColor
Write-Host "  Intelligent SimulIDE Launcher" -ForegroundColor $InfoColor
Write-Host "========================================`n" -ForegroundColor $InfoColor

# Resolve project directory
if (-not [System.IO.Path]::IsPathRooted($ProjectDir)) {
    $ProjectDir = Join-Path $WorkspaceRoot $ProjectDir
}

$ProjectName = Split-Path $ProjectDir -Leaf

# Project-specific compatibility matrix (based on real testing)
$ProjectCompatibility = @{
    "Graphics_Display" = @{
        "PreferredVersion" = "0.4.15"
        "Reason"           = "GLCD library has compatibility issues with SimulIDE 1.1.0"
        "Circuit"          = "Simulator0415.simu"
    }
    "Default"          = @{
        "PreferredVersion" = "1.1.0"
        "Reason"           = "Latest features and improvements"
        "Circuit"          = "Simulator110.simu"
    }
}

# Determine best version
if ($ForceVersion -ne "auto") {
    $SelectedVersion = $ForceVersion
    $SelectionReason = "User forced version"
    Write-Host "🔧 Force using SimulIDE $SelectedVersion" -ForegroundColor $WarningColor
}
elseif ($ProjectCompatibility.ContainsKey($ProjectName)) {
    $ProjectConfig = $ProjectCompatibility[$ProjectName]
    $SelectedVersion = $ProjectConfig.PreferredVersion
    $SelectionReason = $ProjectConfig.Reason
    Write-Host "📁 Project: $ProjectName" -ForegroundColor $InfoColor
    Write-Host "✅ Recommended: SimulIDE $SelectedVersion" -ForegroundColor $SuccessColor
    Write-Host "   Reason: $SelectionReason" -ForegroundColor White
}
else {
    $ProjectConfig = $ProjectCompatibility["Default"]
    $SelectedVersion = $ProjectConfig.PreferredVersion
    $SelectionReason = $ProjectConfig.Reason
    Write-Host "📁 Project: $ProjectName (using default)" -ForegroundColor $InfoColor
    Write-Host "✅ Using: SimulIDE $SelectedVersion" -ForegroundColor $SuccessColor
}

# Build SimulIDE path
$SimulIDEPath = ""
$CircuitFile = ""

if ($SelectedVersion -eq "0.4.15") {
    $SimulIDEPath = Join-Path $WorkspaceRoot "tools\simulide\SimulIDE_0.4.15-SR10_Win64\bin\simulide.exe"
    $CircuitFile = Join-Path $WorkspaceRoot "tools\simulide\Simulator0415.simu"
}
else {
    $SimulIDEPath = Join-Path $WorkspaceRoot "tools\simulide\SimulIDE_1.1.0-SR1_Win64\simulide.exe"
    $CircuitFile = Join-Path $WorkspaceRoot "tools\simulide\Simulator110.simu"
}

# Verify SimulIDE exists
if (-not (Test-Path $SimulIDEPath)) {
    Write-Host "❌ Error: SimulIDE $SelectedVersion not found!" -ForegroundColor $ErrorColor
    Write-Host "   Expected: $SimulIDEPath" -ForegroundColor White
    Write-Host "   Please ensure both SimulIDE versions are installed in tools/simulide/" -ForegroundColor $WarningColor
    exit 1
}

# Verify circuit file exists
if (-not (Test-Path $CircuitFile)) {
    Write-Host "❌ Error: Circuit file not found!" -ForegroundColor $ErrorColor
    Write-Host "   Expected: $CircuitFile" -ForegroundColor White
    exit 1
}

Write-Host "🔧 SimulIDE: $SelectedVersion" -ForegroundColor $InfoColor
Write-Host "⚡ Circuit: $(Split-Path $CircuitFile -Leaf)" -ForegroundColor $InfoColor

# Launch with the appropriate script
$LaunchArgs = @(
    "-ProjectDir", $ProjectDir
    "-CircuitFile", $CircuitFile
    "-SimulIDEPath", $SimulIDEPath
)

$CliScript = Join-Path $PSScriptRoot "cli-simulide.ps1"
& $CliScript @LaunchArgs

Write-Host "`n========================================" -ForegroundColor $InfoColor
Write-Host "  Launcher completed" -ForegroundColor $InfoColor
Write-Host "========================================`n" -ForegroundColor $InfoColor