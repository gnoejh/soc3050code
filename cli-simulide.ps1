#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Launch SimulIDE with ATmega128 project
    
.DESCRIPTION
    Build current project and launch SimulIDE 1.1.0 SR1 with the compiled HEX file.
    Automatically updates the circuit file to point to the project's HEX file.
    
.PARAMETER ProjectDir
    Path to project directory containing Main.c (default: current directory)
    
.PARAMETER CircuitFile
    Path to .simu circuit file (default: Simulator.simu in workspace root)
    
.PARAMETER BuildFirst
    Build the project before launching SimulIDE (default: true)
    
.PARAMETER SimulIDEPath
    Path to SimulIDE executable (auto-detected if not specified)
    
.EXAMPLE
    .\cli-simulide.ps1
    Launch SimulIDE with current project
    
.EXAMPLE
    .\cli-simulide.ps1 -ProjectDir "projects/Port_Basic"
    Launch SimulIDE with specific project
    
.EXAMPLE
    .\cli-simulide.ps1 -BuildFirst $false
    Launch without rebuilding
#>

param(
    [string]$ProjectDir = $PWD.Path,
    [string]$CircuitFile = "",
    [bool]$BuildFirst = $true,
    [string]$SimulIDEPath = ""
)

# Colors for output
$ErrorColor = "Red"
$SuccessColor = "Green"
$InfoColor = "Cyan"
$WarningColor = "Yellow"

# Get workspace root
$WorkspaceRoot = $PSScriptRoot

Write-Host "`n========================================" -ForegroundColor $InfoColor
Write-Host "  SimulIDE ATmega128 Launcher" -ForegroundColor $InfoColor
Write-Host "========================================`n" -ForegroundColor $InfoColor

# Resolve project directory
if (-not [System.IO.Path]::IsPathRooted($ProjectDir)) {
    $ProjectDir = Join-Path $WorkspaceRoot $ProjectDir
}

if (-not (Test-Path $ProjectDir)) {
    Write-Host "❌ Error: Project directory not found: $ProjectDir" -ForegroundColor $ErrorColor
    exit 1
}

Write-Host "📁 Project: " -NoNewline -ForegroundColor $InfoColor
Write-Host "$ProjectDir" -ForegroundColor White

# Find SimulIDE executable
if ([string]::IsNullOrEmpty($SimulIDEPath)) {
    $PossiblePaths = @(
        Join-Path $WorkspaceRoot "SimulIDE_1.1.0-SR1_Win64\simulide.exe"
        Join-Path $WorkspaceRoot "SimulIDE_1.1.0-SR1_Win64\bin\simulide.exe"
        Join-Path $WorkspaceRoot "SimulIDE\simulide.exe"
        Join-Path $WorkspaceRoot "SimulIDE\bin\simulide.exe"
        "C:\Program Files\SimulIDE\simulide.exe"
        "C:\Program Files\SimulIDE\bin\simulide.exe"
        "C:\Program Files (x86)\SimulIDE\simulide.exe"
        "C:\Program Files (x86)\SimulIDE\bin\simulide.exe"
    )
    
    foreach ($path in $PossiblePaths) {
        if (Test-Path $path) {
            $SimulIDEPath = $path
            break
        }
    }
}

if ([string]::IsNullOrEmpty($SimulIDEPath) -or -not (Test-Path $SimulIDEPath)) {
    Write-Host "❌ Error: SimulIDE executable not found!" -ForegroundColor $ErrorColor
    Write-Host "   Please specify path with -SimulIDEPath parameter" -ForegroundColor $WarningColor
    Write-Host "   or install SimulIDE to default location" -ForegroundColor $WarningColor
    exit 1
}

Write-Host "🔧 SimulIDE: " -NoNewline -ForegroundColor $InfoColor
Write-Host "$SimulIDEPath" -ForegroundColor White

# Build project if requested
if ($BuildFirst) {
    Write-Host "`n🔨 Building project..." -ForegroundColor $InfoColor
    
    $BuildScript = Join-Path $WorkspaceRoot "cli-build-project.ps1"
    
    if (Test-Path $BuildScript) {
        & $BuildScript -ProjectDir $ProjectDir
        
        if ($LASTEXITCODE -ne 0) {
            Write-Host "❌ Build failed!" -ForegroundColor $ErrorColor
            exit 1
        }
    } else {
        Write-Host "⚠️  Warning: Build script not found, skipping build" -ForegroundColor $WarningColor
    }
}

# Find HEX file
$HexFile = Join-Path $ProjectDir "Main.hex"

if (-not (Test-Path $HexFile)) {
    Write-Host "❌ Error: HEX file not found: $HexFile" -ForegroundColor $ErrorColor
    Write-Host "   Try building the project first" -ForegroundColor $WarningColor
    exit 1
}

Write-Host "✅ HEX file: " -NoNewline -ForegroundColor $SuccessColor
Write-Host "$HexFile" -ForegroundColor White

# Determine circuit file
if ([string]::IsNullOrEmpty($CircuitFile)) {
    $CircuitFile = Join-Path $WorkspaceRoot "Simulator.simu"
}

if (-not (Test-Path $CircuitFile)) {
    Write-Host "❌ Error: Circuit file not found: $CircuitFile" -ForegroundColor $ErrorColor
    Write-Host "   Expected: $CircuitFile" -ForegroundColor $WarningColor
    exit 1
}

Write-Host "📐 Circuit: " -NoNewline -ForegroundColor $InfoColor
Write-Host "$CircuitFile" -ForegroundColor White

# Create temporary circuit file with updated HEX path
Write-Host "`n🔄 Updating circuit file..." -ForegroundColor $InfoColor

$TempCircuitFile = Join-Path $env:TEMP "atmega128_temp.simu"

# Function to calculate relative path (compatible with PowerShell 5.1)
function Get-RelativePath {
    param(
        [string]$From,
        [string]$To
    )
    
    $FromUri = New-Object System.Uri($From)
    $ToUri = New-Object System.Uri($To)
    
    $RelativeUri = $FromUri.MakeRelativeUri($ToUri)
    $RelativePath = [System.Uri]::UnescapeDataString($RelativeUri.ToString())
    
    return $RelativePath
}

try {
    # Read circuit file
    $CircuitContent = Get-Content $CircuitFile -Raw
    
    # Use absolute path for SimulIDE (relative paths may not work reliably)
    $AbsoluteHexPath = $HexFile -replace '\\', '/'
    
    Write-Host "   Circuit directory: $CircuitDir" -ForegroundColor $InfoColor
    Write-Host "   HEX file: $HexFile" -ForegroundColor $InfoColor
    Write-Host "   Program path for SimulIDE: " -NoNewline -ForegroundColor $InfoColor
    Write-Host "$AbsoluteHexPath" -ForegroundColor White
    
    # Update Program path in MCU item
    if ($CircuitContent -match 'Program="[^"]*"') {
        $OldPath = $Matches[0]
        $CircuitContent = $CircuitContent -replace 'Program="[^"]*"', "Program=`"$AbsoluteHexPath`""
        Write-Host "   ✅ Updated MCU program path" -ForegroundColor $SuccessColor
        Write-Host "      Old: $OldPath" -ForegroundColor Gray
        Write-Host "      New: Program=`"$AbsoluteHexPath`"" -ForegroundColor Gray
    } else {
        Write-Host "   ⚠️  Warning: Could not find Program attribute" -ForegroundColor $WarningColor
    }
    
    # Enable Auto_Load
    if ($CircuitContent -match 'Auto_Load="false"') {
        $CircuitContent = $CircuitContent -replace 'Auto_Load="false"', 'Auto_Load="true"'
        Write-Host "   ✅ Enabled Auto_Load" -ForegroundColor $SuccessColor
    }
    
    # Save temporary circuit file
    $CircuitContent | Set-Content $TempCircuitFile -Encoding UTF8
    
} catch {
    Write-Host "❌ Error updating circuit file: $_" -ForegroundColor $ErrorColor
    exit 1
}

# Launch SimulIDE
Write-Host "`n🚀 Launching SimulIDE..." -ForegroundColor $SuccessColor
Write-Host "   Press Ctrl+C here to close this window (SimulIDE will keep running)" -ForegroundColor $WarningColor
Write-Host ""

try {
    # Launch SimulIDE with the temporary circuit file
    Start-Process -FilePath $SimulIDEPath -ArgumentList "`"$TempCircuitFile`"" -PassThru
    
    Write-Host "✅ SimulIDE launched successfully!" -ForegroundColor $SuccessColor
    Write-Host "`nTips:" -ForegroundColor $InfoColor
    Write-Host "  • Click the Play button (▶) to start simulation" -ForegroundColor White
    Write-Host "  • Right-click MCU → Load Firmware to reload HEX" -ForegroundColor White
    Write-Host "  • Press Ctrl+R in SimulIDE to reset the MCU" -ForegroundColor White
    Write-Host "  • Check SerialTerm for UART output" -ForegroundColor White
    Write-Host ""
    
} catch {
    Write-Host "❌ Error launching SimulIDE: $_" -ForegroundColor $ErrorColor
    exit 1
}

exit 0
