#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Launch SimulIDE with ATmega128 project
    
.DESCRIPTION
    Build current project and launch SimulIDE 1.1.0 SR1 with the compiled HEX file.
    Automatically updates the circuit file to point to the project's HEX file.
    Works in various environments: portable SimulIDE, installed versions, different Windows versions.
    
.PARAMETER ProjectDir
    Path to project directory containing Main.c (default: current directory)
    Can be absolute or relative path
    
.PARAMETER CircuitFile
    Path to .simu circuit file (default: auto-detect Simulator.simu)
    
.PARAMETER BuildFirst
    Build the project before launching SimulIDE (default: true)
    
.PARAMETER SimulIDEPath
    Path to SimulIDE executable (auto-detected if not specified)
    Auto-detection searches: workspace, Program Files, user directories
    
.PARAMETER CreateDesktopShortcut
    Create desktop shortcut for this project (default: false)
    
.EXAMPLE
    .\cli-simulide.ps1
    Launch SimulIDE with current project (auto-detect everything)
    
.EXAMPLE
    .\cli-simulide.ps1 -ProjectDir "projects/Port_Basic"
    Launch SimulIDE with specific project
    
.EXAMPLE
    .\cli-simulide.ps1 -BuildFirst $false
    Launch without rebuilding (faster for testing)
    
.EXAMPLE
    .\cli-simulide.ps1 -SimulIDEPath "C:\SimulIDE\simulide.exe"
    Use specific SimulIDE installation
#>

param(
    [string]$ProjectDir = $PWD.Path,
    [string]$CircuitFile = "",
    [bool]$BuildFirst = $true,
    [string]$SimulIDEPath = "",
    [bool]$CreateDesktopShortcut = $false
)

# Colors for output
$ErrorColor = "Red"
$SuccessColor = "Green"
$InfoColor = "Cyan"
$WarningColor = "Yellow"

# Get workspace root (navigate up from tools/simulide to workspace root)
$WorkspaceRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent

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
    Write-Host "🔍 Searching for SimulIDE..." -ForegroundColor $InfoColor
    
    # Build comprehensive search list
    $PossiblePaths = @(
        # In workspace simulators folder (portable installation)
        Join-Path $WorkspaceRoot "simulators\SimulIDE_1.1.0-SR1_Win64\simulide.exe"
        Join-Path $WorkspaceRoot "simulators\SimulIDE_1.1.0-SR1_Win64\bin\simulide.exe"
        Join-Path $WorkspaceRoot "simulators\SimulIDE_1.1.0-SR1\simulide.exe"
        Join-Path $WorkspaceRoot "simulators\SimulIDE_1.1.0-SR1\bin\simulide.exe"
        Join-Path $WorkspaceRoot "simulators\SimulIDE\simulide.exe"
        Join-Path $WorkspaceRoot "simulators\SimulIDE\bin\simulide.exe"
        
        # In workspace root (legacy/portable installation)
        Join-Path $WorkspaceRoot "SimulIDE_1.1.0-SR1_Win64\simulide.exe"
        Join-Path $WorkspaceRoot "SimulIDE_1.1.0-SR1_Win64\bin\simulide.exe"
        Join-Path $WorkspaceRoot "SimulIDE_1.1.0-SR1\simulide.exe"
        Join-Path $WorkspaceRoot "SimulIDE_1.1.0-SR1\bin\simulide.exe"
        Join-Path $WorkspaceRoot "SimulIDE\simulide.exe"
        Join-Path $WorkspaceRoot "SimulIDE\bin\simulide.exe"
        
        # User's home directory
        Join-Path $env:USERPROFILE "SimulIDE\simulide.exe"
        Join-Path $env:USERPROFILE "SimulIDE\bin\simulide.exe"
        Join-Path $env:USERPROFILE "Downloads\SimulIDE_1.1.0-SR1_Win64\simulide.exe"
        Join-Path $env:USERPROFILE "Downloads\SimulIDE\simulide.exe"
        
        # Program Files (64-bit)
        "C:\Program Files\SimulIDE\simulide.exe"
        "C:\Program Files\SimulIDE\bin\simulide.exe"
        "C:\Program Files\SimulIDE_1.1.0-SR1\simulide.exe"
        "C:\Program Files\SimulIDE_1.1.0-SR1_Win64\simulide.exe"
        
        # Program Files (32-bit)
        "C:\Program Files (x86)\SimulIDE\simulide.exe"
        "C:\Program Files (x86)\SimulIDE\bin\simulide.exe"
        "C:\Program Files (x86)\SimulIDE_1.1.0-SR1\simulide.exe"
        
        # Common portable locations
        "D:\SimulIDE\simulide.exe"
        "E:\SimulIDE\simulide.exe"
    )
    
    # Also search PATH environment variable
    $PathSimulIDE = Get-Command "simulide.exe" -ErrorAction SilentlyContinue
    if ($PathSimulIDE) {
        $PossiblePaths = @($PathSimulIDE.Source) + $PossiblePaths
    }
    
    foreach ($path in $PossiblePaths) {
        if (Test-Path $path) {
            $SimulIDEPath = $path
            Write-Host "   ✅ Found: " -NoNewline -ForegroundColor $SuccessColor
            Write-Host "$SimulIDEPath" -ForegroundColor White
            break
        }
    }
}

if ([string]::IsNullOrEmpty($SimulIDEPath) -or -not (Test-Path $SimulIDEPath)) {
    Write-Host "❌ Error: SimulIDE executable not found!" -ForegroundColor $ErrorColor
    Write-Host ""
    Write-Host "Please download SimulIDE 1.1.0-SR1 from:" -ForegroundColor $WarningColor
    Write-Host "   https://simulide.com/p/downloads/" -ForegroundColor White
    Write-Host ""
    Write-Host "Installation options:" -ForegroundColor $InfoColor
    Write-Host "   1. Extract to workspace: $WorkspaceRoot\simulators\SimulIDE_1.1.0-SR1_Win64\" -ForegroundColor White
    Write-Host "   2. Install to Program Files" -ForegroundColor White
    Write-Host "   3. Specify custom path with -SimulIDEPath parameter" -ForegroundColor White
    Write-Host ""
    Write-Host "Example:" -ForegroundColor $InfoColor
    Write-Host "   .\cli-simulide.ps1 -SimulIDEPath 'C:\SimulIDE\simulide.exe'" -ForegroundColor White
    Write-Host ""
    Write-Host ""
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
    }
    else {
        Write-Host "⚠️  Warning: Build script not found, skipping build" -ForegroundColor $WarningColor
    }
}

# Find HEX file
$HexFile = Join-Path $ProjectDir "Main.hex"

if (-not (Test-Path $HexFile)) {
    Write-Host "❌ Error: HEX file not found: $HexFile" -ForegroundColor $ErrorColor
    
    if ($BuildFirst) {
        Write-Host "   Build completed but HEX file is missing" -ForegroundColor $WarningColor
        Write-Host "   Check build output for errors" -ForegroundColor $WarningColor
    }
    else {
        Write-Host "   Try building the project first:" -ForegroundColor $WarningColor
        Write-Host "   .\cli-simulide.ps1 -ProjectDir '$ProjectDir' -BuildFirst `$true" -ForegroundColor White
    }
    exit 1
}

Write-Host "✅ HEX file: " -NoNewline -ForegroundColor $SuccessColor
Write-Host "$HexFile" -ForegroundColor White

# Determine circuit file with auto-detection
if ([string]::IsNullOrEmpty($CircuitFile)) {
    Write-Host "🔍 Searching for circuit file..." -ForegroundColor $InfoColor
    Write-Host "   Priority: simulators/Simulator.simu (official circuit)" -ForegroundColor $InfoColor
    
    $PossibleCircuits = @(
        Join-Path $WorkspaceRoot "simulators\Simulator.simu"          # PRIMARY: Official circuit
        Join-Path $WorkspaceRoot "simulators\SimulIDE_1.1.0-SR1_Win64\Simulator.simu"
        Join-Path $WorkspaceRoot "SimulIDE_1.1.0-SR1_Win64\Simulator.simu"  # Legacy location
        Join-Path $WorkspaceRoot "Simulator.simu"                    # Workspace root fallback
        Join-Path $WorkspaceRoot "atmega128.simu"                    # Alternative names
        Join-Path $WorkspaceRoot "circuit.simu"
        Join-Path $ProjectDir "circuit.simu"                         # Project-specific
        Join-Path $ProjectDir "Simulator.simu"
    )
    
    foreach ($circuit in $PossibleCircuits) {
        if (Test-Path $circuit) {
            $CircuitFile = $circuit
            Write-Host "   ✅ Found: " -NoNewline -ForegroundColor $SuccessColor
            Write-Host "$CircuitFile" -ForegroundColor White
            
            # Highlight if using the official circuit
            if ($circuit -like "*simulators\Simulator.simu") {
                Write-Host "   🎯 Using official circuit file (recommended)" -ForegroundColor $SuccessColor
            }
            break
        }
    }
}

if ([string]::IsNullOrEmpty($CircuitFile) -or -not (Test-Path $CircuitFile)) {
    Write-Host "❌ Error: Circuit file not found!" -ForegroundColor $ErrorColor
    Write-Host ""
    Write-Host "Please ensure the official SimulIDE circuit file exists:" -ForegroundColor $WarningColor
    Write-Host ""
    Write-Host "Required:" -ForegroundColor $InfoColor
    Write-Host "   📂 $WorkspaceRoot\simulators\Simulator.simu (official circuit)" -ForegroundColor White
    Write-Host ""
    Write-Host "Alternative options:" -ForegroundColor $InfoColor
    Write-Host "   1. Check if simulators/Simulator.simu exists in workspace" -ForegroundColor White
    Write-Host "   2. Specify custom circuit: -CircuitFile 'path\to\circuit.simu'" -ForegroundColor White
    Write-Host ""
    Write-Host "Example:" -ForegroundColor $InfoColor
    Write-Host "   .\cli-simulide.ps1 -CircuitFile 'simulators\Simulator.simu'" -ForegroundColor White
    Write-Host ""
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
    $CircuitContent = Get-Content $CircuitFile -Raw -ErrorAction Stop
    
    if ([string]::IsNullOrWhiteSpace($CircuitContent)) {
        throw "Circuit file is empty or invalid"
    }
    
    # Use absolute path for SimulIDE (relative paths may not work reliably)
    $AbsoluteHexPath = $HexFile -replace '\\', '/'
    
    Write-Host "   HEX file: $HexFile" -ForegroundColor $InfoColor
    Write-Host "   Program path for SimulIDE: " -NoNewline -ForegroundColor $InfoColor
    Write-Host "$AbsoluteHexPath" -ForegroundColor White
    
    # Verify circuit file contains ATmega128 MCU
    if ($CircuitContent -notmatch 'itemtype="MCU"') {
        Write-Host "   ⚠️  Warning: Circuit file doesn't contain MCU component" -ForegroundColor $WarningColor
        Write-Host "      SimulIDE may not load the firmware automatically" -ForegroundColor $WarningColor
    }
    
    # Update Program path in MCU item
    if ($CircuitContent -match 'Program="[^"]*"') {
        $OldPath = $Matches[0]
        $CircuitContent = $CircuitContent -replace 'Program="[^"]*"', "Program=`"$AbsoluteHexPath`""
        Write-Host "   ✅ Updated MCU program path" -ForegroundColor $SuccessColor
        Write-Host "      Old: $OldPath" -ForegroundColor Gray
        Write-Host "      New: Program=`"$AbsoluteHexPath`"" -ForegroundColor Gray
    }
    else {
        Write-Host "   ⚠️  Warning: Could not find Program attribute in circuit" -ForegroundColor $WarningColor
        Write-Host "      You may need to manually load firmware in SimulIDE" -ForegroundColor $WarningColor
    }
    
    # Enable Auto_Load
    if ($CircuitContent -match 'Auto_Load="false"') {
        $CircuitContent = $CircuitContent -replace 'Auto_Load="false"', 'Auto_Load="true"'
        Write-Host "   ✅ Enabled Auto_Load" -ForegroundColor $SuccessColor
    }
    elseif ($CircuitContent -match 'Auto_Load="true"') {
        Write-Host "   ✅ Auto_Load already enabled" -ForegroundColor $SuccessColor
    }
    
    # Save temporary circuit file with error handling
    try {
        $CircuitContent | Set-Content $TempCircuitFile -Encoding UTF8 -ErrorAction Stop
        
        # Verify temp file was created successfully
        if (-not (Test-Path $TempCircuitFile)) {
            throw "Failed to create temporary circuit file"
        }
        
        Write-Host "   ✅ Created temporary circuit: $TempCircuitFile" -ForegroundColor $SuccessColor
    }
    catch {
        throw "Failed to save temporary circuit file: $_"
    }
    
}
catch {
    Write-Host "❌ Error updating circuit file: $_" -ForegroundColor $ErrorColor
    exit 1
}

# Launch SimulIDE
Write-Host "`n🚀 Launching SimulIDE..." -ForegroundColor $SuccessColor
Write-Host "   Press Ctrl+C here to close this window (SimulIDE will keep running)" -ForegroundColor $WarningColor
Write-Host ""

try {
    # Verify SimulIDE is not already running with the same circuit
    $RunningSimulIDE = Get-Process -Name "simulide" -ErrorAction SilentlyContinue
    if ($RunningSimulIDE) {
        Write-Host "⚠️  SimulIDE is already running" -ForegroundColor $WarningColor
        Write-Host "   Close existing instance or use File → Open in SimulIDE" -ForegroundColor $InfoColor
        Write-Host "   Launching new instance anyway..." -ForegroundColor $InfoColor
        Write-Host ""
    }
    
    # Launch SimulIDE with the temporary circuit file
    $Process = Start-Process -FilePath $SimulIDEPath -ArgumentList "`"$TempCircuitFile`"" -PassThru -ErrorAction Stop
    
    # Wait a moment to verify process started
    Start-Sleep -Milliseconds 500
    
    if ($Process.HasExited) {
        throw "SimulIDE process exited immediately (exit code: $($Process.ExitCode))"
    }
    
    Write-Host "✅ SimulIDE launched successfully! (PID: $($Process.Id))" -ForegroundColor $SuccessColor
    Write-Host "`n📋 Next Steps:" -ForegroundColor $InfoColor
    Write-Host "  1️⃣  Click the Play button (▶) to start simulation" -ForegroundColor White
    Write-Host "  2️⃣  Interact with components (buttons, potentiometers, etc.)" -ForegroundColor White
    Write-Host "  3️⃣  Double-click SerialTerm component to see UART output" -ForegroundColor White
    Write-Host ""
    Write-Host "💡 Troubleshooting:" -ForegroundColor $InfoColor
    Write-Host "  • Firmware not loading? → Right-click ATmega128 → Properties" -ForegroundColor White
    Write-Host "  • Need to reload? → Right-click ATmega128 → Load Firmware" -ForegroundColor White
    Write-Host "  • Reset MCU → Press Ctrl+R in SimulIDE" -ForegroundColor White
    Write-Host "  • Circuit file: $TempCircuitFile" -ForegroundColor Gray
    Write-Host ""
    
}
catch {
    Write-Host "❌ Error launching SimulIDE: $_" -ForegroundColor $ErrorColor
    Write-Host ""
    Write-Host "Troubleshooting steps:" -ForegroundColor $InfoColor
    Write-Host "  1. Verify SimulIDE path: $SimulIDEPath" -ForegroundColor White
    Write-Host "  2. Check if SimulIDE.exe is blocked (Right-click → Properties → Unblock)" -ForegroundColor White
    Write-Host "  3. Try running SimulIDE manually:" -ForegroundColor White
    Write-Host "     Start-Process '$SimulIDEPath'" -ForegroundColor Gray
    Write-Host "  4. Check Windows Defender or antivirus settings" -ForegroundColor White
    Write-Host ""
    exit 1
}

exit 0
