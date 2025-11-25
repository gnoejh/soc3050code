#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Launch SimulIDE 0.4.15 with ATmega128 project
    
.DESCRIPTION
    Build current project and launch SimulIDE 0.4.15-SR10 with the compiled HEX file.
    This is the LEGACY VERSION for testing UART functionality comparison.
    Automatically updates Simulator0415.simu to point to the project's HEX file.
    
.PARAMETER ProjectDir
    Path to project directory containing Main.c (default: current directory)
    Can be absolute or relative path
    
.PARAMETER CircuitFile
    Path to .simu circuit file (default: auto-detect Simulator0415.simu)
    
.PARAMETER BuildFirst
    Build the project before launching SimulIDE (default: true)
    
.PARAMETER SimulIDEPath
    Path to SimulIDE 0.4.15 executable (auto-detected if not specified)
    
.EXAMPLE
    .\cli-simulide-0415.ps1
    Launch SimulIDE 0.4.15 with current project
    
.EXAMPLE
    .\cli-simulide-0415.ps1 -ProjectDir "projects/Serial_Communications"
    Test serial communications in old version
    
.EXAMPLE
    .\cli-simulide-0415.ps1 -BuildFirst $false
    Launch without rebuilding (faster for testing)
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

# Get workspace root (navigate up from tools/simulide to workspace root)
$WorkspaceRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent

Write-Host "`n========================================" -ForegroundColor $InfoColor
Write-Host "  SimulIDE 0.4.15-SR10 Legacy Launcher" -ForegroundColor $InfoColor
Write-Host "  (For UART Testing & Comparison)" -ForegroundColor $WarningColor
Write-Host "========================================`n" -ForegroundColor $InfoColor

# Resolve project directory
if (-not [System.IO.Path]::IsPathRooted($ProjectDir)) {
    $ProjectDir = Join-Path $WorkspaceRoot $ProjectDir
}

# Smart project detection for non-project directories
if (-not (Test-Path $ProjectDir) -or -not (Test-Path (Join-Path $ProjectDir "Main.c"))) {
    Write-Host "[WARNING] Not in a valid project directory. Searching for recent projects..." -ForegroundColor $WarningColor
    
    # Try to find the most recently modified project
    $ProjectsDir = Join-Path $WorkspaceRoot "projects"
    
    if (Test-Path $ProjectsDir) {
        $RecentProjects = Get-ChildItem $ProjectsDir -Directory | Where-Object { 
            Test-Path (Join-Path $_.FullName "Main.c") 
        } | Sort-Object LastWriteTime -Descending | Select-Object -First 5
        
        if ($RecentProjects) {
            Write-Host ""
            Write-Host "[PROJECTS] Available projects:" -ForegroundColor $InfoColor
            for ($i = 0; $i -lt $RecentProjects.Count; $i++) {
                $project = $RecentProjects[$i]
                Write-Host "  $($i + 1). $($project.Name)" -ForegroundColor White
            }
            Write-Host ""
            
            # Auto-select the most recent project
            $ProjectDir = $RecentProjects[0].FullName
            Write-Host "[AUTO] Auto-selecting most recent: $($RecentProjects[0].Name)" -ForegroundColor $SuccessColor
        }
        else {
            Write-Host "[ERROR] Error: No projects found with Main.c files" -ForegroundColor $ErrorColor
            exit 1
        }
    }
    else {
        Write-Host "[ERROR] Error: Projects directory not found: $ProjectsDir" -ForegroundColor $ErrorColor
        exit 1
    }
}

if (-not (Test-Path $ProjectDir)) {
    Write-Host "[ERROR] Error: Project directory not found: $ProjectDir" -ForegroundColor $ErrorColor
    exit 1
}

Write-Host "[PROJECT] Project: " -NoNewline -ForegroundColor $InfoColor
Write-Host "$ProjectDir" -ForegroundColor White

# Find SimulIDE 0.4.15 executable - ONLY search for legacy version
if ([string]::IsNullOrEmpty($SimulIDEPath)) {
    Write-Host "[SEARCH] Searching for SimulIDE 0.4.15-SR10..." -ForegroundColor $InfoColor
    
    # Only search for 0.4.15 version
    $PossiblePaths = @(
        Join-Path $WorkspaceRoot "tools\simulide\SimulIDE_0.4.15-SR10_Win64\simulide.exe"
        Join-Path $WorkspaceRoot "tools\simulide\SimulIDE_0.4.15-SR10_Win64\bin\simulide.exe"
        Join-Path $WorkspaceRoot "tools\simulide\SimulIDE_0.4.15\simulide.exe"
        Join-Path $WorkspaceRoot "tools\simulide\SimulIDE_0.4.15\bin\simulide.exe"
    )

    foreach ($path in $PossiblePaths) {
        if (Test-Path $path) {
            $SimulIDEPath = $path
            Write-Host "   [OK] Found: " -NoNewline -ForegroundColor $SuccessColor
            Write-Host "$SimulIDEPath" -ForegroundColor White
            break
        }
    }
}

if ([string]::IsNullOrEmpty($SimulIDEPath) -or -not (Test-Path $SimulIDEPath)) {
    Write-Host "[ERROR] Error: SimulIDE 0.4.15-SR10 executable not found!" -ForegroundColor $ErrorColor
    Write-Host ""
    Write-Host "Expected location:" -ForegroundColor $WarningColor
    Write-Host "   $WorkspaceRoot\tools\simulide\SimulIDE_0.4.15-SR10_Win64\simulide.exe" -ForegroundColor White
    Write-Host ""
    Write-Host "This script requires SimulIDE 0.4.15-SR10 for legacy testing." -ForegroundColor $InfoColor
    Write-Host "For SimulIDE 1.1.0, use: .\cli-simulide.ps1" -ForegroundColor $InfoColor
    Write-Host ""
    exit 1
}

Write-Host "[CONFIG] SimulIDE: " -NoNewline -ForegroundColor $InfoColor
Write-Host "$SimulIDEPath" -ForegroundColor White

# Build project if requested
if ($BuildFirst) {
    Write-Host "`n[BUILD] Building project..." -ForegroundColor $InfoColor
    
    $BuildScript = Join-Path $WorkspaceRoot "tools\cli\cli-build-project.ps1"
    
    if (Test-Path $BuildScript) {
        & $BuildScript -ProjectDir $ProjectDir
        
        if ($LASTEXITCODE -ne 0) {
            Write-Host "[ERROR] Build failed!" -ForegroundColor $ErrorColor
            exit 1
        }
    }
    else {
        Write-Host "[WARNING] Warning: Build script not found, skipping build" -ForegroundColor $WarningColor
    }
}

# Find HEX file
$HexFile = Join-Path $ProjectDir "Main.hex"

if (-not (Test-Path $HexFile)) {
    Write-Host "[ERROR] Error: HEX file not found: $HexFile" -ForegroundColor $ErrorColor
    exit 1
}

Write-Host "[OK] HEX file: " -NoNewline -ForegroundColor $SuccessColor
Write-Host "$HexFile" -ForegroundColor White

# Determine circuit file - ONLY search for 0415 circuit
if ([string]::IsNullOrEmpty($CircuitFile)) {
    Write-Host "[SEARCH] Searching for Simulator0415.simu circuit file..." -ForegroundColor $InfoColor
    
    $PossibleCircuits = @(
        Join-Path $WorkspaceRoot "tools\simulide\Simulator0415.simu"
        Join-Path $WorkspaceRoot "tools\simulide\SimulIDE_0.4.15-SR10_Win64\Simulator0415.simu"
        Join-Path $WorkspaceRoot "Simulator0415.simu"
    )

    foreach ($circuit in $PossibleCircuits) {
        if (Test-Path $circuit) {
            $CircuitFile = $circuit
            Write-Host "   [OK] Found: " -NoNewline -ForegroundColor $SuccessColor
            Write-Host "$CircuitFile" -ForegroundColor White
            break
        }
    }
}

if ([string]::IsNullOrEmpty($CircuitFile) -or -not (Test-Path $CircuitFile)) {
    Write-Host "[ERROR] Error: Simulator0415.simu circuit file not found!" -ForegroundColor $ErrorColor
    Write-Host ""
    Write-Host "Expected location:" -ForegroundColor $WarningColor
    Write-Host "   $WorkspaceRoot\tools\simulide\Simulator0415.simu" -ForegroundColor White
    Write-Host ""
    Write-Host "This circuit is required for SimulIDE 0.4.15 compatibility testing." -ForegroundColor $InfoColor
    Write-Host ""
    exit 1
}

Write-Host "[CIRCUIT] Circuit: " -NoNewline -ForegroundColor $InfoColor
Write-Host "$CircuitFile" -ForegroundColor White

# Update the circuit file
Write-Host "`n[UPDATE] Updating circuit file..." -ForegroundColor $InfoColor

# Backup if not exists
$BackupCircuitFile = $CircuitFile -replace '\.simu$', '_backup.simu'
if (-not (Test-Path $BackupCircuitFile)) {
    Copy-Item $CircuitFile $BackupCircuitFile
    Write-Host "   [BACKUP] Created backup: $BackupCircuitFile" -ForegroundColor $InfoColor
}

try {
    # Read circuit file
    $CircuitContent = Get-Content $CircuitFile -Raw -ErrorAction Stop
    
    if ([string]::IsNullOrWhiteSpace($CircuitContent)) {
        throw "Circuit file is empty or invalid"
    }
    
    # Use absolute path for SimulIDE
    $AbsoluteHexPath = $HexFile -replace '\\', '/'
    
    Write-Host "   HEX file: $HexFile" -ForegroundColor $InfoColor
    Write-Host "   Program path: " -NoNewline -ForegroundColor $InfoColor
    Write-Host "$AbsoluteHexPath" -ForegroundColor White
    
    # Update Program path in MCU item
    if ($CircuitContent -match 'Program="[^"]*"') {
        $OldPath = $Matches[0]
        $CircuitContent = $CircuitContent -replace 'Program="[^"]*"', "Program=`"$AbsoluteHexPath`""
        Write-Host "   [OK] Updated MCU program path" -ForegroundColor $SuccessColor
    }
    else {
        Write-Host "   [WARNING] Warning: Could not find Program attribute in circuit" -ForegroundColor $WarningColor
    }
    
    # Enable Auto_Load
    if ($CircuitContent -match 'Auto_Load="false"') {
        $CircuitContent = $CircuitContent -replace 'Auto_Load="false"', 'Auto_Load="true"'
        Write-Host "   [OK] Enabled Auto_Load" -ForegroundColor $SuccessColor
    }
    
    # Save updated circuit file
    $CircuitContent | Set-Content $CircuitFile -Encoding UTF8 -ErrorAction Stop
    Write-Host "   [OK] Updated circuit: $CircuitFile" -ForegroundColor $SuccessColor
    
}
catch {
    Write-Host "[ERROR] Error updating circuit file: $_" -ForegroundColor $ErrorColor
    exit 1
}

# Launch SimulIDE
Write-Host "`n[START] Launching SimulIDE 0.4.15-SR10..." -ForegroundColor $SuccessColor
Write-Host "   Press Ctrl+C here to close this window (SimulIDE will keep running)" -ForegroundColor $WarningColor
Write-Host ""

try {
    # Check if SimulIDE is already running
    $RunningSimulIDE = Get-Process -Name "simulide" -ErrorAction SilentlyContinue
    if ($RunningSimulIDE) {
        Write-Host "[WARNING] SimulIDE is already running!" -ForegroundColor $WarningColor
        Write-Host "   [ACTION] Please close all SimulIDE instances to avoid version conflicts" -ForegroundColor $WarningColor
        Write-Host "   [INFO] Running: PID $($RunningSimulIDE.Id -join ', ')" -ForegroundColor $InfoColor
        Write-Host ""
        Write-Host "Close SimulIDE and run this script again." -ForegroundColor $InfoColor
        Write-Host ""
        return
    }
    
    # Launch new SimulIDE 0.4.15 instance
    Write-Host "[START] Launching new SimulIDE 0.4.15 instance..." -ForegroundColor $InfoColor
    $Process = Start-Process -FilePath $SimulIDEPath -ArgumentList "`"$CircuitFile`"" -PassThru -ErrorAction Stop
    
    # Wait a moment to verify process started
    Start-Sleep -Milliseconds 500
    
    if ($Process.HasExited) {
        throw "SimulIDE process exited immediately (exit code: $($Process.ExitCode))"
    }
    
    Write-Host "[OK] SimulIDE 0.4.15 launched successfully! (PID: $($Process.Id))" -ForegroundColor $SuccessColor
    Write-Host "`n[TESTING] UART Testing Guide:" -ForegroundColor $InfoColor
    Write-Host "  [1] Click the Play button (PLAY) to start simulation" -ForegroundColor White
    Write-Host "  [2] Open Serial Terminal/Port component to test UART" -ForegroundColor White
    Write-Host "  [3] Compare behavior with SimulIDE 1.1.0 version" -ForegroundColor White
    Write-Host ""
    Write-Host "[COMPARE] To test with SimulIDE 1.1.0:" -ForegroundColor $InfoColor
    Write-Host "  Close this SimulIDE and run: .\cli-simulide.ps1" -ForegroundColor White
    Write-Host ""
    Write-Host "[NOTE] Circuit file: $CircuitFile" -ForegroundColor Gray
    Write-Host ""
    
}
catch {
    Write-Host "[ERROR] Error launching SimulIDE: $_" -ForegroundColor $ErrorColor
    Write-Host ""
    Write-Host "Troubleshooting:" -ForegroundColor $InfoColor
    Write-Host "  1. Verify path: $SimulIDEPath" -ForegroundColor White
    Write-Host "  2. Check if blocked (Right-click > Properties > Unblock)" -ForegroundColor White
    Write-Host ""
    exit 1
}

exit 0
