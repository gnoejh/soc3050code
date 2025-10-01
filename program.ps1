# Universal ATmega128 Programming Script for All Projects
# Usage: .\program_universal.ps1 [-ProjectFolder "01_Assembly_Blink_Basic"] [-ComPort COM3] [-Programmer arduino]

param(
    [string]$ProjectFolder = "",
    [string]$ComPort = "",
    [string]$Programmer = "arduino",
    [string]$HexFile = "Main.hex",
    [switch]$ListPorts,
    [switch]$ListProjects,
    [switch]$Help
)

if ($Help) {
    Write-Host "🔧 Universal ATmega128 AVRDUDE Programming Script" -ForegroundColor Green
    Write-Host "===============================================" -ForegroundColor Green
    Write-Host ""
    Write-Host "Usage:" -ForegroundColor Yellow
    Write-Host "  .\program_universal.ps1                                    # Auto-detect project from current folder"
    Write-Host "  .\program_universal.ps1 -ProjectFolder 01_Assembly_Blink_Basic # Specify project"
    Write-Host "  .\program_universal.ps1 -ComPort COM5                     # Specify COM port"
    Write-Host "  .\program_universal.ps1 -Programmer usbasp                # Use different programmer"
    Write-Host "  .\program_universal.ps1 -ListPorts                        # List available COM ports"
    Write-Host "  .\program_universal.ps1 -ListProjects                     # List all projects"
    Write-Host ""
    Write-Host "Common Programmers:" -ForegroundColor Cyan
    Write-Host "  arduino    - Arduino as ISP"
    Write-Host "  usbasp     - USBasp programmer"
    Write-Host "  avrisp     - AVRISP mkII"
    Write-Host "  stk500v2   - STK500 v2"
    Write-Host "  dragon_isp - AVR Dragon"
    exit 0
}

$AVRDUDE = "C:\Program Files (x86)\AVRDUDESS\avrdude.exe"
$MCU = "m128"
$WorkspaceRoot = $PSScriptRoot

Write-Host "🔧 Universal ATmega128 AVRDUDE Programming" -ForegroundColor Green
Write-Host "=========================================" -ForegroundColor Green

# Check if AVRDUDE exists
if (-not (Test-Path $AVRDUDE)) {
    Write-Host "❌ AVRDUDE not found at: $AVRDUDE" -ForegroundColor Red
    Write-Host "💡 Please install AVRDUDESS or update the path" -ForegroundColor Yellow
    exit 1
}

# List projects if requested
if ($ListProjects) {
    Write-Host "📁 Available projects:" -ForegroundColor Yellow
    $projects = Get-ChildItem -Path "$WorkspaceRoot\projects" -Directory | Sort-Object Name
    $projects | ForEach-Object { Write-Host "  $($_.Name)" -ForegroundColor White }
    exit 0
}

# List ports if requested
if ($ListPorts) {
    Write-Host "📡 Available COM ports:" -ForegroundColor Yellow
    $ports = [System.IO.Ports.SerialPort]::GetPortNames()
    if ($ports.Count -eq 0) {
        Write-Host "  No COM ports found" -ForegroundColor Gray
    } else {
        $ports | ForEach-Object { Write-Host "  $_" -ForegroundColor White }
    }
    exit 0
}

# Auto-detect project folder if not specified
if ($ProjectFolder -eq "") {
    $currentPath = Get-Location
    if ($currentPath.Path -like "*\projects\*") {
        # We're inside a project folder
        $pathParts = $currentPath.Path.Split('\')
        $projectIndex = [array]::IndexOf($pathParts, "projects")
        if ($projectIndex -ge 0 -and ($projectIndex + 1) -lt $pathParts.Length) {
            $ProjectFolder = $pathParts[$projectIndex + 1]
            Write-Host "🔍 Auto-detected project: $ProjectFolder" -ForegroundColor Cyan
        }
    }
    
    if ($ProjectFolder -eq "") {
        Write-Host "📁 Available projects:" -ForegroundColor Yellow
        $projects = Get-ChildItem -Path "$WorkspaceRoot\projects" -Directory | Sort-Object Name
        for ($i = 0; $i -lt $projects.Count; $i++) {
            Write-Host "  [$($i+1)] $($projects[$i].Name)" -ForegroundColor White
        }
        
        $selection = Read-Host "Select project (1-$($projects.Count)) or Enter project name"
        
        if ($selection -match '^\d+$' -and [int]$selection -le $projects.Count -and [int]$selection -gt 0) {
            $ProjectFolder = $projects[[int]$selection - 1].Name
        } elseif ($selection -ne "") {
            $ProjectFolder = $selection
        } else {
            Write-Host "❌ No project selected" -ForegroundColor Red
            exit 1
        }
    }
}

# Validate project folder exists
$projectPath = "$WorkspaceRoot\projects\$ProjectFolder"
if (-not (Test-Path $projectPath)) {
    Write-Host "❌ Project folder not found: $projectPath" -ForegroundColor Red
    Write-Host "💡 Use -ListProjects to see available projects" -ForegroundColor Yellow
    exit 1
}

# Check if HEX file exists in project folder
$hexPath = "$projectPath\$HexFile"
if (-not (Test-Path $hexPath)) {
    Write-Host "❌ HEX file not found: $hexPath" -ForegroundColor Red
    Write-Host "💡 Build the project first to generate the HEX file" -ForegroundColor Yellow
    Write-Host "   • Open the project in VS Code" -ForegroundColor Gray
    Write-Host "   • Run 'Generate HEX File' task" -ForegroundColor Gray
    exit 1
}

# Auto-detect COM port if not specified
if ($ComPort -eq "") {
    Write-Host "🔍 Auto-detecting COM ports..." -ForegroundColor Cyan
    $availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
    
    if ($availablePorts.Count -eq 0) {
        Write-Host "❌ No COM ports found!" -ForegroundColor Red
        Write-Host "💡 Connect your programmer and try again" -ForegroundColor Yellow
        exit 1
    }
    
    if ($availablePorts.Count -eq 1) {
        $ComPort = $availablePorts[0]
        Write-Host "✅ Using COM port: $ComPort" -ForegroundColor Green
    } else {
        Write-Host "📡 Multiple COM ports found:" -ForegroundColor Yellow
        for ($i = 0; $i -lt $availablePorts.Count; $i++) {
            Write-Host "  [$($i+1)] $($availablePorts[$i])" -ForegroundColor White
        }
        
        $selection = Read-Host "Select COM port (1-$($availablePorts.Count)) or press Enter for $($availablePorts[0])"
        
        if ($selection -eq "") {
            $ComPort = $availablePorts[0]
        } elseif ($selection -match '^\d+$' -and [int]$selection -le $availablePorts.Count -and [int]$selection -gt 0) {
            $ComPort = $availablePorts[[int]$selection - 1]
        } else {
            Write-Host "❌ Invalid selection" -ForegroundColor Red
            exit 1
        }
    }
}

# Display programming info
Write-Host ""
Write-Host "📋 Programming Configuration:" -ForegroundColor Cyan
Write-Host "  Project:     $ProjectFolder" -ForegroundColor White
Write-Host "  HEX Path:    '$hexPath'" -ForegroundColor White
Write-Host "  HEX Exists:  $(Test-Path $hexPath)" -ForegroundColor White
Write-Host "  MCU:         $MCU" -ForegroundColor White  
Write-Host "  Programmer:  $Programmer" -ForegroundColor White
Write-Host "  COM Port:    $ComPort" -ForegroundColor White
Write-Host ""

# Build AVRDUDE command
$flashArg = "flash:w:$hexPath:a"
$avrdudeArgs = @(
    "-c", $Programmer,
    "-p", $MCU,
    "-P", $ComPort,
    "-U", $flashArg
)

Write-Host "🚀 Starting programming..." -ForegroundColor Yellow
$flashCommand = "flash:w:" + $hexPath + ":a"
$arguments = @("-c", $Programmer, "-p", $MCU, "-P", $ComPort, "-U", $flashCommand)
Write-Host "Command: avrdude.exe $($arguments -join ' ')" -ForegroundColor Gray
Write-Host ""

# Execute AVRDUDE
try {
    $process = Start-Process -FilePath $AVRDUDE -ArgumentList $arguments -Wait -PassThru -NoNewWindow
    
    if ($process.ExitCode -eq 0) {
        Write-Host ""
        Write-Host "✅ Programming completed successfully!" -ForegroundColor Green
        Write-Host "🎯 ATmega128 is now running: $ProjectFolder" -ForegroundColor Cyan
        
        # Get file size info
        $hexSize = (Get-Item $hexPath).Length
        Write-Host "📊 HEX file size: $hexSize bytes" -ForegroundColor White
    } else {
        Write-Host ""
        Write-Host "❌ Programming failed!" -ForegroundColor Red
        Write-Host ""
        Write-Host "🔧 Troubleshooting tips:" -ForegroundColor Yellow
        Write-Host "  • Check COM port: $ComPort" -ForegroundColor Gray
        Write-Host "  • Verify programmer connection" -ForegroundColor Gray
        Write-Host "  • Ensure ATmega128 is powered" -ForegroundColor Gray
        Write-Host "  • Try different programmer: -Programmer usbasp" -ForegroundColor Gray
        Write-Host "  • List ports: .\program_universal.ps1 -ListPorts" -ForegroundColor Gray
    }
} catch {
    Write-Host "❌ Error executing AVRDUDE: $($_.Exception.Message)" -ForegroundColor Red
}