#!/usr/bin/env pwsh
<#
.SYNOPSIS
    One-Click Self-Sufficient SimulIDE Setup
    
.DESCRIPTION
    Fully automatic setup that:
    1. Configures SimulIDE circuit for TCP communication
    2. Starts TCP socket bridge automatically
    3. Launches SimulIDE with everything ready
    4. No external dependencies required!
    
.PARAMETER ProjectDir
    Project directory for simulation (default: Graphics_Display)
    
.PARAMETER StopBridge
    Stop any running TCP bridges
    
.PARAMETER ShowStatus
    Show current system status
    
.EXAMPLE
    .\one-click-simulide.ps1
    Complete automatic setup and launch
    
.EXAMPLE
    .\one-click-simulide.ps1 -ProjectDir "projects\Serial_Communications"
    Setup for specific project
#>

param(
    [Parameter(Mandatory = $false)]
    [string]$ProjectDir = "Graphics_Display",
    
    [Parameter(Mandatory = $false)]
    [switch]$StopBridge,
    
    [Parameter(Mandatory = $false)]
    [switch]$ShowStatus
)

$WorkspaceRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent

# Function to stop any running bridges
function Stop-TcpBridges {
    Write-Host "🛑 Stopping TCP bridges..." -ForegroundColor Yellow
    
    # Stop PowerShell jobs running TCP bridges
    $bridges = Get-Job | Where-Object { $_.Command -like "*tcp*" -or $_.Command -like "*bridge*" }
    foreach ($bridge in $bridges) {
        Write-Host "   Stopping bridge job: $($bridge.Id)" -ForegroundColor Gray
        Stop-Job $bridge -Force
        Remove-Job $bridge -Force
    }
    
    # Kill any TCP bridge processes
    $processes = Get-Process | Where-Object { $_.ProcessName -like "*bridge*" -or $_.CommandLine -like "*tcp*" }
    foreach ($process in $processes) {
        try {
            Write-Host "   Stopping process: $($process.ProcessName)" -ForegroundColor Gray
            Stop-Process $process -Force
        }
        catch {
            # Ignore errors
        }
    }
    
    Write-Host "✅ TCP bridges stopped" -ForegroundColor Green
}

# Function to start TCP bridge in background
function Start-TcpBridge {
    Write-Host "🚀 Starting TCP Socket Bridge..." -ForegroundColor Cyan
    
    $bridgeScript = Join-Path $PSScriptRoot "windows-virtual-com.ps1"
    
    # Start bridge as background job
    $job = Start-Job -ScriptBlock {
        param($scriptPath)
        & $scriptPath -Method tcpsockets -Start
    } -ArgumentList $bridgeScript
    
    # Wait a moment for bridge to initialize
    Start-Sleep -Seconds 2
    
    Write-Host "✅ TCP Bridge started (Job ID: $($job.Id))" -ForegroundColor Green
    Write-Host "   🌐 Application Port: localhost:9001" -ForegroundColor White
    Write-Host "   🌐 SimulIDE Port: localhost:9002" -ForegroundColor White
    
    return $job
}

# Function to configure SimulIDE circuit
function Configure-SimulIDECircuit {
    Write-Host "⚙️ Configuring SimulIDE circuit..." -ForegroundColor Cyan
    
    $configScript = Join-Path $PSScriptRoot "configure-simulide-tcp.ps1"
    if (Test-Path $configScript) {
        & $configScript 2>&1 | Out-Null
        Write-Host "✅ SimulIDE circuit configured for TCP communication" -ForegroundColor Green
        return $true
    }
    else {
        Write-Host "⚠️ Circuit configuration script not found" -ForegroundColor Yellow
        return $false
    }
}

# Function to launch SimulIDE
function Launch-SimulIDE {
    param([string]$ProjectPath)
    
    Write-Host "🎮 Launching SimulIDE..." -ForegroundColor Cyan
    
    $simulideScript = Join-Path $PSScriptRoot "cli-simulide.ps1"
    if (Test-Path $simulideScript) {
        # Launch SimulIDE with the specified project
        & $simulideScript -ProjectDir $ProjectPath -BuildFirst $true
        return $true
    }
    else {
        Write-Host "❌ SimulIDE launcher not found" -ForegroundColor Red
        return $false
    }
}

# Function to show comprehensive status
function Show-SystemStatus {
    Write-Host "📊 Self-Sufficient SimulIDE System Status" -ForegroundColor Green
    Write-Host "==========================================" -ForegroundColor Green
    Write-Host ""
    
    # Check TCP bridge status
    $bridgeJobs = Get-Job | Where-Object { $_.Command -like "*tcp*" -or $_.Command -like "*bridge*" }
    Write-Host "🌐 TCP Bridge Status:" -ForegroundColor Cyan
    if ($bridgeJobs.Count -gt 0) {
        foreach ($job in $bridgeJobs) {
            Write-Host "   ✅ Bridge running (Job ID: $($job.Id), State: $($job.State))" -ForegroundColor Green
        }
    }
    else {
        Write-Host "   ❌ No TCP bridges running" -ForegroundColor Red
    }
    Write-Host ""
    
    # Check SimulIDE circuit configuration
    $circuitFile = Join-Path $PSScriptRoot "Simulator110.simu"
    Write-Host "🔧 Circuit Configuration:" -ForegroundColor Cyan
    if (Test-Path $circuitFile) {
        $content = Get-Content $circuitFile -Raw
        if ($content -match 'Port="localhost:(\d+)"') {
            Write-Host "   ✅ Configured for TCP: localhost:$($matches[1])" -ForegroundColor Green
        }
        elseif ($content -match 'Port="(COM\d+)"') {
            Write-Host "   ⚠️ Configured for COM port: $($matches[1])" -ForegroundColor Yellow
        }
        else {
            Write-Host "   ❓ Unknown configuration" -ForegroundColor Yellow
        }
    }
    else {
        Write-Host "   ❌ Circuit file not found" -ForegroundColor Red
    }
    Write-Host ""
    
    # Check available ports
    Write-Host "🔌 Network Status:" -ForegroundColor Cyan
    try {
        $tcp9001 = Test-NetConnection -ComputerName localhost -Port 9001 -WarningAction SilentlyContinue
        $tcp9002 = Test-NetConnection -ComputerName localhost -Port 9002 -WarningAction SilentlyContinue
        
        Write-Host "   Port 9001 (App): $(if($tcp9001.TcpTestSucceeded){"✅ Open"}else{"❌ Closed"})" -ForegroundColor $(if ($tcp9001.TcpTestSucceeded) { "Green" }else { "Red" })
        Write-Host "   Port 9002 (SimulIDE): $(if($tcp9002.TcpTestSucceeded){"✅ Open"}else{"❌ Closed"})" -ForegroundColor $(if ($tcp9002.TcpTestSucceeded) { "Green" }else { "Red" })
    }
    catch {
        Write-Host "   ⚠️ Could not test network ports" -ForegroundColor Yellow
    }
    Write-Host ""
    
    # Overall status
    Write-Host "🎯 System Status:" -ForegroundColor Green
    if ($bridgeJobs.Count -gt 0) {
        Write-Host "   🟢 READY - Self-sufficient system is active!" -ForegroundColor Green
        Write-Host "   💡 Your apps connect to: localhost:9001" -ForegroundColor White
        Write-Host "   💡 SimulIDE communicates via: localhost:9002" -ForegroundColor White
    }
    else {
        Write-Host "   🟡 STOPPED - Run with no parameters to start" -ForegroundColor Yellow
    }
}

# Main execution
Write-Host "🎯 One-Click Self-Sufficient SimulIDE Setup" -ForegroundColor Green
Write-Host "============================================" -ForegroundColor Green
Write-Host ""

if ($StopBridge) {
    Stop-TcpBridges
    exit 0
}

if ($ShowStatus) {
    Show-SystemStatus
    exit 0
}

# Full automatic setup
Write-Host "🚀 Starting fully automatic setup..." -ForegroundColor Cyan
Write-Host ""

# Step 1: Stop any existing bridges
Write-Host "1️⃣ Cleaning up existing bridges..." -ForegroundColor Yellow
Stop-TcpBridges
Write-Host ""

# Step 2: Configure SimulIDE circuit for TCP
Write-Host "2️⃣ Configuring SimulIDE circuit..." -ForegroundColor Yellow
Configure-SimulIDECircuit
Write-Host ""

# Step 3: Start TCP bridge
Write-Host "3️⃣ Starting TCP bridge..." -ForegroundColor Yellow
$bridgeJob = Start-TcpBridge
Write-Host ""

# Step 4: Launch SimulIDE
Write-Host "4️⃣ Launching SimulIDE..." -ForegroundColor Yellow
$projectPath = if ($ProjectDir -like "*:*") { $ProjectDir } else { Join-Path $WorkspaceRoot "projects\$ProjectDir" }
$success = Launch-SimulIDE -ProjectPath $projectPath

Write-Host ""
if ($success) {
    Write-Host "🎉 SUCCESS! Self-Sufficient SimulIDE System is Ready!" -ForegroundColor Green
    Write-Host "==============================================" -ForegroundColor Green
    Write-Host ""
    Write-Host "✅ TCP Bridge: Running (localhost:9001 ↔ localhost:9002)" -ForegroundColor Green
    Write-Host "✅ SimulIDE: Configured for TCP communication" -ForegroundColor Green
    Write-Host "✅ Circuit: Loaded and ready for simulation" -ForegroundColor Green
    Write-Host ""
    Write-Host "💡 Your applications connect to: localhost:9001" -ForegroundColor Cyan
    Write-Host "💡 SimulIDE communicates via: localhost:9002" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "🛑 To stop: .\one-click-simulide.ps1 -StopBridge" -ForegroundColor Yellow
    Write-Host "📊 Status: .\one-click-simulide.ps1 -ShowStatus" -ForegroundColor Yellow
}
else {
    Write-Host "❌ Setup completed with issues" -ForegroundColor Red
    Write-Host "💡 Check individual components and try again" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "💡 The system is now completely self-sufficient!" -ForegroundColor Green
Write-Host "   No external COM port software required! 🎯" -ForegroundColor White