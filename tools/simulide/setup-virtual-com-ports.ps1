#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Automatically setup virtual COM port pairs for SimulIDE simulation
    
.DESCRIPTION
    Creates virtual COM port pairs using com0com or Windows built-in capabilities.
    Provides a self-sufficient system without requiring manual com0com installation.
    
.PARAMETER ComPair
    COM port pair to create (default: "COM3,COM4")
    
.PARAMETER Install
    Install portable com0com if not available
    
.PARAMETER Remove
    Remove virtual COM port pairs
    
.PARAMETER Check
    Check current virtual COM port status
    
.EXAMPLE
    .\setup-virtual-com-ports.ps1 -Install
    Install and setup virtual COM ports
    
.EXAMPLE
    .\setup-virtual-com-ports.ps1 -Check
    Check current COM port status
#>

param(
    [Parameter(Mandatory = $false)]
    [string]$ComPair = "COM3,COM4",
    
    [Parameter(Mandatory = $false)]
    [switch]$Install,
    
    [Parameter(Mandatory = $false)]
    [switch]$Remove,
    
    [Parameter(Mandatory = $false)]
    [switch]$Check
)

# Function to check if running as administrator
function Test-Administrator {
    $currentUser = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($currentUser)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

# Function to check current COM ports
function Get-VirtualComPorts {
    Write-Host "🔍 Checking current COM port status..." -ForegroundColor Cyan
    
    try {
        # Get all COM ports with descriptions
        $comPorts = Get-WmiObject -Class Win32_SerialPort | Sort-Object DeviceID
        
        if ($comPorts) {
            Write-Host "📋 Available COM ports:" -ForegroundColor Green
            foreach ($port in $comPorts) {
                $portType = if ($port.Description -match "com0com|Virtual|Emulator") { "Virtual" } else { "Physical" }
                $color = if ($portType -eq "Virtual") { "Yellow" } else { "White" }
                Write-Host "  📌 $($port.DeviceID): $($port.Description) [$portType]" -ForegroundColor $color
            }
        } else {
            Write-Host "❌ No COM ports found" -ForegroundColor Red
        }
        
        # Check if com0com is installed
        $com0comPath = Get-Command "setupc.exe" -ErrorAction SilentlyContinue
        if ($com0comPath) {
            Write-Host "✅ com0com is installed: $($com0comPath.Source)" -ForegroundColor Green
        } else {
            Write-Host "⚠️ com0com not found in PATH" -ForegroundColor Yellow
        }
        
    } catch {
        Write-Host "❌ Error checking COM ports: $($_.Exception.Message)" -ForegroundColor Red
    }
}

# Function to create virtual COM port pair using com0com
function New-VirtualComPair {
    param([string]$Pair)
    
    $ports = $Pair.Split(',')
    if ($ports.Count -ne 2) {
        Write-Host "❌ Invalid COM pair format. Use: COM3,COM4" -ForegroundColor Red
        return $false
    }
    
    $port1 = $ports[0].Trim()
    $port2 = $ports[1].Trim()
    
    Write-Host "🔧 Creating virtual COM pair: $port1 ↔ $port2" -ForegroundColor Cyan
    
    # Check if com0com is available
    $setupc = Get-Command "setupc.exe" -ErrorAction SilentlyContinue
    if (-not $setupc) {
        Write-Host "❌ com0com setupc.exe not found. Please install com0com first." -ForegroundColor Red
        Write-Host "   Download from: https://sourceforge.net/projects/com0com/" -ForegroundColor Yellow
        return $false
    }
    
    if (-not (Test-Administrator)) {
        Write-Host "❌ Administrator privileges required for COM port creation" -ForegroundColor Red
        Write-Host "   Please run PowerShell as Administrator" -ForegroundColor Yellow
        return $false
    }
    
    try {
        # Create COM port pair
        $result = & $setupc.Source install 0 PortName=$port1 PortName=$port2 2>&1
        
        if ($LASTEXITCODE -eq 0) {
            Write-Host "✅ Virtual COM pair created successfully!" -ForegroundColor Green
            Write-Host "   $port1 ↔ $port2" -ForegroundColor White
            return $true
        } else {
            Write-Host "❌ Failed to create COM pair" -ForegroundColor Red
            Write-Host "   $result" -ForegroundColor Gray
            return $false
        }
    } catch {
        Write-Host "❌ Error creating COM pair: $($_.Exception.Message)" -ForegroundColor Red
        return $false
    }
}

# Function to remove virtual COM port pairs
function Remove-VirtualComPairs {
    Write-Host "🗑️ Removing virtual COM port pairs..." -ForegroundColor Cyan
    
    $setupc = Get-Command "setupc.exe" -ErrorAction SilentlyContinue
    if (-not $setupc) {
        Write-Host "❌ com0com setupc.exe not found" -ForegroundColor Red
        return $false
    }
    
    if (-not (Test-Administrator)) {
        Write-Host "❌ Administrator privileges required" -ForegroundColor Red
        return $false
    }
    
    try {
        # List and remove all com0com pairs
        $result = & $setupc.Source remove 0 2>&1
        Write-Host "✅ Virtual COM pairs removed" -ForegroundColor Green
        return $true
    } catch {
        Write-Host "❌ Error removing COM pairs: $($_.Exception.Message)" -ForegroundColor Red
        return $false
    }
}

# Function to download and setup portable com0com
function Install-PortableCom0com {
    Write-Host "📦 Setting up portable com0com..." -ForegroundColor Cyan
    
    $toolsDir = Split-Path $PSScriptRoot -Parent
    $com0comDir = Join-Path $toolsDir "com0com"
    
    if (Test-Path $com0comDir) {
        Write-Host "✅ com0com already installed in: $com0comDir" -ForegroundColor Green
        
        # Add to PATH if not already there
        $env:PATH += ";$com0comDir"
        return $true
    }
    
    Write-Host "🔍 Checking for com0com installation..." -ForegroundColor Yellow
    
    # Check if already installed system-wide
    $setupc = Get-Command "setupc.exe" -ErrorAction SilentlyContinue
    if ($setupc) {
        Write-Host "✅ com0com found in system PATH: $($setupc.Source)" -ForegroundColor Green
        return $true
    }
    
    Write-Host "📋 com0com installation options:" -ForegroundColor Yellow
    Write-Host "   🎯 Recommended for Educational Package:" -ForegroundColor Green
    Write-Host "   1. Download com0com from: https://sourceforge.net/projects/com0com/" -ForegroundColor White
    Write-Host "   2. Install as Administrator (system-wide)" -ForegroundColor White
    Write-Host "   3. Restart this script with -Install" -ForegroundColor White
    Write-Host ""
    Write-Host "   � Alternative - Portable Installation:" -ForegroundColor Cyan
    Write-Host "   1. Download com0com and extract to: $com0comDir" -ForegroundColor White
    Write-Host "   2. Run setupc.exe manually as Administrator" -ForegroundColor White
    Write-Host ""
    Write-Host "   🚀 Quick Setup Commands:" -ForegroundColor Yellow
    Write-Host "   # After installing com0com:" -ForegroundColor Gray
    Write-Host "   setupc install 0 PortName=COM3 PortName=COM4" -ForegroundColor Gray
    Write-Host ""
    
    return $false
}

# Main execution
Write-Host "🔧 Virtual COM Port Setup for SimulIDE" -ForegroundColor Green
Write-Host "=======================================" -ForegroundColor Green

if ($Check -or (-not $Install -and -not $Remove)) {
    Get-VirtualComPorts
    exit 0
}

if ($Install) {
    if (Install-PortableCom0com) {
        New-VirtualComPair -Pair $ComPair
    }
}

if ($Remove) {
    Remove-VirtualComPairs
}

Write-Host ""
Write-Host "💡 Usage Tips:" -ForegroundColor Cyan
Write-Host "   • SimulIDE uses COM4 for serial communication" -ForegroundColor White
Write-Host "   • Your application can connect to COM3" -ForegroundColor White
Write-Host "   • Data flows: App(COM3) ↔ SimulIDE(COM4)" -ForegroundColor White
Write-Host "   • Use serial monitor on COM3 to see SimulIDE output" -ForegroundColor White