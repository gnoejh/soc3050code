#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Automatically configure SimulIDE circuit for TCP socket communication
    
.DESCRIPTION
    Updates the SimulIDE circuit file to use TCP socket communication instead
    of COM ports, making the system fully self-sufficient.
    
.PARAMETER CircuitFile
    Path to SimulIDE circuit file (default: auto-detect)
    
.PARAMETER TcpPort
    TCP port for SimulIDE communication (default: 9002)
    
.PARAMETER Restore
    Restore original COM port configuration
    
.EXAMPLE
    .\configure-simulide-tcp.ps1
    Auto-configure circuit for TCP communication
    
.EXAMPLE
    .\configure-simulide-tcp.ps1 -Restore
    Restore original COM port settings
#>

param(
    [Parameter(Mandatory = $false)]
    [string]$CircuitFile = "",
    
    [Parameter(Mandatory = $false)]
    [int]$TcpPort = 9002,
    
    [Parameter(Mandatory = $false)]
    [switch]$Restore
)

# Function to find SimulIDE circuit file
function Find-SimulIDECircuit {
    $possiblePaths = @(
        ".\Simulator110.simu",
        "..\Simulator110.simu", 
        "tools\simulide\Simulator110.simu",
        "w:\soc3050code\tools\simulide\Simulator110.simu"
    )
    
    foreach ($path in $possiblePaths) {
        if (Test-Path $path) {
            return (Resolve-Path $path).Path
        }
    }
    
    return $null
}

# Function to backup circuit file
function Backup-CircuitFile {
    param([string]$FilePath)
    
    $backupPath = $FilePath + ".backup"
    if (-not (Test-Path $backupPath)) {
        Copy-Item $FilePath $backupPath
        Write-Host "✅ Circuit backup created: $backupPath" -ForegroundColor Green
    }
    return $backupPath
}

# Function to configure TCP socket communication
function Set-TcpSocketCommunication {
    param([string]$FilePath, [int]$Port)
    
    Write-Host "🔧 Configuring SimulIDE for TCP socket communication..." -ForegroundColor Cyan
    Write-Host "   Circuit: $FilePath" -ForegroundColor White
    Write-Host "   TCP Port: localhost:$Port" -ForegroundColor White
    
    try {
        # Read circuit file
        $content = Get-Content $FilePath -Raw
        
        # Backup original
        Backup-CircuitFile -FilePath $FilePath
        
        # Replace SerialPort COM configuration with TCP
        # Look for SerialPort component and update its Port property
        $pattern = '(<item itemtype="SerialPort"[^>]*)(Port="[^"]*")([^>]*>)'
        $replacement = "`$1Port=`"localhost:$Port`"`$3"
        
        $newContent = $content -replace $pattern, $replacement
        
        # Also update any other COM port references
        $newContent = $newContent -replace 'Port="COM\d+"', "Port=`"localhost:$Port`""
        
        # Write updated content
        Set-Content $FilePath $newContent -NoNewline
        
        Write-Host "✅ Circuit configured for TCP socket communication" -ForegroundColor Green
        Write-Host "💡 SimulIDE will now use: localhost:$Port" -ForegroundColor Yellow
        Write-Host "💡 Your applications connect to: localhost:9001" -ForegroundColor Yellow
        
        return $true
        
    }
    catch {
        Write-Host "❌ Error configuring circuit: $($_.Exception.Message)" -ForegroundColor Red
        return $false
    }
}

# Function to restore original COM port settings
function Restore-ComPortSettings {
    param([string]$FilePath)
    
    $backupPath = $FilePath + ".backup"
    
    if (Test-Path $backupPath) {
        Write-Host "🔄 Restoring original COM port settings..." -ForegroundColor Cyan
        Copy-Item $backupPath $FilePath -Force
        Write-Host "✅ Original settings restored from backup" -ForegroundColor Green
        return $true
    }
    else {
        Write-Host "❌ No backup file found: $backupPath" -ForegroundColor Red
        Write-Host "💡 Manually edit circuit to restore COM4 settings" -ForegroundColor Yellow
        return $false
    }
}

# Function to show current circuit configuration
function Show-CircuitConfiguration {
    param([string]$FilePath)
    
    Write-Host "📋 Current SimulIDE Circuit Configuration" -ForegroundColor Cyan
    Write-Host "=========================================" -ForegroundColor Cyan
    Write-Host "Circuit File: $FilePath" -ForegroundColor White
    
    if (Test-Path $FilePath) {
        $content = Get-Content $FilePath -Raw
        
        # Extract SerialPort configuration
        if ($content -match '<item itemtype="SerialPort"[^>]*Port="([^"]*)"[^>]*>') {
            $portConfig = $matches[1]
            Write-Host "Serial Port: $portConfig" -ForegroundColor White
            
            if ($portConfig -like "localhost:*") {
                Write-Host "✅ Configured for TCP socket communication" -ForegroundColor Green
            }
            elseif ($portConfig -like "COM*") {
                Write-Host "⚠️ Configured for COM port communication" -ForegroundColor Yellow
            }
            else {
                Write-Host "❓ Unknown port configuration" -ForegroundColor Yellow
            }
        }
        else {
            Write-Host "❌ No SerialPort component found" -ForegroundColor Red
        }
        
        # Check for backup
        $backupPath = $FilePath + ".backup"
        if (Test-Path $backupPath) {
            Write-Host "✅ Backup available: $backupPath" -ForegroundColor Green
        }
        else {
            Write-Host "⚠️ No backup found" -ForegroundColor Yellow
        }
    }
    else {
        Write-Host "❌ Circuit file not found" -ForegroundColor Red
    }
}

# Main execution
Write-Host "🔧 SimulIDE TCP Socket Configuration" -ForegroundColor Green
Write-Host "====================================" -ForegroundColor Green
Write-Host ""

# Find circuit file if not specified
if ([string]::IsNullOrEmpty($CircuitFile)) {
    $CircuitFile = Find-SimulIDECircuit
    if (-not $CircuitFile) {
        Write-Host "❌ Could not find SimulIDE circuit file" -ForegroundColor Red
        Write-Host "💡 Specify path with -CircuitFile parameter" -ForegroundColor Yellow
        exit 1
    }
}

if ($Restore) {
    $success = Restore-ComPortSettings -FilePath $CircuitFile
    if ($success) {
        Write-Host "🚀 Restart SimulIDE to use COM port communication" -ForegroundColor Cyan
    }
}
else {
    Show-CircuitConfiguration -FilePath $CircuitFile
    Write-Host ""
    
    $success = Set-TcpSocketCommunication -FilePath $CircuitFile -Port $TcpPort
    if ($success) {
        Write-Host ""
        Write-Host "🚀 Next Steps:" -ForegroundColor Cyan
        Write-Host "   1. Start TCP bridge: .\windows-virtual-com.ps1 -Method tcpsockets -Start" -ForegroundColor White
        Write-Host "   2. Launch SimulIDE - it will automatically use TCP communication" -ForegroundColor White
        Write-Host "   3. Your applications connect to: localhost:9001" -ForegroundColor White
        Write-Host ""
        Write-Host "💡 The system is now fully self-sufficient!" -ForegroundColor Yellow
    }
}

Write-Host ""
Write-Host "💡 Quick Commands:" -ForegroundColor Cyan
Write-Host "   .\configure-simulide-tcp.ps1           # Configure for TCP" -ForegroundColor White
Write-Host "   .\configure-simulide-tcp.ps1 -Restore  # Restore COM ports" -ForegroundColor White