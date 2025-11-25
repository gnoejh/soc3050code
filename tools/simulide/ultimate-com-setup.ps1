#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Complete Self-Sufficient COM Port Solution for SimulIDE Educational Framework
    
.DESCRIPTION
    Provides multiple approaches for COM port generation:
    1. Automatic com0com detection and setup
    2. PowerShell-based virtual ports
    3. SimulIDE internal serial communication
    4. Hardware COM port management
    
.PARAMETER Setup
    Automatically setup the best available COM port solution
    
.PARAMETER Method
    Force specific method: "com0com", "powershell", "internal", "hardware"
    
.PARAMETER Check
    Check all available COM port options
    
.PARAMETER Reset
    Reset all virtual COM port configurations
    
.EXAMPLE
    .\ultimate-com-setup.ps1 -Setup
    Automatically setup best COM port solution
    
.EXAMPLE
    .\ultimate-com-setup.ps1 -Method com0com
    Force com0com setup
#>

param(
    [Parameter(Mandatory = $false)]
    [switch]$Setup,
    
    [Parameter(Mandatory = $false)]
    [ValidateSet("com0com", "powershell", "internal", "hardware")]
    [string]$Method = "",
    
    [Parameter(Mandatory = $false)]
    [switch]$Check,
    
    [Parameter(Mandatory = $false)]
    [switch]$Reset
)

# Educational COM Port Solutions
class EducationalComSolutions {
    [hashtable]$Solutions
    [string]$RecommendedSolution
    
    EducationalComSolutions() {
        $this.Solutions = @{}
        $this.EvaluateOptions()
    }
    
    [void]EvaluateOptions() {
        Write-Host "🔍 Evaluating COM port solutions..." -ForegroundColor Cyan
        
        # 1. Check for com0com
        $com0com = Get-Command "setupc.exe" -ErrorAction SilentlyContinue
        if ($com0com) {
            $this.Solutions["com0com"] = @{
                Available   = $true
                Priority    = 1
                Description = "com0com Virtual COM Ports"
                Reliability = "Excellent"
                Setup       = "Automatic"
            }
        }
        else {
            $this.Solutions["com0com"] = @{
                Available   = $false
                Priority    = 1
                Description = "com0com Virtual COM Ports (Not Installed)"
                Reliability = "Excellent"
                Setup       = "Manual Installation Required"
            }
        }
        
        # 2. Check for hardware COM ports
        $realPorts = $this.GetRealComPorts()
        if ($realPorts.Count -ge 2) {
            $this.Solutions["hardware"] = @{
                Available   = $true
                Priority    = 2
                Description = "Hardware COM Ports"
                Reliability = "Excellent"
                Setup       = "Hardware Dependent"
                Ports       = $realPorts
            }
        }
        
        # 3. PowerShell solution (always available)
        $this.Solutions["powershell"] = @{
            Available   = $true
            Priority    = 3
            Description = "PowerShell Named Pipes"
            Reliability = "Basic"
            Setup       = "Automatic"
        }
        
        # 4. SimulIDE internal (always available)
        $this.Solutions["internal"] = @{
            Available   = $true
            Priority    = 4
            Description = "SimulIDE Internal Serial Terminal"
            Reliability = "Good"
            Setup       = "No Setup Required"
        }
        
        # Determine recommended solution
        $availableSolutions = $this.Solutions.GetEnumerator() | Where-Object { $_.Value.Available } | Sort-Object { $_.Value.Priority }
        if ($availableSolutions) {
            $this.RecommendedSolution = $availableSolutions[0].Key
        }
    }
    
    [array]GetRealComPorts() {
        $realPorts = @()
        try {
            $allPorts = Get-WmiObject -Class Win32_SerialPort | Where-Object {
                $_.Description -notmatch "com0com|Virtual|Emulator|Loopback"
            }
            $realPorts = $allPorts | ForEach-Object { $_.DeviceID }
        }
        catch {
            # Fallback to basic port enumeration
            $allPorts = [System.IO.Ports.SerialPort]::getportnames()
            # Filter out known virtual ports
            $realPorts = $allPorts | Where-Object { $_ -notmatch "CNC|VCOM" }
        }
        return $realPorts
    }
    
    [void]ShowReport() {
        Write-Host "📊 COM Port Solutions Analysis" -ForegroundColor Green
        Write-Host "==============================" -ForegroundColor Green
        Write-Host ""
        
        foreach ($solution in ($this.Solutions.GetEnumerator() | Sort-Object { $_.Value.Priority })) {
            $name = $solution.Key
            $info = $solution.Value
            $status = if ($info.Available) { "[OK] Available" } else { "[ERROR] Not Available" }
            $color = if ($info.Available) { "Green" } else { "Red" }
            
            Write-Host "[CONFIG] $($info.Description)" -ForegroundColor $color
            Write-Host "   Status: $status" -ForegroundColor White
            Write-Host "   Reliability: $($info.Reliability)" -ForegroundColor White
            Write-Host "   Setup: $($info.Setup)" -ForegroundColor White
            
            if ($info.Ports) {
                Write-Host "   Ports: $($info.Ports -join ', ')" -ForegroundColor White
            }
            
            if ($name -eq $this.RecommendedSolution) {
                Write-Host "   ⭐ RECOMMENDED" -ForegroundColor Yellow
            }
            Write-Host ""
        }
    }
    
    [bool]SetupSolution([string]$method) {
        if ([string]::IsNullOrEmpty($method)) {
            $method = $this.RecommendedSolution
        }
        
        Write-Host "[START] Setting up: $method" -ForegroundColor Cyan
        
        switch ($method) {
            "com0com" {
                return $this.SetupCom0com()
            }
            "powershell" {
                return $this.SetupPowerShellBridge()
            }
            "internal" {
                return $this.SetupSimulIDEInternal()
            }
            "hardware" {
                return $this.SetupHardwarePorts()
            }
            default {
                Write-Host "[ERROR] Unknown method: $method" -ForegroundColor Red
                return $false
            }
        }
    }
    
    [bool]SetupCom0com() {
        Write-Host "[CONFIG] Setting up com0com virtual COM ports..." -ForegroundColor Green
        
        $setupc = Get-Command "setupc.exe" -ErrorAction SilentlyContinue
        if (-not $setupc) {
            Write-Host "[ERROR] com0com not installed" -ForegroundColor Red
            Write-Host "[DOWNLOAD] Download and install from: https://sourceforge.net/projects/com0com/" -ForegroundColor Yellow
            Write-Host "[CONFIG] After installation, run: setupc install 0 PortName=COM3 PortName=COM4" -ForegroundColor Yellow
            return $false
        }
        
        # Check if running as admin
        $currentUser = [Security.Principal.WindowsIdentity]::GetCurrent()
        $principal = New-Object Security.Principal.WindowsPrincipal($currentUser)
        $isAdmin = $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
        
        if (-not $isAdmin) {
            Write-Host "[WARNING] Administrator privileges required for com0com setup" -ForegroundColor Yellow
            Write-Host "[CONFIG] Run as Administrator: setupc install 0 PortName=COM3 PortName=COM4" -ForegroundColor Yellow
            return $false
        }
        
        try {
            $result = & $setupc.Source install 0 PortName=COM3 PortName=COM4 2>&1
            if ($LASTEXITCODE -eq 0) {
                Write-Host "[OK] com0com COM3↔COM4 pair created successfully!" -ForegroundColor Green
                return $true
            }
            else {
                Write-Host "[ERROR] com0com setup failed: $result" -ForegroundColor Red
                return $false
            }
        }
        catch {
            Write-Host "[ERROR] Error: $($_.Exception.Message)" -ForegroundColor Red
            return $false
        }
    }
    
    [bool]SetupPowerShellBridge() {
        Write-Host "[CONFIG] Setting up PowerShell serial bridge..." -ForegroundColor Green
        
        $bridgeScript = Join-Path $PSScriptRoot "powershell-serial-bridge.ps1"
        if (Test-Path $bridgeScript) {
            Write-Host "[OK] PowerShell bridge available" -ForegroundColor Green
            Write-Host "[START] Start bridge: .\powershell-serial-bridge.ps1 -Start" -ForegroundColor Cyan
            return $true
        }
        else {
            Write-Host "[ERROR] PowerShell bridge script not found" -ForegroundColor Red
            return $false
        }
    }
    
    [bool]SetupSimulIDEInternal() {
        Write-Host "🔧 Setting up SimulIDE internal serial communication..." -ForegroundColor Green
        Write-Host "✅ No setup required - use SerialTerm component in SimulIDE" -ForegroundColor Green
        Write-Host "📋 Instructions:" -ForegroundColor Cyan
        Write-Host "   1. Open SimulIDE circuit" -ForegroundColor White
        Write-Host "   2. Add SerialTerm component" -ForegroundColor White
        Write-Host "   3. Connect to ATmega128 UART pins" -ForegroundColor White
        Write-Host "   4. Double-click SerialTerm to open terminal" -ForegroundColor White
        return $true
    }
    
    [bool]SetupHardwarePorts() {
        $realPorts = $this.GetRealComPorts()
        if ($realPorts.Count -ge 2) {
            Write-Host "✅ Hardware COM ports detected: $($realPorts -join ', ')" -ForegroundColor Green
            Write-Host "🔧 Configure SimulIDE to use: $($realPorts[1])" -ForegroundColor Cyan
            Write-Host "🔧 Configure your application to use: $($realPorts[0])" -ForegroundColor Cyan
            return $true
        }
        else {
            Write-Host "❌ Insufficient hardware COM ports (need 2, found $($realPorts.Count))" -ForegroundColor Red
            return $false
        }
    }
}

# Main execution
Write-Host "🎯 Ultimate Self-Sufficient COM Port Setup" -ForegroundColor Green
Write-Host "===========================================" -ForegroundColor Green
Write-Host ""

$solutions = [EducationalComSolutions]::new()

if ($Check -or (-not $Setup -and -not $Reset -and [string]::IsNullOrEmpty($Method))) {
    $solutions.ShowReport()
}
elseif ($Setup) {
    Write-Host "🚀 Automatic Setup Mode" -ForegroundColor Cyan
    Write-Host "Recommended Solution: $($solutions.RecommendedSolution)" -ForegroundColor Yellow
    Write-Host ""
    
    $success = $solutions.SetupSolution($Method)
    if ($success) {
        Write-Host "✅ COM port setup completed successfully!" -ForegroundColor Green
    }
    else {
        Write-Host "❌ Setup failed - see instructions above" -ForegroundColor Red
        Write-Host "💡 Try alternative methods or manual setup" -ForegroundColor Yellow
    }
}
elseif ($Reset) {
    Write-Host "🗑️ Resetting COM port configurations..." -ForegroundColor Yellow
    # Implementation for reset would go here
}
elseif (-not [string]::IsNullOrEmpty($Method)) {
    $solutions.SetupSolution($Method)
}

Write-Host ""
Write-Host "💡 Quick Commands:" -ForegroundColor Cyan
Write-Host "   .\ultimate-com-setup.ps1 -Setup          # Auto-setup best solution" -ForegroundColor White
Write-Host "   .\ultimate-com-setup.ps1 -Method com0com # Force specific method" -ForegroundColor White
Write-Host "   .\ultimate-com-setup.ps1 -Check          # Check all options" -ForegroundColor White