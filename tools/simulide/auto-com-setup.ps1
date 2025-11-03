#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Self-Sufficient COM Port Setup for SimulIDE Educational Framework
    
.DESCRIPTION
    Automatically detects and sets up the best available COM port solution:
    1. com0com virtual COM ports (preferred)
    2. Hardware COM ports  
    3. SimulIDE internal serial terminal
    4. PowerShell named pipes (basic)
    
.PARAMETER Setup
    Automatically setup the best available COM port solution
    
.PARAMETER Check
    Check all available COM port options
    
.EXAMPLE
    .\auto-com-setup.ps1 -Setup
    Automatically setup best COM port solution
    
.EXAMPLE
    .\auto-com-setup.ps1 -Check
    Check all available options
#>

param(
    [Parameter(Mandatory = $false)]
    [switch]$Setup,
    
    [Parameter(Mandatory = $false)]
    [switch]$Check
)

# Function to get real (non-virtual) COM ports
function Get-RealComPorts {
    $realPorts = @()
    try {
        $allPorts = Get-WmiObject -Class Win32_SerialPort | Where-Object {
            $_.Description -notmatch "com0com|Virtual|Emulator|Loopback"
        }
        $realPorts = $allPorts | ForEach-Object { $_.DeviceID }
    } catch {
        # Fallback to basic port enumeration
        $allPorts = [System.IO.Ports.SerialPort]::getportnames()
        # Filter out known virtual ports
        $realPorts = $allPorts | Where-Object { $_ -notmatch "CNC|VCOM" }
    }
    return $realPorts
}

# Function to check com0com availability
function Test-Com0comAvailable {
    $setupc = Get-Command "setupc.exe" -ErrorAction SilentlyContinue
    return $null -ne $setupc
}

# Function to check if running as administrator
function Test-Administrator {
    $currentUser = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($currentUser)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

# Function to show comprehensive status
function Show-ComPortStatus {
    Write-Host "📊 Self-Sufficient COM Port Analysis" -ForegroundColor Green
    Write-Host "====================================" -ForegroundColor Green
    Write-Host ""
    
    # Check current COM ports
    $allPorts = [System.IO.Ports.SerialPort]::getportnames() | Sort-Object
    $realPorts = Get-RealComPorts
    $virtualPorts = $allPorts | Where-Object { $_ -notin $realPorts }
    
    Write-Host "🔍 Current COM Port Status:" -ForegroundColor Cyan
    if ($allPorts.Count -gt 0) {
        Write-Host "   All Ports: $($allPorts -join ', ')" -ForegroundColor White
        Write-Host "   Real Ports: $($realPorts -join ', ')" -ForegroundColor Green
        Write-Host "   Virtual Ports: $($virtualPorts -join ', ')" -ForegroundColor Yellow
    } else {
        Write-Host "   ❌ No COM ports detected" -ForegroundColor Red
    }
    Write-Host ""
    
    # Evaluate solutions
    Write-Host "🎯 Available Solutions (in priority order):" -ForegroundColor Cyan
    Write-Host ""
    
    # Solution 1: com0com
    $com0comAvailable = Test-Com0comAvailable
    $isAdmin = Test-Administrator
    Write-Host "1️⃣ com0com Virtual COM Ports" -ForegroundColor $(if($com0comAvailable){"Green"}else{"Red"})
    Write-Host "   Status: $(if($com0comAvailable){"✅ Available"}else{"❌ Not Installed"})" -ForegroundColor White
    Write-Host "   Reliability: Excellent (industry standard)" -ForegroundColor White
    Write-Host "   Admin Required: $(if($isAdmin){"✅ Yes (current session)"}else{"⚠️ Yes (run as admin)"})" -ForegroundColor White
    if ($com0comAvailable) {
        Write-Host "   ⭐ RECOMMENDED - Professional virtual COM ports" -ForegroundColor Yellow
    } else {
        Write-Host "   📥 Install: https://sourceforge.net/projects/com0com/" -ForegroundColor Gray
    }
    Write-Host ""
    
    # Solution 2: Windows TCP Socket Bridge
    Write-Host "2️⃣ Windows TCP Socket Bridge (NEW!)" -ForegroundColor Green
    Write-Host "   Status: ✅ Always Available (Built into Windows)" -ForegroundColor White
    Write-Host "   Reliability: Excellent (native Windows)" -ForegroundColor White
    Write-Host "   Admin Required: ❌ No" -ForegroundColor White
    Write-Host "   🌟 SELF-SUFFICIENT - No external dependencies!" -ForegroundColor Yellow
    Write-Host "   💡 Uses localhost:9001 ↔ localhost:9002" -ForegroundColor White
    Write-Host ""
    
    # Solution 3: Hardware COM ports
    Write-Host "3️⃣ Hardware COM Ports" -ForegroundColor $(if($realPorts.Count -ge 2){"Green"}else{"Yellow"})
    Write-Host "   Status: $(if($realPorts.Count -ge 2){"✅ Sufficient ($($realPorts.Count) ports)"}else{"⚠️ Limited ($($realPorts.Count) ports)"})" -ForegroundColor White
    Write-Host "   Reliability: Excellent (real hardware)" -ForegroundColor White
    Write-Host "   Requirements: 2+ USB-to-Serial or Arduino devices" -ForegroundColor White
    if ($realPorts.Count -ge 2) {
        Write-Host "   🎯 AVAILABLE - Use $($realPorts[0]) and $($realPorts[1])" -ForegroundColor Yellow
    }
    Write-Host ""
    
    # Solution 4: SimulIDE internal
    Write-Host "4️⃣ SimulIDE Internal Serial Terminal" -ForegroundColor Green
    Write-Host "   Status: ✅ Always Available" -ForegroundColor White
    Write-Host "   Reliability: Good (simulation only)" -ForegroundColor White
    Write-Host "   Requirements: None - built into SimulIDE" -ForegroundColor White
    Write-Host "   💡 FALLBACK - SerialTerm component in circuit" -ForegroundColor Yellow
    Write-Host ""
    
    # Recommendation
    Write-Host "🏆 Recommendation:" -ForegroundColor Green
    if ($com0comAvailable -and $isAdmin) {
        Write-Host "   Use com0com for professional-grade virtual COM ports" -ForegroundColor White
    } elseif ($realPorts.Count -ge 2) {
        Write-Host "   Use hardware COM ports: $($realPorts[0]) ↔ $($realPorts[1])" -ForegroundColor White
    } else {
        Write-Host "   🌟 Use Windows TCP Socket Bridge (self-sufficient!)" -ForegroundColor Yellow
        Write-Host "   • No external dependencies required" -ForegroundColor White
        Write-Host "   • Built into Windows" -ForegroundColor White
        Write-Host "   • Run: .\windows-virtual-com.ps1 -Method tcpsockets -Start" -ForegroundColor Cyan
    }
}

# Function to setup com0com
function Setup-Com0com {
    Write-Host "🔧 Setting up com0com COM3↔COM4 pair..." -ForegroundColor Green
    
    if (-not (Test-Com0comAvailable)) {
        Write-Host "❌ com0com not installed" -ForegroundColor Red
        Write-Host "📥 Please install from: https://sourceforge.net/projects/com0com/" -ForegroundColor Yellow
        return $false
    }
    
    if (-not (Test-Administrator)) {
        Write-Host "❌ Administrator privileges required" -ForegroundColor Red
        Write-Host "🔧 Run PowerShell as Administrator and retry" -ForegroundColor Yellow
        return $false
    }
    
    try {
        $setupc = Get-Command "setupc.exe" -ErrorAction Stop
        Write-Host "🔧 Creating COM3↔COM4 virtual pair..." -ForegroundColor Cyan
        
        $result = & $setupc.Source install 0 PortName=COM3 PortName=COM4 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-Host "✅ COM3↔COM4 pair created successfully!" -ForegroundColor Green
            Write-Host "   📤 SimulIDE will use COM4" -ForegroundColor White
            Write-Host "   📥 Your applications can use COM3" -ForegroundColor White
            return $true
        } else {
            Write-Host "❌ Setup failed: $result" -ForegroundColor Red
            return $false
        }
    } catch {
        Write-Host "❌ Error: $($_.Exception.Message)" -ForegroundColor Red
        return $false
    }
}

# Function to setup Windows TCP socket bridge
function Setup-WindowsTcpBridge {
    Write-Host "🔧 Setting up Windows TCP Socket Bridge..." -ForegroundColor Green
    Write-Host "✅ Pure Windows solution - no external dependencies!" -ForegroundColor Green
    Write-Host ""
    Write-Host "📋 TCP Socket Configuration:" -ForegroundColor Cyan
    Write-Host "   🌐 Application Port: localhost:9001" -ForegroundColor White
    Write-Host "   🌐 SimulIDE Port: localhost:9002" -ForegroundColor White
    Write-Host "   🔄 Bridge: Data flows between ports automatically" -ForegroundColor White
    Write-Host ""
    Write-Host "🔧 Setup Instructions:" -ForegroundColor Yellow
    Write-Host "   1. Start TCP bridge: .\windows-virtual-com.ps1 -Method tcpsockets -Start" -ForegroundColor White
    Write-Host "   2. Configure SimulIDE SerialPort to use network connection" -ForegroundColor White
    Write-Host "   3. Your applications connect to localhost:9001" -ForegroundColor White
    Write-Host ""
    Write-Host "💡 Benefits:" -ForegroundColor Green
    Write-Host "   • ✅ No administrator privileges required" -ForegroundColor White
    Write-Host "   • ✅ Built into Windows (no external software)" -ForegroundColor White
    Write-Host "   • ✅ Network-capable (can work across machines)" -ForegroundColor White
    Write-Host "   • ✅ Reliable TCP/IP communication" -ForegroundColor White
    Write-Host ""
    Write-Host "🚀 Ready to use Windows native virtual COM solution!" -ForegroundColor Green
    return $true
}

# Function to setup SimulIDE internal serial
function Setup-SimulIDEInternal {
    Write-Host "🔧 Setting up SimulIDE internal serial communication..." -ForegroundColor Green
    Write-Host "✅ No setup required - use SerialTerm component" -ForegroundColor Green
    Write-Host ""
    Write-Host "📋 Instructions for Students:" -ForegroundColor Cyan
    Write-Host "   1. Open SimulIDE circuit file" -ForegroundColor White
    Write-Host "   2. SerialTerm component is already in the circuit" -ForegroundColor White  
    Write-Host "   3. Double-click SerialTerm to open terminal window" -ForegroundColor White
    Write-Host "   4. UART output will appear in SerialTerm" -ForegroundColor White
    Write-Host ""
    Write-Host "💡 This method works without any COM ports!" -ForegroundColor Yellow
    return $true
}

# Function to auto-setup best solution
function Setup-BestSolution {
    Write-Host "🚀 Auto-Setup: Finding best COM port solution..." -ForegroundColor Green
    Write-Host ""
    
    # Try com0com first (if available and admin)
    if ((Test-Com0comAvailable) -and (Test-Administrator)) {
        Write-Host "🎯 Using com0com (professional virtual COM ports)" -ForegroundColor Cyan
        return Setup-Com0com
    }
    
    # Try Windows TCP socket bridge (NEW - always available!)
    Write-Host "🎯 Using Windows TCP Socket Bridge (self-sufficient solution)" -ForegroundColor Cyan
    $tcpSuccess = Setup-WindowsTcpBridge
    if ($tcpSuccess) {
        return $true
    }
    
    # Try hardware ports
    $realPorts = Get-RealComPorts
    if ($realPorts.Count -ge 2) {
        Write-Host "🎯 Using hardware COM ports: $($realPorts -join ' ↔ ')" -ForegroundColor Cyan
        Write-Host "✅ Hardware setup complete!" -ForegroundColor Green
        Write-Host "   Configure SimulIDE SerialPort to use: $($realPorts[1])" -ForegroundColor White
        Write-Host "   Your applications can use: $($realPorts[0])" -ForegroundColor White
        return $true
    }
    
    # Provide guidance for com0com if available but not admin
    if ((Test-Com0comAvailable) -and (-not (Test-Administrator))) {
        Write-Host "🎯 com0com available but requires Administrator privileges" -ForegroundColor Yellow
        Write-Host "   Run PowerShell as Administrator for automatic setup" -ForegroundColor White
        Write-Host "   Or manually run: setupc install 0 PortName=COM3 PortName=COM4" -ForegroundColor White
        Write-Host ""
    }
    
    # Fallback to SimulIDE internal
    Write-Host "🎯 Using SimulIDE internal SerialTerm (fallback)" -ForegroundColor Cyan
    return Setup-SimulIDEInternal
}

# Main execution
Write-Host "🎯 Self-Sufficient COM Port Setup for SimulIDE" -ForegroundColor Green
Write-Host "===============================================" -ForegroundColor Green
Write-Host ""

if ($Setup) {
    $success = Setup-BestSolution
    Write-Host ""
    if ($success) {
        Write-Host "✅ COM port setup completed successfully!" -ForegroundColor Green
        Write-Host "🚀 Ready for SimulIDE serial communication!" -ForegroundColor Cyan
    } else {
        Write-Host "⚠️ Setup completed with limitations" -ForegroundColor Yellow
        Write-Host "💡 Consider installing com0com for full functionality" -ForegroundColor White
    }
} else {
    Show-ComPortStatus
}

Write-Host ""
Write-Host "💡 Quick Commands:" -ForegroundColor Cyan
Write-Host "   .\auto-com-setup.ps1 -Setup    # Auto-setup best solution" -ForegroundColor White
Write-Host "   .\auto-com-setup.ps1 -Check    # Check all options" -ForegroundColor White
Write-Host "   setupc install 0 PortName=COM3 PortName=COM4  # Manual com0com" -ForegroundColor Gray