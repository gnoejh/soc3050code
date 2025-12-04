#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Deploy ATmega128 Multi-User Dashboard for Classroom
    
.DESCRIPTION
    Sets up the multi-user web dashboard for classroom use.
    Configures networking, creates service, and provides deployment instructions.
    
.PARAMETER ServerIP
    IP address of the server (default: auto-detect)
    
.PARAMETER Port
    Port for the web dashboard (default: 5001)
    
.PARAMETER MaxStudents
    Maximum number of concurrent students (default: 50)
    
.EXAMPLE
    .\deploy-classroom-dashboard.ps1
    .\deploy-classroom-dashboard.ps1 -ServerIP "192.168.1.100" -Port 8080
#>

param(
    [string]$ServerIP = "",
    [int]$Port = 5001,
    [int]$MaxStudents = 50
)

$WorkspaceRoot = Split-Path $PSScriptRoot -Parent
$InfoColor = "Cyan"
$SuccessColor = "Green"
$WarningColor = "Yellow"
$ErrorColor = "Red"

Write-Host "=" * 70 -ForegroundColor $InfoColor
Write-Host "🏫 ATmega128 CLASSROOM DASHBOARD DEPLOYMENT" -ForegroundColor $InfoColor
Write-Host "=" * 70 -ForegroundColor $InfoColor

# Auto-detect server IP if not provided
if (-not $ServerIP) {
    try {
        $ServerIP = (Get-NetIPAddress -AddressFamily IPv4 | Where-Object { 
                $_.IPAddress -ne "127.0.0.1" -and 
                $_.AddressState -eq "Preferred" -and
                $_.IPAddress -like "192.168.*" -or $_.IPAddress -like "10.*" -or $_.IPAddress -like "172.*"
            } | Select-Object -First 1).IPAddress
        
        if ($ServerIP) {
            Write-Host "✅ Auto-detected server IP: $ServerIP" -ForegroundColor $SuccessColor
        }
        else {
            $ServerIP = "127.0.0.1"
            Write-Host "⚠️  Could not auto-detect IP, using localhost: $ServerIP" -ForegroundColor $WarningColor
        }
    }
    catch {
        $ServerIP = "127.0.0.1"
        Write-Host "⚠️  Network detection failed, using localhost: $ServerIP" -ForegroundColor $WarningColor
    }
}

Write-Host ""
Write-Host "CONFIGURATION" -ForegroundColor $InfoColor
Write-Host "─────────────────" -ForegroundColor Gray
Write-Host "Server IP: $ServerIP" -ForegroundColor White
Write-Host "Port: $Port" -ForegroundColor White
Write-Host "Max Students: $MaxStudents" -ForegroundColor White
Write-Host ""

# Check prerequisites
Write-Host "CHECKING PREREQUISITES" -ForegroundColor $InfoColor
Write-Host "──────────────────────────" -ForegroundColor Gray

$pythonPath = Get-Command python -ErrorAction SilentlyContinue
if (-not $pythonPath) {
    Write-Host "❌ Python not found!" -ForegroundColor $ErrorColor
    Write-Host "   Please install Python 3.8+ from https://python.org" -ForegroundColor $WarningColor
    exit 1
}

$pythonVersion = python --version 2>&1
Write-Host "✅ Python found: $pythonVersion" -ForegroundColor $SuccessColor

# Check Python packages
$requiredPackages = @("flask", "flask-socketio", "eventlet")
$missingPackages = @()

foreach ($package in $requiredPackages) {
    try {
        $result = python -c "import $package" 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-Host "✅ $package installed" -ForegroundColor $SuccessColor
        }
        else {
            $missingPackages += $package
        }
    }
    catch {
        $missingPackages += $package
    }
}

if ($missingPackages.Count -gt 0) {
    Write-Host "📦 Installing missing packages..." -ForegroundColor $InfoColor
    foreach ($package in $missingPackages) {
        Write-Host "   Installing $package..." -ForegroundColor White
        pip install $package
        if ($LASTEXITCODE -eq 0) {
            Write-Host "   ✅ $package installed successfully" -ForegroundColor $SuccessColor
        }
        else {
            Write-Host "   ❌ Failed to install $package" -ForegroundColor $ErrorColor
            exit 1
        }
    }
}

Write-Host ""

# Create configuration file
Write-Host "CREATING CONFIGURATION" -ForegroundColor $InfoColor
Write-Host "─────────────────────────" -ForegroundColor Gray

$configDir = Join-Path $WorkspaceRoot "dashboards\config"
if (-not (Test-Path $configDir)) {
    New-Item -ItemType Directory -Path $configDir -Force | Out-Null
}

$configFile = Join-Path $configDir "multi_user_config.json"
$config = @{
    server_ip              = $ServerIP
    port                   = $Port
    max_students           = $MaxStudents
    session_timeout_hours  = 8
    workspace_cleanup_days = 7
    auto_backup            = $true
    teacher_password       = "admin123"  # Change this!
    debug                  = $false
} | ConvertTo-Json -Depth 3

Set-Content -Path $configFile -Value $config -Encoding UTF8
Write-Host "✅ Configuration saved: $configFile" -ForegroundColor $SuccessColor

# Create startup script
$startupScript = Join-Path $WorkspaceRoot "start-classroom-dashboard.bat"
$startupContent = @"
@echo off
cd /d "$WorkspaceRoot"
echo Starting ATmega128 Classroom Dashboard...
echo Server: http://$ServerIP`:$Port
echo.
python dashboards\multi_user_dashboard.py
pause
"@

Set-Content -Path $startupScript -Value $startupContent -Encoding ASCII
Write-Host "✅ Startup script created: $startupScript" -ForegroundColor $SuccessColor

# Create Windows service script (optional)
$serviceScript = Join-Path $WorkspaceRoot "install-dashboard-service.ps1"
$serviceContent = @"
# Run as Administrator to install Windows service
`$serviceName = "ATmega128Dashboard"
`$serviceDescription = "ATmega128 Multi-User Educational Dashboard"
`$pythonPath = (Get-Command python).Source
`$scriptPath = "$WorkspaceRoot\dashboards\multi_user_dashboard.py"

# Install service using NSSM (Non-Sucking Service Manager)
# Download NSSM from: https://nssm.cc/download
# nssm install `$serviceName `$pythonPath `$scriptPath
# nssm set `$serviceName Description "`$serviceDescription"
# nssm start `$serviceName

Write-Host "Service installation script created."
Write-Host "To install as Windows service:"
Write-Host "1. Download NSSM from https://nssm.cc/download"
Write-Host "2. Extract nssm.exe to System32"
Write-Host "3. Run this script as Administrator"
"@

Set-Content -Path $serviceScript -Value $serviceContent -Encoding UTF8
Write-Host "✅ Service script created: $serviceScript" -ForegroundColor $SuccessColor

Write-Host ""

# Check firewall
Write-Host "FIREWALL CONFIGURATION" -ForegroundColor $InfoColor
Write-Host "─────────────────────────" -ForegroundColor Gray

try {
    $firewallRule = Get-NetFirewallRule -DisplayName "ATmega128 Dashboard" -ErrorAction SilentlyContinue
    if (-not $firewallRule) {
        Write-Host "📡 Creating firewall rule for port $Port..." -ForegroundColor $InfoColor
        New-NetFirewallRule -DisplayName "ATmega128 Dashboard" -Direction Inbound -Protocol TCP -LocalPort $Port -Action Allow | Out-Null
        Write-Host "✅ Firewall rule created" -ForegroundColor $SuccessColor
    }
    else {
        Write-Host "✅ Firewall rule already exists" -ForegroundColor $SuccessColor
    }
}
catch {
    Write-Host "⚠️  Could not configure firewall automatically" -ForegroundColor $WarningColor
    Write-Host "   Manual firewall configuration may be required" -ForegroundColor $WarningColor
}

Write-Host ""

# Network testing
Write-Host "NETWORK TESTING" -ForegroundColor $InfoColor
Write-Host "───────────────────" -ForegroundColor Gray

$testPort = Test-NetConnection -ComputerName $ServerIP -Port $Port -WarningAction SilentlyContinue
Write-Host "📡 Testing network connectivity..." -ForegroundColor $InfoColor
Write-Host "   Server: $ServerIP`:$Port" -ForegroundColor White

Write-Host ""

# Create student access information
$studentInfoFile = Join-Path $WorkspaceRoot "STUDENT_ACCESS_INFO.txt"
$studentInfo = @"
ATmega128 CLASSROOM DASHBOARD - STUDENT ACCESS
============================================

🌐 STUDENT LOGIN URL:
   http://$ServerIP`:$Port

📱 QR CODE ACCESS:
   Generate QR code for the URL above using any QR generator

👥 CLASSROOM INSTRUCTIONS:

1. Students connect to classroom WiFi
2. Open web browser (Chrome, Firefox, Edge, Safari)
3. Go to: http://$ServerIP`:$Port
4. Enter their full name
5. Click "Start Coding Session"
6. Begin working on ATmega128 projects

🏫 TEACHER DASHBOARD:
   http://$ServerIP`:$Port/teacher

📊 FEATURES:
   • Each student gets isolated workspace
   • Real-time activity monitoring
   • Automatic session management
   • Build and compile support
   • SimulIDE integration ready

⚙️  SYSTEM REQUIREMENTS:
   • Modern web browser
   • Network connection to server
   • No software installation needed

📞 TECHNICAL SUPPORT:
   • Check server is running: $startupScript
   • Verify network connectivity
   • Check firewall settings
   • Contact system administrator

Generated: $(Get-Date)
Server: $ServerIP`:$Port
Max Students: $MaxStudents
"@

Set-Content -Path $studentInfoFile -Value $studentInfo -Encoding UTF8
Write-Host "✅ Student access info: $studentInfoFile" -ForegroundColor $SuccessColor

Write-Host ""

# Final deployment summary
Write-Host "DEPLOYMENT COMPLETE" -ForegroundColor $SuccessColor
Write-Host "══════════════════════════════════════════════════════════════════════" -ForegroundColor $SuccessColor
Write-Host "🎉 Classroom dashboard is ready for deployment!" -ForegroundColor $SuccessColor
Write-Host ""
Write-Host "🚀 TO START THE DASHBOARD:" -ForegroundColor $InfoColor
Write-Host "   Double-click: $startupScript" -ForegroundColor White
Write-Host "   Or run: python dashboards\multi_user_dashboard.py" -ForegroundColor White
Write-Host ""
Write-Host "👥 STUDENT ACCESS:" -ForegroundColor $InfoColor
Write-Host "   URL: http://$ServerIP`:$Port" -ForegroundColor White
Write-Host "   Share this URL with students" -ForegroundColor White
Write-Host ""
Write-Host "🏫 TEACHER MONITORING:" -ForegroundColor $InfoColor
Write-Host "   URL: http://$ServerIP`:$Port/teacher" -ForegroundColor White
Write-Host "   Monitor all student activity in real-time" -ForegroundColor White
Write-Host ""
Write-Host "📋 NEXT STEPS:" -ForegroundColor $InfoColor
Write-Host "   1. Start the dashboard using the startup script" -ForegroundColor White
Write-Host "   2. Test access from a student device" -ForegroundColor White
Write-Host "   3. Share student access URL with class" -ForegroundColor White
Write-Host "   4. Monitor activity via teacher dashboard" -ForegroundColor White
Write-Host ""
Write-Host "📁 FILES CREATED:" -ForegroundColor $InfoColor
Write-Host "   • $configFile" -ForegroundColor White
Write-Host "   • $startupScript" -ForegroundColor White
Write-Host "   • $serviceScript" -ForegroundColor White
Write-Host "   • $studentInfoFile" -ForegroundColor White
Write-Host ""
Write-Host "🔧 SYSTEM CAPACITY:" -ForegroundColor $InfoColor
Write-Host "   • Max concurrent students: $MaxStudents" -ForegroundColor White
Write-Host "   • Session timeout: 8 hours" -ForegroundColor White
Write-Host "   • Isolated workspaces for each student" -ForegroundColor White
Write-Host "   • Automatic cleanup and management" -ForegroundColor White
Write-Host "══════════════════════════════════════════════════════════════════════" -ForegroundColor $SuccessColor