# Run as Administrator to install Windows service
$serviceName = "ATmega128Dashboard"
$serviceDescription = "ATmega128 Multi-User Educational Dashboard"
$pythonPath = (Get-Command python).Source
$scriptPath = "W:\soc3050code\tools\dashboards\multi_user_dashboard.py"

# Install service using NSSM (Non-Sucking Service Manager)
# Download NSSM from: https://nssm.cc/download
# nssm install $serviceName $pythonPath $scriptPath
# nssm set $serviceName Description "$serviceDescription"
# nssm start $serviceName

Write-Host "Service installation script created."
Write-Host "To install as Windows service:"
Write-Host "1. Download NSSM from https://nssm.cc/download"
Write-Host "2. Extract nssm.exe to System32"
Write-Host "3. Run this script as Administrator"
