# Check what's using COM4
Write-Host "Checking what's using serial ports..." -ForegroundColor Cyan
Write-Host ""

# Method 1: WMI Query
try {
    $ports = Get-WmiObject Win32_SerialPort | Select-Object DeviceID, Name, Status
    Write-Host "Available Serial Ports:" -ForegroundColor Green
    $ports | Format-Table -AutoSize
}
catch {
    Write-Host "Could not query serial ports: $_" -ForegroundColor Yellow
}

# Method 2: Check if COM4 is accessible
Write-Host ""
Write-Host "Testing COM4 access..." -ForegroundColor Cyan
try {
    $testPort = New-Object System.IO.Ports.SerialPort "COM4", 9600
    $testPort.Open()
    Write-Host "✓ COM4 is available!" -ForegroundColor Green
    $testPort.Close()
}
catch {
    Write-Host "✗ COM4 is in use or unavailable" -ForegroundColor Red
    Write-Host "  Error: $($_.Exception.Message)" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Possible causes:" -ForegroundColor Cyan
    Write-Host "  1. VS Code Serial Monitor is already connected"
    Write-Host "  2. Another application is using COM4"
    Write-Host "  3. com0com driver has COM4 locked"
    Write-Host ""
    Write-Host "Solution: Close VS Code Serial Monitor first, then run bridge" -ForegroundColor Green
}

Write-Host ""
Write-Host "Press any key to continue..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
