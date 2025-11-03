# ATmega128 Reset Utility for VS Code
# This will close/reopen the serial connection to trigger a reset

Write-Host "=== ATmega128 Reset Utility ===" -ForegroundColor Green
Write-Host "This will reset the serial connection..." -ForegroundColor Yellow

try {
    # Quick connect/disconnect to trigger reset
    $port = New-Object System.IO.Ports.SerialPort("COM3", 9600)
    $port.DtrEnable = $true
    $port.RtsEnable = $true
    $port.Open()
    
    Write-Host "Triggering reset..." -ForegroundColor Cyan
    $port.DtrEnable = $false  # This often triggers a reset
    Start-Sleep -Milliseconds 100
    $port.DtrEnable = $true
    Start-Sleep -Milliseconds 500
    
    $port.Close()
    Write-Host "Reset completed! Reconnect with Serial Monitor." -ForegroundColor Green
    
} catch {
    Write-Host "Reset attempt failed: $_" -ForegroundColor Red
    Write-Host "Try using the hardware reset button instead." -ForegroundColor Yellow
}

Write-Host "Now reconnect with VS Code Serial Monitor." -ForegroundColor Cyan