# Minimal Serial Test in PowerShell
# Run this in VS Code terminal

Write-Host "Testing COM3 connection..." -ForegroundColor Yellow

try {
    # Quick connection test
    $port = New-Object System.IO.Ports.SerialPort("COM3", 9600)
    $port.Open()
    Write-Host "SUCCESS: COM3 is accessible!" -ForegroundColor Green
    
    # Send a test command (mode 4)
    $port.WriteLine("4")
    Start-Sleep -Seconds 1
    
    # Read response
    if ($port.BytesToRead -gt 0) {
        $response = $port.ReadExisting()
        Write-Host "Device Response:" -ForegroundColor Cyan
        Write-Host $response -ForegroundColor White
    }
    
    $port.Close()
    Write-Host "Connection test completed." -ForegroundColor Green
    
} catch {
    Write-Host "FAILED: $_" -ForegroundColor Red
    Write-Host "Try closing any Arduino IDE, PuTTY, or other serial programs" -ForegroundColor Yellow
}