# Quick Serial Monitor for VS Code Terminal
Write-Host "=== ATmega128 Serial Communication Monitor ===" -ForegroundColor Green
Write-Host "Device: COM3 @ 9600 baud" -ForegroundColor Yellow
Write-Host "Press Ctrl+C to exit" -ForegroundColor Cyan
Write-Host ""

try {
    $port = New-Object System.IO.Ports.SerialPort("COM3", 9600)
    $port.Parity = [System.IO.Ports.Parity]::None
    $port.DataBits = 8
    $port.StopBits = [System.IO.Ports.StopBits]::One
    $port.Open()
    
    Write-Host "Connected! Receiving data..." -ForegroundColor Green
    Write-Host "----------------------------------------" -ForegroundColor Gray
    
    while ($true) {
        if ($port.BytesToRead -gt 0) {
            $data = $port.ReadExisting()
            Write-Host $data -NoNewline -ForegroundColor White
        }
        Start-Sleep -Milliseconds 50
    }
}
catch {
    Write-Host "Error: $_" -ForegroundColor Red
    Write-Host "Make sure no other program is using COM3" -ForegroundColor Yellow
}
finally {
    if ($port -and $port.IsOpen) {
        $port.Close()
    }
}