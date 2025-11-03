# Simple Serial Reader
$port = New-Object System.IO.Ports.SerialPort("COM3", 9600)
$port.Open()
Write-Host "Connected to COM3 - Reading data..." -ForegroundColor Green
Write-Host "Press Ctrl+C to stop" -ForegroundColor Yellow

try {
    while ($true) {
        if ($port.BytesToRead -gt 0) {
            Write-Host $port.ReadExisting() -NoNewline
        }
        Start-Sleep -Milliseconds 50
    }
} finally {
    $port.Close()
}