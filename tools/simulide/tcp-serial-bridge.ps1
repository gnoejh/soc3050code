# TCP to Serial Bridge for SimulIDE
# Connects VS Code Serial Monitor to SimulIDE via TCP

param(
    [string]$SerialPort = "COM5",
    [int]$BaudRate = 9600,
    [int]$TcpPort = 1234
)

Write-Host "=== TCP to Serial Bridge ===" -ForegroundColor Cyan
Write-Host "Serial: $SerialPort @ $BaudRate baud" -ForegroundColor Gray
Write-Host "TCP: localhost:$TcpPort" -ForegroundColor Gray
Write-Host ""

# Open Serial Port
try {
    $port = New-Object System.IO.Ports.SerialPort $SerialPort, $BaudRate
    $port.Open()
    Write-Host "✓ Serial port opened: $SerialPort" -ForegroundColor Green
}
catch {
    Write-Host "✗ Failed to open serial port: $_" -ForegroundColor Red
    Write-Host ""
    Write-Host "SOLUTION:" -ForegroundColor Yellow
    Write-Host "  1. Close VS Code Serial Monitor" -ForegroundColor White
    Write-Host "  2. Close any other programs using $SerialPort" -ForegroundColor White
    Write-Host "  3. Run this bridge script again" -ForegroundColor White
    Write-Host "  4. Then open VS Code Serial Monitor" -ForegroundColor White
    Write-Host ""
    exit 1
}

# Connect to TCP (with retry)
Write-Host "Connecting to SimulIDE TCP server..." -ForegroundColor Yellow
$maxRetries = 10
$retryCount = 0
$connected = $false

while (-not $connected -and $retryCount -lt $maxRetries) {
    try {
        $tcpClient = New-Object System.Net.Sockets.TcpClient
        $tcpClient.Connect("localhost", $TcpPort)
        $stream = $tcpClient.GetStream()
        $connected = $true
        Write-Host "✓ Connected to TCP port $TcpPort" -ForegroundColor Green
    }
    catch {
        $retryCount++
        if ($retryCount -lt $maxRetries) {
            Write-Host "  Waiting for SimulIDE... (attempt $retryCount/$maxRetries)" -ForegroundColor Gray
            Start-Sleep -Seconds 1
        }
        else {
            Write-Host "✗ Failed to connect to TCP after $maxRetries attempts" -ForegroundColor Red
            Write-Host ""
            Write-Host "SOLUTION:" -ForegroundColor Yellow
            Write-Host "  1. Make sure SimulIDE is running" -ForegroundColor White
            Write-Host "  2. Load the simulation circuit (Simulator110.simu)" -ForegroundColor White
            Write-Host "  3. Click the 'Power' button to START the simulation" -ForegroundColor White
            Write-Host "  4. Wait for SerialPort TCP to initialize" -ForegroundColor White
            Write-Host "  5. Run this bridge script again" -ForegroundColor White
            Write-Host ""
            $port.Close()
            exit 1
        }
    }
}

Write-Host ""
Write-Host "=== Bridge Running ===" -ForegroundColor Green
Write-Host ""
Write-Host "✓ COM4 (Serial) ↔ localhost:1234 (TCP) ↔ SimulIDE" -ForegroundColor Cyan
Write-Host ""
Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "  1. Open VS Code Serial Monitor" -ForegroundColor White
Write-Host "  2. Select Port: COM4" -ForegroundColor White
Write-Host "  3. Set Baud rate: 9600" -ForegroundColor White
Write-Host "  4. Start monitoring and send messages!" -ForegroundColor White
Write-Host ""
Write-Host "Press Ctrl+C to stop bridge" -ForegroundColor Gray
Write-Host ""

# Bridge loop
try {
    $buffer = New-Object byte[] 1024
    
    while ($true) {
        # TCP → Serial
        if ($stream.DataAvailable) {
            $bytesRead = $stream.Read($buffer, 0, $buffer.Length)
            if ($bytesRead -gt 0) {
                $port.Write($buffer, 0, $bytesRead)
                $text = [System.Text.Encoding]::UTF8.GetString($buffer, 0, $bytesRead)
                Write-Host "TCP→Serial: $text" -NoNewline -ForegroundColor Cyan
            }
        }
        
        # Serial → TCP
        if ($port.BytesToRead -gt 0) {
            $bytesToRead = [Math]::Min($port.BytesToRead, $buffer.Length)
            $bytesRead = $port.Read($buffer, 0, $bytesToRead)
            if ($bytesRead -gt 0) {
                $stream.Write($buffer, 0, $bytesRead)
                $text = [System.Text.Encoding]::UTF8.GetString($buffer, 0, $bytesRead)
                Write-Host "Serial→TCP: $text" -NoNewline -ForegroundColor Yellow
            }
        }
        
        Start-Sleep -Milliseconds 10
    }
}
finally {
    Write-Host "`n`nShutting down..." -ForegroundColor Red
    $stream.Close()
    $tcpClient.Close()
    $port.Close()
    Write-Host "Bridge stopped." -ForegroundColor Gray
}
