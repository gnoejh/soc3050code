# TCP Server Bridge for SimulIDE
# Acts as TCP SERVER - SimulIDE connects to this bridge as CLIENT
# Bridge forwards data between TCP client (SimulIDE) and COM4 (VS Code)

param(
    [string]$SerialPort = "COM5",
    [int]$BaudRate = 9600,
    [int]$TcpPort = 1234
)

Write-Host "=== TCP Server Bridge (SimulIDE Client Mode) ===" -ForegroundColor Cyan
Write-Host "Serial: $SerialPort @ $BaudRate baud" -ForegroundColor Gray
Write-Host "TCP Server: listening on localhost:$TcpPort" -ForegroundColor Gray
Write-Host ""

# Open Serial Port first
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

# Create TCP Listener
try {
    $listener = New-Object System.Net.Sockets.TcpListener([System.Net.IPAddress]::Loopback, $TcpPort)
    $listener.Start()
    Write-Host "✓ TCP server listening on port $TcpPort" -ForegroundColor Green
    Write-Host ""
    Write-Host "Waiting for SimulIDE to connect..." -ForegroundColor Yellow
    
    # Wait with timeout for SimulIDE to connect
    $timeout = 30 # seconds
    $elapsed = 0
    $listener.Server.ReceiveTimeout = 1000
    
    while ($elapsed -lt $timeout) {
        if ($listener.Pending()) {
            $tcpClient = $listener.AcceptTcpClient()
            $stream = $tcpClient.GetStream()
            Write-Host "✓ SimulIDE connected!" -ForegroundColor Green
            break
        }
        Start-Sleep -Milliseconds 500
        $elapsed += 0.5
        if (($elapsed % 5) -eq 0) {
            Write-Host "  Still waiting... ($elapsed/$timeout seconds)" -ForegroundColor Gray
        }
    }
    
    if ($elapsed -ge $timeout) {
        throw "SimulIDE did not connect within $timeout seconds"
    }
}
catch {
    Write-Host "✗ Failed to create TCP server: $_" -ForegroundColor Red
    $port.Close()
    if ($listener) { $listener.Stop() }
    exit 1
}

Write-Host ""
Write-Host "=== Bridge Running ===" -ForegroundColor Green
Write-Host "Forwarding data: COM4 ↔ TCP:1234 ↔ SimulIDE" -ForegroundColor Cyan
Write-Host "Press Ctrl+C to stop" -ForegroundColor Gray
Write-Host ""

# Bidirectional forwarding
$buffer = New-Object byte[] 1024
try {
    while ($true) {
        # TCP → Serial (SimulIDE sends to VS Code)
        if ($stream.DataAvailable) {
            $count = $stream.Read($buffer, 0, $buffer.Length)
            if ($count -gt 0) {
                $port.Write($buffer, 0, $count)
                $data = [System.Text.Encoding]::ASCII.GetString($buffer, 0, $count)
                Write-Host "TCP→Serial: $data" -ForegroundColor Cyan -NoNewline
            }
        }
        
        # Serial → TCP (VS Code sends to SimulIDE)
        if ($port.BytesToRead -gt 0) {
            $count = $port.Read($buffer, 0, [Math]::Min($buffer.Length, $port.BytesToRead))
            if ($count -gt 0) {
                $stream.Write($buffer, 0, $count)
                $data = [System.Text.Encoding]::ASCII.GetString($buffer, 0, $count)
                Write-Host "Serial→TCP: $data" -ForegroundColor Yellow -NoNewline
            }
        }
        
        Start-Sleep -Milliseconds 10
    }
}
finally {
    Write-Host ""
    Write-Host "Closing connections..." -ForegroundColor Gray
    $stream.Close()
    $tcpClient.Close()
    $listener.Stop()
    $port.Close()
    Write-Host "Bridge stopped." -ForegroundColor Gray
}
