# Serial Communication Script for VS Code
# Connect to ATmega128 Serial_interrupt demo

Write-Host "=== ATmega128 Serial Communication ===" -ForegroundColor Green
Write-Host "Connecting to COM3 at 9600 baud..." -ForegroundColor Yellow

try {
    # Create serial port object
    $serialPort = New-Object System.IO.Ports.SerialPort("COM3", 9600)
    $serialPort.Parity = [System.IO.Ports.Parity]::None
    $serialPort.DataBits = 8
    $serialPort.StopBits = [System.IO.Ports.StopBits]::One
    $serialPort.Handshake = [System.IO.Ports.Handshake]::None
    $serialPort.ReadTimeout = 1000
    $serialPort.WriteTimeout = 1000
    
    # Open the port
    $serialPort.Open()
    Write-Host "Connected successfully!" -ForegroundColor Green
    Write-Host ""
    Write-Host "INSTRUCTIONS:" -ForegroundColor Cyan
    Write-Host "- Type commands and press Enter to send" -ForegroundColor White
    Write-Host "- Available demo modes: 1, 2, 3, 4, 5, h, q" -ForegroundColor White
    Write-Host "- Type 'EXIT' to quit this session" -ForegroundColor White
    Write-Host "- In Mode 4, try commands: 'led on', 'led off', 'status'" -ForegroundColor White
    Write-Host ""
    
    # Start reading in background
    $readJob = Start-Job -ScriptBlock {
        param($portName)
        $port = New-Object System.IO.Ports.SerialPort($portName, 9600)
        $port.Open()
        while ($true) {
            try {
                if ($port.BytesToRead -gt 0) {
                    $data = $port.ReadExisting()
                    Write-Host $data -NoNewline -ForegroundColor Yellow
                }
                Start-Sleep -Milliseconds 50
            }
            catch {
                break
            }
        }
        $port.Close()
    } -ArgumentList "COM3"
    
    Write-Host "Waiting for device menu..." -ForegroundColor Gray
    Start-Sleep -Seconds 2
    
    # Interactive loop
    while ($true) {
        $input = Read-Host "> "
        
        if ($input -eq "EXIT") {
            break
        }
        
        if ($serialPort.IsOpen) {
            $serialPort.WriteLine($input)
        }
        
        Start-Sleep -Milliseconds 100
    }
    
} catch {
    Write-Host "Error: $_" -ForegroundColor Red
    Write-Host "Make sure:" -ForegroundColor Yellow
    Write-Host "1. Device is connected to COM3" -ForegroundColor White
    Write-Host "2. Device is programmed with Serial_interrupt firmware" -ForegroundColor White
    Write-Host "3. No other program is using COM3" -ForegroundColor White
} finally {
    if ($serialPort -and $serialPort.IsOpen) {
        $serialPort.Close()
        Write-Host "Serial port closed." -ForegroundColor Gray
    }
    if ($readJob) {
        Stop-Job $readJob -ErrorAction SilentlyContinue
        Remove-Job $readJob -ErrorAction SilentlyContinue
    }
}

Write-Host "Session ended." -ForegroundColor Green