# Simple Serial Monitor for ATmega128
# Usage: .\serial.ps1 [COMPort] [BaudRate]

param(
    [string]$Port = "",
    [int]$BaudRate = 9600
)

Write-Host "🔌 ATmega128 Serial Monitor" -ForegroundColor Green
Write-Host "================================" -ForegroundColor Green

# Auto-detect COM ports if not specified
if ($Port -eq "") {
    Write-Host "📡 Available COM ports:" -ForegroundColor Yellow
    $availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
    
    if ($availablePorts.Count -eq 0) {
        Write-Host "❌ No COM ports found!" -ForegroundColor Red
        exit 1
    }
    
    for ($i = 0; $i -lt $availablePorts.Count; $i++) {
        Write-Host "  [$($i+1)] $($availablePorts[$i])" -ForegroundColor White
    }
    
    Write-Host ""
    $selection = Read-Host "Select COM port (1-$($availablePorts.Count)) or enter port name (e.g., COM3)"
    
    # Check if it's a number selection
    if ($selection -match '^\d+$' -and [int]$selection -le $availablePorts.Count -and [int]$selection -gt 0) {
        $Port = $availablePorts[[int]$selection - 1]
    } else {
        $Port = $selection.ToUpper()
    }
}

Write-Host "🔗 Connecting to $Port at $BaudRate baud..." -ForegroundColor Cyan

try {
    $serialPort = New-Object System.IO.Ports.SerialPort($Port, $BaudRate)
    $serialPort.ReadTimeout = 500
    $serialPort.WriteTimeout = 500
    $serialPort.Open()
    
    Write-Host "✅ Connected! Press Ctrl+C to exit" -ForegroundColor Green
    Write-Host "📤 Type messages and press Enter to send" -ForegroundColor Yellow
    Write-Host "----------------------------------------" -ForegroundColor Gray
    
    # Background job to read from serial port
    $readJob = Start-Job -ScriptBlock {
        param($portName, $baud)
        $port = New-Object System.IO.Ports.SerialPort($portName, $baud)
        $port.Open()
        while ($true) {
            try {
                if ($port.BytesToRead -gt 0) {
                    $data = $port.ReadExisting()
                    Write-Output "RX: $data"
                }
                Start-Sleep -Milliseconds 50
            } catch {
                break
            }
        }
        $port.Close()
    } -ArgumentList $Port, $BaudRate
    
    # Main loop for user input
    while ($true) {
        $input = Read-Host "TX"
        if ($input -eq "exit" -or $input -eq "quit") {
            break
        }
        
        if ($input -ne "") {
            $serialPort.WriteLine($input)
            Write-Host "📤 Sent: $input" -ForegroundColor Green
        }
        
        # Check for received data
        Start-Sleep -Milliseconds 100
        $jobOutput = Receive-Job -Job $readJob -ErrorAction SilentlyContinue
        if ($jobOutput) {
            Write-Host $jobOutput -ForegroundColor Yellow
        }
    }
    
} catch {
    Write-Host "❌ Error: $($_.Exception.Message)" -ForegroundColor Red
} finally {
    if ($serialPort -and $serialPort.IsOpen) {
        $serialPort.Close()
    }
    if ($readJob) {
        Stop-Job -Job $readJob
        Remove-Job -Job $readJob
    }
    Write-Host "🔌 Serial connection closed." -ForegroundColor Cyan
}