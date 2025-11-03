# COM Port Troubleshooting Guide for VS Code

## Quick COM3 Access Fix

### 1. Close all potential conflicts:
```powershell
# Stop any running serial monitor processes
Get-Process | Where-Object {$_.ProcessName -like "*putty*" -or $_.ProcessName -like "*arduino*" -or $_.ProcessName -like "*serial*"} | Stop-Process -Force

# Check COM port availability
mode COM3
```

### 2. Programming sequence:
```batch
# Build first
.\build_custom.bat

# Then program (make sure no serial monitor is open)
avrdude -c arduino -p m128 -P COM3 -U flash:w:Main.hex:i
```

### 3. Start communication after programming:
```powershell
# Simple read-only monitor
$port = New-Object System.IO.Ports.SerialPort("COM3", 9600)
$port.Open()
while($true) { 
    if($port.BytesToRead -gt 0) { 
        Write-Host $port.ReadExisting() -NoNewline 
    } 
    Start-Sleep -Milliseconds 100 
}
```

### 4. VS Code Serial Monitor:
- Ctrl+Shift+P
- "Serial Monitor: Start Monitoring"  
- Port: COM3, Baud: 9600, 8N1

## Device Menu After Programming:
```
========================================
  SERIAL INTERRUPT COMMUNICATION DEMO
========================================
Select demonstration mode:
  1 - Basic RX Interrupt Echo
  2 - TX Interrupt Queue  
  3 - Bidirectional Interrupts
  4 - Command Processing ← RECOMMENDED
  5 - Advanced Buffering
  h - Help and Information
  q - Quit Program
```

## Mode 4 Commands:
- `led on` - Turn on LED
- `led off` - Turn off LED  
- `status` - Show system status
- `reset` - Reset counters
- `quit` - Exit mode