param(
    [Parameter(Mandatory = $true)]
    [string]$ProjectDir,
    
    [Parameter(Mandatory = $false)]
    [string]$Programmer = "usbasp",
    
    [Parameter(Mandatory = $false)]
    [string]$Port = "",
    
    [Parameter(Mandatory = $false)]
    [switch]$DryRun
)

# Function to auto-detect Arduino COM port
function Find-ArduinoPort {
    # Get all available COM ports
    $availablePorts = [System.IO.Ports.SerialPort]::getportnames() | Sort-Object
    
    if ($availablePorts.Count -eq 0) {
        Write-Host "[ERROR] No COM ports found!" -ForegroundColor Red
        Write-Host "   Please check if Arduino/programmer is connected" -ForegroundColor Yellow
        return $null
    }
    
    Write-Host "Available COM ports:" -ForegroundColor Green
    foreach ($port in $availablePorts) {
        Write-Host "  [PORT] $port" -ForegroundColor White
    }
    
    # For Arduino programmer, try to auto-detect
    if ($availablePorts.Count -eq 1) {
        $singlePort = $availablePorts[0]
        # Check if it's a virtual port
        try {
            $deviceInfo = Get-WmiObject -Class Win32_SerialPort | Where-Object { $_.DeviceID -eq $singlePort }
            if ($deviceInfo -and ($deviceInfo.Description -match "com0com|Virtual|Emulator|Loopback")) {
                Write-Host "[WARNING] Only port found is virtual: $singlePort ($($deviceInfo.Description))" -ForegroundColor Yellow
                Write-Host "[ERROR] No real Arduino device detected. Please connect Arduino and try again." -ForegroundColor Red
                return $null
            }
        }
        catch {
            # If WMI fails, assume it's a real port
        }
        Write-Host "[OK] Auto-detected: $singlePort (only available port)" -ForegroundColor Green
        return $singlePort
    }
    
    # Multiple ports - try to find Arduino-like device
    foreach ($port in $availablePorts) {
        try {
            # Try to get device information (Windows specific)
            $deviceInfo = Get-WmiObject -Class Win32_SerialPort | Where-Object { $_.DeviceID -eq $port }
            if ($deviceInfo) {
                $description = $deviceInfo.Description
                Write-Host "  [CHECK] $port`: $description" -ForegroundColor Gray
                
                # Exclude virtual/emulator ports
                if ($description -match "com0com|Virtual|Emulator|Loopback") {
                    Write-Host "    [SKIP] Skipping virtual port: $port" -ForegroundColor Yellow
                    continue
                }
                
                # Look for real Arduino/USB devices
                if ($description -match "Arduino|CH340|CP210|FTDI|USB.*Serial") {
                    Write-Host "[OK] Auto-detected: $port ($description)" -ForegroundColor Green
                    return $port
                }
            }
        }
        catch {
            # If WMI fails, continue to next port
        }
    }
    
    # No Arduino device found - check for non-virtual ports as fallback
    $realPorts = @()
    foreach ($port in $availablePorts) {
        try {
            $deviceInfo = Get-WmiObject -Class Win32_SerialPort | Where-Object { $_.DeviceID -eq $port }
            if ($deviceInfo) {
                if (-not ($deviceInfo.Description -match "com0com|Virtual|Emulator|Loopback")) {
                    $realPorts += $port
                    Write-Host "  📍 Real port found: $port ($($deviceInfo.Description))" -ForegroundColor Cyan
                }
            }
            else {
                # If no WMI info, assume it might be real
                $realPorts += $port
            }
        }
        catch {
            # If WMI fails, assume it might be real
            $realPorts += $port
        }
    }
    
    if ($realPorts.Count -gt 0) {
        $detectedPort = $realPorts[0]
        Write-Host "[WARNING] No Arduino detected, using first real port: $detectedPort" -ForegroundColor Yellow
        Write-Host "   If this fails, manually specify: -Port COM#" -ForegroundColor Yellow
        return $detectedPort
    }
    
    # Last resort: no real ports found
    Write-Host "[ERROR] No real Arduino/serial devices detected!" -ForegroundColor Red
    Write-Host "   Available ports are virtual/emulators:" -ForegroundColor Yellow
    foreach ($port in $availablePorts) {
        Write-Host "     $port (virtual)" -ForegroundColor Gray
    }
    Write-Host "   Please connect a real Arduino device and try again." -ForegroundColor Yellow
    return $null
}

Write-Host "Programming ATmega128 from project: $ProjectDir" -ForegroundColor Cyan

# Setup portable tools paths with fallback to system tools
$WorkspaceRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent

# Try portable AVR tools first, fallback to system Atmel Studio
$portableAvrGcc = Join-Path $WorkspaceRoot "tools\avr-toolchain\bin\avr-gcc.exe"
$systemAvrGcc = "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe"

if (Test-Path $portableAvrGcc) {
    $avrGccExe = $portableAvrGcc
    $avrObjcopyExe = Join-Path $WorkspaceRoot "tools\avr-toolchain\bin\avr-objcopy.exe"
    Write-Host "[OK] Using portable AVR toolchain" -ForegroundColor Green
}
elseif (Test-Path $systemAvrGcc) {
    $avrGccExe = $systemAvrGcc
    $avrObjcopyExe = "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe"
    Write-Host "[OK] Using system AVR toolchain (Atmel Studio)" -ForegroundColor Yellow
}
else {
    Write-Host "[ERROR] No AVR toolchain found! Please install Atmel Studio or portable AVR toolchain." -ForegroundColor Red
    exit 1
}

if ($ProjectDir -like '*\projects\*' -or $ProjectDir -like '*\Main') {
    $projectName = Split-Path $ProjectDir -Leaf
    Write-Host "Project: $projectName" -ForegroundColor Green
    
    # Change to project directory
    Set-Location $ProjectDir
    
    # Check if HEX file exists
    $hexFile = "Main.hex"
    if (-not (Test-Path $hexFile)) {
        Write-Host "HEX file not found. Building project first..." -ForegroundColor Yellow
        
        # Build the project
        $buildResult = & $avrGccExe -mmcu=atmega128 -DF_CPU=16000000UL -Os -Wall -I. -I../../shared_libs Main.c ../../shared_libs/_port.c ../../shared_libs/_init.c -o Main.elf 2>&1
        
        if ($LASTEXITCODE -ne 0) {
            Write-Host "Build failed!" -ForegroundColor Red
            Write-Host $buildResult -ForegroundColor Red
            exit 1
        }
        
        # Generate HEX
        $hexResult = & $avrObjcopyExe -O ihex -R .eeprom Main.elf Main.hex 2>&1
        
        if ($LASTEXITCODE -ne 0) {
            Write-Host "HEX generation failed!" -ForegroundColor Red
            Write-Host $hexResult -ForegroundColor Red
            exit 1
        }
        
        Write-Host "Build completed successfully!" -ForegroundColor Green
    }
    
    Write-Host "Programming with $Programmer..." -ForegroundColor Yellow
    
    # Program based on programmer type - prioritize portable AVRDUDE for self-sufficiency
    $WorkspaceRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
    $avrdudeExe = Join-Path $WorkspaceRoot "tools\avrdudess\avrdude.exe"
    
    # Check portable AVRDUDE first, fallback to system if needed
    if (-not (Test-Path $avrdudeExe)) {
        Write-Host "[ERROR] Portable AVRDUDE not found: $avrdudeExe" -ForegroundColor Red
        Write-Host "   Falling back to system AVRDUDE..." -ForegroundColor Yellow
        $avrdudeExe = "C:\Program Files (x86)\AVRDUDESS\avrdude.exe"
        
        if (-not (Test-Path $avrdudeExe)) {
            Write-Host "[ERROR] System AVRDUDE also not found!" -ForegroundColor Red
            exit 1
        }
    }
    
    Write-Host "[OK] Using AVRDUDE: $avrdudeExe" -ForegroundColor Green
    
    # Check for Debug folder hex file first, then main folder
    $debugHexFile = "Debug\Main.hex"
    $mainHexFile = "Main.hex"
    
    if (Test-Path $debugHexFile) {
        $hexFilePath = $debugHexFile
    }
    elseif (Test-Path $mainHexFile) {
        $hexFilePath = $mainHexFile
    }
    else {
        Write-Host "No HEX file found in Debug\ or main directory!" -ForegroundColor Red
        exit 1
    }
    
    # Use absolute path for HEX file (matching MC Studio format)
    $fullHexPath = (Resolve-Path $hexFilePath).Path
    Write-Host "Using HEX file: $fullHexPath" -ForegroundColor Green
    
    # Construct the flash parameter exactly like MC Studio
    # Use single quotes to avoid PowerShell interpretation issues
    $flashParam = 'flash:w:"' + $fullHexPath + '":a'
    
    # Auto-detect COM port if needed (for arduino, avrisp, stk500v2 programmers)
    if ($Programmer.ToLower() -in @("arduino", "avrisp", "stk500v2") -and [string]::IsNullOrEmpty($Port)) {
        Write-Host "[CHECK] Auto-detecting COM port..." -ForegroundColor Cyan
        $Port = Find-ArduinoPort
        if ([string]::IsNullOrEmpty($Port)) {
            Write-Host "[ERROR] Could not auto-detect COM port. Please specify -Port parameter." -ForegroundColor Red
            exit 1
        }
        Write-Host "[OK] Using auto-detected port: $Port" -ForegroundColor Green
    }
    
    switch ($Programmer.ToLower()) {
        "usbasp" {
            if ($DryRun) {
                Write-Host "DRY RUN: Would execute:" -ForegroundColor Yellow
                Write-Host "`"$avrdudeExe`" -c usbasp -p m128 -U `"$flashParam`"" -ForegroundColor Gray
                $LASTEXITCODE = 0
                $programResult = "Dry run - would program successfully"
            }
            else {
                $programResult = & $avrdudeExe -c usbasp -p m128 -U $flashParam 2>&1
            }
        }
        "arduino" {
            if ($DryRun) {
                Write-Host "DRY RUN: Would execute:" -ForegroundColor Yellow
                Write-Host "`"$avrdudeExe`" -c arduino -p m128 -P $Port -U `"$flashParam`"" -ForegroundColor Gray
                $LASTEXITCODE = 0
                $programResult = "Dry run - would program successfully"
            }
            else {
                # Use exact MC Studio format: avrdude.exe -c arduino -p m128 -P COM3 -U flash:w:"file.hex":a
                $programResult = & $avrdudeExe -c arduino -p m128 -P $Port -U $flashParam 2>&1
            }
        }
        "avrisp" {
            if ($DryRun) {
                Write-Host "DRY RUN: Would execute:" -ForegroundColor Yellow
                Write-Host "`"$avrdudeExe`" -c avrisp -p m128 -P $Port -U `"$flashParam`"" -ForegroundColor Gray
                $LASTEXITCODE = 0
                $programResult = "Dry run - would program successfully"
            }
            else {
                $programResult = & $avrdudeExe -c avrisp -p m128 -P $Port -U $flashParam 2>&1
            }
        }
        "stk500v2" {
            if ($DryRun) {
                Write-Host "DRY RUN: Would execute:" -ForegroundColor Yellow
                Write-Host "`"$avrdudeExe`" -c stk500v2 -p m128 -P $Port -U `"$flashParam`"" -ForegroundColor Gray
                $LASTEXITCODE = 0
                $programResult = "Dry run - would program successfully"
            }
            else {
                $programResult = & $avrdudeExe -c stk500v2 -p m128 -P $Port -U $flashParam 2>&1
            }
        }
        default {
            Write-Host "Unsupported programmer: $Programmer" -ForegroundColor Red
            Write-Host "Supported: usbasp, arduino, avrisp, stk500v2" -ForegroundColor Yellow
            exit 1
        }
    }
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "Programming completed successfully!" -ForegroundColor Green
        Write-Host "ATmega128 has been programmed with $projectName" -ForegroundColor Cyan
    }
    else {
        Write-Host "Programming failed!" -ForegroundColor Red
        Write-Host $programResult -ForegroundColor Red
        Write-Host "" -ForegroundColor White
        Write-Host "Troubleshooting tips:" -ForegroundColor Yellow
        Write-Host "1. Check programmer connection" -ForegroundColor White
        Write-Host "2. Verify correct COM port (if using serial programmer)" -ForegroundColor White
        Write-Host "3. Ensure ATmega128 is powered" -ForegroundColor White
        Write-Host "4. Try different programmer type with -Programmer parameter" -ForegroundColor White
        exit 1
    }
    
}
else {
    Write-Host "Error: Not a valid project directory" -ForegroundColor Red
    Write-Host "Current directory: $ProjectDir" -ForegroundColor Gray
    exit 1
}