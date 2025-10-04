param(
    [Parameter(Mandatory = $true)]
    [string]$ProjectDir,
    
    [Parameter(Mandatory = $false)]
    [string]$Programmer = "usbasp",
    
    [Parameter(Mandatory = $false)]
    [string]$Port = "COM3",
    
    [Parameter(Mandatory = $false)]
    [switch]$DryRun
)

Write-Host "Programming ATmega128 from project: $ProjectDir" -ForegroundColor Cyan

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
        $buildResult = & "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe" -mmcu=atmega128 -DF_CPU=7372800UL -Os -Wall -I. -I../../shared_libs Main.c ../../shared_libs/_port.c ../../shared_libs/_init.c -o Main.elf 2>&1
        
        if ($LASTEXITCODE -ne 0) {
            Write-Host "Build failed!" -ForegroundColor Red
            Write-Host $buildResult -ForegroundColor Red
            exit 1
        }
        
        # Generate HEX
        $hexResult = & "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom Main.elf Main.hex 2>&1
        
        if ($LASTEXITCODE -ne 0) {
            Write-Host "HEX generation failed!" -ForegroundColor Red
            Write-Host $hexResult -ForegroundColor Red
            exit 1
        }
        
        Write-Host "Build completed successfully!" -ForegroundColor Green
    }
    
    Write-Host "Programming with $Programmer..." -ForegroundColor Yellow
    
    # Program based on programmer type
    $avrdudeExe = "C:\Program Files (x86)\AVRDUDESS\avrdude.exe"
    
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
                Write-Host "`"$avrdudeExe`" -c arduino -p m128 -P $Port -b 115200 -D -U `"$flashParam`"" -ForegroundColor Gray
                $LASTEXITCODE = 0
                $programResult = "Dry run - would program successfully"
            }
            else {
                # Match MC Studio: -b 115200 for baud rate, -D to disable erase
                $programResult = & $avrdudeExe -c arduino -p m128 -P $Port -b 115200 -D -U $flashParam 2>&1
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