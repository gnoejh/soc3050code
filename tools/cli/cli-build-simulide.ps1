param(
    [string]$ProjectDir,
    [string]$SourceFile = ""  # Optional: specify which .c file to build
)

# SIMULIDE BUILD: Optimized for SimulIDE 1.1.0+ with full ELPM support
Write-Host "[TARGET] Building for SimulIDE 1.1.0+ (ELPM Support Fixed)" -ForegroundColor Cyan

# PORTABLE SYSTEM - Use only included tools, no external dependencies
$WorkspaceRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent

# Portable AVR tools (REQUIRED - no external fallbacks)
$portableAvrGcc = Join-Path $WorkspaceRoot "tools\avr-toolchain\bin\avr-gcc.exe"
$portableAvrObjcopy = Join-Path $WorkspaceRoot "tools\avr-toolchain\bin\avr-objcopy.exe"

if (Test-Path $portableAvrGcc) {
    $avrGccExe = $portableAvrGcc
    $avrObjcopyExe = $portableAvrObjcopy
    Write-Host "[OK] Using portable AVR toolchain" -ForegroundColor Green
}
else {
    Write-Host "[ERROR] PORTABLE SYSTEM ERROR: AVR toolchain not found!" -ForegroundColor Red
    Write-Host "Expected location: $portableAvrGcc" -ForegroundColor Yellow
    Write-Host "This system requires NO external installations - all tools should be included." -ForegroundColor Yellow
    Write-Host "Please ensure the tools/avr-toolchain directory is complete." -ForegroundColor Yellow
    exit 1
}

if ($ProjectDir -like "*\projects\*") {
    $name = Split-Path $ProjectDir -Leaf
    Write-Host "Building: $name"
    Set-Location $ProjectDir
    
    # Auto-detect source file if not specified
    if ([string]::IsNullOrEmpty($SourceFile)) {
        # Check for Lab.c first (lab exercises take priority)
        if (Test-Path "Lab.c") {
            $SourceFile = "Lab.c"
            Write-Host "[AUTO-DETECT] Found Lab.c - building lab exercise" -ForegroundColor Cyan
        }
        elseif (Test-Path "Main.c") {
            $SourceFile = "Main.c"
            Write-Host "[AUTO-DETECT] Found Main.c - building main project" -ForegroundColor Cyan
        }
        else {
            Write-Host "[ERROR] No Main.c or Lab.c found in $ProjectDir" -ForegroundColor Red
            exit 1
        }
    }
    
    # Get base name without extension for output files
    $BaseName = [System.IO.Path]::GetFileNameWithoutExtension($SourceFile)
    Write-Host "Source: $SourceFile -> Output: $BaseName.elf, $BaseName.hex" -ForegroundColor Gray
    
    # Standard AVR compiler flags (compatible with avr/io.h)
    $McStudioFlags = "-funsigned-char -funsigned-bitfields -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax"
    $ExactMcStudioFlags = "-O3"  # EXACT from MC Studio: "Optimize most (-O3)" - NOT -Os!
    $CommonFlags = "-mmcu=atmega128 -DF_CPU=16000000UL -DBAUD=9600 -DDEBUG -Wall -I. -I../../shared_libs"
    
    # UNIFIED BUILD: Generate standard hex file compatible with both hardware and SimulIDE
    
    # Special handling for projects with educational interrupt programming
    if ($name -eq "Serial_interrupt") {
        Write-Host "Building Serial_interrupt for SimulIDE..." -ForegroundColor Yellow
        & $avrGccExe $CommonFlags.Split(' ') $McStudioFlags.Split(' ') $ExactMcStudioFlags.Split(' ') $SourceFile ../../shared_libs/_port.c ../../shared_libs/_uart.c ../../shared_libs/_interrupt_manager.c -lm -o "$BaseName.elf"
    }
    elseif ($name -eq "Serial_Communications") {
        Write-Host "Building Serial_Communications for SimulIDE..." -ForegroundColor Cyan
        
        # Check if building Lab.c (needs GLCD library)
        if ($SourceFile -eq "Lab.c") {
            Write-Host "  [LAB BUILD] Including GLCD library for Lab.c" -ForegroundColor Yellow
            & $avrGccExe $CommonFlags.Split(' ') $McStudioFlags.Split(' ') $ExactMcStudioFlags.Split(' ') $SourceFile ../../shared_libs/_glcd.c ../../shared_libs/_port.c -lm -o "$BaseName.elf"
        }
        else {
            # Main.c build (no GLCD)
            Write-Host "  [MAIN BUILD] Standard build without GLCD" -ForegroundColor Yellow
            & $avrGccExe $CommonFlags.Split(' ') $McStudioFlags.Split(' ') $ExactMcStudioFlags.Split(' ') $SourceFile ../../shared_libs/_port.c -lm -o "$BaseName.elf"
        }
    }
    elseif ($name -eq "Graphics_Display") {
        Write-Host "Building Graphics_Display with GLCD library..." -ForegroundColor Magenta
        # Graphics Display project uses GLCD library
        # Add flag for newer SimulIDE versions (1.1.0+) that need different timing
        $SimulIDENewFlag = "-DSIMULIDE_NEW_VERSION"
        & $avrGccExe $CommonFlags.Split(' ') $McStudioFlags.Split(' ') $ExactMcStudioFlags.Split(' ') $SimulIDENewFlag $SourceFile `
            ../../shared_libs/_glcd.c `
            ../../shared_libs/_port.c `
            ../../shared_libs/_init.c `
            -lm -o "$BaseName.elf"
    }
    elseif ($name -eq "Interrupt_Basic") {
        Write-Host "Building Interrupt_Basic for SimulIDE..." -ForegroundColor Yellow
        & $avrGccExe $CommonFlags.Split(' ') $McStudioFlags.Split(' ') $ExactMcStudioFlags.Split(' ') $SourceFile ../../shared_libs/_port.c ../../shared_libs/_init.c -lm -o "$BaseName.elf"
    }
    elseif ($name -eq "Port_Basic") {
        Write-Host "Building Port_Basic for SimulIDE..." -ForegroundColor Cyan
        & $avrGccExe $CommonFlags.Split(' ') $McStudioFlags.Split(' ') $ExactMcStudioFlags.Split(' ') $SourceFile -lm -o "$BaseName.elf"
    }
    else {
        # Standard build for SimulIDE for all projects
        Write-Host "Building $name for SimulIDE..." -ForegroundColor Green
        & $avrGccExe $CommonFlags.Split(' ') $McStudioFlags.Split(' ') $ExactMcStudioFlags.Split(' ') $SourceFile ../../shared_libs/_port.c ../../shared_libs/_init.c ../../shared_libs/_uart.c ../../shared_libs/_glcd.c ../../shared_libs/_timer2.c ../../shared_libs/_adc.c ../../shared_libs/_buzzer.c ../../shared_libs/_eeprom.c -lm -o "$BaseName.elf"
    }
    
    if ($LASTEXITCODE -eq 0) {
        & $avrObjcopyExe -O ihex -R .eeprom "$BaseName.elf" "$BaseName.hex"
        Write-Host "Build completed!" -ForegroundColor Green
        Write-Host "Files: $BaseName.elf, $BaseName.hex" -ForegroundColor Gray
    }
    else {
        Write-Host "Build failed!" -ForegroundColor Red
    }
}
else {
    Write-Host "Error: Not a project folder" -ForegroundColor Red
}
