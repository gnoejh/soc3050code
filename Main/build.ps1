# ATmega128 Build Script for VS Code
# This script mimics Microchip Studio's smart compilation

param(
    [string]$Config = "Release"
)

$AVR_GCC = "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe"
$AVR_OBJCOPY = "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe"
$AVR_SIZE = "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe"

$CFLAGS = @(
    "-mmcu=atmega128",
    "-DF_CPU=16000000UL",
    "-Os",
    "-Wall", 
    "-std=gnu99",
    "-I."
)

Write-Host "🔧 Building ATmega128 Project..." -ForegroundColor Green

# Always needed files
$CORE_FILES = @("Main.c")

# Parse config.h to determine which modules are needed
$configContent = Get-Content "config.h"
$activeDefines = @()

foreach ($line in $configContent) {
    # Match lines like: #define SYMBOL or #define SYMBOL // comment
    if ($line -match '^#define\s+(\w+)(?:\s|//|$)') {
        $activeDefines += $Matches[1]
        Write-Host "✓ Active: $($Matches[1])" -ForegroundColor Yellow
    }
}

# Determine required source files based on active defines
# Start with core files
$sourceFiles = @("Main.c", "_init.c")

# Add files based on which examples are actually active (not just capability flags)
if ($activeDefines | Where-Object { $_ -like "ASSEMBLY_BLINK*" }) {
    $sourceFiles += "_port.c"
    # Only add main_blink_asm.c for assembly examples
    $sourceFiles += "main_blink_asm.c"
    # Note: main_blink_asm.c has its own UART ISR implementations, so don't include _uart.c
}

# Only add library files if specific features are being used
if ($activeDefines | Where-Object { $_ -like "TIMER_*" -or $_ -like "*TIMER*" }) {
    $sourceFiles += "_timer2.c", "main_timer.c"
}

if ($activeDefines | Where-Object { $_ -like "INTERRUPT_*" -or $_ -like "*INTERRUPT*" }) {
    $sourceFiles += "_interrupt.c", "main_interrupt.c"
}

if ($activeDefines | Where-Object { $_ -like "SERIAL_*" -and $_ -notlike "ASSEMBLY_*" }) {
    $sourceFiles += "_uart.c", "main_serial.c"
}

if ($activeDefines | Where-Object { $_ -like "ADC_*" }) {
    $sourceFiles += "_adc.c", "main_adc.c"
}

if ($activeDefines | Where-Object { $_ -like "GRAPHICS_*" -or $_ -like "*GLCD*" }) {
    $sourceFiles += "_glcd.c", "main_graphics.c"
}

if ($activeDefines | Where-Object { $_ -like "BUZZER_*" -or $_ -like "*BUZZER*" }) {
    $sourceFiles += "_buzzer.c"
}

if ($activeDefines | Where-Object { $_ -like "EEPROM_*" }) {
    $sourceFiles += "_eeprom.c", "main_memory.c"
}

if ($activeDefines -contains "EDUCATIONAL_DEMO") {
    $sourceFiles += "educational_demo.c"
}

if ($activeDefines -contains "ASSEMBLY_PROGRESSION_EXAMPLE") {
    $sourceFiles += "main_assembly_progression.c"
}

if ($activeDefines -contains "PYTHON_INTERFACE_EXAMPLE") {
    $sourceFiles += "main_python_interface.c", "_port_optimized.c", "_uart_optimized.c", "_init_optimized.c", "_adc_optimized.c", "_buzzer_optimized.c"
}

# Remove duplicates
$sourceFiles = $sourceFiles | Sort-Object | Get-Unique

Write-Host "📁 Source files to compile:" -ForegroundColor Cyan
$sourceFiles | ForEach-Object { Write-Host "   • $_" -ForegroundColor White }

# Compile
Write-Host "⚙️ Compiling..." -ForegroundColor Green
$compileArgs = $CFLAGS + $sourceFiles + @("-o", "Main.elf")

try {
    & $AVR_GCC @compileArgs
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✅ Compilation successful!" -ForegroundColor Green
        
        # Generate HEX file
        Write-Host "📦 Generating HEX file..." -ForegroundColor Green
        & $AVR_OBJCOPY -O ihex Main.elf Main.hex
        
        # Show memory usage
        Write-Host "📊 Memory usage:" -ForegroundColor Green
        & $AVR_SIZE Main.elf
        
        Write-Host "`n🎉 Build completed successfully!" -ForegroundColor Green
        Write-Host "   • ELF: Main.elf" -ForegroundColor White
        Write-Host "   • HEX: Main.hex" -ForegroundColor White
    }
}
catch {
    Write-Host "❌ Compilation failed!" -ForegroundColor Red
    Write-Host $_.Exception.Message -ForegroundColor Red
    exit 1
}