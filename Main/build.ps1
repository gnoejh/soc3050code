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
    if ($line -match '^#define\s+(\w+)' -and $line -notmatch '//') {
        $activeDefines += $Matches[1]
        Write-Host "✓ Active: $($Matches[1])" -ForegroundColor Yellow
    }
}

# Determine required source files based on active defines
$sourceFiles = @("Main.c", "_port.c", "variables.c")

# Add conditional source files  
if ($activeDefines -contains "BLINK_PORT" -or $activeDefines -contains "BLINK_PIN") {
    $sourceFiles += "main_blink.c"
    # For simple blink, we don't need full initialization
}

if ($activeDefines | Where-Object { $_ -like "SERIAL_*" }) {
    $sourceFiles += "main_serial.c", "_uart.c", "_init.c", "_interrupt.c", "_timer2.c", "_adc.c", "_glcd.c"
}

if ($activeDefines | Where-Object { $_ -like "ADC_*" }) {
    $sourceFiles += "_adc.c", "_init.c"
}

if ($activeDefines | Where-Object { $_ -like "GRAPHICS_*" }) {
    $sourceFiles += "_glcd.c", "_init.c"
}

if ($activeDefines | Where-Object { $_ -like "TIMER_*" }) {
    $sourceFiles += "_timer2.c", "_init.c"
}

if ($activeDefines | Where-Object { $_ -like "INTERRUPT_*" }) {
    $sourceFiles += "_interrupt.c", "_init.c"
}

# Educational examples
if ($activeDefines -contains "EDUCATIONAL_FLOW_TEST") {
    $sourceFiles += "educational_flow_test.c", "_init.c", "_uart.c", "_port.c", "_adc.c", "_timer2.c", "_interrupt.c"
}

if ($activeDefines -contains "EDUCATIONAL_SIMPLE_TEST") {
    $sourceFiles += "educational_simple_test.c", "_init.c", "_uart.c", "_port.c", "_adc.c", "_timer2.c", "_interrupt.c", "_glcd.c"
}

if ($activeDefines -contains "EDUCATIONAL_DEMO") {
    $sourceFiles += "educational_demo.c", "_init.c", "_uart.c", "_port.c", "_adc.c", "_timer2.c", "_interrupt.c", "_glcd.c"
}

if ($activeDefines -contains "ASSEMBLY_PROGRESSION_EXAMPLE") {
    $sourceFiles += "main_assembly_progression.c", "_port_optimized.c", "_init_optimized.c"
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