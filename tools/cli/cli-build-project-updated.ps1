param([string]$ProjectDir)

Write-Host "Analyzing: $ProjectDir"

if ($ProjectDir -like "*\projects\*") {
    $name = Split-Path $ProjectDir -Leaf
    Write-Host "Building: $name"
    Set-Location $ProjectDir
    
    # MC Studio compatible compiler flags (based on working MC Studio build)
    $McStudioFlags = "-funsigned-char -funsigned-bitfields -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax"
    $CommonFlags = "-mmcu=atmega128 -DF_CPU=16000000UL -DBAUD=9600 -O3 -Wall -I. -I../../shared_libs"
    $AvrGccPath = "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe"
    
    # Special handling for Serial_interrupt project to avoid ISR conflicts
    if ($name -eq "Serial_interrupt") {
        Write-Host "Building Serial_interrupt with custom UART interrupt handling..." -ForegroundColor Yellow
        & $AvrGccPath $CommonFlags.Split(' ') $McStudioFlags.Split(' ') -DCUSTOM_UART_INTERRUPTS Main.c ../../shared_libs/_port.c ../../shared_libs/_init.c ../../shared_libs/_uart.c ../../shared_libs/_glcd.c ../../shared_libs/_timer2.c ../../shared_libs/_adc.c ../../shared_libs/_buzzer.c ../../shared_libs/_eeprom.c ../../shared_libs/_interrupt.c -lm -o Main.elf
    } else {
        # Standard build for other projects
        & $AvrGccPath $CommonFlags.Split(' ') $McStudioFlags.Split(' ') Main.c ../../shared_libs/_port.c ../../shared_libs/_init.c ../../shared_libs/_uart.c ../../shared_libs/_glcd.c ../../shared_libs/_timer2.c ../../shared_libs/_adc.c ../../shared_libs/_buzzer.c ../../shared_libs/_eeprom.c ../../shared_libs/_interrupt.c -lm -o Main.elf
    }
    
    if ($LASTEXITCODE -eq 0) {
        & "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom Main.elf Main.hex
        Write-Host "Build completed!" -ForegroundColor Green
        Write-Host "Files: Main.elf, Main.hex" -ForegroundColor Gray
    } else {
        Write-Host "Build failed!" -ForegroundColor Red
    }
} else {
    Write-Host "Error: Not a project folder" -ForegroundColor Red
}