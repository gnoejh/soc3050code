param(
    [Parameter(Mandatory=$true)]
    [string]$FilePath
)

# Extract project name from file path
Write-Host "🔍 Analyzing file path: $FilePath" -ForegroundColor Cyan

if ($FilePath -match '\\projects\\([^\\]+)\\') {
    $projectName = $matches[1]
    Write-Host "🎯 Detected project: $projectName" -ForegroundColor Green
    
    # Change to workspace directory
    Set-Location "w:\soc3050code"
    
    # Get project directory
    $projectDir = "w:\soc3050code\projects\$projectName"
    $hexFile = "$projectDir\Main.hex"
    
    # Check if HEX file exists, if not build it
    if (-not (Test-Path $hexFile)) {
        Write-Host "🔨 HEX file not found, building project first..." -ForegroundColor Yellow
        Set-Location $projectDir
        
        # Build ELF file
        & "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe" -mmcu=atmega128 -DF_CPU=7372800UL -Os -Wall -Wextra -I. -I../../shared_libs Main.c ../../shared_libs/_port.c ../../shared_libs/_init.c -o Main.elf
        
        if ($LASTEXITCODE -eq 0) {
            Write-Host "✅ Build successful" -ForegroundColor Green
            
            # Generate HEX file
            & "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom Main.elf Main.hex
            
            if ($LASTEXITCODE -eq 0) {
                Write-Host "✅ HEX file generated" -ForegroundColor Green
            } else {
                Write-Host "❌ HEX generation failed" -ForegroundColor Red
                exit 1
            }
        } else {
            Write-Host "❌ Build failed" -ForegroundColor Red
            exit 1
        }
        
        # Change back to workspace directory
        Set-Location "w:\soc3050code"
    }
    
    # Call the universal programming script
    Write-Host "📡 Programming project..." -ForegroundColor Yellow
    & ".\program.ps1" -ProjectFolder $projectName
} else {
    Write-Host "❌ Error: Please open a file from a project folder (path should contain \projects\)" -ForegroundColor Red
    Write-Host "   Current path: $FilePath" -ForegroundColor Gray
    exit 1
}