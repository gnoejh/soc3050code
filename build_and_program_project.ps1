param(
    [Parameter(Mandatory=$true)]
    [string]$ProjectDir
)

# Extract project name from directory path
Write-Host "🔍 Analyzing project directory: $ProjectDir" -ForegroundColor Cyan

if ($ProjectDir -like '*\projects\*') {
    $projectName = Split-Path $ProjectDir -Leaf
    Write-Host "🎯 Detected project: $projectName" -ForegroundColor Green
    
    Write-Host "🔨 Building and programming: $projectName" -ForegroundColor Green
    
    # Change to the project directory for building
    Set-Location $ProjectDir
    
    # Step 1: Build the ELF file
    Write-Host "📦 Step 1: Building ELF file..." -ForegroundColor Yellow
    $buildResult = & "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe" -mmcu=atmega128 -DF_CPU=7372800UL -Os -Wall -Wextra -I. -I../../shared_libs Main.c ../../shared_libs/_port.c ../../shared_libs/_init.c -o Main.elf 2>&1
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "❌ Build failed!" -ForegroundColor Red
        Write-Host $buildResult -ForegroundColor Red
        exit 1
    }
    Write-Host "✅ Build successful" -ForegroundColor Green
    
    # Step 2: Generate HEX file
    Write-Host "🔧 Step 2: Generating HEX file..." -ForegroundColor Yellow
    $hexResult = & "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom Main.elf Main.hex 2>&1
    
    if ($LASTEXITCODE -ne 0) {
        Write-Host "❌ HEX generation failed!" -ForegroundColor Red
        Write-Host $hexResult -ForegroundColor Red
        exit 1
    }
    Write-Host "✅ HEX file generated" -ForegroundColor Green
    
    # Step 3: Program the device
    Write-Host "📡 Step 3: Programming device..." -ForegroundColor Yellow
    
    # Change to workspace directory for programming script
    Set-Location "w:\soc3050code"
    
    # Call the universal programming script
    & ".\program.ps1" -ProjectFolder $projectName
    
} else {
    Write-Host "❌ Error: Please open a file from a project folder" -ForegroundColor Red
    Write-Host "   Current directory: $ProjectDir" -ForegroundColor Gray
    exit 1
}