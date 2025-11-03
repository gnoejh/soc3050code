# SimulIDE 1.1.0 Patch Verification Script
# Verifies that both ELPM and UART patches are correctly applied

param(
    [string]$SimulIDEPath = "tools\simulide\SimulIDE_1.1.0-SR1_Win64"
)

$WorkspaceRoot = $PSScriptRoot
if (-not $WorkspaceRoot) {
    $WorkspaceRoot = Get-Location
}

# If running from tools/simulide, go up two levels
if ($WorkspaceRoot -match 'tools[\\/]simulide$') {
    $WorkspaceRoot = Split-Path (Split-Path $WorkspaceRoot -Parent) -Parent
}

$ErrorCount = 0
$WarningCount = 0

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "SimulIDE 1.1.0 Patch Verification" -ForegroundColor Cyan
Write-Host "========================================`n" -ForegroundColor Cyan

# Check if SimulIDE exists
$SimulIDEExe = Join-Path $WorkspaceRoot "$SimulIDEPath\simulide.exe"
if (-not (Test-Path $SimulIDEExe)) {
    Write-Host "❌ ERROR: SimulIDE not found at: $SimulIDEExe" -ForegroundColor Red
    $ErrorCount++
    exit 1
}
Write-Host "✅ SimulIDE found: $SimulIDEExe" -ForegroundColor Green

# Check ELPM Patch
Write-Host "`n--- Checking ELPM Patch ---" -ForegroundColor Yellow
$MCUFile = Join-Path $WorkspaceRoot "$SimulIDEPath\data\AVR\mega128.mcu"
$MCUModified = Join-Path $WorkspaceRoot "$SimulIDEPath\data\AVR\mega128modified.mcu"

if (Test-Path $MCUFile) {
    $content = Get-Content $MCUFile -Raw
    if ($content -match 'register\s+name="RAMPZ"') {
        Write-Host "✅ ELPM patch applied: RAMPZ register found" -ForegroundColor Green
    }
    else {
        Write-Host "❌ ELPM patch NOT applied: RAMPZ register missing" -ForegroundColor Red
        $ErrorCount++
    }
}
else {
    Write-Host "❌ mega128.mcu file not found!" -ForegroundColor Red
    $ErrorCount++
}

if (Test-Path $MCUModified) {
    Write-Host "✅ Reference file exists: mega128modified.mcu" -ForegroundColor Green
}
else {
    Write-Host "⚠️  WARNING: Reference file missing: mega128modified.mcu" -ForegroundColor Yellow
    $WarningCount++
}

# Check UART Patch
Write-Host "`n--- Checking UART Patch ---" -ForegroundColor Yellow
$PerifFile = Join-Path $WorkspaceRoot "$SimulIDEPath\data\AVR\mega64_128\mega64_perif.xml"
$PerifModified = Join-Path $WorkspaceRoot "$SimulIDEPath\data\AVR\mega64_128\mega64_perif_modified.xml"
$PerifBackup = Join-Path $WorkspaceRoot "$SimulIDEPath\data\AVR\mega64_128\mega64_perif_original_backup.xml"

if (Test-Path $PerifFile) {
    $content = Get-Content $PerifFile -Raw
    
    $hasUSART0 = $content -match '<usart\s+name="USART0"'
    $hasUSART1 = $content -match '<usart\s+name="USART1"'
    
    if ($hasUSART0 -and $hasUSART1) {
        Write-Host "✅ UART patch applied: Both USART0 and USART1 defined" -ForegroundColor Green
        
        # Check pin assignments
        if ($content -match 'pin="PORTE1".*register="UDR0"') {
            Write-Host "  ✅ USART0 TX pin: PORTE1 (correct)" -ForegroundColor Green
        }
        else {
            Write-Host "  ⚠️  USART0 TX pin may be incorrect" -ForegroundColor Yellow
            $WarningCount++
        }
        
        if ($content -match 'pin="PORTD3".*register="UDR1"') {
            Write-Host "  ✅ USART1 TX pin: PORTD3 (correct)" -ForegroundColor Green
        }
        else {
            Write-Host "  ⚠️  USART1 TX pin may be incorrect" -ForegroundColor Yellow
            $WarningCount++
        }
    }
    else {
        Write-Host "❌ UART patch NOT applied: USART peripherals missing" -ForegroundColor Red
        $ErrorCount++
    }
}
else {
    Write-Host "❌ mega64_perif.xml file not found!" -ForegroundColor Red
    $ErrorCount++
}

if (Test-Path $PerifModified) {
    Write-Host "✅ Reference file exists: mega64_perif_modified.xml" -ForegroundColor Green
}
else {
    Write-Host "⚠️  WARNING: Reference file missing: mega64_perif_modified.xml" -ForegroundColor Yellow
    $WarningCount++
}

if (Test-Path $PerifBackup) {
    Write-Host "✅ Backup exists: mega64_perif_original_backup.xml" -ForegroundColor Green
}
else {
    Write-Host "⚠️  WARNING: Backup missing: mega64_perif_original_backup.xml" -ForegroundColor Yellow
    $WarningCount++
}

# Summary
Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "Verification Summary" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

if ($ErrorCount -eq 0 -and $WarningCount -eq 0) {
    Write-Host "✅ ALL PATCHES VERIFIED SUCCESSFULLY!" -ForegroundColor Green
    Write-Host "   SimulIDE 1.1.0 is fully patched for ATmega128" -ForegroundColor Green
    exit 0
}
elseif ($ErrorCount -eq 0) {
    Write-Host "⚠️  PATCHES APPLIED BUT WARNINGS PRESENT" -ForegroundColor Yellow
    Write-Host "   Errors: $ErrorCount" -ForegroundColor White
    Write-Host "   Warnings: $WarningCount" -ForegroundColor Yellow
    Write-Host "`n   Patches are functional but reference files may be missing." -ForegroundColor Yellow
    exit 0
}
else {
    Write-Host "❌ PATCH VERIFICATION FAILED!" -ForegroundColor Red
    Write-Host "   Errors: $ErrorCount" -ForegroundColor Red
    Write-Host "   Warnings: $WarningCount" -ForegroundColor Yellow
    Write-Host "`n   Please reapply patches. See: SIMULIDE_PATCHES_QUICK_REF.md" -ForegroundColor Red
    exit 1
}
