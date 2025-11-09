# PowerShell script to unlock and delete PWM_Motor_Stepper folder
# Run this script as Administrator for best results

Write-Host "=== Unlocking PWM_Motor_Stepper folder ===" -ForegroundColor Yellow
Write-Host ""

$folderPath = "w:\soc3050code\projects\PWM_Motor_Stepper"

# Check if folder exists
if (-not (Test-Path $folderPath)) {
    Write-Host "Folder already deleted or doesn't exist!" -ForegroundColor Green
    exit 0
}

Write-Host "Attempting to close handles to the folder..." -ForegroundColor Cyan

# Try to remove read-only attributes
Get-ChildItem -Path $folderPath -Recurse -Force | ForEach-Object {
    $_.IsReadOnly = $false
}

# Try to delete the folder
try {
    Remove-Item -Path $folderPath -Recurse -Force -ErrorAction Stop
    Write-Host ""
    Write-Host "SUCCESS: Folder deleted!" -ForegroundColor Green
} catch {
    Write-Host ""
    Write-Host "ERROR: Could not delete folder" -ForegroundColor Red
    Write-Host $_.Exception.Message -ForegroundColor Red
    Write-Host ""
    Write-Host "Please try:" -ForegroundColor Yellow
    Write-Host "1. Close ALL Cursor windows completely" -ForegroundColor White
    Write-Host "2. Close SimulIDE if running" -ForegroundColor White
    Write-Host "3. Close any Windows Explorer windows showing this folder" -ForegroundColor White
    Write-Host "4. Restart your computer if needed" -ForegroundColor White
    Write-Host ""
    Write-Host "Or manually delete after closing all applications." -ForegroundColor Cyan
}

