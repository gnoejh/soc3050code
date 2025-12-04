# Remove com0com Virtual Port Pair
# Run this to clean up COM4-COM5 pair before using TCP bridge

Write-Host "=== Remove com0com Port Pair ===" -ForegroundColor Cyan
Write-Host ""
Write-Host "This script will remove the COM4-COM5 virtual port pair" -ForegroundColor Yellow
Write-Host "so that COM4 can be used for the TCP bridge." -ForegroundColor Yellow
Write-Host ""

# Check if running as Administrator
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)

if (-not $isAdmin) {
    Write-Host "✗ This script requires Administrator privileges" -ForegroundColor Red
    Write-Host ""
    Write-Host "Please run as Administrator:" -ForegroundColor Yellow
    Write-Host "  1. Right-click PowerShell" -ForegroundColor White
    Write-Host "  2. Select 'Run as Administrator'" -ForegroundColor White
    Write-Host "  3. Run this script again" -ForegroundColor White
    Write-Host ""
    pause
    exit 1
}

# Find com0com installation
$com0comPaths = @(
    "C:\Program Files (x86)\com0com",
    "C:\Program Files\com0com",
    "C:\com0com"
)

$setupcPath = $null
foreach ($path in $com0comPaths) {
    $testPath = Join-Path $path "setupc.exe"
    if (Test-Path $testPath) {
        $setupcPath = $testPath
        break
    }
}

if (-not $setupcPath) {
    Write-Host "✗ com0com not found in standard locations" -ForegroundColor Red
    Write-Host ""
    Write-Host "Searched locations:" -ForegroundColor Gray
    foreach ($path in $com0comPaths) {
        Write-Host "  - $path" -ForegroundColor Gray
    }
    Write-Host ""
    Write-Host "If com0com is installed elsewhere, remove the port pair manually:" -ForegroundColor Yellow
    Write-Host "  1. Run 'Setup for com0com' from Start Menu" -ForegroundColor White
    Write-Host "  2. Select the COM4-COM5 pair" -ForegroundColor White
    Write-Host "  3. Click 'Remove Pair'" -ForegroundColor White
    Write-Host ""
    pause
    exit 1
}

Write-Host "✓ Found com0com at: $setupcPath" -ForegroundColor Green
Write-Host ""

# List current pairs
Write-Host "Current com0com port pairs:" -ForegroundColor Cyan
& $setupcPath list

Write-Host ""
Write-Host "Removing port pair 0 (COM4-COM5)..." -ForegroundColor Yellow

try {
    & $setupcPath remove 0
    Write-Host ""
    Write-Host "✓ Port pair removed successfully!" -ForegroundColor Green
    Write-Host ""
    Write-Host "You can now use the TCP bridge with COM4" -ForegroundColor Cyan
}
catch {
    Write-Host ""
    Write-Host "✗ Failed to remove port pair: $_" -ForegroundColor Red
    Write-Host ""
    Write-Host "Try removing manually:" -ForegroundColor Yellow
    Write-Host "  1. Run 'Setup for com0com' from Start Menu as Administrator" -ForegroundColor White
    Write-Host "  2. Select the pair with COM4" -ForegroundColor White
    Write-Host "  3. Click 'Remove Pair'" -ForegroundColor White
}

Write-Host ""
pause
