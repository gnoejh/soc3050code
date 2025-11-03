# SOC3050 PowerShell Execution Policy Fix
# Copy and paste these commands into PowerShell (Run as Administrator)

Write-Host "🔧 SOC3050 PowerShell Setup" -ForegroundColor Cyan
Write-Host "=" * 40 -ForegroundColor Cyan
Write-Host ""

Write-Host "Current Execution Policy:" -ForegroundColor Yellow
Get-ExecutionPolicy

Write-Host ""
Write-Host "To run SOC3050 scripts, you have these options:" -ForegroundColor Green
Write-Host ""

Write-Host "Option 1 (Recommended): Use the batch file wrapper" -ForegroundColor Cyan
Write-Host "   Double-click: setup.bat" -ForegroundColor Gray
Write-Host ""

Write-Host "Option 2: Run with bypass flag" -ForegroundColor Cyan
Write-Host "   powershell -ExecutionPolicy Bypass -File verify-environment.ps1" -ForegroundColor Gray
Write-Host ""

Write-Host "Option 3: Change policy temporarily (requires Admin)" -ForegroundColor Cyan
Write-Host "   Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser" -ForegroundColor Gray
Write-Host ""

Write-Host "Option 4: Use VS Code directly" -ForegroundColor Cyan
Write-Host "   code ." -ForegroundColor Gray
Write-Host "   Then use Ctrl+Shift+P -> Tasks: Run Task" -ForegroundColor Gray
Write-Host ""

$choice = Read-Host "Do you want to set RemoteSigned policy for current user? (y/N)"
if ($choice -eq 'y' -or $choice -eq 'Y') {
    try {
        Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser -Force
        Write-Host "✅ Execution policy updated!" -ForegroundColor Green
        Write-Host "You can now run: .\verify-environment.ps1" -ForegroundColor Green
    }
    catch {
        Write-Host "❌ Failed to set execution policy. You may need to run as Administrator." -ForegroundColor Red
        Write-Host "Use Option 1 (batch file) instead." -ForegroundColor Yellow
    }
}
else {
    Write-Host "No changes made. Use Option 1 (setup.bat wrapper) for verification." -ForegroundColor Yellow
}

Write-Host ""
Write-Host "Press any key to continue..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")