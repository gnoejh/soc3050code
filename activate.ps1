# ATmega128 Student Environment Activation
# Run this to start working with your ATmega128

Write-Host "🎓 ATmega128 Student Environment" -ForegroundColor Green
Write-Host "=================================" -ForegroundColor Green

# Activate Python environment
if (Test-Path ".venv\Scripts\Activate.ps1") {
    . .venv\Scripts\Activate.ps1
    Write-Host "✅ Python environment activated!" -ForegroundColor Green
    Write-Host ""
    
    # Show quick commands
    Write-Host "🚀 Quick Commands:" -ForegroundColor Cyan
    Write-Host "  F7            - Build C code (or: cd Main; .\build.ps1)" -ForegroundColor White
    Write-Host "  F8            - Program ATmega128 (or: cd Main; .\program.ps1)" -ForegroundColor White
    Write-Host "  Ctrl+Shift+``  - Open terminal" -ForegroundColor White
    Write-Host ""
    
    Write-Host "🐍 Python Examples:" -ForegroundColor Cyan
    Write-Host "  python python\examples\quick_start.py     - Start here!" -ForegroundColor White
    Write-Host "  python python\examples\example_01_basic.py - Basic communication" -ForegroundColor White
    Write-Host "  python python\examples\example_02_sensors.py - Read sensors" -ForegroundColor White
    Write-Host "  python python\examples\example_03_realtime.py - Live graphs" -ForegroundColor White
    Write-Host "  python python\examples\example_04_iot.py - Web dashboard" -ForegroundColor White
    Write-Host "  python python\examples\example_05_analysis.py - Data analysis" -ForegroundColor White
    Write-Host ""
    
    Write-Host "🔧 Hardware Setup:" -ForegroundColor Cyan
    Write-Host "  1. Connect ATmega128 via USB" -ForegroundColor White
    Write-Host "  2. Check device manager for COM port" -ForegroundColor White
    Write-Host "  3. Update port in Python examples" -ForegroundColor White
    Write-Host ""
    
    Write-Host "📚 Documentation:" -ForegroundColor Cyan
    Write-Host "  README.md                    - Main documentation" -ForegroundColor White
    Write-Host "  python\examples\README.md    - Python examples guide" -ForegroundColor White
    Write-Host ""
    
    Write-Host "💡 Need help? Check the documentation or ask your instructor!" -ForegroundColor Yellow
    
}
else {
    Write-Host "❌ Virtual environment not found!" -ForegroundColor Red
    Write-Host "💡 Run setup.ps1 to create the environment" -ForegroundColor Yellow
}