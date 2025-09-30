# Node.js and npm Verification Script
# Run this to test if Node.js and npm are working correctly in VS Code

Write-Host "🔍 Node.js and npm Verification" -ForegroundColor Green
Write-Host "===============================" -ForegroundColor Green

# Test Node.js
Write-Host "📦 Testing Node.js..." -ForegroundColor Yellow
try {
    $nodeVersion = node --version
    Write-Host "✅ Node.js: $nodeVersion" -ForegroundColor Green
    
    $nodePath = (Get-Command node).Source
    Write-Host "📁 Path: $nodePath" -ForegroundColor White
} catch {
    Write-Host "❌ Node.js not found or not working" -ForegroundColor Red
    Write-Host "Error: $($_.Exception.Message)" -ForegroundColor Red
}

# Test npm
Write-Host "`n📦 Testing npm..." -ForegroundColor Yellow
try {
    $npmVersion = npm --version
    Write-Host "✅ npm: $npmVersion" -ForegroundColor Green
    
    $npmPath = (Get-Command npm).Source
    Write-Host "📁 Path: $npmPath" -ForegroundColor White
} catch {
    Write-Host "❌ npm not found or not working" -ForegroundColor Red
    Write-Host "Error: $($_.Exception.Message)" -ForegroundColor Red
}

# Test npm basic functionality
Write-Host "`n🧪 Testing npm functionality..." -ForegroundColor Yellow
try {
    $npmConfig = npm config get registry
    Write-Host "✅ npm registry: $npmConfig" -ForegroundColor Green
    
    # Test npm list (should work even with no packages)
    npm list --depth=0 2>$null | Out-Null
    Write-Host "✅ npm list command works" -ForegroundColor Green
} catch {
    Write-Host "❌ npm functionality test failed" -ForegroundColor Red
    Write-Host "Error: $($_.Exception.Message)" -ForegroundColor Red
}

# Environment check
Write-Host "`n🌍 Environment Check..." -ForegroundColor Yellow
$pathEntries = $env:PATH -split ';' | Where-Object { $_ -like '*nodejs*' }
if ($pathEntries) {
    Write-Host "✅ Node.js in PATH: $($pathEntries -join ', ')" -ForegroundColor Green
} else {
    Write-Host "⚠️ Node.js not found in PATH" -ForegroundColor Yellow
}

# VS Code specific check
Write-Host "`n💻 VS Code Integration Check..." -ForegroundColor Yellow
if (Test-Path "$env:USERPROFILE\.vscode") {
    Write-Host "✅ VS Code user directory found" -ForegroundColor Green
} else {
    Write-Host "⚠️ VS Code user directory not found" -ForegroundColor Yellow
}

# Test in current directory context
Write-Host "`n📂 Current Directory Test..." -ForegroundColor Yellow
$currentDir = Get-Location
Write-Host "📁 Current directory: $currentDir" -ForegroundColor White

# Try to run node in current context
try {
    $testResult = node -e "console.log('Node.js working in current directory')" 2>&1
    if ($testResult -like "*Node.js working*") {
        Write-Host "✅ Node.js execution test passed" -ForegroundColor Green
    } else {
        Write-Host "⚠️ Node.js execution test gave unexpected result: $testResult" -ForegroundColor Yellow
    }
} catch {
    Write-Host "❌ Node.js execution test failed" -ForegroundColor Red
    Write-Host "Error: $($_.Exception.Message)" -ForegroundColor Red
}

Write-Host "`n🎯 Summary:" -ForegroundColor Cyan
Write-Host "If all tests show ✅, Node.js and npm are properly configured for VS Code." -ForegroundColor White
Write-Host "If you see ❌ or ⚠️, there might be configuration issues." -ForegroundColor White
Write-Host "`n💡 If VS Code still complains, try:" -ForegroundColor Yellow
Write-Host "1. Restart VS Code completely" -ForegroundColor Gray
Write-Host "2. Reload window (Ctrl+Shift+P → 'Developer: Reload Window')" -ForegroundColor Gray
Write-Host "3. Check VS Code's integrated terminal settings" -ForegroundColor Gray