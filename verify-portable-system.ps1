#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Verify SOC3050 Portable System Setup
    
.DESCRIPTION
    Comprehensive verification that all required tools are present for the SOC3050 
    embedded systems course. This ensures students can use the system without any 
    external installations or dependencies.
    
    Checks:
    - AVR toolchain (gcc, objcopy, etc.)
    - SimulIDE simulators (both versions)
    - Project structure and build scripts
    - Shared libraries and headers
    
.EXAMPLE
    .\verify-portable-system.ps1
    Run complete verification
    
.EXAMPLE
    .\verify-portable-system.ps1 -Detailed
    Run with detailed output
#>

param(
    [switch]$Detailed = $false
)

# Colors for output
$SuccessColor = "Green"
$ErrorColor = "Red"
$WarningColor = "Yellow"
$InfoColor = "Cyan"

$script:ErrorCount = 0
$script:WarningCount = 0

function Test-Component {
    param(
        [string]$Name,
        [string]$Path,
        [string]$Type = "File",
        [switch]$Critical = $false
    )
    
    $exists = if ($Type -eq "File") { Test-Path $Path -PathType Leaf } else { Test-Path $Path -PathType Container }
    
    if ($exists) {
        Write-Host "[OK] $Name" -ForegroundColor $SuccessColor
        if ($Detailed) {
            Write-Host "   Location: $Path" -ForegroundColor Gray
        }
        return $true
    }
    else {
        $level = if ($Critical) { $ErrorColor; $script:ErrorCount++ } else { $WarningColor; $script:WarningCount++ }
        $prefix = if ($Critical) { "[ERROR]" } else { "[WARNING]" }
        Write-Host "$prefix $Name" -ForegroundColor $level
        Write-Host "   Missing: $Path" -ForegroundColor Gray
        return $false
    }
}

function Test-ExecutableVersion {
    param(
        [string]$Name,
        [string]$Path,
        [string]$VersionFlag = "--version"
    )
    
    if (Test-Path $Path) {
        try {
            $version = & $Path $VersionFlag 2>$null | Select-Object -First 1
            Write-Host "[OK] $Name" -ForegroundColor $SuccessColor
            if ($Detailed -and $version) {
                Write-Host "   Version: $version" -ForegroundColor Gray
            }
            return $true
        }
        catch {
            Write-Host "[WARNING] $Name (present but version check failed)" -ForegroundColor $WarningColor
            $script:WarningCount++
            return $false
        }
    }
    else {
        Write-Host "[ERROR] $Name" -ForegroundColor $ErrorColor
        Write-Host "   Missing: $Path" -ForegroundColor Gray
        $script:ErrorCount++
        return $false
    }
}

Write-Host "═══════════════════════════════════════════════════════════" -ForegroundColor $InfoColor
Write-Host "SOC3050 PORTABLE SYSTEM VERIFICATION" -ForegroundColor $InfoColor
Write-Host "═══════════════════════════════════════════════════════════" -ForegroundColor $InfoColor
Write-Host ""

$WorkspaceRoot = $PSScriptRoot

# 1. Core Directory Structure
Write-Host "CORE DIRECTORY STRUCTURE" -ForegroundColor $InfoColor
Write-Host "─────────────────────────────" -ForegroundColor Gray

Test-Component "Projects Directory" (Join-Path $WorkspaceRoot "projects") -Type Directory -Critical
Test-Component "Shared Libraries" (Join-Path $WorkspaceRoot "shared_libs") -Type Directory -Critical
Test-Component "Tools Directory" (Join-Path $WorkspaceRoot "tools") -Type Directory -Critical
Test-Component "Documentation" (Join-Path $WorkspaceRoot "docs") -Type Directory

Write-Host ""

# 2. AVR Toolchain (CRITICAL)
Write-Host "AVR TOOLCHAIN (CRITICAL)" -ForegroundColor $InfoColor
Write-Host "────────────────────────────" -ForegroundColor Gray

$avrToolchainDir = Join-Path $WorkspaceRoot "tools\avr-toolchain"
Test-Component "AVR Toolchain Directory" $avrToolchainDir -Type Directory -Critical

$avrBinDir = Join-Path $avrToolchainDir "bin"
if (Test-Component "AVR Bin Directory" $avrBinDir -Type Directory -Critical) {
    $avrGcc = Join-Path $avrBinDir "avr-gcc.exe"
    $avrObjcopy = Join-Path $avrBinDir "avr-objcopy.exe"
    $avrSize = Join-Path $avrBinDir "avr-size.exe"
    
    Test-ExecutableVersion "AVR-GCC Compiler" $avrGcc
    Test-ExecutableVersion "AVR-Objcopy" $avrObjcopy
    Test-ExecutableVersion "AVR-Size" $avrSize
    
    # Check for other essential tools
    Test-Component "AVR-AR" (Join-Path $avrBinDir "avr-ar.exe") -Critical
    Test-Component "AVR-LD" (Join-Path $avrBinDir "avr-ld.exe") -Critical
}

Write-Host ""

# 3. SimulIDE Simulators
Write-Host "SIMULIDE SIMULATORS" -ForegroundColor $InfoColor
Write-Host "──────────────────────" -ForegroundColor Gray

$simulideDir = Join-Path $WorkspaceRoot "tools\simulide"
Test-Component "SimulIDE Tools Directory" $simulideDir -Type Directory -Critical

# Check for both SimulIDE versions
$newSimulIDE = Join-Path $simulideDir "SimulIDE_1.1.0-SR1_Win64\simulide.exe"
$oldSimulIDE = Join-Path $simulideDir "SimulIDE_0.4.15-SR10_Win64\bin\simulide.exe"

Test-Component "SimulIDE 1.1.0 (New)" $newSimulIDE -Critical
Test-Component "SimulIDE 0.4.15 (Legacy)" $oldSimulIDE -Critical

# Check circuit files
Test-Component "Main Circuit File" (Join-Path $simulideDir "Simulator110.simu")
Test-Component "Legacy Circuit File" (Join-Path $simulideDir "Simulator0415.simu")

Write-Host ""

# 4. Build Scripts
Write-Host "BUILD SCRIPTS" -ForegroundColor $InfoColor
Write-Host "─────────────────" -ForegroundColor Gray

$cliDir = Join-Path $WorkspaceRoot "tools\cli"
Test-Component "CLI Tools Directory" $cliDir -Type Directory -Critical
Test-Component "Hardware Build Script" (Join-Path $cliDir "cli-build-project.ps1") -Critical
Test-Component "SimulIDE Build Script" (Join-Path $cliDir "cli-build-simulide.ps1") -Critical
Test-Component "SimulIDE Launch Script" (Join-Path $simulideDir "cli-simulide.ps1") -Critical

Write-Host ""

# 5. Shared Libraries
Write-Host "SHARED LIBRARIES" -ForegroundColor $InfoColor
Write-Host "───────────────────" -ForegroundColor Gray

$sharedLibs = Join-Path $WorkspaceRoot "shared_libs"
if (Test-Path $sharedLibs) {
    $criticalLibs = @(
        "_glcd.c", "_glcd.h",      # Graphics LCD
        "_init.c", "_init.h",      # System initialization
        "_port.c", "_port.h",      # Port control
        "_uart.c", "_uart.h"       # Serial communication
    )
    
    foreach ($lib in $criticalLibs) {
        Test-Component $lib (Join-Path $sharedLibs $lib) -Critical
    }
    
    # Optional libraries
    $optionalLibs = @("_timer2.c", "_timer2.h", "_adc.c", "_adc.h", "_buzzer.c", "_buzzer.h", "_eeprom.c", "_eeprom.h")
    foreach ($lib in $optionalLibs) {
        Test-Component $lib (Join-Path $sharedLibs $lib)
    }
}

Write-Host ""

# 6. Sample Projects
Write-Host "SAMPLE PROJECTS" -ForegroundColor $InfoColor
Write-Host "──────────────────" -ForegroundColor Gray

$projectsDir = Join-Path $WorkspaceRoot "projects"
if (Test-Path $projectsDir) {
    $sampleProjects = Get-ChildItem $projectsDir -Directory | Where-Object { Test-Path (Join-Path $_.FullName "Main.c") }
    
    if ($sampleProjects.Count -gt 0) {
        Write-Host "[OK] Found $($sampleProjects.Count) sample projects" -ForegroundColor $SuccessColor
        if ($Detailed) {
            foreach ($project in $sampleProjects) {
                Write-Host "   Project: $($project.Name)" -ForegroundColor Gray
            }
        }
    }
    else {
        Write-Host "[WARNING] No sample projects found" -ForegroundColor $WarningColor
        $script:WarningCount++
    }
}

Write-Host ""

# 7. Test Basic Build
Write-Host "BASIC BUILD TEST" -ForegroundColor $InfoColor
Write-Host "───────────────────" -ForegroundColor Gray

$testProject = Join-Path $projectsDir "Graphics_Display"
if (Test-Path (Join-Path $testProject "Main.c")) {
    Write-Host "Testing build for Graphics_Display project..." -ForegroundColor $InfoColor
    
    try {
        $buildScript = Join-Path $WorkspaceRoot "tools\cli\cli-build-project.ps1"
        $result = & powershell.exe -ExecutionPolicy Bypass -File $buildScript -ProjectDir $testProject 2>&1
        
        if ($LASTEXITCODE -eq 0 -and (Test-Path (Join-Path $testProject "Main.hex"))) {
            Write-Host "[OK] Build test successful" -ForegroundColor $SuccessColor
        }
        else {
            Write-Host "[ERROR] Build test failed" -ForegroundColor $ErrorColor
            $script:ErrorCount++
            if ($Detailed) {
                Write-Host "   Build output: $result" -ForegroundColor Gray
            }
        }
    }
    catch {
        Write-Host "[ERROR] Build test error: $($_.Exception.Message)" -ForegroundColor $ErrorColor
        $script:ErrorCount++
    }
}
else {
    Write-Host "[WARNING] Graphics_Display project not found - skipping build test" -ForegroundColor $WarningColor
    $script:WarningCount++
}

# 8. Summary
Write-Host ""
Write-Host "═══════════════════════════════════════════════════════════" -ForegroundColor $InfoColor
Write-Host "VERIFICATION SUMMARY" -ForegroundColor $InfoColor
Write-Host "═══════════════════════════════════════════════════════════" -ForegroundColor $InfoColor

if ($script:ErrorCount -eq 0 -and $script:WarningCount -eq 0) {
    Write-Host "PERFECT! Portable system is complete and ready to use." -ForegroundColor $SuccessColor
    Write-Host ""
    Write-Host "[OK] All critical components present" -ForegroundColor $SuccessColor
    Write-Host "[OK] Build system functional" -ForegroundColor $SuccessColor
    Write-Host "[OK] Students can download and start immediately" -ForegroundColor $SuccessColor
    
}
elseif ($script:ErrorCount -eq 0) {
    Write-Host "[OK] GOOD: System is functional with minor warnings" -ForegroundColor $SuccessColor
    Write-Host "[WARNING] Warnings: $script:WarningCount (non-critical)" -ForegroundColor $WarningColor
    Write-Host ""
    Write-Host "[OK] Students can use the system successfully" -ForegroundColor $SuccessColor
    
}
else {
    Write-Host "[ERROR] ISSUES FOUND: System needs attention" -ForegroundColor $ErrorColor
    Write-Host "[ERROR] Critical errors: $script:ErrorCount" -ForegroundColor $ErrorColor
    Write-Host "[WARNING] Warnings: $script:WarningCount" -ForegroundColor $WarningColor
    Write-Host ""
    Write-Host "[FIX] Please fix critical errors before distributing to students" -ForegroundColor $WarningColor
}

Write-Host ""
Write-Host "USAGE INSTRUCTIONS:" -ForegroundColor $InfoColor
Write-Host "  1. Download/clone the repository" -ForegroundColor White
Write-Host "  2. Run this verification script" -ForegroundColor White
Write-Host "  3. Navigate to projects/[ProjectName]" -ForegroundColor White
Write-Host "  4. Run: ..\..\tools\cli\cli-build-project.ps1" -ForegroundColor White
Write-Host "  5. For simulation: ..\..\tools\simulide\cli-simulide.ps1" -ForegroundColor White
Write-Host ""

# Return appropriate exit code
if ($script:ErrorCount -gt 0) {
    exit 1
}
else {
    exit 0
}