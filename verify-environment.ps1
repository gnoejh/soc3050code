#!/usr/bin/env pwsh
# SOC3050 Self-Sufficient Environment Verification Script
# This script verifies that all required tools are bundled and working
# Run this after cloning the repository to ensure everything is ready

param(
    [switch]$Verbose,
    [switch]$Quick,
    [switch]$Fix
)

$ErrorActionPreference = "Stop"
$SuccessColor = "Green"
$WarningColor = "Yellow" 
$ErrorColor = "Red"
$InfoColor = "Cyan"

function Write-Status {
    param($Message, $Status, $Details = "")
    
    $statusSymbol = switch ($Status) {
        "Success" { "[OK] "; $color = $SuccessColor }
        "Warning" { "[WARN] "; $color = $WarningColor }
        "Error" { "[ERROR] "; $color = $ErrorColor }
        "Info" { "[INFO] "; $color = $InfoColor }
        default { ""; $color = "White" }
    }
    
    Write-Host "$statusSymbol $Message" -ForegroundColor $color
    if ($Details -and $Verbose) {
        Write-Host "   $Details" -ForegroundColor Gray
    }
}

function Test-Tool {
    param($Name, $Path, $TestCommand = $null, $RequiredFor = "")
    
    if (Test-Path $Path) {
        if ($TestCommand) {
            try {
                $result = & $Path $TestCommand 2>&1
                if ($LASTEXITCODE -eq 0) {
                    $version = ($result -split "`n")[0]
                    Write-Status "$Name found and working" "Success" "$version"
                    return $true
                }
                else {
                    Write-Status "$Name found but not responding correctly" "Warning" $result
                    return $false
                }
            }
            catch {
                Write-Status "$Name found but execution failed" "Warning" $_.Exception.Message
                return $false
            }
        }
        else {
            Write-Status "$Name found" "Success" $Path
            return $true
        }
    }
    else {
        Write-Status "$Name not found" "Error" "Required for: $RequiredFor"
        return $false
    }
}

Write-Host "[START] SOC3050 Self-Sufficient Environment Verification" -ForegroundColor $InfoColor
Write-Host "=" * 50 -ForegroundColor $InfoColor
Write-Host ""

# Determine workspace root
$WorkspaceRoot = $PSScriptRoot
Write-Status "Workspace Root" "Info" $WorkspaceRoot

$allPassed = $true

Write-Host "`n=== Core Development Tools ===" -ForegroundColor $InfoColor

# 1. AVR Toolchain
$avrGcc = Join-Path $WorkspaceRoot "tools\avr-toolchain\bin\avr-gcc.exe"
$passed = Test-Tool "AVR-GCC Compiler" $avrGcc "--version" "Building C code for ATmega128"
$allPassed = $allPassed -and $passed

$avrObjcopy = Join-Path $WorkspaceRoot "tools\avr-toolchain\bin\avr-objcopy.exe"
$passed = Test-Tool "AVR-Objcopy" $avrObjcopy "--version" "Generating HEX files"
$allPassed = $allPassed -and $passed

$avrSize = Join-Path $WorkspaceRoot "tools\avr-toolchain\bin\avr-size.exe"
$passed = Test-Tool "AVR-Size" $avrSize "--version" "Memory usage analysis"
$allPassed = $allPassed -and $passed

# 2. ATmega DFP
$dfpPath = Join-Path $WorkspaceRoot "tools\packs\atmel\ATmega_DFP\1.7.374"
$dfpInclude = Join-Path $dfpPath "include"
$dfpDevice = Join-Path $dfpPath "gcc\dev\atmega128"
$passed = (Test-Path $dfpInclude) -and (Test-Path $dfpDevice)
if ($passed) {
    Write-Status "ATmega128 Device Family Pack" "Success" $dfpPath
}
else {
    Write-Status "ATmega128 Device Family Pack incomplete" "Error" "Required for device-specific optimizations"
    $allPassed = $false
}

Write-Host "`n=== Simulation Tools ===" -ForegroundColor $InfoColor

# 3. SimulIDE
$simulideExe = Join-Path $WorkspaceRoot "tools\simulide\SimulIDE_1.1.0-SR1_Win64\simulide.exe"
$passed = Test-Tool "SimulIDE Simulator" $simulideExe $null "Circuit simulation and testing"
$allPassed = $allPassed -and $passed

Write-Host "`n=== Build Scripts ===" -ForegroundColor $InfoColor

# 4. Core build scripts
$buildScript = Join-Path $WorkspaceRoot "tools\cli\cli-build-project.ps1"
$passed = Test-Tool "Build Script" $buildScript $null "Building projects"
$allPassed = $allPassed -and $passed

$programScript = Join-Path $WorkspaceRoot "tools\cli\cli-program-project.ps1"
$passed = Test-Tool "Programming Script" $programScript $null "Programming microcontrollers"
$allPassed = $allPassed -and $passed

if (-not $Quick) {
    Write-Host "`n=== VS Code Configuration ===" -ForegroundColor $InfoColor
    
    # 5. VS Code configuration files
    $tasksJson = Join-Path $WorkspaceRoot ".vscode\tasks.json"
    $passed = Test-Tool "VS Code Tasks" $tasksJson $null "Build and run tasks in VS Code"
    $allPassed = $allPassed -and $passed
    
    $settingsJson = Join-Path $WorkspaceRoot ".vscode\settings.json"
    $passed = Test-Tool "VS Code Settings" $settingsJson $null "IntelliSense and editor configuration"
    $allPassed = $allPassed -and $passed
    
    Write-Host "`n=== Sample Projects ===" -ForegroundColor $InfoColor
    
    # 6. Check for sample projects
    $projectsDir = Join-Path $WorkspaceRoot "projects"
    $sampleProjects = @("Port_Basic", "Timer_Basic", "Serial_Communications", "Graphics_Display")
    
    foreach ($project in $sampleProjects) {
        $projectPath = Join-Path $projectsDir $project
        $mainC = Join-Path $projectPath "Main.c"
        $passed = Test-Path $mainC
        if ($passed) {
            Write-Status "Sample Project: $project" "Success" $projectPath
        }
        else {
            Write-Status "Sample Project: $project" "Warning" "Main.c not found"
        }
    }
}

Write-Host "`n=== Test Build ===" -ForegroundColor $InfoColor

if (-not $Quick) {
    # 7. Test build a simple project
    $testProject = Join-Path $WorkspaceRoot "projects\Port_Basic"
    if (Test-Path $testProject) {
        try {
            Push-Location $testProject
            Write-Status "Testing build process..." "Info"
            
            $buildResult = & (Join-Path $WorkspaceRoot "tools\cli\cli-build-project.ps1") -ProjectDir $testProject 2>&1
            
            if ($LASTEXITCODE -eq 0 -and (Test-Path "Main.elf")) {
                Write-Status "Test build successful" "Success" "Port_Basic project compiled"
                
                # Clean up test artifacts
                Remove-Item -Path "*.o", "*.elf", "*.hex", "*.map" -Force -ErrorAction SilentlyContinue
            }
            else {
                Write-Status "Test build failed" "Error" $buildResult
                $allPassed = $false
            }
        }
        catch {
            Write-Status "Test build error" "Error" $_.Exception.Message
            $allPassed = $false
        }
        finally {
            Pop-Location
        }
    }
    else {
        Write-Status "Test project not found" "Warning" "Cannot verify build process"
    }
}

Write-Host "`n" + "=" * 50 -ForegroundColor $InfoColor

if ($allPassed) {
    Write-Status "Environment verification PASSED" "Success" "Ready for development!"
    Write-Host ""
    Write-Host "[NEXT] Next steps:" -ForegroundColor $InfoColor
    Write-Host "   1. Open VS Code in this directory" -ForegroundColor Gray
    Write-Host "   2. Navigate to projects/ folder" -ForegroundColor Gray
    Write-Host "   3. Choose a project and start coding!" -ForegroundColor Gray
    Write-Host "   4. Use Ctrl+Shift+P -> 'Tasks: Run Task' to build" -ForegroundColor Gray
    exit 0
}
else {
    Write-Status "Environment verification FAILED" "Error" "Some components are missing or not working"
    Write-Host ""
    Write-Host "[FIX] Troubleshooting:" -ForegroundColor $WarningColor
    Write-Host "   1. Ensure you cloned the complete repository" -ForegroundColor Gray
    Write-Host "   2. Check that tools/ directory is present and complete" -ForegroundColor Gray
    Write-Host "   3. Try running: git lfs pull (if using Git LFS)" -ForegroundColor Gray
    Write-Host "   4. Contact instructor if problems persist" -ForegroundColor Gray
    exit 1
}