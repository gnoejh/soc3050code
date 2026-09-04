#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Check SimulIDE environment and dependencies
    
.DESCRIPTION
    Verifies that all required components are installed and configured correctly
    for SimulIDE integration. Provides detailed diagnostics and suggestions.
    
.EXAMPLE
    .\check-simulide-environment.ps1
    Run complete environment check
#>

param(
    [switch]$Detailed = $false,
    [switch]$Fix = $false
)

$WorkspaceRoot = $PSScriptRoot

# Colors
$ErrorColor = "Red"
$SuccessColor = "Green"
$InfoColor = "Cyan"
$WarningColor = "Yellow"

Write-Host "`n========================================" -ForegroundColor $InfoColor
Write-Host "  SimulIDE Environment Checker" -ForegroundColor $InfoColor
Write-Host "========================================`n" -ForegroundColor $InfoColor

$Issues = @()
$Warnings = @()
$Passed = 0
$Failed = 0

# Helper function to test component
function Test-Component {
    param(
        [string]$Name,
        [scriptblock]$Test,
        [string]$SuccessMessage,
        [string]$FailMessage,
        [string]$FixCommand = "",
        [bool]$Critical = $true
    )
    
    Write-Host "🔍 Checking: " -NoNewline -ForegroundColor $InfoColor
    Write-Host "$Name... " -NoNewline -ForegroundColor White
    
    try {
        $Result = & $Test
        if ($Result) {
            Write-Host "✅ PASS" -ForegroundColor $SuccessColor
            if ($Detailed -and $SuccessMessage) {
                Write-Host "   $SuccessMessage" -ForegroundColor Gray
            }
            $script:Passed++
            return $true
        }
        else {
            if ($Critical) {
                Write-Host "❌ FAIL" -ForegroundColor $ErrorColor
                $script:Failed++
                $script:Issues += @{
                    Name    = $Name
                    Message = $FailMessage
                    Fix     = $FixCommand
                }
            }
            else {
                Write-Host "⚠️  WARNING" -ForegroundColor $WarningColor
                $script:Warnings += @{
                    Name    = $Name
                    Message = $FailMessage
                    Fix     = $FixCommand
                }
            }
            if ($FailMessage) {
                Write-Host "   $FailMessage" -ForegroundColor $(if ($Critical) { $ErrorColor } else { $WarningColor })
            }
            if ($FixCommand) {
                Write-Host "   Fix: $FixCommand" -ForegroundColor $InfoColor
            }
            return $false
        }
    }
    catch {
        Write-Host "❌ ERROR" -ForegroundColor $ErrorColor
        Write-Host "   Exception: $_" -ForegroundColor $ErrorColor
        $script:Failed++
        return $false
    }
}

Write-Host "=== Core Requirements ===" -ForegroundColor $InfoColor
Write-Host ""

# 1. PowerShell version
Test-Component -Name "PowerShell Version" -Test {
    $PSVersionTable.PSVersion.Major -ge 5
} -SuccessMessage "PowerShell $($PSVersionTable.PSVersion) detected" `
    -FailMessage "PowerShell 5.0 or higher required" `
    -FixCommand "Download PowerShell 7+ from https://aka.ms/powershell"

# 2. Workspace structure
Test-Component -Name "Workspace Structure" -Test {
    Test-Path (Join-Path $WorkspaceRoot "cli-simulide.ps1")
} -SuccessMessage "Workspace root: $WorkspaceRoot" `
    -FailMessage "cli-simulide.ps1 not found in workspace" `
    -FixCommand "Ensure you're running from workspace root"

# 3. Projects directory
Test-Component -Name "Projects Directory" -Test {
    Test-Path (Join-Path $WorkspaceRoot "projects")
} -SuccessMessage "Projects directory found" `
    -FailMessage "projects/ directory not found" `
    -FixCommand "Create projects/ directory or check workspace structure"

# 4. Circuit file
$CircuitFile = Join-Path $WorkspaceRoot "SimulIDE_1.1.0-SR1_Win64\Simulator110.simu"
Test-Component -Name "Circuit File" -Test {
    Test-Path $CircuitFile
} -SuccessMessage "Circuit file: $CircuitFile" `
    -FailMessage "Simulator110.simu not found in SimulIDE folder" `
    -FixCommand "Create Simulator110.simu in SimulIDE_1.1.0-SR1_Win64 folder"

Write-Host "`n=== SimulIDE Installation ===" -ForegroundColor $InfoColor
Write-Host ""

# 5. SimulIDE executable
$SimulIDEPaths = @(
    Join-Path $WorkspaceRoot "SimulIDE_1.1.0-SR1_Win64\simulide.exe"
    Join-Path $WorkspaceRoot "SimulIDE\simulide.exe"
    "C:\Program Files\SimulIDE\simulide.exe"
    "C:\Program Files (x86)\SimulIDE\simulide.exe"
)

$SimulIDEFound = $null
foreach ($Path in $SimulIDEPaths) {
    if (Test-Path $Path) {
        $SimulIDEFound = $Path
        break
    }
}

Test-Component -Name "SimulIDE Executable" -Test {
    $null -ne $SimulIDEFound
} -SuccessMessage "Found: $SimulIDEFound" `
    -FailMessage "SimulIDE not found in any standard location" `
    -FixCommand "Download from https://simulide.com/p/downloads/ and extract to workspace"

# 6. SimulIDE version
if ($SimulIDEFound) {
    Test-Component -Name "SimulIDE Version" -Test {
        $SimulIDEFound -match "1\.1\.0"
    } -SuccessMessage "SimulIDE 1.1.0-SR1 detected" `
        -FailMessage "Different SimulIDE version detected" `
        -FixCommand "Download SimulIDE 1.1.0-SR1 for compatibility" `
        -Critical $false
}

Write-Host "`n=== Build Tools ===" -ForegroundColor $InfoColor
Write-Host ""

# 7. AVR-GCC
$WorkspaceRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$portableAvrGcc = Join-Path $WorkspaceRoot "tools\avr-toolchain\bin\avr-gcc.exe"
$systemAvrGcc = "C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe"

Test-Component -Name "AVR-GCC Compiler" -Test {
    (Test-Path $portableAvrGcc) -or (Test-Path $systemAvrGcc)
} -SuccessMessage $(if (Test-Path $portableAvrGcc) { "Found: $portableAvrGcc (portable)" } else { "Found: $systemAvrGcc (system)" }) `
    -FailMessage "AVR-GCC not found (required for building)" `
    -FixCommand "Portable toolchain should be bundled. Check tools/avr-toolchain/ directory."

# 8. Build script
Test-Component -Name "Build Script" -Test {
    Test-Path (Join-Path $WorkspaceRoot "cli-build-project.ps1")
} -SuccessMessage "cli-build-project.ps1 found" `
    -FailMessage "Build script missing" `
    -FixCommand "Restore cli-build-project.ps1 from repository"

Write-Host "`n=== Project Files ===" -ForegroundColor $InfoColor
Write-Host ""

# 9. Sample project
$SampleProject = Join-Path $WorkspaceRoot "projects\Port_Basic"
Test-Component -Name "Sample Project" -Test {
    Test-Path $SampleProject
} -SuccessMessage "Port_Basic project found" `
    -FailMessage "Sample project not found" `
    -FixCommand "Ensure projects are properly extracted" `
    -Critical $false

# 10. Sample project Main.c
if (Test-Path $SampleProject) {
    Test-Component -Name "Sample Project Main.c" -Test {
        Test-Path (Join-Path $SampleProject "Main.c")
    } -SuccessMessage "Main.c exists in Port_Basic" `
        -FailMessage "Main.c not found in sample project" `
        -Critical $false
}

Write-Host "`n=== Configuration ===" -ForegroundColor $InfoColor
Write-Host ""

# 11. VS Code tasks
$TasksFile = Join-Path $WorkspaceRoot ".vscode\tasks.json"
Test-Component -Name "VS Code Tasks" -Test {
    Test-Path $TasksFile
} -SuccessMessage "tasks.json found" `
    -FailMessage "VS Code tasks.json not found" `
    -FixCommand "Restore .vscode/tasks.json for build integration" `
    -Critical $false

# 12. Temp directory writable
Test-Component -Name "Temp Directory Access" -Test {
    $TestFile = Join-Path $env:TEMP "simulide_test_$(Get-Random).tmp"
    try {
        "test" | Out-File $TestFile -ErrorAction Stop
        Remove-Item $TestFile -ErrorAction Stop
        return $true
    }
    catch {
        return $false
    }
} -SuccessMessage "Can write to: $env:TEMP" `
    -FailMessage "Cannot write to temp directory" `
    -FixCommand "Check temp folder permissions: $env:TEMP"

Write-Host "`n=== Optional Components ===" -ForegroundColor $InfoColor
Write-Host ""

# 13. Documentation
Test-Component -Name "SimulIDE Documentation" -Test {
    $newPath = Test-Path (Join-Path $WorkspaceRoot "docs\simulide\SIMULIDE_GUIDE.md")
    $oldPath = Test-Path (Join-Path $WorkspaceRoot "SIMULIDE_GUIDE.md")
    $newPath -or $oldPath
} -SuccessMessage "Complete documentation available" `
    -FailMessage "Documentation files missing" `
    -FixCommand "Restore documentation files" `
    -Critical $false

# 14. Lab files
$LabProjects = @("Port_Basic", "ADC_Basic", "Graphics_Display")
$LabsFound = 0
foreach ($Lab in $LabProjects) {
    $LabFile = Join-Path $WorkspaceRoot "projects\$Lab\Lab.c"
    if (Test-Path $LabFile) {
        $LabsFound++
    }
}

Test-Component -Name "Lab Exercise Files" -Test {
    $LabsFound -ge 3
} -SuccessMessage "$LabsFound lab projects found" `
    -FailMessage "Lab.c files missing" `
    -Critical $false

# Summary
Write-Host ""
Write-Host "========================================" -ForegroundColor $InfoColor
Write-Host "  Summary" -ForegroundColor $InfoColor
Write-Host "========================================" -ForegroundColor $InfoColor
Write-Host ""
Write-Host "✅ Passed: " -NoNewline -ForegroundColor $SuccessColor
Write-Host "$Passed" -ForegroundColor White
Write-Host "❌ Failed: " -NoNewline -ForegroundColor $ErrorColor
Write-Host "$Failed" -ForegroundColor White
Write-Host "⚠️  Warnings: " -NoNewline -ForegroundColor $WarningColor
Write-Host "$($Warnings.Count)" -ForegroundColor White
Write-Host ""

if ($Failed -eq 0 -and $Warnings.Count -eq 0) {
    Write-Host "🎉 All checks passed! Environment is ready for SimulIDE." -ForegroundColor $SuccessColor
    Write-Host ""
    Write-Host "Next steps:" -ForegroundColor $InfoColor
    Write-Host "  1. Open a project file in VS Code" -ForegroundColor White
    Write-Host "  2. Press Ctrl+Shift+B" -ForegroundColor White
    Write-Host "  3. Select 'Build and Simulate Current Project'" -ForegroundColor White
    Write-Host ""
}
elseif ($Failed -eq 0) {
    Write-Host "⚠️  Environment is functional with minor warnings." -ForegroundColor $WarningColor
    Write-Host ""
}
else {
    Write-Host "❌ Environment setup incomplete. Please address the issues above." -ForegroundColor $ErrorColor
    Write-Host ""
    
    if ($Issues.Count -gt 0) {
        Write-Host "Critical Issues to Fix:" -ForegroundColor $ErrorColor
        Write-Host ""
        $i = 1
        foreach ($Issue in $Issues) {
            Write-Host "  $i. " -NoNewline -ForegroundColor White
            Write-Host "$($Issue.Name)" -ForegroundColor $ErrorColor
            Write-Host "     $($Issue.Message)" -ForegroundColor White
            if ($Issue.Fix) {
                Write-Host "     → $($Issue.Fix)" -ForegroundColor $InfoColor
            }
            Write-Host ""
            $i++
        }
    }
}

if ($Warnings.Count -gt 0 -and $Detailed) {
    Write-Host "Warnings:" -ForegroundColor $WarningColor
    Write-Host ""
    foreach ($Warning in $Warnings) {
        Write-Host "  • " -NoNewline -ForegroundColor $WarningColor
        Write-Host "$($Warning.Name)" -ForegroundColor White
        Write-Host "    $($Warning.Message)" -ForegroundColor Gray
        if ($Warning.Fix) {
            Write-Host "    → $($Warning.Fix)" -ForegroundColor $InfoColor
        }
        Write-Host ""
    }
}

exit $(if ($Failed -eq 0) { 0 } else { 1 })
