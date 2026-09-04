# ATmega128 Educational Framework - System Debugger and Tester
# Comprehensive testing and debugging tool for educational projects

param(
    [Parameter(Mandatory=$false)]
    [string]$ProjectPath = "",
    
    [Parameter(Mandatory=$false)]
    [switch]$FullTest = $false,
    
    [Parameter(Mandatory=$false)]
    [switch]$MemoryTest = $false,
    
    [Parameter(Mandatory=$false)]
    [switch]$HardwareTest = $false,
    
    [Parameter(Mandatory=$false)]
    [switch]$Interactive = $false
)

function Write-Log {
    param([string]$Message, [string]$Level = "INFO")
    
    $color = switch ($Level) {
        "ERROR" { "Red" }
        "WARN"  { "Yellow" }
        "SUCCESS" { "Green" }
        "INFO"  { "Cyan" }
        "DEBUG" { "Gray" }
        default { "White" }
    }
    
    Write-Host "[$Level] $Message" -ForegroundColor $color
}

# Test results storage
$global:testResults = @{
    TotalTests = 0
    PassedTests = 0
    FailedTests = 0
    WarningTests = 0
    TestDetails = @()
    StartTime = Get-Date
}

function Add-TestResult {
    param(
        [string]$TestName,
        [string]$Status,  # PASS, FAIL, WARN
        [string]$Details = "",
        [string]$Recommendation = ""
    )
    
    $global:testResults.TotalTests++
    
    switch ($Status) {
        "PASS" { $global:testResults.PassedTests++ }
        "FAIL" { $global:testResults.FailedTests++ }
        "WARN" { $global:testResults.WarningTests++ }
    }
    
    $global:testResults.TestDetails += @{
        Name = $TestName
        Status = $Status
        Details = $Details
        Recommendation = $Recommendation
        Timestamp = Get-Date
    }
    
    $color = switch ($Status) {
        "PASS" { "Green" }
        "FAIL" { "Red" }
        "WARN" { "Yellow" }
        default { "White" }
    }
    
    Write-Host "[TEST] $TestName : $Status" -ForegroundColor $color
    if ($Details) {
        Write-Host "       $Details" -ForegroundColor Gray
    }
}

function Test-ProjectStructure {
    param([string]$ProjectPath)
    
    Write-Log "Testing project structure..." "INFO"
    
    # Test for required files
    $requiredFiles = @("Main.c", "config.h")
    foreach ($file in $requiredFiles) {
        $filePath = Join-Path $ProjectPath $file
        if (Test-Path $filePath) {
            Add-TestResult "Required File: $file" "PASS" "File exists and accessible"
        } else {
            Add-TestResult "Required File: $file" "FAIL" "Missing required file" "Create $file with proper template"
        }
    }
    
    # Test for project file
    $cprojFiles = Get-ChildItem -Path $ProjectPath -Filter "*.cproj"
    if ($cprojFiles.Count -eq 1) {
        Add-TestResult "Project File" "PASS" "Single .cproj file found: $($cprojFiles[0].Name)"
    } elseif ($cprojFiles.Count -eq 0) {
        Add-TestResult "Project File" "FAIL" "No .cproj file found" "Create project file for Microchip Studio"
    } else {
        Add-TestResult "Project File" "WARN" "Multiple .cproj files found" "Remove duplicate project files"
    }
    
    # Test directory structure
    $buildDirs = @("Debug", "Release")
    foreach ($dir in $buildDirs) {
        $dirPath = Join-Path $ProjectPath $dir
        if (Test-Path $dirPath) {
            Add-TestResult "Build Directory: $dir" "PASS" "Build output directory exists"
        } else {
            Add-TestResult "Build Directory: $dir" "WARN" "Build directory not found" "Build project to create output directories"
        }
    }
}

function Test-CodeQuality {
    param([string]$ProjectPath)
    
    Write-Log "Testing code quality..." "INFO"
    
    $mainFile = Join-Path $ProjectPath "Main.c"
    if (Test-Path $mainFile) {
        $content = Get-Content $mainFile -Raw
        $lines = Get-Content $mainFile
        
        # Test for main function
        if ($content -match "int\s+main\s*\(") {
            Add-TestResult "Main Function" "PASS" "main() function found"
        } else {
            Add-TestResult "Main Function" "FAIL" "No main() function found" "Add proper main() function"
        }
        
        # Test for includes
        if ($content -match '#include\s+"config\.h"') {
            Add-TestResult "Config Include" "PASS" "config.h is included"
        } else {
            Add-TestResult "Config Include" "FAIL" "config.h not included" "Add #include \"config.h\""
        }
        
        # Test for educational comments
        $commentCount = ($content | Select-String -Pattern "/\*.*\*/" -AllMatches).Matches.Count
        $commentCount += ($content | Select-String -Pattern "//.*" -AllMatches).Matches.Count
        
        if ($commentCount -gt 10) {
            Add-TestResult "Code Documentation" "PASS" "$commentCount comments found"
        } elseif ($commentCount -gt 5) {
            Add-TestResult "Code Documentation" "WARN" "Limited documentation ($commentCount comments)" "Add more educational comments"
        } else {
            Add-TestResult "Code Documentation" "FAIL" "Poor documentation ($commentCount comments)" "Add comprehensive educational comments"
        }
        
        # Test for infinite loop
        if ($content -match "while\s*\(\s*1\s*\)") {
            Add-TestResult "Main Loop" "PASS" "Main infinite loop found"
        } else {
            Add-TestResult "Main Loop" "WARN" "No main infinite loop detected" "Consider adding main event loop"
        }
        
        # Test for UART usage
        if ($content -match "puts_USART1|USART1|uart") {
            Add-TestResult "UART Communication" "PASS" "UART functions detected"
        } else {
            Add-TestResult "UART Communication" "WARN" "No UART communication found" "Consider adding UART for debugging"
        }
        
        # Test for delay usage
        if ($content -match "_delay_ms|_delay_us") {
            Add-TestResult "Timing Control" "PASS" "Delay functions used"
        } else {
            Add-TestResult "Timing Control" "WARN" "No delay functions found" "Add timing control if needed"
        }
        
        # Test for proper initialization
        if ($content -match "init.*\(\)|.*_init\(\)") {
            Add-TestResult "System Initialization" "PASS" "Initialization functions found"
        } else {
            Add-TestResult "System Initialization" "FAIL" "No initialization detected" "Add proper system initialization"
        }
    }
}

function Test-HardwareConfiguration {
    param([string]$ProjectPath)
    
    Write-Log "Testing hardware configuration..." "INFO"
    
    $configFile = Join-Path $ProjectPath "config.h"
    if (Test-Path $configFile) {
        $content = Get-Content $configFile -Raw
        
        # Test for F_CPU definition
        if ($content -match "#define\s+F_CPU") {
            $fcpuMatch = [regex]::Match($content, "#define\s+F_CPU\s+(\d+)UL")
            if ($fcpuMatch.Success) {
                $fcpuValue = [long]$fcpuMatch.Groups[1].Value
                if ($fcpuValue -eq 16000000) {
                    Add-TestResult "CPU Frequency" "PASS" "F_CPU correctly set to 16MHz"
                } elseif ($fcpuValue -eq 16000000) {
                    Add-TestResult "CPU Frequency" "WARN" "F_CPU set to 16MHz" "Verify crystal frequency matches"
                } else {
                    Add-TestResult "CPU Frequency" "WARN" "F_CPU set to $fcpuValue Hz" "Verify frequency matches hardware"
                }
            }
        } else {
            Add-TestResult "CPU Frequency" "FAIL" "F_CPU not defined" "Add #define F_CPU 16000000UL"
        }
        
        # Test for required includes
        $requiredIncludes = @("avr/io.h", "util/delay.h")
        foreach ($include in $requiredIncludes) {
            if ($content -match "#include\s*<$([regex]::Escape($include))>") {
                Add-TestResult "Include: $include" "PASS" "Required header included"
            } else {
                Add-TestResult "Include: $include" "FAIL" "Missing required header" "Add #include <$include>"
            }
        }
        
        # Test for shared library includes
        if ($content -match '#include\s+".*\.h"') {
            Add-TestResult "Shared Libraries" "PASS" "Shared library headers included"
        } else {
            Add-TestResult "Shared Libraries" "WARN" "No shared library headers" "Consider using educational libraries"
        }
    }
}

function Test-BuildConfiguration {
    param([string]$ProjectPath)
    
    Write-Log "Testing build configuration..." "INFO"
    
    $cprojFile = Get-ChildItem -Path $ProjectPath -Filter "*.cproj" | Select-Object -First 1
    if ($cprojFile) {
        $content = Get-Content $cprojFile.FullName -Raw
        
        # Test for ATmega128 target
        if ($content -match "ATmega128|atmega128") {
            Add-TestResult "Target Device" "PASS" "ATmega128 target configured"
        } else {
            Add-TestResult "Target Device" "FAIL" "Wrong or missing target device" "Set target to ATmega128"
        }
        
        # Test for include paths
        if ($content -match "shared_libs|\.\.\\\.\.\\shared_libs") {
            Add-TestResult "Include Paths" "PASS" "Shared libraries path configured"
        } else {
            Add-TestResult "Include Paths" "WARN" "Shared libraries path not found" "Add shared_libs to include paths"
        }
        
        # Test for optimization settings
        if ($content -match "Optimize.*size|Os") {
            Add-TestResult "Optimization" "PASS" "Size optimization enabled"
        } elseif ($content -match "Optimize.*debugging|Og") {
            Add-TestResult "Optimization" "PASS" "Debug optimization enabled"
        } else {
            Add-TestResult "Optimization" "WARN" "Optimization not configured" "Set appropriate optimization level"
        }
        
        # Test for warnings
        if ($content -match "AllWarnings.*True") {
            Add-TestResult "Compiler Warnings" "PASS" "All warnings enabled"
        } else {
            Add-TestResult "Compiler Warnings" "WARN" "Warnings not fully enabled" "Enable all compiler warnings"
        }
    }
}

function Test-MemoryUsage {
    param([string]$ProjectPath)
    
    Write-Log "Testing memory usage..." "INFO"
    
    # Look for build output files
    $buildDirs = @("Debug", "Release")
    $memoryTested = $false
    
    foreach ($buildDir in $buildDirs) {
        $buildPath = Join-Path $ProjectPath $buildDir
        $mapFile = Get-ChildItem -Path $buildPath -Filter "*.map" -ErrorAction SilentlyContinue | Select-Object -First 1
        
        if ($mapFile) {
            $memoryTested = $true
            $content = Get-Content $mapFile.FullName -Raw
            
            # Extract memory usage information
            if ($content -match "\.text\s+0x\w+\s+(\w+)") {
                $textSize = [Convert]::ToInt32($matches[1], 16)
                $flashUsage = ($textSize / 131072.0) * 100  # ATmega128 has 128KB flash
                
                if ($flashUsage -lt 50) {
                    Add-TestResult "Flash Usage ($buildDir)" "PASS" "Flash usage: $($flashUsage.ToString('F1'))% ($textSize bytes)"
                } elseif ($flashUsage -lt 80) {
                    Add-TestResult "Flash Usage ($buildDir)" "WARN" "Flash usage: $($flashUsage.ToString('F1'))% ($textSize bytes)" "Consider code optimization"
                } else {
                    Add-TestResult "Flash Usage ($buildDir)" "FAIL" "Flash usage: $($flashUsage.ToString('F1'))% ($textSize bytes)" "Reduce code size"
                }
            }
            
            if ($content -match "\.data\s+0x\w+\s+(\w+)") {
                $dataSize = [Convert]::ToInt32($matches[1], 16)
                Add-TestResult "Data Section ($buildDir)" "PASS" "Data size: $dataSize bytes"
            }
            
            if ($content -match "\.bss\s+0x\w+\s+(\w+)") {
                $bssSize = [Convert]::ToInt32($matches[1], 16)
                $ramUsage = (($dataSize + $bssSize) / 4096.0) * 100  # ATmega128 has 4KB RAM
                
                if ($ramUsage -lt 50) {
                    Add-TestResult "RAM Usage ($buildDir)" "PASS" "RAM usage: $($ramUsage.ToString('F1'))% ($($dataSize + $bssSize) bytes)"
                } elseif ($ramUsage -lt 80) {
                    Add-TestResult "RAM Usage ($buildDir)" "WARN" "RAM usage: $($ramUsage.ToString('F1'))% ($($dataSize + $bssSize) bytes)" "Monitor RAM usage"
                } else {
                    Add-TestResult "RAM Usage ($buildDir)" "FAIL" "RAM usage: $($ramUsage.ToString('F1'))% ($($dataSize + $bssSize) bytes)" "Reduce RAM usage"
                }
            }
        }
    }
    
    if (-not $memoryTested) {
        Add-TestResult "Memory Analysis" "WARN" "No build output found for memory analysis" "Build project to generate memory reports"
    }
}

function Test-EducationalContent {
    param([string]$ProjectPath)
    
    Write-Log "Testing educational content..." "INFO"
    
    $readmeFile = Join-Path $ProjectPath "README.md"
    if (Test-Path $readmeFile) {
        $content = Get-Content $readmeFile -Raw
        
        # Test for educational sections
        $requiredSections = @("Learning Objectives", "Hardware", "Usage", "Troubleshooting")
        foreach ($section in $requiredSections) {
            if ($content -match $section) {
                Add-TestResult "README Section: $section" "PASS" "Educational section present"
            } else {
                Add-TestResult "README Section: $section" "WARN" "Missing educational section" "Add $section section to README"
            }
        }
        
        # Test for documentation quality
        $wordCount = ($content -split '\s+').Count
        if ($wordCount -gt 500) {
            Add-TestResult "Documentation Quality" "PASS" "Comprehensive documentation ($wordCount words)"
        } elseif ($wordCount -gt 200) {
            Add-TestResult "Documentation Quality" "WARN" "Basic documentation ($wordCount words)" "Expand documentation for better learning"
        } else {
            Add-TestResult "Documentation Quality" "FAIL" "Poor documentation ($wordCount words)" "Add comprehensive educational documentation"
        }
    } else {
        Add-TestResult "README Documentation" "FAIL" "No README.md found" "Create educational documentation"
    }
}

function Test-InteractiveMode {
    Write-Log "Starting interactive testing mode..." "INFO"
    
    while ($true) {
        Write-Host "`nATmega128 Educational Framework - Interactive Debugger" -ForegroundColor Cyan
        Write-Host "========================================================" -ForegroundColor Cyan
        Write-Host "1. Test specific project"
        Write-Host "2. Run code quality analysis"
        Write-Host "3. Check memory usage"
        Write-Host "4. Validate educational content"
        Write-Host "5. Generate test report"
        Write-Host "6. Exit"
        Write-Host ""
        
        $choice = Read-Host "Select option (1-6)"
        
        switch ($choice) {
            "1" {
                $projectPath = Read-Host "Enter project path"
                if (Test-Path $projectPath) {
                    Test-SingleProject $projectPath
                } else {
                    Write-Log "Project path not found: $projectPath" "ERROR"
                }
            }
            
            "2" {
                $projectPath = Read-Host "Enter project path"
                if (Test-Path $projectPath) {
                    Test-CodeQuality $projectPath
                } else {
                    Write-Log "Project path not found: $projectPath" "ERROR"
                }
            }
            
            "3" {
                $projectPath = Read-Host "Enter project path"
                if (Test-Path $projectPath) {
                    Test-MemoryUsage $projectPath
                } else {
                    Write-Log "Project path not found: $projectPath" "ERROR"
                }
            }
            
            "4" {
                $projectPath = Read-Host "Enter project path"
                if (Test-Path $projectPath) {
                    Test-EducationalContent $projectPath
                } else {
                    Write-Log "Project path not found: $projectPath" "ERROR"
                }
            }
            
            "5" {
                Generate-TestReport
            }
            
            "6" {
                Write-Log "Exiting interactive mode" "INFO"
                return
            }
            
            default {
                Write-Log "Invalid selection. Please choose 1-6." "WARN"
            }
        }
    }
}

function Test-SingleProject {
    param([string]$ProjectPath)
    
    Write-Log "Testing project: $ProjectPath" "INFO"
    
    # Reset test results
    $global:testResults = @{
        TotalTests = 0
        PassedTests = 0
        FailedTests = 0
        WarningTests = 0
        TestDetails = @()
        StartTime = Get-Date
    }
    
    Test-ProjectStructure $ProjectPath
    Test-CodeQuality $ProjectPath
    Test-HardwareConfiguration $ProjectPath
    Test-BuildConfiguration $ProjectPath
    
    if ($MemoryTest) {
        Test-MemoryUsage $ProjectPath
    }
    
    Test-EducationalContent $ProjectPath
    
    Generate-TestSummary
}

function Test-AllProjects {
    Write-Log "Testing all projects..." "INFO"
    
    $projectsRoot = Join-Path $PSScriptRoot "projects"
    $projects = Get-ChildItem -Path $projectsRoot -Directory | Sort-Object Name
    
    foreach ($project in $projects) {
        Write-Log "Testing project: $($project.Name)" "INFO"
        Test-SingleProject $project.FullName
        Write-Log "---" "INFO"
    }
}

function Generate-TestSummary {
    $duration = (Get-Date) - $global:testResults.StartTime
    
    Write-Host "`n" -NoNewline
    Write-Host "TEST SUMMARY" -ForegroundColor White -BackgroundColor Blue
    Write-Host "============" -ForegroundColor White -BackgroundColor Blue
    Write-Host "Total Tests: $($global:testResults.TotalTests)" -ForegroundColor White
    Write-Host "Passed: $($global:testResults.PassedTests)" -ForegroundColor Green
    Write-Host "Failed: $($global:testResults.FailedTests)" -ForegroundColor Red
    Write-Host "Warnings: $($global:testResults.WarningTests)" -ForegroundColor Yellow
    Write-Host "Duration: $($duration.TotalSeconds.ToString('F2')) seconds" -ForegroundColor White
    
    $successRate = if ($global:testResults.TotalTests -gt 0) {
        ($global:testResults.PassedTests / $global:testResults.TotalTests) * 100
    } else { 0 }
    
    Write-Host "Success Rate: $($successRate.ToString('F1'))%" -ForegroundColor $(if ($successRate -gt 80) { "Green" } elseif ($successRate -gt 60) { "Yellow" } else { "Red" })
}

function Generate-TestReport {
    $reportPath = Join-Path $PSScriptRoot "test_report.html"
    
    $html = @"
<!DOCTYPE html>
<html>
<head>
    <title>ATmega128 Educational Framework - Test Report</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .header { background-color: #2E86AB; color: white; padding: 20px; border-radius: 5px; }
        .summary { background-color: #f0f0f0; padding: 15px; margin: 20px 0; border-radius: 5px; }
        .test-result { margin: 10px 0; padding: 10px; border-radius: 3px; }
        .pass { background-color: #d4edda; border-left: 4px solid #28a745; }
        .fail { background-color: #f8d7da; border-left: 4px solid #dc3545; }
        .warn { background-color: #fff3cd; border-left: 4px solid #ffc107; }
        .details { font-style: italic; color: #666; }
        .recommendation { font-weight: bold; color: #007bff; }
    </style>
</head>
<body>
    <div class="header">
        <h1>ATmega128 Educational Framework - Test Report</h1>
        <p>Generated: $(Get-Date)</p>
    </div>
    
    <div class="summary">
        <h2>Test Summary</h2>
        <p><strong>Total Tests:</strong> $($global:testResults.TotalTests)</p>
        <p><strong>Passed:</strong> <span style="color: green;">$($global:testResults.PassedTests)</span></p>
        <p><strong>Failed:</strong> <span style="color: red;">$($global:testResults.FailedTests)</span></p>
        <p><strong>Warnings:</strong> <span style="color: orange;">$($global:testResults.WarningTests)</span></p>
    </div>
    
    <div class="test-details">
        <h2>Test Details</h2>
"@

    foreach ($test in $global:testResults.TestDetails) {
        $cssClass = switch ($test.Status) {
            "PASS" { "pass" }
            "FAIL" { "fail" }
            "WARN" { "warn" }
        }
        
        $html += @"
        <div class="test-result $cssClass">
            <strong>$($test.Name)</strong> - $($test.Status)
            <div class="details">$($test.Details)</div>
"@
        
        if ($test.Recommendation) {
            $html += "<div class='recommendation'>Recommendation: $($test.Recommendation)</div>"
        }
        
        $html += "</div>`n"
    }
    
    $html += @"
    </div>
</body>
</html>
"@

    Set-Content -Path $reportPath -Value $html
    Write-Log "Test report generated: $reportPath" "SUCCESS"
    
    # Open report in default browser
    Start-Process $reportPath
}

# Main execution
if ($Interactive) {
    Test-InteractiveMode
} elseif ($ProjectPath -ne "") {
    Test-SingleProject $ProjectPath
} elseif ($FullTest) {
    Test-AllProjects
} else {
    Write-Log "ATmega128 Educational Framework - System Debugger and Tester" "INFO"
    Write-Log "Usage:" "INFO"
    Write-Log "  -ProjectPath <path>  : Test specific project" "INFO"
    Write-Log "  -FullTest           : Test all projects" "INFO"
    Write-Log "  -MemoryTest         : Include memory usage analysis" "INFO"
    Write-Log "  -Interactive        : Interactive testing mode" "INFO"
    Write-Log "" "INFO"
    Write-Log "Examples:" "INFO"
    Write-Log "  .\system_debugger.ps1 -ProjectPath 'projects\Port'" "INFO"
    Write-Log "  .\system_debugger.ps1 -FullTest -MemoryTest" "INFO"
    Write-Log "  .\system_debugger.ps1 -Interactive" "INFO"
}