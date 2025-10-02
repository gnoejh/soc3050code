# ATmega128 Educational Framework - Enhanced Build System
# Version 2.0 - Optimized for Educational Use

param(
    [Parameter(Mandatory=$false)]
    [string]$ProjectDir = "",
    
    [Parameter(Mandatory=$false)]
    [string]$BuildMode = "Debug",
    
    [Parameter(Mandatory=$false)]
    [switch]$CleanBuild = $false,
    
    [Parameter(Mandatory=$false)]
    [switch]$Program = $false,
    
    [Parameter(Mandatory=$false)]
    [switch]$TestAll = $false,
    
    [Parameter(Mandatory=$false)]
    [switch]$VerboseOutput = $false,
    
    [Parameter(Mandatory=$false)]
    [string]$LogLevel = "INFO"
)

# Enhanced logging system
function Write-Log {
    param(
        [string]$Message,
        [string]$Level = "INFO"
    )
    
    $timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    $color = switch ($Level) {
        "ERROR" { "Red" }
        "WARN"  { "Yellow" }
        "SUCCESS" { "Green" }
        "INFO"  { "White" }
        default { "Gray" }
    }
    
    if ($VerboseOutput -or $Level -ne "DEBUG") {
        Write-Host "[$timestamp] [$Level] $Message" -ForegroundColor $color
    }
    
    # Also log to file for debugging
    $logFile = Join-Path $PSScriptRoot "build.log"
    "[$timestamp] [$Level] $Message" | Out-File -FilePath $logFile -Append
}

# Enhanced project validation
function Test-ProjectStructure {
    param([string]$ProjectPath)
    
    Write-Log "Validating project structure: $ProjectPath" "INFO"
    
    $required = @("Main.c", "*.cproj", "config.h")
    $missing = @()
    
    foreach ($file in $required) {
        if (!(Get-ChildItem -Path $ProjectPath -Filter $file -ErrorAction SilentlyContinue)) {
            $missing += $file
        }
    }
    
    if ($missing.Count -gt 0) {
        Write-Log "Missing required files: $($missing -join ', ')" "ERROR"
        return $false
    }
    
    Write-Log "Project structure validation passed" "SUCCESS"
    return $true
}

# Enhanced build function with better error handling
function Build-Project {
    param(
        [string]$ProjectPath,
        [string]$Mode = "Debug"
    )
    
    Write-Log "Building project: $ProjectPath (Mode: $Mode)" "INFO"
    
    # Validate project structure first
    if (!(Test-ProjectStructure $ProjectPath)) {
        Write-Log "Project validation failed" "ERROR"
        return $false
    }
    
    # Find .cproj file
    $cprojFile = Get-ChildItem -Path $ProjectPath -Filter "*.cproj" | Select-Object -First 1
    if (!$cprojFile) {
        Write-Log "No .cproj file found in $ProjectPath" "ERROR"
        return $false
    }
    
    Write-Log "Using project file: $($cprojFile.Name)" "INFO"
    
    # Clean build if requested
    if ($CleanBuild) {
        Write-Log "Performing clean build..." "INFO"
        $cleanResult = & "C:\Program Files (x86)\Atmel\Studio\7.0\AtmelStudio.exe" /build "$($cprojFile.FullName)" /clean
        if ($LASTEXITCODE -ne 0) {
            Write-Log "Clean failed with exit code: $LASTEXITCODE" "WARN"
        }
    }
    
    # Build the project
    try {
        Write-Log "Starting build process..." "INFO"
        
        # Use atprogram for building (more reliable than GUI)
        $buildCmd = @(
            "C:\Program Files (x86)\Atmel\Studio\7.0\atbackend\atprogram.exe"
            "program"
            "-t"
            "simulator"
            "-d"
            "atmega128"
        )
        
        # Alternative: Use msbuild directly
        $msbuildPath = "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\MSBuild\Current\Bin\MSBuild.exe"
        if (Test-Path $msbuildPath) {
            $buildResult = & $msbuildPath $cprojFile.FullName /p:Configuration=$Mode /p:Platform="AVR" /verbosity:minimal
            
            if ($LASTEXITCODE -eq 0) {
                Write-Log "Build completed successfully" "SUCCESS"
                return $true
            } else {
                Write-Log "Build failed with exit code: $LASTEXITCODE" "ERROR"
                return $false
            }
        } else {
            Write-Log "MSBuild not found, falling back to Atmel Studio" "WARN"
            
            # Use AtmelStudio command line
            $studioResult = & "C:\Program Files (x86)\Atmel\Studio\7.0\AtmelStudio.exe" /build "$($cprojFile.FullName)"
            
            if ($LASTEXITCODE -eq 0) {
                Write-Log "Build completed successfully" "SUCCESS"
                return $true
            } else {
                Write-Log "Build failed with exit code: $LASTEXITCODE" "ERROR"
                return $false
            }
        }
    }
    catch {
        Write-Log "Build error: $($_.Exception.Message)" "ERROR"
        return $false
    }
}

# Enhanced project testing
function Test-ProjectBuild {
    param([string]$ProjectPath)
    
    Write-Log "Testing project build: $ProjectPath" "INFO"
    
    # Check for build outputs
    $outputDir = Join-Path $ProjectPath "$BuildMode"
    if (!(Test-Path $outputDir)) {
        Write-Log "Build output directory not found: $outputDir" "ERROR"
        return $false
    }
    
    # Check for .hex file
    $hexFile = Get-ChildItem -Path $outputDir -Filter "*.hex" | Select-Object -First 1
    if (!$hexFile) {
        Write-Log "No .hex file found in build output" "ERROR"
        return $false
    }
    
    # Check file size (should be reasonable)
    $fileSize = (Get-Item $hexFile.FullName).Length
    if ($fileSize -lt 100) {
        Write-Log "Generated .hex file is suspiciously small: $fileSize bytes" "WARN"
    } elseif ($fileSize -gt 1048576) {
        Write-Log "Generated .hex file is suspiciously large: $fileSize bytes" "WARN"
    } else {
        Write-Log ".hex file size looks good: $fileSize bytes" "SUCCESS"
    }
    
    return $true
}

# Main execution logic
function Main {
    Write-Log "ATmega128 Educational Framework - Enhanced Build System v2.0" "INFO"
    Write-Log "Build Mode: $BuildMode, Clean: $CleanBuild, Program: $Program, TestAll: $TestAll" "INFO"
    
    # Initialize counters
    $successCount = 0
    $failCount = 0
    $totalCount = 0
    
    if ($TestAll) {
        Write-Log "Testing all projects..." "INFO"
        
        # Get all project directories
        $projectsRoot = Join-Path $PSScriptRoot "projects"
        $projects = Get-ChildItem -Path $projectsRoot -Directory | Sort-Object Name
        
        foreach ($project in $projects) {
            $totalCount++
            Write-Log "Processing project: $($project.Name)" "INFO"
            
            if (Build-Project $project.FullName $BuildMode) {
                if (Test-ProjectBuild $project.FullName) {
                    $successCount++
                    Write-Log "Project $($project.Name) - SUCCESS" "SUCCESS"
                } else {
                    $failCount++
                    Write-Log "Project $($project.Name) - FAILED (Test)" "ERROR"
                }
            } else {
                $failCount++
                Write-Log "Project $($project.Name) - FAILED (Build)" "ERROR"
            }
            
            Write-Log "---" "INFO"
        }
        
        # Summary
        Write-Log "Build Summary:" "INFO"
        Write-Log "Total Projects: $totalCount" "INFO"
        Write-Log "Successful: $successCount" "SUCCESS"
        Write-Log "Failed: $failCount" $(if ($failCount -gt 0) { "ERROR" } else { "SUCCESS" })
        Write-Log "Success Rate: $([math]::Round(($successCount / $totalCount) * 100, 1))%" "INFO"
        
    } elseif ($ProjectDir -ne "") {
        Write-Log "Building single project: $ProjectDir" "INFO"
        
        if (!(Test-Path $ProjectDir)) {
            Write-Log "Project directory not found: $ProjectDir" "ERROR"
            exit 1
        }
        
        if (Build-Project $ProjectDir $BuildMode) {
            if (Test-ProjectBuild $ProjectDir) {
                Write-Log "Project build and test completed successfully" "SUCCESS"
                exit 0
            } else {
                Write-Log "Project build succeeded but test failed" "ERROR"
                exit 1
            }
        } else {
            Write-Log "Project build failed" "ERROR"
            exit 1
        }
    } else {
        Write-Log "No project specified. Use -ProjectDir or -TestAll" "ERROR"
        Write-Log "Usage examples:" "INFO"
        Write-Log "  .\enhanced_build_system.ps1 -ProjectDir 'projects\Port'" "INFO"
        Write-Log "  .\enhanced_build_system.ps1 -TestAll" "INFO"
        Write-Log "  .\enhanced_build_system.ps1 -ProjectDir 'projects\Port' -CleanBuild -Program" "INFO"
        exit 1
    }
}

# Performance monitoring
$stopwatch = [System.Diagnostics.Stopwatch]::StartNew()

try {
    Main
}
finally {
    $stopwatch.Stop()
    Write-Log "Total execution time: $($stopwatch.Elapsed.TotalSeconds.ToString('F2')) seconds" "INFO"
}