# ATmega128 Educational Framework - Code Quality Optimizer
# Analyzes and optimizes code across all projects

param(
    [Parameter(Mandatory=$false)]
    [switch]$AnalyzeOnly = $false,
    
    [Parameter(Mandatory=$false)]
    [switch]$Fix = $false,
    
    [Parameter(Mandatory=$false)]
    [switch]$Verbose = $false
)

# Analysis results storage
$global:analysisResults = @{
    TotalFiles = 0
    Issues = @()
    MemoryUsage = @()
    CodeMetrics = @()
    Recommendations = @()
}

function Write-Log {
    param([string]$Message, [string]$Level = "INFO")
    
    $color = switch ($Level) {
        "ERROR" { "Red" }
        "WARN"  { "Yellow" }
        "SUCCESS" { "Green" }
        "INFO"  { "Cyan" }
        default { "White" }
    }
    
    if ($Verbose -or $Level -ne "DEBUG") {
        Write-Host "[$Level] $Message" -ForegroundColor $color
    }
}

# Analyze C code for common issues
function Analyze-CFile {
    param([string]$FilePath)
    
    $content = Get-Content $FilePath -Raw
    $lines = Get-Content $FilePath
    $issues = @()
    
    Write-Log "Analyzing: $FilePath" "DEBUG"
    
    # Check for memory leaks (malloc without free)
    $mallocCount = ($content | Select-String -Pattern "malloc\s*\(" -AllMatches).Matches.Count
    $freeCount = ($content | Select-String -Pattern "free\s*\(" -AllMatches).Matches.Count
    
    if ($mallocCount -gt $freeCount) {
        $issues += @{
            Type = "Memory Leak"
            Severity = "HIGH"
            Description = "Found $mallocCount malloc() but only $freeCount free()"
            File = $FilePath
        }
    }
    
    # Check for buffer overflows (gets, strcpy without bounds)
    if ($content -match "gets\s*\(") {
        $issues += @{
            Type = "Buffer Overflow Risk"
            Severity = "HIGH"
            Description = "Use of unsafe gets() function"
            File = $FilePath
        }
    }
    
    # Check for uninitialized variables
    $uninitPattern = "^\s*(int|char|float|double|uint8_t|uint16_t|uint32_t)\s+\w+\s*;"
    for ($i = 0; $i -lt $lines.Count; $i++) {
        if ($lines[$i] -match $uninitPattern) {
            $varName = ($lines[$i] -replace ".*\s+(\w+)\s*;.*", '$1')
            
            # Check if variable is used before assignment in next 10 lines
            $used = $false
            $assigned = $false
            
            for ($j = $i + 1; $j -lt [Math]::Min($i + 10, $lines.Count); $j++) {
                if ($lines[$j] -match "$varName\s*=" -and !($lines[$j] -match "=.*$varName")) {
                    $assigned = $true
                    break
                }
                if ($lines[$j] -match "$varName" -and !($lines[$j] -match "$varName\s*=")) {
                    $used = $true
                }
            }
            
            if ($used -and !$assigned) {
                $issues += @{
                    Type = "Uninitialized Variable"
                    Severity = "MEDIUM"
                    Description = "Variable '$varName' may be used before initialization"
                    File = $FilePath
                    Line = $i + 1
                }
            }
        }
    }
    
    # Check for hardcoded magic numbers
    $magicNumbers = ($content | Select-String -Pattern "\b\d{3,}\b" -AllMatches).Matches
    foreach ($match in $magicNumbers) {
        $number = $match.Value
        if ($number -notin @("255", "256", "1024", "2048", "4096", "8192", "16000000")) {
            $issues += @{
                Type = "Magic Number"
                Severity = "LOW"
                Description = "Hardcoded number '$number' should be a named constant"
                File = $FilePath
            }
        }
    }
    
    # Check for proper function documentation
    $functions = ($content | Select-String -Pattern "^\s*\w+\s+\w+\s*\([^)]*\)\s*\{" -AllMatches).Matches
    foreach ($function in $functions) {
        $lineIndex = ($content.Substring(0, $function.Index) | Measure-Object -Line).Lines
        
        # Check if there's a comment block before the function
        $hasDoc = $false
        for ($i = $lineIndex - 5; $i -lt $lineIndex; $i++) {
            if ($i -ge 0 -and $i -lt $lines.Count) {
                if ($lines[$i] -match "/\*.*\*/" -or $lines[$i] -match "//.*") {
                    $hasDoc = $true
                    break
                }
            }
        }
        
        if (!$hasDoc -and $function.Value -notmatch "main\s*\(") {
            $issues += @{
                Type = "Missing Documentation"
                Severity = "LOW"
                Description = "Function lacks documentation comment"
                File = $FilePath
                Line = $lineIndex
            }
        }
    }
    
    # Estimate memory usage
    $memoryUsage = @{
        File = $FilePath
        GlobalVariables = ($content | Select-String -Pattern "^\s*(int|char|float|double|uint8_t|uint16_t|uint32_t)\s+\w+" -AllMatches).Matches.Count
        LocalVariables = ($content | Select-String -Pattern "{\s*(int|char|float|double|uint8_t|uint16_t|uint32_t)\s+\w+" -AllMatches).Matches.Count
        Arrays = ($content | Select-String -Pattern "\w+\s*\[\s*\d+\s*\]" -AllMatches).Matches.Count
        EstimatedBytes = 0
    }
    
    # Calculate estimated memory usage
    $intVars = ($content | Select-String -Pattern "(int|uint16_t)" -AllMatches).Matches.Count
    $charVars = ($content | Select-String -Pattern "(char|uint8_t)" -AllMatches).Matches.Count
    $floatVars = ($content | Select-String -Pattern "float" -AllMatches).Matches.Count
    
    $memoryUsage.EstimatedBytes = ($intVars * 2) + ($charVars * 1) + ($floatVars * 4)
    
    return @{
        Issues = $issues
        Memory = $memoryUsage
        LinesOfCode = $lines.Count
        Functions = $functions.Count
    }
}

# Generate optimization recommendations
function Generate-Recommendations {
    param([array]$AllIssues)
    
    $recommendations = @()
    
    # Group issues by type
    $issueGroups = $AllIssues | Group-Object -Property Type
    
    foreach ($group in $issueGroups) {
        switch ($group.Name) {
            "Memory Leak" {
                $recommendations += @{
                    Priority = "HIGH"
                    Category = "Memory Management"
                    Recommendation = "Implement proper memory management with matching malloc/free pairs"
                    AffectedFiles = $group.Group.Count
                }
            }
            "Buffer Overflow Risk" {
                $recommendations += @{
                    Priority = "HIGH"
                    Category = "Security"
                    Recommendation = "Replace unsafe functions (gets) with safe alternatives (fgets)"
                    AffectedFiles = $group.Group.Count
                }
            }
            "Magic Number" {
                $recommendations += @{
                    Priority = "MEDIUM"
                    Category = "Code Quality"
                    Recommendation = "Define constants for magic numbers to improve readability"
                    AffectedFiles = $group.Group.Count
                }
            }
            "Missing Documentation" {
                $recommendations += @{
                    Priority = "LOW"
                    Category = "Documentation"
                    Recommendation = "Add function documentation to improve code maintainability"
                    AffectedFiles = $group.Group.Count
                }
            }
        }
    }
    
    return $recommendations
}

# Fix common issues automatically
function Fix-CommonIssues {
    param([string]$FilePath, [array]$Issues)
    
    if (!$Fix) {
        Write-Log "Fix mode disabled. Use -Fix to apply automatic fixes." "INFO"
        return
    }
    
    $content = Get-Content $FilePath -Raw
    $modified = $false
    
    foreach ($issue in $Issues) {
        switch ($issue.Type) {
            "Magic Number" {
                # This would require more sophisticated analysis
                # For now, just flag for manual review
                Write-Log "Magic number in $FilePath requires manual review" "WARN"
            }
            
            "Buffer Overflow Risk" {
                if ($content -match "gets\s*\(") {
                    Write-Log "Found unsafe gets() in $FilePath - requires manual replacement" "WARN"
                }
            }
        }
    }
    
    if ($modified) {
        Set-Content -Path $FilePath -Value $content
        Write-Log "Applied automatic fixes to $FilePath" "SUCCESS"
    }
}

# Main analysis function
function Start-Analysis {
    Write-Log "Starting comprehensive code analysis..." "INFO"
    
    $projectsRoot = Join-Path $PSScriptRoot "projects"
    $mainRoot = Join-Path $PSScriptRoot "Main"
    $sharedRoot = Join-Path $PSScriptRoot "shared_libs"
    
    $allFiles = @()
    $allFiles += Get-ChildItem -Path $projectsRoot -Filter "*.c" -Recurse
    $allFiles += Get-ChildItem -Path $mainRoot -Filter "*.c" -Recurse
    $allFiles += Get-ChildItem -Path $sharedRoot -Filter "*.c" -Recurse
    
    $global:analysisResults.TotalFiles = $allFiles.Count
    Write-Log "Found $($allFiles.Count) C files to analyze" "INFO"
    
    $totalIssues = @()
    $totalMemory = @()
    
    foreach ($file in $allFiles) {
        $result = Analyze-CFile $file.FullName
        
        $totalIssues += $result.Issues
        $totalMemory += $result.Memory
        
        $global:analysisResults.CodeMetrics += @{
            File = $file.FullName
            LinesOfCode = $result.LinesOfCode
            Functions = $result.Functions
            Issues = $result.Issues.Count
        }
        
        # Apply fixes if requested
        if ($Fix -and $result.Issues.Count -gt 0) {
            Fix-CommonIssues $file.FullName $result.Issues
        }
    }
    
    $global:analysisResults.Issues = $totalIssues
    $global:analysisResults.MemoryUsage = $totalMemory
    $global:analysisResults.Recommendations = Generate-Recommendations $totalIssues
    
    # Generate report
    Generate-Report
}

# Generate comprehensive report
function Generate-Report {
    Write-Log "Generating analysis report..." "INFO"
    
    $report = @"
# ATmega128 Educational Framework - Code Quality Report
Generated: $(Get-Date)

## Summary
- Total Files Analyzed: $($global:analysisResults.TotalFiles)
- Total Issues Found: $($global:analysisResults.Issues.Count)
- High Priority Issues: $(($global:analysisResults.Issues | Where-Object { $_.Severity -eq "HIGH" }).Count)
- Medium Priority Issues: $(($global:analysisResults.Issues | Where-Object { $_.Severity -eq "MEDIUM" }).Count)
- Low Priority Issues: $(($global:analysisResults.Issues | Where-Object { $_.Severity -eq "LOW" }).Count)

## Issue Breakdown by Type
"@

    $issueGroups = $global:analysisResults.Issues | Group-Object -Property Type
    foreach ($group in $issueGroups) {
        $report += "`n- $($group.Name): $($group.Count) occurrences"
    }
    
    $report += @"

## Memory Usage Analysis
- Average estimated memory per file: $([math]::Round(($global:analysisResults.MemoryUsage | Measure-Object -Property EstimatedBytes -Average).Average, 2)) bytes
- Total estimated memory usage: $(($global:analysisResults.MemoryUsage | Measure-Object -Property EstimatedBytes -Sum).Sum) bytes
- Files with highest memory usage:
"@

    $topMemoryFiles = $global:analysisResults.MemoryUsage | Sort-Object -Property EstimatedBytes -Descending | Select-Object -First 5
    foreach ($file in $topMemoryFiles) {
        $report += "`n  - $(Split-Path $file.File -Leaf): $($file.EstimatedBytes) bytes"
    }
    
    $report += @"

## Code Metrics
- Average lines of code per file: $([math]::Round(($global:analysisResults.CodeMetrics | Measure-Object -Property LinesOfCode -Average).Average, 2))
- Total functions: $(($global:analysisResults.CodeMetrics | Measure-Object -Property Functions -Sum).Sum)
- Files with most issues:
"@

    $problemFiles = $global:analysisResults.CodeMetrics | Sort-Object -Property Issues -Descending | Select-Object -First 5
    foreach ($file in $problemFiles) {
        $report += "`n  - $(Split-Path $file.File -Leaf): $($file.Issues) issues"
    }
    
    $report += @"

## Recommendations
"@

    foreach ($rec in $global:analysisResults.Recommendations) {
        $report += @"

### $($rec.Category) [$($rec.Priority) Priority]
$($rec.Recommendation)
Affected files: $($rec.AffectedFiles)
"@
    }
    
    # Save report
    $reportPath = Join-Path $PSScriptRoot "code_quality_report.md"
    Set-Content -Path $reportPath -Value $report
    
    Write-Log "Report saved to: $reportPath" "SUCCESS"
    
    # Display summary
    Write-Log "Analysis Complete!" "SUCCESS"
    Write-Log "Files analyzed: $($global:analysisResults.TotalFiles)" "INFO"
    Write-Log "Issues found: $($global:analysisResults.Issues.Count)" "INFO"
    Write-Log "High priority: $(($global:analysisResults.Issues | Where-Object { $_.Severity -eq "HIGH" }).Count)" "ERROR"
    Write-Log "Medium priority: $(($global:analysisResults.Issues | Where-Object { $_.Severity -eq "MEDIUM" }).Count)" "WARN"
    Write-Log "Low priority: $(($global:analysisResults.Issues | Where-Object { $_.Severity -eq "LOW" }).Count)" "INFO"
}

# Main execution
if ($AnalyzeOnly -and $Fix) {
    Write-Log "Cannot use both -AnalyzeOnly and -Fix. Choose one." "ERROR"
    exit 1
}

Start-Analysis