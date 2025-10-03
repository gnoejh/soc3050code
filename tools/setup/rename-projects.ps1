# Script to remove number prefixes from project folders and update all references
# Example: 01_Port -> Port, 01_Port.cproj -> Port.cproj

param(
    [switch]$DryRun = $false
)

$ErrorActionPreference = "Stop"
$projectsPath = Join-Path $PSScriptRoot "projects"
$slnFile = Join-Path $PSScriptRoot "Main.atsln"

Write-Host "=== Project Renaming Script ===" -ForegroundColor Cyan
Write-Host "Projects Path: $projectsPath" -ForegroundColor Yellow
Write-Host "DryRun Mode: $DryRun" -ForegroundColor Yellow
Write-Host ""

# Get all project folders
$projectFolders = Get-ChildItem -Path $projectsPath -Directory | Sort-Object Name

# Build mapping of old to new names
$renameMap = @{}
foreach ($folder in $projectFolders) {
    $oldName = $folder.Name
    # Remove leading digits followed by underscore (e.g., "01_", "02_", etc.)
    $newName = $oldName -replace '^\d+_', ''
    
    if ($oldName -ne $newName) {
        $renameMap[$oldName] = $newName
        Write-Host "  $oldName -> $newName" -ForegroundColor Green
    }
    else {
        Write-Host "  $oldName (no change)" -ForegroundColor Gray
    }
}

Write-Host ""
Write-Host "Found $($renameMap.Count) projects to rename" -ForegroundColor Cyan
Write-Host ""

if ($renameMap.Count -eq 0) {
    Write-Host "No projects to rename. Exiting." -ForegroundColor Yellow
    exit 0
}

if ($DryRun) {
    Write-Host "DRY RUN MODE - No changes will be made" -ForegroundColor Yellow
    Write-Host ""
}

# Step 1: Rename .cproj files within folders
Write-Host "Step 1: Renaming .cproj files..." -ForegroundColor Cyan
foreach ($oldName in $renameMap.Keys) {
    $newName = $renameMap[$oldName]
    $oldCprojPath = Join-Path $projectsPath "$oldName\$oldName.cproj"
    $newCprojPath = Join-Path $projectsPath "$oldName\$newName.cproj"
    
    if (Test-Path $oldCprojPath) {
        Write-Host "  Renaming: $oldName.cproj -> $newName.cproj" -ForegroundColor Yellow
        if (-not $DryRun) {
            Move-Item -Path $oldCprojPath -Destination $newCprojPath -Force
        }
    }
}

# Step 1b: Rename .componentinfo.xml files within folders
Write-Host ""
Write-Host "Step 1b: Renaming .componentinfo.xml files..." -ForegroundColor Cyan
foreach ($oldName in $renameMap.Keys) {
    $newName = $renameMap[$oldName]
    $oldComponentInfoPath = Join-Path $projectsPath "$oldName\$oldName.componentinfo.xml"
    $newComponentInfoPath = Join-Path $projectsPath "$oldName\$newName.componentinfo.xml"
    
    if (Test-Path $oldComponentInfoPath) {
        Write-Host "  Renaming: $oldName.componentinfo.xml -> $newName.componentinfo.xml" -ForegroundColor Yellow
        if (-not $DryRun) {
            Move-Item -Path $oldComponentInfoPath -Destination $newComponentInfoPath -Force
        }
    }
}

# Step 2: Update content within .cproj files
Write-Host ""
Write-Host "Step 2: Updating .cproj file contents..." -ForegroundColor Cyan
foreach ($oldName in $renameMap.Keys) {
    $newName = $renameMap[$oldName]
    $cprojPath = Join-Path $projectsPath "$oldName\$newName.cproj"
    
    if (Test-Path $cprojPath) {
        Write-Host "  Updating: $newName.cproj" -ForegroundColor Yellow
        if (-not $DryRun) {
            $content = Get-Content -Path $cprojPath -Raw -Encoding UTF8
            # Replace AssemblyName
            $content = $content -replace "<AssemblyName>$oldName</AssemblyName>", "<AssemblyName>$newName</AssemblyName>"
            # Replace Name
            $content = $content -replace "<Name>$oldName</Name>", "<Name>$newName</Name>"
            # Replace RootNamespace
            $content = $content -replace "<RootNamespace>$oldName</RootNamespace>", "<RootNamespace>$newName</RootNamespace>"
            Set-Content -Path $cprojPath -Value $content -Encoding UTF8 -NoNewline
        }
    }
}

# Step 3: Rename project folders
Write-Host ""
Write-Host "Step 3: Renaming project folders..." -ForegroundColor Cyan
# Sort by name descending to avoid conflicts (rename longer names first)
$sortedOldNames = $renameMap.Keys | Sort-Object -Descending
foreach ($oldName in $sortedOldNames) {
    $newName = $renameMap[$oldName]
    $oldFolderPath = Join-Path $projectsPath $oldName
    $newFolderPath = Join-Path $projectsPath $newName
    
    if (Test-Path $oldFolderPath) {
        Write-Host "  Renaming folder: $oldName -> $newName" -ForegroundColor Yellow
        if (-not $DryRun) {
            # Check if target already exists (case-insensitive file systems)
            if ((Test-Path $newFolderPath) -and ($oldFolderPath -ne $newFolderPath)) {
                Write-Host "    WARNING: Target folder already exists: $newFolderPath" -ForegroundColor Red
                continue
            }
            Move-Item -Path $oldFolderPath -Destination $newFolderPath -Force
        }
    }
}

# Step 4: Update Main.atsln solution file
Write-Host ""
Write-Host "Step 4: Updating solution file..." -ForegroundColor Cyan
if (Test-Path $slnFile) {
    Write-Host "  Updating: Main.atsln" -ForegroundColor Yellow
    if (-not $DryRun) {
        $slnContent = Get-Content -Path $slnFile -Raw -Encoding UTF8
        
        foreach ($oldName in $renameMap.Keys) {
            $newName = $renameMap[$oldName]
            # Update project name
            $slnContent = $slnContent -replace "= `"$oldName`"", "= `"$newName`""
            # Update project path
            $slnContent = $slnContent -replace "projects\\$oldName\\$oldName\.cproj", "projects\$newName\$newName.cproj"
        }
        
        Set-Content -Path $slnFile -Value $slnContent -Encoding UTF8 -NoNewline
    }
}
else {
    Write-Host "  Solution file not found: $slnFile" -ForegroundColor Gray
}

# Step 5: Update build scripts
Write-Host ""
Write-Host "Step 5: Checking build scripts for updates..." -ForegroundColor Cyan

$buildScripts = @(
    "cli-build-project.ps1",
    "cli-build-all.ps1",
    "cli-program-project.ps1",
    "cli-test-system.ps1",
    "cli-analyze-code.ps1"
)

foreach ($scriptName in $buildScripts) {
    $scriptPath = Join-Path $PSScriptRoot $scriptName
    if (Test-Path $scriptPath) {
        Write-Host "  Checking: $scriptName" -ForegroundColor Yellow
        $scriptContent = Get-Content -Path $scriptPath -Raw -Encoding UTF8
        $modified = $false
        
        foreach ($oldName in $renameMap.Keys) {
            $newName = $renameMap[$oldName]
            if ($scriptContent -match $oldName) {
                Write-Host "    Found reference to: $oldName" -ForegroundColor Gray
                if (-not $DryRun) {
                    $scriptContent = $scriptContent -replace $oldName, $newName
                    $modified = $true
                }
            }
        }
        
        if ($modified -and -not $DryRun) {
            Set-Content -Path $scriptPath -Value $scriptContent -Encoding UTF8 -NoNewline
            Write-Host "    Updated: $scriptName" -ForegroundColor Green
        }
    }
}

Write-Host ""
if ($DryRun) {
    Write-Host "=== DRY RUN COMPLETED ===" -ForegroundColor Yellow
    Write-Host "No changes were made. Run without -DryRun to apply changes." -ForegroundColor Yellow
}
else {
    Write-Host "=== RENAMING COMPLETED SUCCESSFULLY ===" -ForegroundColor Green
}
Write-Host ""
Write-Host "Summary:" -ForegroundColor Cyan
Write-Host "  Projects renamed: $($renameMap.Count)" -ForegroundColor White
Write-Host "  .cproj files updated: $($renameMap.Count)" -ForegroundColor White
Write-Host "  Solution file updated: $(if (Test-Path $slnFile) { 'Yes' } else { 'N/A' })" -ForegroundColor White
Write-Host ""
