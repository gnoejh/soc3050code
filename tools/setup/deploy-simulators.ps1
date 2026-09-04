#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Deploy simulate.bat files to all project directories
    
.DESCRIPTION
    Creates convenient simulate.bat launcher files in each project directory
    for one-click building and simulation.
    
.EXAMPLE
    .\deploy-simulators.ps1
    Deploy to all project directories
#>

param(
    [string]$ProjectsRoot = "projects",
    [switch]$Force = $false
)

$WorkspaceRoot = $PSScriptRoot
$ProjectsPath = Join-Path $WorkspaceRoot $ProjectsRoot
$TemplatePath = Join-Path $WorkspaceRoot "simulate-template.bat"

if (-not (Test-Path $TemplatePath)) {
    Write-Host "❌ Error: Template file not found: $TemplatePath" -ForegroundColor Red
    exit 1
}

if (-not (Test-Path $ProjectsPath)) {
    Write-Host "❌ Error: Projects directory not found: $ProjectsPath" -ForegroundColor Red
    exit 1
}

Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "  Deploy SimulIDE Batch Launchers" -ForegroundColor Cyan
Write-Host "========================================`n" -ForegroundColor Cyan

# Get all project directories
$Projects = Get-ChildItem -Path $ProjectsPath -Directory

Write-Host "Found $($Projects.Count) project directories" -ForegroundColor Green
Write-Host ""

$Created = 0
$Skipped = 0
$Updated = 0

foreach ($Project in $Projects) {
    $DestFile = Join-Path $Project.FullName "simulate.bat"
    
    $Exists = Test-Path $DestFile
    
    if ($Exists -and -not $Force) {
        Write-Host "⏭️  Skipped: " -NoNewline -ForegroundColor Yellow
        Write-Host "$($Project.Name) " -NoNewline -ForegroundColor White
        Write-Host "(already exists, use -Force to overwrite)" -ForegroundColor Gray
        $Skipped++
        continue
    }
    
    try {
        Copy-Item -Path $TemplatePath -Destination $DestFile -Force
        
        if ($Exists) {
            Write-Host "🔄 Updated: " -NoNewline -ForegroundColor Cyan
            Write-Host "$($Project.Name)" -ForegroundColor White
            $Updated++
        } else {
            Write-Host "✅ Created: " -NoNewline -ForegroundColor Green
            Write-Host "$($Project.Name)" -ForegroundColor White
            $Created++
        }
    } catch {
        Write-Host "❌ Failed: " -NoNewline -ForegroundColor Red
        Write-Host "$($Project.Name) - $_" -ForegroundColor White
    }
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Summary:" -ForegroundColor Cyan
Write-Host "  ✅ Created: $Created" -ForegroundColor Green
Write-Host "  🔄 Updated: $Updated" -ForegroundColor Cyan
Write-Host "  ⏭️  Skipped: $Skipped" -ForegroundColor Yellow
Write-Host "  📁 Total projects: $($Projects.Count)" -ForegroundColor White
Write-Host "========================================`n" -ForegroundColor Cyan

Write-Host "Students can now double-click simulate.bat in any project folder!" -ForegroundColor Green
Write-Host ""

exit 0
