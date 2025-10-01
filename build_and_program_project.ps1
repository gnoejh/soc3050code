param(
    [Parameter(Mandatory=$true)]
    [string]$ProjectDir
)

# Extract project name from directory path
Write-Host "🔍 Analyzing project directory: $ProjectDir" -ForegroundColor Cyan

if ($ProjectDir -like '*\projects\*') {
    $projectName = Split-Path $ProjectDir -Leaf
    Write-Host "🎯 Detected project: $projectName" -ForegroundColor Green
    
    # Change to workspace directory
    Set-Location "w:\soc3050code"
    
    Write-Host "🔨 Building and programming: $projectName" -ForegroundColor Green
    
    # Call the universal programming script (which will build if needed)
    & ".\program.ps1" -ProjectFolder $projectName
} else {
    Write-Host "❌ Error: Please open a file from a project folder" -ForegroundColor Red
    Write-Host "   Current directory: $ProjectDir" -ForegroundColor Gray
    exit 1
}