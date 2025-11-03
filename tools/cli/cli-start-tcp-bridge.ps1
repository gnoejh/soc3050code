param(
    [string]$Method = 'tcpsockets',
    [string]$Port1 = 'VCOM3',
    [string]$Port2 = 'VCOM4'
)

$scriptDir = Split-Path $MyInvocation.MyCommand.Path -Parent
$repoRoot = (Get-Item $scriptDir).Parent.Parent.FullName
$target = Join-Path $repoRoot "tools\simulide\windows-virtual-com.ps1"

if (-not (Test-Path $target)) {
    Write-Host "Error: target script not found: $target" -ForegroundColor Red
    exit 1
}

pwsh -ExecutionPolicy Bypass -File $target -Method $Method -Start
