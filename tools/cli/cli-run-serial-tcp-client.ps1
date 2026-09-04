param(
    [string]$HostName = '127.0.0.1',
    [int]$Port = 9001
)

$scriptDir = Split-Path $MyInvocation.MyCommand.Path -Parent
$repoRoot = (Get-Item $scriptDir).Parent.Parent.FullName
$target = Join-Path $repoRoot "projects\Serial_Communications\tcp_test_client.ps1"

if (-not (Test-Path $target)) {
    Write-Host "Error: target client not found: $target" -ForegroundColor Red
    exit 1
}

pwsh -NoExit -ExecutionPolicy Bypass -File $target -HostName $HostName -Port $Port
