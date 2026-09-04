param(
    [string]$CircuitFile = "",
    [int]$TcpPort = 9002,
    [switch]$Restore
)

$scriptDir = Split-Path $MyInvocation.MyCommand.Path -Parent
$repoRoot = (Get-Item $scriptDir).Parent.Parent.FullName
$target = Join-Path $repoRoot "tools\simulide\configure-simulide-tcp.ps1"

if (-not (Test-Path $target)) {
    Write-Host "Error: target script not found: $target" -ForegroundColor Red
    exit 1
}

$argsList = @()
if ($CircuitFile -ne "") { $argsList += "-CircuitFile"; $argsList += $CircuitFile }
if ($TcpPort -ne 9002) { $argsList += "-TcpPort"; $argsList += $TcpPort }
if ($Restore) { $argsList += "-Restore" }

pwsh -ExecutionPolicy Bypass -File $target $argsList
