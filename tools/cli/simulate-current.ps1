<#
.SYNOPSIS
    Open whichever lesson the current file belongs to in SimulIDE.

.DESCRIPTION
    For the 2026 edition this runs the lesson's simulate.bat, which builds if
    needed, drops a copy of the shared board next to the lesson's own Main.hex
    and opens it in SimulIDE 1.1.0-SR2.

    For the legacy projects/ tree it falls through to the older SR1 launcher.

.PARAMETER ProjectDir
    Directory of the lesson. Defaults to the current directory.
#>
param(
    [string]$ProjectDir = (Get-Location).Path
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $ProjectDir -PathType Container)) {
    Write-Host "[ERROR] Not a directory: $ProjectDir" -ForegroundColor Red
    exit 1
}
$ProjectDir = (Resolve-Path -LiteralPath $ProjectDir).Path
$name = Split-Path $ProjectDir -Leaf

# --- 2026 edition -----------------------------------------------------------
# A couple of legacy projects also carry a simulate.bat, so identify the 2026
# tree by the shared launcher in the _build directory beside the lesson.
$lessonSim = Join-Path $ProjectDir "simulate.bat"
$engine = Join-Path (Split-Path $ProjectDir -Parent) "_build\simulate-lesson.bat"
if ((Test-Path -LiteralPath $lessonSim) -and (Test-Path -LiteralPath $engine)) {
    Write-Host "[2026] Simulating $name in SimulIDE 1.1.0-SR2" -ForegroundColor Cyan
    & cmd.exe /c "cd /d `"$ProjectDir`" && call .\simulate.bat"
    exit $LASTEXITCODE
}

# --- legacy edition ---------------------------------------------------------
if ($ProjectDir -like "*\projects\*" -or $ProjectDir -like "*/projects/*") {
    Write-Host "[legacy] Simulating $name via cli-simulide.ps1 (SimulIDE 1.1.0-SR1)" -ForegroundColor Yellow
    $legacy = Join-Path (Split-Path $PSScriptRoot -Parent) "simulide\cli-simulide.ps1"
    & $legacy -ProjectDir $ProjectDir
    exit $LASTEXITCODE
}

Write-Host "[ERROR] $name is not a lesson folder." -ForegroundColor Red
Write-Host "        Open a file inside projects2026_avr\<lesson>\ and try again." -ForegroundColor Yellow
exit 1
