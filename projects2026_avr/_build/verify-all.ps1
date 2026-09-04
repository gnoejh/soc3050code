<#
.SYNOPSIS
    Build every lesson in projects2026_avr and check the result is real firmware.

.DESCRIPTION
    Run this before a lecture, or after touching anything shared - the build
    engine, shared_libs, or config.h - to confirm all 22 lessons still work.

    It checks more than the exit code. A batch file's "echo ... -> Main.hex"
    is a redirect, and that bug once left every lesson with a 13-byte Main.hex
    containing the word "Build OK" while every existence check still passed.
    So each hex is validated as Intel HEX: records begin with ':', the file
    ends with an end-of-file record, and there is a plausible amount of code.

.PARAMETER Lesson
    Verify one lesson instead of all, e.g. -Lesson 11_ADC_Basic

.PARAMETER ShowWarnings
    List compiler warnings from lesson code. Warnings originating inside
    shared_libs are counted separately and not listed, since they come from
    library stubs no lesson calls.

.PARAMETER Clean
    Delete build output afterwards, leaving the tree as committed.

.EXAMPLE
    pwsh _build\verify-all.ps1
    pwsh _build\verify-all.ps1 -ShowWarnings -Clean
#>
param(
    [string]$Lesson = "",
    [switch]$ShowWarnings,
    [switch]$Clean
)

$ErrorActionPreference = "Stop"
$root = Split-Path $PSScriptRoot -Parent

$lessons = Get-ChildItem $root -Directory |
    Where-Object { $_.Name -match '^\d\d_' } |
    Sort-Object Name
if ($Lesson) {
    $lessons = $lessons | Where-Object { $_.Name -eq $Lesson }
    if (-not $lessons) { Write-Host "No such lesson: $Lesson" -ForegroundColor Red; exit 1 }
}

function Test-IntelHex {
    <# Returns $null if the file is a valid Intel HEX image, else a reason. #>
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path)) { return "Main.hex missing" }
    $lines = @(Get-Content -LiteralPath $Path)
    if ($lines.Count -lt 2) { return "only $($lines.Count) line(s) - output was clobbered" }

    $data = 0
    foreach ($l in $lines) {
        if ($l -notmatch '^:[0-9A-Fa-f]{8,}$') { return "bad record: '$l'" }
        if ($l.Substring(7, 2) -eq '00') { $data += [Convert]::ToInt32($l.Substring(1, 2), 16) }
    }
    if ($lines[-1] -notmatch '^:00000001FF$') { return "no end-of-file record" }
    if ($data -lt 64) { return "only $data bytes of code" }
    return $null
}

$results = @()
foreach ($d in $lessons) {
    $p = $d.FullName
    Remove-Item "$p\Main.hex", "$p\Main.elf" -ErrorAction SilentlyContinue

    $out = & cmd.exe /c "cd /d `"$p`" && call .\build.bat" 2>&1
    $exit = $LASTEXITCODE

    $warnLines = @($out | Select-String 'warning:')
    $libWarn = @($warnLines | Where-Object { "$_" -match 'shared_libs' }).Count
    $ownWarn = @($warnLines | Where-Object { "$_" -notmatch 'shared_libs' })

    $reason = Test-IntelHex "$p\Main.hex"
    $bytes = 0
    if (-not $reason) { $bytes = (Get-Item "$p\Main.hex").Length }

    $results += [pscustomobject]@{
        Lesson   = $d.Name
        Ok       = ($exit -eq 0 -and -not $reason)
        Reason   = $reason
        HexBytes = $bytes
        OwnWarn  = $ownWarn
        LibWarn  = $libWarn
        Errors   = @($out | Select-String 'error:' | Select-Object -First 4)
    }
}

# ---- report ---------------------------------------------------------------
"`n  SOC3050 2026 edition - build verification`n"
foreach ($r in $results) {
    if ($r.Ok) {
        $w = if ($r.OwnWarn.Count) { "  ({0} warning{1})" -f $r.OwnWarn.Count, (@('s','')[[int]($r.OwnWarn.Count -eq 1)]) } else { "" }
        "  PASS  {0,-26} {1,6} bytes{2}" -f $r.Lesson, $r.HexBytes, $w
    }
    else {
        "  FAIL  {0,-26} {1}" -f $r.Lesson, $r.Reason
        foreach ($e in $r.Errors) { "          $e" }
    }
    if ($ShowWarnings) {
        foreach ($w in $r.OwnWarn) {
            "          " + (("$w" -replace '^.*warning: ', '') -replace '\s+', ' ')
        }
    }
}

$pass = @($results | Where-Object Ok).Count
$fail = $results.Count - $pass
$ownWarnTotal = ($results | ForEach-Object { $_.OwnWarn.Count } | Measure-Object -Sum).Sum
$libWarnTotal = ($results | ForEach-Object { $_.LibWarn } | Measure-Object -Sum).Sum

""
"  {0} passed, {1} failed" -f $pass, $fail
"  {0} warnings in lesson code, {1} in shared_libs" -f $ownWarnTotal, $libWarnTotal
if (-not $ShowWarnings -and $ownWarnTotal) { "  re-run with -ShowWarnings to list them" }

if ($Clean) {
    foreach ($d in $lessons) {
        Remove-Item "$($d.FullName)\Main.hex", "$($d.FullName)\Main.elf",
                    "$($d.FullName)\Board.simu" -ErrorAction SilentlyContinue
    }
    "  build output cleaned"
}
""
exit ([int]($fail -gt 0))
