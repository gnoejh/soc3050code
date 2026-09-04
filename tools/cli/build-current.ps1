<#
.SYNOPSIS
    Build whichever lesson or project the open file belongs to.

.DESCRIPTION
    The repository holds two lesson trees with different build systems:

      projects2026_avr/   current edition. Each lesson has a build.bat that
                          names its shared_libs set and calls the shared
                          engine in _build/build-lesson.bat.

      projects/           legacy edition, built by cli-build-project.ps1,
                          which guesses the library set from the filenames
                          present in the folder.

    VS Code's "Build Current Project" task calls this, so the same keystroke
    works in either tree. Without it the task fails outright on the 2026
    lessons: cli-build-project.ps1 gates on the path matching "*\projects\*".

.PARAMETER ProjectDir
    Directory of the lesson to build. Defaults to the current directory.

.PARAMETER SourceFile
    Legacy tree only; ignored by the 2026 engine, which always builds Main.c.
#>
param(
    [string]$ProjectDir = (Get-Location).Path,
    [string]$SourceFile = ""
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $ProjectDir -PathType Container)) {
    Write-Host "[ERROR] Not a directory: $ProjectDir" -ForegroundColor Red
    exit 1
}
$ProjectDir = (Resolve-Path -LiteralPath $ProjectDir).Path
$name = Split-Path $ProjectDir -Leaf

# --- 2026 edition -----------------------------------------------------------
# Legacy projects also contain a build.bat, so the presence of one proves
# nothing. What identifies the 2026 tree is the shared engine sitting in a
# _build directory beside the lesson.
$lessonBuild = Join-Path $ProjectDir "build.bat"
$engine = Join-Path (Split-Path $ProjectDir -Parent) "_build\build-lesson.bat"
if ((Test-Path -LiteralPath $lessonBuild) -and (Test-Path -LiteralPath $engine)) {
    Write-Host "[2026] Building $name" -ForegroundColor Cyan
    & cmd.exe /c "cd /d `"$ProjectDir`" && call .\build.bat"
    $code = $LASTEXITCODE

    # The engine reports success before writing the hex, so confirm the
    # artefact really is an Intel HEX file rather than trusting the exit code.
    $hex = Join-Path $ProjectDir "Main.hex"
    if ($code -eq 0 -and (Test-Path -LiteralPath $hex)) {
        $first = Get-Content -LiteralPath $hex -TotalCount 1
        if ($first -notlike ':*') {
            Write-Host "[ERROR] Main.hex is not Intel HEX - build output was clobbered." -ForegroundColor Red
            exit 1
        }
    }
    elseif ($code -eq 0) {
        Write-Host "[ERROR] Build reported success but Main.hex is missing." -ForegroundColor Red
        exit 1
    }
    exit $code
}

# --- legacy edition ---------------------------------------------------------
if ($ProjectDir -like "*\projects\*" -or $ProjectDir -like "*/projects/*") {
    Write-Host "[legacy] Building $name via cli-build-project.ps1" -ForegroundColor Yellow
    $legacy = Join-Path $PSScriptRoot "cli-build-project.ps1"
    & $legacy -ProjectDir $ProjectDir -SourceFile $SourceFile
    exit $LASTEXITCODE
}

Write-Host "[ERROR] $name is not a lesson folder." -ForegroundColor Red
Write-Host "        Open a file inside projects2026_avr\<lesson>\ (current edition)" -ForegroundColor Yellow
Write-Host "        or projects\<project>\ (legacy), then run the task again." -ForegroundColor Yellow
exit 1
