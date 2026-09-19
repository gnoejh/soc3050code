@echo off
REM ============================================================================
REM  Shared build engine - SOC3050 2026 ARM edition
REM
REM  Not run directly.  Each lesson's build.bat sets TARGET and LIBS, then:
REM      set TARGET=c031c6
REM      set LIBS=
REM      call "%~dp0..\_build\build-lesson.bat"
REM
REM  A lesson is SELF-SUFFICIENT if it carries its own startup.c and link.ld.
REM  This engine prefers the lesson's copies and falls back to the target's
REM  defaults in _startup\%TARGET%\ only if the lesson has none.  Lesson 04
REM  exists precisely so the student edits those two files, so it carries them.
REM
REM  Emits all three artefacts: Main.elf (Wokwi loads this, and it carries the
REM  symbols GDB wants), Main.hex (Wokwi also accepts this) and Main.bin.
REM ============================================================================
setlocal enabledelayedexpansion

set "ROOT=%~dp0..\.."
set "ARMBIN=%ROOT%\tools\arm-toolchain\bin"
set "GCC=%ARMBIN%\arm-none-eabi-gcc.exe"
set "OBJCOPY=%ARMBIN%\arm-none-eabi-objcopy.exe"
set "SIZE=%ARMBIN%\arm-none-eabi-size.exe"
set "CMSIS=%ROOT%\tools\cmsis"

if not exist "%GCC%" (
    echo ERROR: ARM toolchain not found at "%GCC%"
    echo        Expected the vendored xPack arm-none-eabi-gcc under tools\.
    exit /b 1
)
if not exist "Main.c" (
    echo ERROR: Main.c not found. Run build.bat from inside a lesson folder.
    exit /b 1
)
if "%TARGET%"=="" set "TARGET=c031c6"

set "TARGETCFG=%~dp0..\_targets\%TARGET%.bat"
if not exist "%TARGETCFG%" (
    echo ERROR: unknown target "%TARGET%" - no %TARGETCFG%
    exit /b 1
)
call "%TARGETCFG%"

REM  A target file that does not execute leaves these empty, and an empty
REM  DEVINC makes -I"%CMSIS%\" end in a backslash-quote, which escapes the
REM  quote and swallows the whole source list.  gcc then reports "no input
REM  files" and nothing points at the target file.  Check, do not hope.
REM  (This is why the target file is .bat: CMD's `call` silently ignores an
REM  unknown extension such as .cfg, with no error of any kind.)
if "%MCUFLAGS%"=="" (
    echo ERROR: target "%TARGET%" did not set MCUFLAGS.
    echo        "%TARGETCFG%" exists but produced nothing.
    exit /b 1
)
if "%DEVINC%"=="" (
    echo ERROR: target "%TARGET%" did not set DEVINC.
    exit /b 1
)

REM ---- startup.c and link.ld: lesson's own, else the target default --------
if exist "startup.c" (
    set "STARTUP=startup.c"
) else (
    set "STARTUP=%~dp0..\_startup\%TARGET%\startup.c"
)
if exist "link.ld" (
    set "LDSCRIPT=link.ld"
) else (
    set "LDSCRIPT=%~dp0..\_startup\%TARGET%\link.ld"
)

for %%I in ("%CD%") do set "LESSON=%%~nxI"
echo Building %LESSON%  [target %TARGET%] ...

REM ---- assemble the source list -------------------------------------------
REM  Every .c in the lesson folder is compiled.  That is what makes a lesson
REM  folder self-sufficient: drop retarget.c or a driver in beside Main.c and
REM  it builds, with no list to maintain anywhere.
set "SOURCES="
for %%F in (*.c) do set "SOURCES=!SOURCES! %%F"
if not exist "startup.c" set "SOURCES=!SOURCES! "!STARTUP!""
for %%L in (%LIBS%) do (
    if not exist "%~dp0..\_lib\%%L.c" (
        echo ERROR: shared library "%%L.c" does not exist in _lib\.
        exit /b 1
    )
    set "SOURCES=!SOURCES! "%~dp0..\_lib\%%L.c""
)

REM ---- compile + link ------------------------------------------------------
REM  --specs=nano.specs picks newlib-nano.  We do NOT use nosys.specs: this
REM  edition implements its own _write/_read/_sbrk so printf reaches USART2,
REM  which also removes the seven "not implemented" link warnings nosys emits.
"%GCC%" ^
    %MCUFLAGS% ^
    %DEVDEF% ^
    -Os -g3 -std=c11 ^
    -Wall -Wextra ^
    -ffunction-sections -fdata-sections ^
    -I. ^
    -I"%CMSIS%\Core\Include" ^
    -I"%CMSIS%\%DEVINC%" ^
    -I"%~dp0..\_lib" ^
    !SOURCES! ^
    --specs=nano.specs ^
    -T"!LDSCRIPT!" ^
    -Wl,--gc-sections ^
    -Wl,-Map=Main.map ^
    -Wl,--print-memory-usage ^
    -o Main.elf
if errorlevel 1 (
    echo.
    echo Build FAILED.
    exit /b 1
)

"%OBJCOPY%" -O ihex Main.elf Main.hex
if errorlevel 1 ( echo HEX generation FAILED. & exit /b 1 )
"%OBJCOPY%" -O binary Main.elf Main.bin
if errorlevel 1 ( echo BIN generation FAILED. & exit /b 1 )

REM ---- report ---------------------------------------------------------------
REM  -A gives the real per-section split.  Unlike the AVR toolchain this linker
REM  does not fold .data into text, so these numbers can be trusted directly.
echo.
"%SIZE%" -A Main.elf
echo.
echo Build OK: Main.elf, Main.hex and Main.bin created
echo Simulate: run simulate.bat, or upload Main.elf at
echo           https://wokwi.com/projects/new/%WOKWIBOARD%
exit /b 0
