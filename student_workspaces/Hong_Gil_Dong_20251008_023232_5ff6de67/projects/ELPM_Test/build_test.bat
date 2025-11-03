@echo off
cd /d "w:\soc3050code\projects\ELPM_Test"
echo Building ELMP Test Program...

w:\soc3050code\tools\avr-toolchain\bin\avr-gcc.exe ^
-mmcu=atmega128 ^
-DF_CPU=16000000UL ^
-DBAUD=9600 ^
-O2 ^
-Wall ^
-I. ^
-funsigned-char ^
-funsigned-bitfields ^
-fpack-struct ^
-fshort-enums ^
Main.c ^
-lm ^
-o Main.elf

if %errorlevel% equ 0 (
    echo Build successful, generating HEX...
    w:\soc3050code\tools\avr-toolchain\bin\avr-objcopy.exe -O ihex -R .eeprom Main.elf Main.hex
    echo.
    echo Checking for ELPM instructions...
    w:\soc3050code\tools\avr-toolchain\bin\avr-objdump.exe -d Main.elf | findstr /i "elpm"
    echo.
    if %errorlevel% equ 0 (
        echo ELPM instructions found - this will test SimulIDE's ELPM handling
    ) else (
        echo No ELPM instructions found - compiler optimized them away
    )
    echo.
    echo Done! Load Main.hex into SimulIDE to test ELPM support.
    echo Expected behavior:
    echo - Success: PORTB LEDs blink alternating pattern 10101010/01010101
    echo - Failure: PORTB LEDs stay on for 2 seconds then turn off
) else (
    echo Build failed!
)