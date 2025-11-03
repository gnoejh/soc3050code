@echo off
cd /d "w:\soc3050code\projects\Graphics_Display"
echo Building Graphics_Display with full GLCD library...

w:\soc3050code\tools\avr-toolchain\bin\avr-gcc.exe ^
-mmcu=atmega128 ^
-DF_CPU=16000000UL ^
-DBAUD=9600 ^
-Os ^
-Wall ^
-I. ^
-I../../shared_libs ^
-funsigned-char ^
-funsigned-bitfields ^
-fpack-struct ^
-fshort-enums ^
Main.c ^
../../shared_libs/_glcd.c ^
../../shared_libs/_port.c ^
../../shared_libs/_init.c ^
-lm ^
-o Main.elf

if %errorlevel% equ 0 (
    echo Build successful, generating HEX...
    w:\soc3050code\tools\avr-toolchain\bin\avr-objcopy.exe -O ihex -R .eeprom Main.elf Main.hex
    echo Done! Main.hex generated with full GLCD library.
    echo Note: ELPM errors may occur in SimulIDE 1.1.0 but this demonstrates the library works in 0.4.15
) else (
    echo Build failed!
)