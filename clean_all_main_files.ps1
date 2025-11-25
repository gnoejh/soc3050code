# Batch clean all Main.c files - Remove lecture content, keep demo code
# This script processes all remaining project Main.c files

$projects = @(
    @{Name = "Inline_Assembly"; Header = "INLINE ASSEMBLY - DEMO CODE"; Demos = "AVR assembly language examples, inline assembly syntax, register operations" }
    @{Name = "Watchdog_Fail_Safe"; Header = "WATCHDOG FAIL-SAFE SYSTEM - DEMO CODE"; Demos = "Watchdog timer configuration, system reset scenarios, fail-safe programming" }
    @{Name = "I2C_Master_Basic"; Header = "I2C MASTER BASIC - DEMO CODE"; Demos = "I2C initialization, device addressing, read/write operations" }
    @{Name = "I2C_Sensors_Multi"; Header = "I2C MULTI-SENSOR - DEMO CODE"; Demos = "Multiple I2C devices, sensor reading, data aggregation" }
    @{Name = "Keypad_Advanced_Debounce"; Header = "KEYPAD DEBOUNCING - DEMO CODE"; Demos = "Software debouncing, key scanning, stable input detection" }
    @{Name = "I2C_RTC_DS1307"; Header = "I2C RTC DS1307 - DEMO CODE"; Demos = "Real-time clock interfacing, time/date reading, I2C communication" }
    @{Name = "Keypad_Calculator_App"; Header = "KEYPAD CALCULATOR - DEMO CODE"; Demos = "Key input processing, calculation logic, LCD display output" }
    @{Name = "LCD_Character_Basic"; Header = "LCD CHARACTER DISPLAY - DEMO CODE"; Demos = "LCD initialization, character display, cursor control" }
    @{Name = "Power_Wakeup_Optimization"; Header = "POWER WAKEUP OPTIMIZATION - DEMO CODE"; Demos = "Sleep modes, wakeup sources, power consumption optimization" }
    @{Name = "PWM_Motor_Stepper"; Header = "PWM STEPPER MOTOR - DEMO CODE"; Demos = "Stepper motor control, step sequencing, speed control" }
    @{Name = "SPI_Master_Basic"; Header = "SPI MASTER BASIC - DEMO CODE"; Demos = "SPI initialization, master mode, data transfer" }
    @{Name = "LCD_Sensor_Dashboard"; Header = "LCD SENSOR DASHBOARD - DEMO CODE"; Demos = "Multi-sensor display, LCD formatting, real-time updates" }
    @{Name = "PWM_Motor_DC"; Header = "PWM DC MOTOR - DEMO CODE"; Demos = "DC motor speed control, PWM generation, direction control" }
    @{Name = "SPI_EEPROM_Memory"; Header = "SPI EEPROM MEMORY - DEMO CODE"; Demos = "EEPROM read/write, SPI protocol, data persistence" }
    @{Name = "Graphics_Display"; Header = "GRAPHICS DISPLAY - DEMO CODE"; Demos = "Graphical LCD control, pixel drawing, shape rendering" }
    @{Name = "LCD_Advanced_Features"; Header = "LCD ADVANCED FEATURES - DEMO CODE"; Demos = "Custom characters, scrolling, special effects" }
    @{Name = "Keypad_Matrix_Basic"; Header = "KEYPAD MATRIX - DEMO CODE"; Demos = "Matrix scanning, key detection, input processing" }
    @{Name = "SPI_Multi_Device"; Header = "SPI MULTI-DEVICE - DEMO CODE"; Demos = "Multiple SPI slaves, chip select management, device coordination" }
    @{Name = "PWM_Motor_Servo"; Header = "PWM SERVO MOTOR - DEMO CODE"; Demos = "Servo position control, PWM timing, angle positioning" }
    @{Name = "Power_LowPower_Sensors"; Header = "LOW POWER SENSORS - DEMO CODE"; Demos = "Power-optimized sensor reading, sleep between readings" }
    @{Name = "Watchdog_System_Reset"; Header = "WATCHDOG SYSTEM RESET - DEMO CODE"; Demos = "Watchdog timer setup, reset scenarios, recovery mechanisms" }
    @{Name = "Power_Sleep_Modes"; Header = "POWER SLEEP MODES - DEMO CODE"; Demos = "Sleep mode configuration, wakeup methods, power savings" }
    @{Name = "Port_Basic"; Header = "PORT I/O BASIC - DEMO CODE"; Demos = "Port configuration, digital I/O, LED/button control" }
    @{Name = "Joystick"; Header = "JOYSTICK - DEMO CODE"; Demos = "Analog joystick reading, ADC sampling, position detection" }
    @{Name = "Port_Assembly"; Header = "PORT ASSEMBLY - DEMO CODE"; Demos = "Assembly language port operations, bit manipulation" }
    @{Name = "Accelerometer"; Header = "ACCELEROMETER - DEMO CODE"; Demos = "Accelerometer interfacing, axis reading, motion detection" }
    @{Name = "ELPM_Test"; Header = "ELPM EXTENDED MEMORY - DEMO CODE"; Demos = "Extended memory access, ELPM instruction usage" }
    @{Name = "CDS_Light_Sensor"; Header = "CDS LIGHT SENSOR - DEMO CODE"; Demos = "Light sensor reading, ADC conversion, ambient light detection" }
)

foreach ($proj in $projects) {
    $filePath = "w:\soc3050code\projects\$($proj.Name)\Main.c"
    
    if (-not (Test-Path $filePath)) {
        Write-Host "Skipping $($proj.Name) - file not found" -ForegroundColor Yellow
        continue
    }
    
    $originalLineCount = (Get-Content $filePath).Count
    
    # Find where #include "config.h" appears
    $includeLine = (Select-String -Path $filePath -Pattern '#include "config\.h"' | Select-Object -First 1).LineNumber
    
    if (-not $includeLine) {
        Write-Host "Skipping $($proj.Name) - no #include found" -ForegroundColor Yellow
        continue
    }
    
    $newHeader = @"
/*
 * ==============================================================================
 * $($proj.Header)
 * ==============================================================================
 * PROJECT: $($proj.Name)
 * See Slide.md for complete theory and technical details
 *
 * DEMOS: $($proj.Demos)
 * ==============================================================================
 */
"@
    
    # Extract code portion (from #include line onward)
    $codePart = Get-Content $filePath -TotalCount 99999 | Select-Object -Skip ($includeLine - 1)
    
    # Write new file
    ($newHeader, "", $codePart) | Out-File $filePath -Encoding utf8
    
    $newLineCount = (Get-Content $filePath).Count
    $saved = $originalLineCount - $newLineCount
    
    Write-Host "✓ $($proj.Name): $originalLineCount → $newLineCount lines (saved $saved)" -ForegroundColor Green
}

Write-Host "`nAll files cleaned! Run 'git status' to see changes." -ForegroundColor Cyan
