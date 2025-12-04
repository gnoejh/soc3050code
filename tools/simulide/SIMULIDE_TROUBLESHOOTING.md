# SimulIDE Troubleshooting Guide - ATmega128 Projects

**Course:** SOC 3050 - Embedded Systems and Applications  
**Target Hardware:** ATmega128 @ 16MHz  
**Simulator:** SimulIDE (versions 0.4.15 and 1.1.0-SR1)  
**Last Updated:** November 2, 2025

---

## Table of Contents

1. [SimulIDE Version Comparison](#simulide-version-comparison)
2. [Known Bugs and Workarounds](#known-bugs-and-workarounds)
3. [Fixed Issues in SimulIDE 1.1.0-SR1](#fixed-issues-in-simulide-110-sr1)
4. [Active Bugs in SimulIDE 1.1.0-SR1](#active-bugs-in-simulide-110-sr1)
5. [Project-Specific Workarounds](#project-specific-workarounds)
6. [Verification Procedures](#verification-procedures)
7. [Migration Guide](#migration-guide)

---

## SimulIDE Version Comparison

### Version 0.4.15 (Legacy/Stable)
- **Status:** Stable for most ATmega128 features
- **Known Issues:** ELPM instruction crash, UART1 initialization problems
- **Best For:** Basic I/O, Timer0, Timer1, SPI, I2C projects
- **Recommendation:** Use for production lab exercises

### Version 1.1.0-SR1 (Current/Development)
- **Status:** Beta quality - some regressions from 0.4.15
- **Fixed Issues:** ELPM instruction, UART1 initialization
- **New Issues:** Timer0 CTC mode crash, Timer0 PWM modes unstable
- **Best For:** Testing ELPM code, advanced UART projects
- **Recommendation:** Use cautiously, with fallback to 0.4.15

---

## Known Bugs and Workarounds

### 🔴 **Critical Bugs (System Crashes)**

#### 1. Timer0 CTC Mode Crash (SimulIDE 1.1.0-SR1)
**Status:** ❌ **ACTIVE BUG** in SimulIDE 1.1.0-SR1  
**Affected Projects:** Timer_Programming (Demos 3-4)  
**Symptoms:**
- SimulIDE crashes when Timer0 enters CTC mode
- No error message, just application termination
- Occurs when `WGM01 = 1` (CTC mode bit set)

**Root Cause:**
- Timer0 type 821 implementation bug in SimulIDE 1.1.0-SR1
- CTC mode register handling appears broken
- Possibly related to OCR0 compare match logic

**Workaround:**
```c
// ❌ CRASHES in SimulIDE 1.1.0-SR1:
TCCR0 = (1<<WGM01) | (1<<CS02);  // CTC mode, Timer0

// ✅ WORKAROUND - Use Timer1 instead:
TCCR1A = 0x00;
TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10);  // CTC mode, Timer1
OCR1AH = 0;
OCR1AL = 249;  // Compare value
```

**Project Impact:**
- Timer_Programming: Demos 3-4 switched from Timer0 to Timer1 CTC mode
- All CTC functionality works correctly with Timer1
- Educational objectives achieved with alternative timer

---

#### 2. Timer0 PWM Modes Crash (SimulIDE 1.1.0-SR1)
**Status:** ❌ **ACTIVE BUG** in SimulIDE 1.1.0-SR1  
**Affected Projects:** Timer_Programming (Demos 5-6)  
**Symptoms:**
- Fast PWM mode crashes simulator
- Phase Correct PWM mode unstable
- Crash occurs when `COM01` bits are set with PWM mode

**Root Cause:**
- Timer0 type 821 PWM waveform generator broken
- OC0 pin output logic causes internal error
- Possibly related to compare output mode handling

**Workaround:**
```c
// ❌ CRASHES in SimulIDE 1.1.0-SR1:
TCCR0 = (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS01);  // Fast PWM

// ✅ WORKAROUND - Use Timer1 8-bit PWM:
TCCR1A = (1<<COM1A1) | (1<<WGM10);  // Non-inverting, 8-bit PWM
TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10);  // Fast PWM mode
OCR1AH = 0;
OCR1AL = 128;  // 50% duty cycle
```

**Project Impact:**
- Timer_Programming: Demos 5-6 use Timer1 8-bit PWM instead of Timer0
- PWM functionality identical (frequency differs slightly)
- Students learn 16-bit timer flexibility

---

### 🟡 **Major Bugs (Functional Issues)**

#### 3. UART1 Initialization Hang (SimulIDE 0.4.15)
**Status:** ✅ **FIXED** in SimulIDE 1.1.0-SR1  
**Affected Projects:** All projects using serial communication  
**Symptoms:**
- Calling `Uart1_init()` causes infinite loop in SimulIDE 0.4.15
- Simulator appears frozen, no CPU activity
- printf() statements never execute

**Root Cause:**
- UART1 initialization sequence triggers internal simulator deadlock
- Possibly related to baud rate generator or interrupt flag handling
- Only affects Uart1, not Uart0

**Workaround (0.4.15):**
```c
// ❌ HANGS in SimulIDE 0.4.15:
void main(void) {
    Uart1_init();  // Blocks forever
    printf("Hello\n");
}

// ✅ WORKAROUND for 0.4.15 - Disable UART:
void main(void) {
    // Uart1_init();  // Comment out for simulation
    // printf("Hello\n");  // Comment out
    
    // Use LED indicators instead
    PORTB ^= (1<<PB0);
}
```

**Fix Status:**
- ✅ **Completely fixed in SimulIDE 1.1.0-SR1**
- UART1 initialization works correctly
- printf() and serial output functional
- Can re-enable UART in all projects when using 1.1.0-SR1

---

#### 4. ELPM Instruction Crash (All SimulIDE Versions)
**Status:** ⚠️ **PARTIAL FIX REQUIRED** - RAMPZ register added but ELPM execution still buggy  
**Affected Projects:** Graphics_Display, any project using PROGMEM strings with strcpy_P  
**Symptoms:**
- Error: "AVR Invalid instruction: ELPM with no RAMPZ"
- Occurs when using strcpy_P() or pgm_read_byte() in some cases
- Happens even though RAMPZ register is defined in mega128.mcu

**Root Cause:**
- AVR-GCC generates ELPM instructions for PROGMEM access on ATmega128
- SimulIDE 1.1.0-SR1 has RAMPZ register defined but ELPM execution is buggy
- The compiler assumes 24-bit addressing (128KB) even for small programs
- strcpy_P() from avr-libc generates ELPM instead of LPM

**Compiler-Level Fix:**
```powershell
# Add -mshort-calls flag to force 16-bit addressing (LPM instead of ELPM)
avr-gcc -mmcu=atmega128 -mshort-calls ... Main.c -o Main.elf
```

**Applied in cli-build-project.ps1:**
```powershell
elseif ($name -eq "Graphics_Display") {
    $NoELPMFlag = "-mshort-calls"  # Force LPM instead of ELPM
    & $avrGccExe ... $NoELPMFlag $SourceFile ... -o "$BaseName.elf"
}
```

**Fix Status:**
- ✅ **RAMPZ register added to mega128.mcu** (in tools/simulide/SimulIDE_1.1.0-SR1_Win64/data/AVR/)
- ✅ **Compiler flag -mshort-calls added** to Graphics_Display build script
- ⚠️ **ELPM execution still broken** in SimulIDE - avoid using it
- ✅ **LPM (16-bit) addressing works correctly** with -mshort-calls flag

---

## Fixed Issues in SimulIDE 1.1.0-SR1

### ⚠️ ELPM Instruction - Partially Working
**Status:** RAMPZ register added, but ELPM execution still buggy  
**Workaround:** Use `-mshort-calls` compiler flag to force LPM instead  
**Projects Affected:**
- Graphics_Display: Fixed with -mshort-calls flag
- Any project using strcpy_P() or PROGMEM access
- Projects < 64KB don't need ELPM anyway

**Compiler Fix Applied:**
```powershell
# In cli-build-project.ps1 for Graphics_Display:
$NoELPMFlag = "-mshort-calls"  # Force 16-bit addressing (LPM)
avr-gcc -mmcu=atmega128 -mshort-calls ... Main.c -o Main.elf
```

**Why This Works:**
- `-mshort-calls` tells compiler to use 16-bit addresses (0-64KB range)
- Forces LPM instruction instead of ELPM
- LPM works correctly in SimulIDE
- Sufficient for all current projects (none exceed 64KB)

**If You Need ELPM:**
- Test on real ATmega128 hardware instead
- Or use SimulIDE 0.4.15 and avoid PROGMEM entirely

### ✅ UART1 Initialization
**Benefit:** Serial communication and printf() debugging works  
**Projects Enabled:**
- All projects can now use serial output
- printf() debugging available
- Real-time monitoring via serial terminal

**Verification:**
```c
// Test UART1 functionality
void test_uart1(void) {
    Uart1_init();  // Should not hang
    printf("UART1 Test\n");  // Should print
    Uart1_Tx('A');  // Should transmit
}
```

---

## Active Bugs in SimulIDE 1.1.0-SR1

### ❌ Timer0 Type 821 Bugs

**Bug Summary:**
- Timer0 Normal mode: ✅ Works
- Timer0 CTC mode: ❌ **Crashes simulator**
- Timer0 Fast PWM: ❌ **Crashes simulator**  
- Timer0 Phase Correct PWM: ❌ **Unstable/crashes**

**Affected Registers:**
- `TCCR0`: WGM01, WGM00, COM01, COM00 bits cause issues
- `OCR0`: Compare value setting triggers crash in CTC/PWM modes
- `TCNT0`: Counter works in Normal mode only

**Workaround Strategy:**
1. Use Timer0 only for Normal mode (overflow counting)
2. Switch to Timer1 for CTC mode
3. Switch to Timer1 for PWM applications
4. Accept Timer1 as "the student timer" for this version

**Timer Capability Matrix:**

| Mode              | Timer0 (0.4.15) | Timer0 (1.1.0-SR1) | Timer1 (All Versions) |
|-------------------|-----------------|--------------------|-----------------------|
| Normal/Overflow   | ✅ Works        | ✅ Works           | ✅ Works              |
| CTC               | ✅ Works        | ❌ **CRASH**       | ✅ Works              |
| Fast PWM          | ✅ Works        | ❌ **CRASH**       | ✅ Works              |
| Phase Correct PWM | ✅ Works        | ❌ **CRASH**       | ✅ Works              |

---

## Project-Specific Workarounds

### Timer_Programming Project

**Original Goal:** Use only Timer0 for all demonstrations

**Implemented Solution:** Hybrid Timer0 + Timer1 approach

**Demo Breakdown:**

| Demo | Description              | Timer Used | Reason                        |
|------|--------------------------|------------|-------------------------------|
| 0    | LED Test                 | None       | Uses _delay_ms()              |
| 1    | Normal Mode Polling      | Timer0     | ✅ Normal mode works          |
| 2    | Normal Mode Interrupt    | Timer0     | ✅ Normal mode works          |
| 3    | CTC Mode Polling         | **Timer1** | ❌ Timer0 CTC crashes         |
| 4    | CTC Mode Interrupt       | **Timer1** | ❌ Timer0 CTC crashes         |
| 5    | Fast PWM                 | **Timer1** | ❌ Timer0 PWM crashes         |
| 6    | Phase Correct PWM        | **Timer1** | ❌ Timer0 PWM crashes         |
| 7    | Prescaler Comparison     | Timer0     | ✅ Normal mode works          |
| 8    | Multi-task Interrupt     | Timer0     | ✅ Normal mode works          |

**Educational Impact:**
- ✅ All learning objectives achieved
- ✅ Students learn both Timer0 and Timer1
- ✅ Demonstrates timer flexibility
- ✅ Real-world skill: choosing appropriate timer
- ℹ️ Bonus: 16-bit timer experience

**Code Documentation:**
```c
/*
 * NOTE: Timer1 is used for CTC and PWM demos due to SimulIDE Timer0 bug
 * Timer0 CTC/PWM modes crash in SimulIDE 1.1.0-SR1
 * Timer1 (16-bit) provides same functionality with more precision
 * Educational objectives unchanged - students learn timer concepts
 */
```

---

### Serial_Communications Project

**Version-Specific Code:**

```c
// SimulIDE 1.1.0-SR1: UART works
#define SIMULIDE_VERSION_NEW  // Define for 1.1.0-SR1

void main(void) {
#ifdef SIMULIDE_VERSION_NEW
    Uart1_init();  // Safe to use
    printf("Serial Output Available\n");
#else
    // SimulIDE 0.4.15: UART hangs
    // Use LED indicators instead
    PORTB &= ~(1<<PB0);  // Visual feedback
#endif
}
```

---

## Verification Procedures

### Testing Timer0 Functionality

```c
// Test 1: Normal Mode (Should work in all versions)
void test_timer0_normal(void) {
    TCCR0 = (1<<CS02);  // Normal mode, prescaler 256
    TCNT0 = 0;
    while(!(TIFR & (1<<TOV0)));  // Wait for overflow
    // ✅ Should not crash
}

// Test 2: CTC Mode (Crashes in 1.1.0-SR1)
void test_timer0_ctc(void) {
    TCCR0 = (1<<WGM01) | (1<<CS02);  // CTC mode
    OCR0 = 100;
    // ❌ CRASH in SimulIDE 1.1.0-SR1
}

// Test 3: PWM Mode (Crashes in 1.1.0-SR1)
void test_timer0_pwm(void) {
    TCCR0 = (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS01);
    OCR0 = 128;
    // ❌ CRASH in SimulIDE 1.1.0-SR1
}
```

### Testing UART1 Functionality

```c
// Test in SimulIDE 1.1.0-SR1
void test_uart1(void) {
    Uart1_init();
    printf("UART1 Test: ");
    
    for(uint8_t i = 0; i < 10; i++) {
        printf("%d ", i);
    }
    printf("\n");
    // ✅ Should work in 1.1.0-SR1
    // ❌ Hangs in 0.4.15
}
```

### Testing ELPM Workaround

```c
// ❌ AVOID - Generates ELPM which crashes SimulIDE:
void test_elpm_broken(void) {
    const uint8_t data[] PROGMEM = {1, 2, 3, 4, 5};
    uint8_t value = pgm_read_byte_far((uint32_t)data);  // ELPM crashes!
}

// ✅ USE - With -mshort-calls compiler flag:
void test_lpm_working(void) {
    const uint8_t data[] PROGMEM = {1, 2, 3, 4, 5};
    uint8_t value = pgm_read_byte(&data[0]);  // LPM works!
}

// ✅ BEST - Avoid PROGMEM for strings in SimulIDE:
void test_no_progmem(void) {
    char buffer[] = "Test String";  // RAM instead of PROGMEM
    // Works in SimulIDE, but uses more RAM
}
```

---

## Migration Guide

### When to Use SimulIDE 0.4.15

**Use Cases:**
- ✅ Timer0 CTC mode demonstrations
- ✅ Timer0 PWM applications
- ✅ Stable classroom environment
- ✅ Projects using only basic I/O, Timer0, Timer1, ADC, SPI, I2C

**Limitations:**
- ❌ No ELPM instruction support
- ❌ UART1/printf() unavailable (must disable)
- ❌ Limited to 64KB program space

**Recommended Projects:**
- Port_Basic, Port_Assembly
- ADC_Basic, CDS_Light_Sensor
- SPI_Master_Basic, SPI_EEPROM_Memory, SPI_Multi_Device
- I2C_Master_Basic, I2C_RTC_DS1307, I2C_Sensors_Multi
- PWM_Motor_DC, PWM_Motor_Servo, PWM_Motor_Stepper
- LCD_Character_Basic, LCD_Advanced_Features
- Keypad_Matrix_Basic, Keypad_Calculator_App

---

### When to Use SimulIDE 1.1.0-SR1

**Use Cases:**
- ✅ ELPM instruction testing
- ✅ UART/printf() debugging
- ✅ Projects requiring serial output
- ✅ Advanced memory access demonstrations

**Limitations:**
- ❌ Timer0 CTC mode crashes (use Timer1)
- ❌ Timer0 PWM crashes (use Timer1)
- ⚠️ Beta quality - may have other undiscovered bugs

**Recommended Projects:**
- ELPM_Test, Atmega128_Instructions
- Serial_Communications
- Timer_Programming (with Timer0+Timer1 hybrid)
- Any project needing printf() debugging

---

### Version Selection Decision Tree

```
Start
  │
  ├─ Need ELPM instruction?
  │    Yes → SimulIDE 1.1.0-SR1
  │    No → Continue
  │
  ├─ Need UART/printf() output?
  │    Yes → SimulIDE 1.1.0-SR1
  │    No → Continue
  │
  ├─ Need Timer0 CTC or PWM?
  │    Yes → SimulIDE 0.4.15
  │    No → Either version (prefer 0.4.15 for stability)
```

---

## Bug Reporting

### Reporting New Issues

If you encounter new bugs in SimulIDE:

1. **Verify it's a SimulIDE bug:**
   - Test on real ATmega128 hardware
   - Compare behavior with datasheet specifications
   - Check if workaround with different peripheral works

2. **Document the bug:**
   - SimulIDE version number
   - Minimal reproducible code
   - Expected vs actual behavior
   - System information (OS, CPU)

3. **Report to:**
   - SimulIDE GitHub: https://github.com/SimulIDE/SimulIDE
   - Course instructor for project-specific workarounds

---

## Summary Table

| Issue                    | 0.4.15  | 1.1.0-SR1 | Workaround                    |
|--------------------------|---------|-----------|-------------------------------|
| Timer0 Normal Mode       | ✅ Works | ✅ Works   | None needed                   |
| Timer0 CTC Mode          | ✅ Works | ❌ CRASH   | Use Timer1 CTC                |
| Timer0 PWM Mode          | ✅ Works | ❌ CRASH   | Use Timer1 PWM                |
| UART1 Initialization     | ❌ HANG  | ✅ Fixed   | Disable in 0.4.15             |
| ELPM Instruction         | ❌ CRASH | ⚠️ Buggy  | Use -mshort-calls flag        |
| Timer1 All Modes         | ✅ Works | ✅ Works   | None needed                   |
| SPI, I2C, ADC            | ✅ Works | ✅ Works   | None needed                   |

---

## Recommendations

### For Instructors:
1. **Use SimulIDE 1.1.0-SR1** as default (with -mshort-calls for PROGMEM projects)
2. **Fall back to 0.4.15** only for Timer0 CTC/PWM projects
3. **Always use -mshort-calls flag** for projects with PROGMEM strings
4. **Document compiler flags** in each project README
5. **Test all demos** before lab sessions in target simulator version

### For Students:
1. **Check project README** for recommended SimulIDE version
2. **Keep both versions installed** for flexibility
3. **Report unexpected crashes** to instructor
4. **Use Timer1** as fallback if Timer0 issues occur
5. **Test on hardware** when possible for validation

### For Future Development:
1. **Monitor SimulIDE releases** for Timer0 bug fixes
2. **Update projects** when bugs are resolved
3. **Document version requirements** in all project headers
4. **Maintain legacy compatibility** with 0.4.15
5. **Report bugs upstream** to SimulIDE developers

---

## Version History

| Date       | SimulIDE Version | Major Changes                           |
|------------|------------------|-----------------------------------------|
| 2024-2025  | 0.4.15           | Stable baseline, UART1/ELPM broken      |
| 2025-11-02 | 1.1.0-SR1        | UART1/ELPM fixed, Timer0 CTC/PWM broken |

---

## Contact

**Course:** SOC 3050 - Embedded Systems and Applications  
**Instructor:** Professor Hong Jeong  
**Repository:** https://github.com/gnoejh/soc3050code  
**Documentation:** W:\soc3050code\docs\

---

*Last Updated: November 2, 2025*  
*Verified with: SimulIDE 0.4.15 and SimulIDE 1.1.0-SR1_Win64*
