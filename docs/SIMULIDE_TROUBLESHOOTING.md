# SimulIDE Troubleshooting Guide - ATmega128 Projects

**Course:** SOC 3050 - Embedded Systems and Applications  
**Target Hardware:** ATmega128 @ 16MHz  
**Simulator:** SimulIDE (versions 0.4.15 and 1.1.0-SR1)  
**Last Updated:** December 1, 2025

---

## Document Purpose

This is the **single source of truth** for all SimulIDE bugs, patches, and workarounds for ATmega128 projects. 

**Related Documents:**
- `simulide/SIMULIDE_PATCHES_QUICK_REF.md` - Quick patch restoration guide only
- `simulide/SIMULIDE_1.1.0_PATCHES_APPLIED.md` - Historical patch implementation details

**Use this document for:**
- Understanding bug root causes and technical details
- Implementing workarounds in projects
- Reporting issues to SimulIDE developers
- Version selection for specific features

---

## Table of Contents

1. [SimulIDE Version Comparison](#simulide-version-comparison)
2. [Known Bugs and Workarounds](#known-bugs-and-workarounds)
3. [Fixed Issues in SimulIDE 1.1.0-SR1](#fixed-issues-in-simulide-110-sr1)
4. [Active Bugs in SimulIDE 1.1.0-SR1](#active-bugs-in-simulide-110-sr1)
   - [Timer0 Type 821 CTC Mode Bug](#-timer0-type-821-ctc-mode-bug-critical)
   - [Why XML Patches Don't Work](#-why-xml-patches-dont-work-for-timer0)
   - [Workaround Strategies](#workaround-strategies)
5. [Project-Specific Workarounds](#project-specific-workarounds)
6. [Reporting Bugs to SimulIDE Developers](#reporting-bugs-to-simulide-developers)
7. [Patch Files Reference](#patch-files-reference)
8. [Quick Reference Matrix](#quick-reference-matrix)
9. [Verification Procedures](#verification-procedures)
10. [Migration Guide](#migration-guide)
11. [Version History](#version-history)
12. [Summary for Developers](#summary-for-developers)

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

#### 4. ELPM Instruction Crash (SimulIDE 0.4.15)
**Status:** ✅ **FIXED** in SimulIDE 1.1.0-SR1  
**Affected Projects:** ELPM_Test, Atmega128_Instructions  
**Symptoms:**
- Executing ELPM (Extended Load Program Memory) crashes simulator
- Immediate termination, no error message
- Any code beyond 64KB program space access fails

**Root Cause:**
- ELPM instruction not properly implemented in 0.4.15
- Extended addressing (RAMPZ register) handling broken
- ATmega128 specific feature, not tested in smaller AVRs

**Workaround (0.4.15):**
```c
// ❌ CRASHES in SimulIDE 0.4.15:
uint8_t read_extended_flash(uint32_t address) {
    RAMPZ = (address >> 16);
    return pgm_read_byte_far(address);  // Uses ELPM
}

// ✅ WORKAROUND for 0.4.15 - Avoid extended memory:
// Limit program to < 64KB
// Use only LPM instruction (16-bit addressing)
```

**Fix Status:**
- ✅ **Completely fixed in SimulIDE 1.1.0-SR1**
- ELPM instruction works correctly
- Full 128KB flash access functional
- RAMPZ register properly simulated

---

## Fixed Issues in SimulIDE 1.1.0-SR1

### ✅ ELPM Instruction Support
**Benefit:** Full ATmega128 memory space accessible  
**Projects Enabled:**
- ELPM_Test: Demonstrates extended memory access
- Atmega128_Instructions: Complete instruction set testing
- Large projects: Can now exceed 64KB code size

**Verification:**
```c
// Test ELPM functionality
#include <avr/pgmspace.h>

const char test_string[] PROGMEM = "ELPM Works!";

void test_elpm(void) {
    uint32_t addr = (uint32_t)test_string;
    RAMPZ = (addr >> 16);
    
    for(uint8_t i = 0; i < 11; i++) {
        char c = pgm_read_byte_far(addr + i);
        // Should not crash, returns correct characters
    }
}
```

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

### ❌ Timer0 Type 821 CTC Mode Bug (CRITICAL)

**Status:** ❌ **ACTIVE BUG** - Cannot be fixed with XML patches  
**Bug ID:** Timer0 Type 821 Implementation Bug  
**Discovered:** November 2, 2025  
**Analyzed:** December 1, 2025  
**Affected Projects:** Timer0_CTC_Precision, Timer_Programming (Demos 3-4)

---

#### Technical Details

**Root Cause:**
SimulIDE's C++ implementation of **type 821 timers** has a critical bug in CTC mode handling. When `WGM01=1` is set in `TCCR0`, the simulator crashes.

**Why XML Patches Don't Work:**
Unlike ELPM and UART1 bugs (which were XML configuration issues), this is a **compiled C++ code bug** in SimulIDE's timer simulation engine.

**Timer Type Architecture:**
```
Type 800: Modern 8-bit timer (ATmega328)  - TCCR0A + TCCR0B (✅ Works)
Type 801: Legacy 8-bit timer (ATmega8)    - TCCR0 single   (⚠️ Limited)
Type 820: Modern async timer (Timer2)     - TCCR2A + TCCR2B (✅ Works)  
Type 821: Legacy 8-bit timer (ATmega128)  - TCCR0 single   (❌ CTC Bug)
Type 160: 16-bit timer (Timer1/3)         - TCCR1A + TCCR1B (✅ Works)
```

**ATmega128 Timer Configuration:**
- Timer0: Type 821 (8-bit, single TCCR0 register)
- Timer1: Type 160 (16-bit, dual registers)
- Timer2: Type 821 (8-bit, single TCCR2 register)
- Timer3: Type 160 (16-bit, dual registers)

**File Location:**
```
SimulIDE_1.1.0-SR1_Win64/data/AVR/mega64_128/mega64_perif.xml
Line 28-37: Timer0 definition with type="821"
```

**XML Configuration (Current):**
```xml
<!-- Type 821 has CTC bug in SimulIDE - use Timer1 instead for CTC/PWM demos -->
<timer name="TIMER0" type="821" configregsA="TCCR0"
                counter="TCNT0"
                interrupt="T0_OVF"
                prescalers="0,1,8,32,64,128,256,1024" 
                prselect="CS00,CS01,CS02" >
    <ocunit name="OC0" pin="PORTB4" ocreg="OCR0" bits="COM00,COM01" interrupt="T0CO" />
</timer>
```

---

#### Bug Symptoms

**Crash Behavior:**
- SimulIDE application immediately terminates (no error message)
- Occurs the moment `WGM01=1` is written to `TCCR0`
- Also crashes when setting `OCR0` in CTC mode
- No recovery possible - simulator must be restarted

**Code That Triggers Crash:**
```c
// ❌ CRASHES SimulIDE 1.1.0-SR1:
void timer0_ctc_init(void) {
    TCCR0 = (1<<WGM01) | (1<<CS01);  // CTC mode, prescaler /8
    OCR0 = 249;                       // Compare value
    TIMSK |= (1<<OCIE0);             // Enable compare interrupt
}
```

**Code That Works:**
```c
// ✅ WORKS in SimulIDE 1.1.0-SR1:
void timer0_normal_init(void) {
    TCCR0 = (1<<CS01);               // Normal mode, prescaler /8
    TIMSK |= (1<<TOIE0);             // Enable overflow interrupt
}
```

---

#### Affected Modes and Registers

**Timer0 Mode Compatibility:**

| Mode              | Timer0 (0.4.15) | Timer0 (1.1.0-SR1) | Timer1 (All Versions) |
|-------------------|-----------------|--------------------|-----------------------|
| Normal/Overflow   | ✅ Works        | ✅ Works           | ✅ Works              |
| CTC               | ✅ Works        | ❌ **CRASH**       | ✅ Works              |
| Fast PWM          | ✅ Works        | ❌ **CRASH**       | ✅ Works              |
| Phase Correct PWM | ✅ Works        | ❌ **CRASH**       | ✅ Works              |

**TCCR0 Register Bits (ATmega128):**
```
Bit:     7      6      5      4      3      2      1      0
       FOC0   WGM00  COM01  COM00  WGM01  CS02   CS01   CS00
Status: ✅     ⚠️     ❌     ❌     ❌     ✅     ✅     ✅

✅ Safe: Clock select bits (CS02:00) work correctly
⚠️ Partial: WGM00 works only when WGM01=0 (Normal/PWM split)
❌ Crash: WGM01, COM01, COM00 trigger crashes in CTC/PWM modes
```

**Waveform Generation Mode (WGM) Table:**
```
WGM01  WGM00  Mode                    Status in 1.1.0-SR1
  0      0    Normal (Overflow)       ✅ Works
  0      1    PWM, Phase Correct      ❌ Crashes
  1      0    CTC (OCR0 top)          ❌ Crashes
  1      1    Fast PWM                ❌ Crashes
```

**Affected Registers:**
- `TCCR0`: Cannot set WGM01=1 or COM bits in CTC/PWM modes
- `OCR0`: Setting compare value crashes in CTC/PWM (but works in Normal mode as disconnected register)
- `TCNT0`: Read/write works in all modes
- `TIMSK`: OCIE0 enable works, but comparison never happens (crash before use)
- `TIFR`: OCF0 flag never sets (crash before comparison)

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

### Timer0_CTC_Precision Project

**Status:** ⚠️ **Hardware-only project** (SimulIDE 1.1.0-SR1 incompatible)

**Description:** Educational project demonstrating Timer0 CTC mode for precision timing applications

**SimulIDE Compatibility:**
- SimulIDE 1.1.0-SR1: ❌ Crashes immediately on Timer0 CTC mode
- SimulIDE 0.4.15: ✅ Should work (not fully tested)
- Real Hardware: ✅ Works perfectly

**Project Files:**
- `projects/Timer0_CTC_Precision/Main.c` - Complete with 4 CTC demos
- `projects/Timer0_CTC_Precision/Slide.md` - Educational presentation
- `projects/Timer0_CTC_Precision/README.md` - Documentation with formulas

**Testing Recommendations:**
1. **Hardware Testing:** Use with real ATmega128 board (COM3)
2. **Simulation Testing:** Use Timer1_CTC_Precision project instead
3. **Legacy Simulation:** Try SimulIDE 0.4.15 if needed

**Documentation Notes:**
Project README and Main.c clearly document the SimulIDE incompatibility and recommend Timer1_CTC_Precision for simulation.

---

### ❌ Why XML Patches Don't Work for Timer0

**Comparison with Fixed Bugs:**

| Bug Type | Root Cause | Fix Method | Can Be Patched? |
|----------|------------|------------|-----------------|
| **ELPM Instruction** | Missing RAMPZ register definition in XML | Add proper `<register>` tag in `mega128.mcu` | ✅ Yes - XML config |
| **UART1 Missing** | Peripheral not defined in XML | Add `<usart>` peripheral in `mega64_perif.xml` | ✅ Yes - XML config |
| **UART1 Register Mapping** | Wrong configregsA assignment | Fix `configregsA="UCSR1A"` in XML | ✅ Yes - XML config |
| **Timer0 CTC Mode** | C++ timer821 simulation code bug | Requires source code modification | ❌ No - Compiled code |

**Why ELPM/UART Patches Worked:**
```xml
<!-- ELPM Fix: XML register definition -->
<regblock name="I/O_REG" start="0x003B" end="0x003B" offset="32">
  <register name="RAMPZ" addr="0x003B" bits="" />
</regblock>

<!-- UART1 Fix: XML peripheral addition -->
<usart name="USART1" number="2" 
       configregsA="UCSR1A" configregsB="UCSR1B" configregsC="UCSR1C"
       interrupt="USART1_U">
  <trunit type="tx" pin="PORTD3" register="UDR1" interrupt="USART1_T" />
  <trunit type="rx" pin="PORTD2" interrupt="USART1_R" />
</usart>
```

**Why Timer0 CTC Can't Be Patched:**
```cpp
// SimulIDE C++ source (pseudo-code showing the bug location)
// File: src/simulator/elements/processors/avr/avrTimer821.cpp

void AvrTimer821::setCTC_Mode() {
    if (m_WGM01 == 1) {
        // BUG: Incorrect CTC mode implementation causes crash
        // This code is COMPILED into simulide.exe
        // Cannot be fixed with XML configuration
        m_compareMatch = ???;  // Broken logic here
    }
}
```

**What Would Be Needed to Fix Timer0:**
1. **Access to SimulIDE source code** (C++ files)
2. **Fix bug in `avrTimer821.cpp`** or equivalent timer type 821 implementation
3. **Recompile SimulIDE** from source
4. **Distribute patched executable** (or wait for official fix)

**XML Limitation:**
XML files in `data/AVR/` only define:
- Register addresses and names
- Peripheral connections and pins
- Interrupt vectors
- Configuration parameters

XML **cannot** change:
- Timer counting logic
- Mode switching behavior  
- Interrupt generation timing
- Compare/match algorithms

---

### Workaround Strategies

#### Strategy 1: Use Timer1 for CTC/PWM (Recommended)
**Advantages:**
- ✅ Works in all SimulIDE versions
- ✅ 16-bit precision (0-65535 vs 0-255)
- ✅ More output compare units (3 vs 1)
- ✅ Input capture capability
- ✅ Same code structure as Timer0

**Code Conversion:**
```c
// Timer0 (broken in SimulIDE 1.1.0-SR1):
TCCR0 = (1<<WGM01) | (1<<CS01);  // CTC, /8
OCR0 = 249;                       // 8kHz
TIMSK |= (1<<OCIE0);

// Timer1 (works everywhere):
TCCR1A = 0;
TCCR1B = (1<<WGM12) | (1<<CS11);  // CTC, /8
OCR1A = 249;                       // 8kHz
TIMSK |= (1<<OCIE1A);
```

**When to Use:**
- Educational projects (Timer_Programming)
- Simulation-required projects
- Projects needing >255 range
- Projects with multiple compare channels

---

#### Strategy 2: Use SimulIDE 0.4.15 (Legacy Support)
**Advantages:**
- ✅ Timer0 CTC/PWM modes work correctly
- ✅ All timer features functional
- ✅ Stable for basic peripherals

**Disadvantages:**
- ❌ ELPM instruction crashes
- ❌ UART1 initialization hangs
- ❌ No printf() debugging

**When to Use:**
- Timer0-specific educational demos
- Projects not using UART or ELPM
- Verifying Timer0 behavior
- Comparing simulator versions

---

#### Strategy 3: Hardware-Only Testing
**Advantages:**
- ✅ All features work (no simulator bugs)
- ✅ Real-world timing accuracy
- ✅ Validates production code
- ✅ Students learn hardware debugging

**Disadvantages:**
- ❌ Requires physical hardware
- ❌ No visual waveform display
- ❌ Harder to debug timing issues
- ❌ Cannot test before lab session

**When to Use:**
- Final project validation
- Timer0_CTC_Precision project
- Production code verification
- Advanced student projects

---

#### Strategy 4: Hybrid Approach (Recommended for Course)
**Implementation:**
1. Use SimulIDE 1.1.0-SR1 for UART/ELPM projects
2. Use SimulIDE 0.4.15 for Timer0 CTC/PWM projects
3. Use Timer1 when possible for maximum compatibility
4. Test on hardware for final validation

**Project Documentation Template:**
````markdown
## SimulIDE Compatibility

| Version | Status | Notes |
|---------|--------|-------|
| 0.4.15  | ✅ Recommended | Timer0 works, no UART |
| 1.1.0   | ❌ Incompatible | Timer0 CTC crashes |
| Hardware| ✅ Fully Supported | All features work |

**Simulation:** Use SimulIDE 0.4.15 or use Timer1_CTC_Precision project
**Hardware:** Upload to ATmega128 board on COM3
````

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

### Testing ELPM Functionality

```c
// Test in SimulIDE 1.1.0-SR1
void test_elpm(void) {
    const uint8_t data[] PROGMEM = {1, 2, 3, 4, 5};
    uint32_t addr = (uint32_t)data;
    
    RAMPZ = (addr >> 16);
    uint8_t value = pgm_read_byte_far(addr);
    // ✅ Should work in 1.1.0-SR1
    // ❌ Crashes in 0.4.15
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
| ELPM Instruction         | ❌ CRASH | ✅ Fixed   | Avoid extended memory in 0.4.15 |
| Timer1 All Modes         | ✅ Works | ✅ Works   | None needed                   |
| SPI, I2C, ADC            | ✅ Works | ✅ Works   | None needed                   |

---

## Recommendations

### For Instructors:
1. **Default to SimulIDE 0.4.15** for most lab exercises
2. **Use SimulIDE 1.1.0-SR1** only for ELPM and UART-heavy projects
3. **Document version requirements** in each project README
4. **Prepare both versions** on lab computers
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

## Reporting Bugs to SimulIDE Developers

### Timer0 CTC Bug Report Template

If reporting this bug to SimulIDE developers, use the following information:

**Bug Report:**
```markdown
Title: ATmega128 Timer0 (Type 821) CTC Mode Crash

SimulIDE Version: 1.1.0-SR1 (Windows 64-bit)
MCU: ATmega128
Affected Timer: TIMER0 (Type 821 implementation)

Description:
SimulIDE crashes when Timer0 is configured for CTC mode (WGM01=1) on ATmega128.
The application terminates immediately without error message.

Steps to Reproduce:
1. Load ATmega128 MCU in SimulIDE 1.1.0-SR1
2. Execute code: TCCR0 = (1<<WGM01) | (1<<CS01);
3. Simulator crashes immediately

Expected Behavior:
Timer0 should operate in CTC mode, generating compare match interrupts when TCNT0 == OCR0

Actual Behavior:
SimulIDE application terminates (crash) the moment WGM01=1 is written to TCCR0

Workaround:
Use Timer1 (Type 160) for CTC mode, which works correctly

Additional Information:
- Timer0 Normal mode (WGM01=0, WGM00=0) works correctly
- Timer0 Type 821 CTC worked in SimulIDE 0.4.15
- ATmega32 also uses Type 821 timers and may have same issue
- Timer2 (also Type 821) likely has same bug

Configuration File:
SimulIDE_1.1.0-SR1_Win64/data/AVR/mega64_128/mega64_perif.xml
Line 30: <timer name="TIMER0" type="821" configregsA="TCCR0" ...>

Regression:
This is a regression from SimulIDE 0.4.15 where Timer0 CTC mode worked.

Related Links:
- ATmega128 Datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf
- Timer0 CTC Mode: Section 17.7.2 (Clear Timer on Compare Match)
```

**Where to Report:**
- **GitHub Issues:** https://github.com/SimulIDE/SimulIDE/issues
- **SimulIDE Forum:** https://simulide.forumotion.com/

**Attachments to Include:**
1. Minimal test project (Main.c with Timer0 CTC init)
2. SimulIDE circuit file (.simu)
3. Compiled hex file
4. Screenshot/video of crash (if possible)

**Minimal Test Code:**
```c
// Minimal Timer0 CTC crash test for ATmega128
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

int main(void) {
    DDRB |= (1<<PB0);  // LED output
    
    // ❌ This line crashes SimulIDE 1.1.0-SR1:
    TCCR0 = (1<<WGM01) | (1<<CS01);  // CTC mode, prescaler /8
    OCR0 = 249;                       // 8kHz frequency
    TIMSK |= (1<<OCIE0);             // Enable compare interrupt
    
    sei();
    
    while(1) {
        // Never reaches here - crashes on TCCR0 write
    }
}

ISR(TIMER0_COMP_vect) {
    PORTB ^= (1<<PB0);  // Toggle LED
}
```

---

## Patch Files Reference

### Successfully Applied Patches (SimulIDE 1.1.0-SR1)

#### 1. ELPM Instruction Fix
**File:** `data/AVR/mega128.mcu`  
**Backup:** `data/AVR/mega128modified.mcu`  
**Fix Type:** XML register definition  
**Status:** ✅ Applied and working

**What Was Fixed:**
```xml
<regblock  name="I/O_REG" start="0x003B" end="0x003B" offset="32">
  <register  name="RAMPZ" addr="0x003B" bits="" />
</regblock>
```

---

#### 2. UART1/USART Peripheral Fix
**File:** `data/AVR/mega64_128/mega64_perif.xml`  
**Backup:** `data/AVR/mega64_128/mega64_perif_modified.xml`  
**Fix Type:** XML peripheral addition and register mapping  
**Status:** ✅ Applied and working

**What Was Fixed:**
```xml
<usart name="USART0" number="1" configregsA="UCSR0A" configregsB="UCSR0B" configregsC="UCSR0C"
                     interrupt="USART0_U">
  <trunit type="tx" pin="PORTE1" register="UDR0" interrupt="USART0_T" />
  <trunit type="rx" pin="PORTE0"                 interrupt="USART0_R" />
</usart>

<usart name="USART1" number="2" configregsA="UCSR1A" configregsB="UCSR1B" configregsC="UCSR1C"
                     interrupt="USART1_U">
  <trunit type="tx" pin="PORTD3" register="UDR1" interrupt="USART1_T" />
  <trunit type="rx" pin="PORTD2"                 interrupt="USART1_R" />
</usart>
```

**Critical Detail:** `configregsA` must point to status register (UCSR_A), not config register (UCSR_C)

---

#### 3. Timer0 CTC Mode (NOT PATCHABLE)
**File:** N/A (compiled C++ code in simulide.exe)  
**Backup:** N/A  
**Fix Type:** Requires source code modification  
**Status:** ❌ Cannot be patched - bug in C++ implementation

**Why It Can't Be Patched:**
- Bug is in compiled timer821.cpp code, not XML configuration
- XML only defines registers/connections, not simulation logic
- Would require recompiling SimulIDE from source

**What Would Need to Be Fixed:**
```cpp
// SimulIDE source code location (approximate):
// src/simulator/elements/processors/avr/avrTimer821.cpp

void AvrTimer821::writeTCCR0(uint8_t value) {
    // BUG: This section crashes when WGM01=1
    if (value & (1<<WGM01)) {
        // Broken CTC mode implementation here
        initCTC_Mode();  // ← Likely crash point
    }
}
```

---

### Patch Restoration

If patches are lost after SimulIDE update/reinstall:

```powershell
# Restore ELPM patch
Copy-Item "tools\simulide\SimulIDE_1.1.0-SR1_Win64\data\AVR\mega128modified.mcu" `
          "tools\simulide\SimulIDE_1.1.0-SR1_Win64\data\AVR\mega128.mcu" -Force

# Restore UART patch  
Copy-Item "tools\simulide\SimulIDE_1.1.0-SR1_Win64\data\AVR\mega64_128\mega64_perif_modified.xml" `
          "tools\simulide\SimulIDE_1.1.0-SR1_Win64\data\AVR\mega64_128\mega64_perif.xml" -Force

# Verify patches
& "tools\simulide\verify-patches.ps1"
```

**⚠️ Important:** Never delete `*modified.xml` or `*modified.mcu` backup files!

---

## Quick Reference Matrix

### Bug Status Summary (December 1, 2025)

| Bug/Feature              | 0.4.15    | 1.1.0-SR1 | Fix Method | Can Be Patched? |
|--------------------------|-----------|-----------|------------|-----------------|
| Timer0 Normal Mode       | ✅ Works  | ✅ Works  | N/A        | N/A             |
| Timer0 CTC Mode          | ✅ Works  | ❌ CRASH  | ⚠️ Source code | ❌ No (C++ bug) |
| Timer0 PWM Mode          | ✅ Works  | ❌ CRASH  | ⚠️ Source code | ❌ No (C++ bug) |
| Timer1 All Modes         | ✅ Works  | ✅ Works  | N/A        | N/A             |
| UART1 Initialization     | ❌ HANG   | ✅ Fixed  | XML config | ✅ Yes (Applied) |
| ELPM Instruction         | ❌ CRASH  | ✅ Fixed  | XML config | ✅ Yes (Applied) |
| SPI, I2C, ADC            | ✅ Works  | ✅ Works  | N/A        | N/A             |

---

## Version History

| Date       | SimulIDE Version | Major Changes                                    | Patches Applied           |
|------------|------------------|--------------------------------------------------|---------------------------|
| 2024-2025  | 0.4.15-SR10      | Stable baseline, UART1/ELPM broken              | None                      |
| 2025-10-18 | 1.1.0-SR1        | UART1/ELPM fixed, Timer0 CTC/PWM broken (regression) | ELPM + UART XML patches |
| 2025-11-02 | 1.1.0-SR1        | Timer0 bug documented, workarounds implemented   | No new patches            |
| 2025-12-01 | 1.1.0-SR1        | Comprehensive Timer0 analysis, patch comparison  | No new patches            |

---

## Document Maintenance

**Primary Maintainer:** Professor Hong Jeong  
**Last Comprehensive Update:** December 1, 2025  
**Review Frequency:** After each SimulIDE release or bug discovery

**When to Update This Document:**
1. New SimulIDE version released
2. New bugs discovered in existing versions
3. Patches successfully applied
4. Workarounds developed for projects
5. Upstream bug fixes confirmed

**Related Documentation:**
- `simulide/SIMULIDE_PATCHES_QUICK_REF.md` - Quick patch restore guide
- `simulide/SIMULIDE_1.1.0_PATCHES_APPLIED.md` - Historical patch details
- Each project's README.md - Project-specific compatibility notes

---

## Summary for Developers

### What Works and What Doesn't

**SimulIDE 1.1.0-SR1 (Current Recommended for Most Projects):**
- ✅ **WORKS:** Timer0 Normal mode, Timer1 all modes, UART0/1, ELPM, SPI, I2C, ADC, GPIO
- ❌ **BROKEN:** Timer0 CTC mode, Timer0 PWM modes (crashes)
- ⚠️ **PATCHED:** ELPM instruction (XML fix), UART1 peripheral (XML fix)

**SimulIDE 0.4.15 (Legacy Support for Timer0):**
- ✅ **WORKS:** Timer0 all modes, Timer1 all modes, SPI, I2C, ADC, GPIO
- ❌ **BROKEN:** UART1 initialization (hangs), ELPM instruction (crashes)
- ⚠️ **NO PATCHES:** Use as-is with limitations

**Hardware (Real ATmega128):**
- ✅ **ALL FEATURES WORK PERFECTLY** - no simulator bugs

### Decision Tree for Project Development

```
Need Timer0 CTC/PWM?
├─ YES → Use SimulIDE 0.4.15 OR use Timer1 OR test on hardware only
└─ NO  → Use SimulIDE 1.1.0-SR1 (best UART/ELPM support)

Need UART printf debugging?
├─ YES → Must use SimulIDE 1.1.0-SR1 (UART patched)
└─ NO  → Can use either version

Need ELPM (>64KB flash)?
├─ YES → Must use SimulIDE 1.1.0-SR1 (ELPM patched)
└─ NO  → Can use either version

Need everything working?
└─ Test on real hardware (no simulator limitations)
```

### Bug Severity Classification

**🔴 CRITICAL (Crash/Data Loss):**
- Timer0 CTC mode crash (1.1.0-SR1)
- Timer0 PWM mode crash (1.1.0-SR1)
- ELPM instruction crash (0.4.15) - ✅ Fixed in 1.1.0
- UART1 hang (0.4.15) - ✅ Fixed in 1.1.0

**🟡 HIGH (Feature Unavailable):**
- Timer2 CTC mode (likely same as Timer0, untested)
- Timer2 PWM modes (likely same as Timer0, untested)

**🟢 LOW (Minor Issues):**
- None currently documented

---

## Contact and Support

**Course:** SOC 3050 - Embedded Systems and Applications  
**Instructor:** Professor Hong Jeong  
**Institution:** Kangwon National University  
**Repository:** https://github.com/gnoejh/soc3050code  
**Documentation Path:** `W:\soc3050code\docs\`

**For Questions:**
1. Check this document first (single source of truth)
2. Check project-specific README files
3. Consult `simulide/SIMULIDE_PATCHES_QUICK_REF.md` for patch issues
4. Contact instructor for course-specific issues
5. Report new bugs via GitHub Issues

**For SimulIDE Bug Reports:**
- GitHub: https://github.com/SimulIDE/SimulIDE/issues
- Forum: https://simulide.forumotion.com/

---

*Last Updated: December 1, 2025*  
*SimulIDE Versions Tested: 0.4.15-SR10, 1.1.0-SR1*  
*ATmega128 @ 16MHz*  
*Status: ✅ ELPM Fixed | ✅ UART Fixed | ❌ Timer0 CTC Broken*
