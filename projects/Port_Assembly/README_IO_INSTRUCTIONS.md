# Port_Assembly Project - Essential I/O Instructions

## Overview
This project has been **completely rewritten** to focus exclusively on teaching the 6 most essential AVR assembly I/O instructions for port programming.

## The 6 Essential Instructions

### 1. **OUT** - Write to I/O Register
- **Cycles:** 1
- **Range:** I/O addresses 0x00-0x3F
- **Usage:** `OUT PORTB, r16` - Write register to I/O port
- **Demo:** `demo_01_out()` - Blink all LEDs on/off

### 2. **IN** - Read from I/O Register  
- **Cycles:** 1
- **Range:** I/O addresses 0x00-0x3F
- **Usage:** `IN r16, PIND` - Read I/O port into register
- **Demo:** `demo_02_in()` - Read button, display on LEDs

### 3. **SBI** - Set Bit in I/O (Atomic)
- **Cycles:** 2
- **Range:** I/O addresses 0x00-0x1F only
- **Atomic:** YES - Cannot be interrupted
- **Usage:** `SBI PORTB, 3` - Set bit 3 to 1
- **Demo:** `demo_03_sbi()` - Turn ON LEDs one by one

### 4. **CBI** - Clear Bit in I/O (Atomic)
- **Cycles:** 2
- **Range:** I/O addresses 0x00-0x1F only
- **Atomic:** YES - Cannot be interrupted
- **Usage:** `CBI PORTB, 3` - Clear bit 3 to 0
- **Demo:** `demo_04_cbi()` - Turn OFF LEDs one by one

### 5. **SBIC** - Skip if Bit in I/O is Clear
- **Cycles:** 1-3 (depends on skip)
- **Range:** I/O addresses 0x00-0x1F only
- **Usage:** `SBIC PIND, 7` - Skip next instruction if bit 7 = 0
- **Demo:** `demo_05_sbic()` - Button pressed detection

### 6. **SBIS** - Skip if Bit in I/O is Set
- **Cycles:** 1-3 (depends on skip)
- **Range:** I/O addresses 0x00-0x1F only
- **Usage:** `SBIS PIND, 7` - Skip next instruction if bit 7 = 1
- **Demo:** `demo_06_sbis()` - Button released detection

## Learning Progression

Start with `demo_01_out()` and progress through each demo:

```c
int main(void) {
    demo_01_out();    // ← START HERE: Learn OUT
    // demo_02_in();     // Learn IN
    // demo_03_sbi();    // Learn SBI
    // demo_04_cbi();    // Learn CBI
    // demo_05_sbic();   // Learn SBIC
    // demo_06_sbis();   // Learn SBIS
    
    return 0;
}
```

Uncomment one demo at a time, study the code, build and test on hardware.

## Key Concepts

### Atomic Operations (SBI/CBI)
- **Cannot be interrupted** - Safe for use in ISRs
- **Single operation** - No read-modify-write cycle
- **Fast** - Only 2 cycles

Compare to non-atomic:
```
IN r16, PORTB      ; Read (1 cycle)
ORI r16, 0x08      ; Modify (1 cycle)
OUT PORTB, r16     ; Write (1 cycle)
; Total: 3+ cycles, CAN be interrupted!
```

vs.

```
SBI PORTB, 3       ; Done! (2 cycles, ATOMIC)
```

### Address Range Limitations
- **OUT/IN:** Work with 0x00-0x3F (64 I/O registers)
- **SBI/CBI/SBIC/SBIS:** Work with 0x00-0x1F only (32 I/O registers)

ATmega128 I/O addresses:
- PORTB = 0x18 ✅ (works with all instructions)
- PORTC = 0x15 ✅ (works with all instructions)
- PORTD = 0x12 ✅ (works with all instructions)
- PORTE = 0x03 ✅ (works with all instructions)
- PORTF = 0x62 ❌ (SBI/CBI/SBIC/SBIS won't work, use IN/OUT)

### Skip Instructions (SBIC/SBIS)
These enable conditional logic in pure assembly without C if-statements:

```assembly
sbic PIND, 7         ; Skip next if button pressed (bit=0)
rjmp not_pressed     ; Jump if NOT pressed
; ... code for button pressed ...
not_pressed:
```

## Hardware Setup
- **MCU:** ATmega128 @ 16MHz
- **LEDs:** 8 LEDs on PORTB (PB0-PB7)
- **Button:** 1 button on PORTD.7 (active LOW, pull-up enabled)

## Build and Program
```powershell
# Build
.\cli-build-project.ps1 -ProjectDir "projects\Port_Assembly"

# Program
.\cli-program-project.ps1 -ProjectDir "projects\Port_Assembly" -Programmer arduino -Port COM3
```

## Next Steps
After mastering these 6 instructions:
1. Move to `Port_Configuration` for real-world applications
2. Study interrupt service routines (ISRs)
3. Explore timer/counter programming
4. Learn UART and other peripherals

## Why These 6 Instructions?
- **Foundation** - Every AVR program uses these
- **Efficient** - Fast execution (1-2 cycles)
- **Direct control** - No abstraction, pure hardware access
- **Industry standard** - Used in professional embedded systems
- **Interview questions** - Commonly asked in embedded job interviews

---

**Remember:** The best way to learn is by doing! Build each demo, observe the LEDs, press the button, and understand how each instruction works at the hardware level.
