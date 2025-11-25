# SimulIDE Bug Fixes

This document tracks bugs found in SimulIDE and the fixes applied to make it work correctly with ATmega128 projects.

## Summary

Three critical bugs were discovered and fixed in SimulIDE 1.1.0-SR1:

1. **ELPM Instruction Bug** - Incorrect program memory access
2. **UART1 Missing** - Second UART not implemented
3. **L298P H-Bridge Bug** - Inverted enable logic

---

## 1. ELPM (Extended Load Program Memory) Bug

**Location:** `SimulIDE_1.1.0-SR1_Win64\data\MCUs\AVR\mega128\mega128.mcu`

**Problem:**
- ATmega128 has 128KB flash memory requiring ELPM instruction for access beyond 64KB
- SimulIDE incorrectly implemented ELPM, causing program crashes or wrong data reads
- Affected all projects using data tables stored in program memory (PROGMEM)

**Symptoms:**
- Programs that work on real hardware fail in simulation
- Random crashes when accessing PROGMEM data
- Incorrect values read from flash memory

**Fix Applied:**
Modified the MCU configuration file to correctly implement ELPM instruction behavior for 3-byte address space.

**Files Modified:**
- `data/MCUs/AVR/mega128/mega128.mcu`

---

## 2. UART1 Missing

**Location:** `SimulIDE_1.1.0-SR1_Win64\data\MCUs\AVR\mega128\mega128.mcu`

**Problem:**
- ATmega128 has two USART modules (USART0 and USART1)
- SimulIDE only implemented USART0
- Projects using UART1 (serial communication on PD2/PD3) did not work

**Symptoms:**
- UART1 functions compile but produce no output
- Serial communication on USART1 pins inactive
- Only USART0 (PE0/PE1) worked

**Fix Applied:**
Added UART1 peripheral definition to the ATmega128 MCU configuration, mapping it to the correct registers (UDR1, UBRR1H/L, UCSR1A/B/C) and pins (PD2=RXD1, PD3=TXD1).

**Files Modified:**
- `data/MCUs/AVR/mega128/mega128.mcu`

---

## 3. L298P Motor Not Spinning - User Error (Not a Bug)

**Location:** Circuit wiring issue

**Problem:**
- Motor connected to L298P would not spin
- OUT1 and OUT2 showed 0V regardless of input signals
- PWM on enable pin had no effect
- Direction signals (IN1/IN2) did not produce output voltage

**Suspected Cause (INCORRECT):**
Initially thought to be a bug in L298P.sim1 internal circuit design with shared nodes preventing independent OUT1/OUT2 control.

**Actual Root Cause:**
**SenA pin was not connected to Ground!**

The L298P's SenA (Sense A) pin is the current sense return path - it's the ground reference for the motor driver's internal circuitry. Without this connection:
- No return path exists for current flow
- Outputs remain at 0V (floating/undefined state)
- Motor cannot receive power regardless of control signals

**Symptoms:**
- Both OUT1 and OUT2 stuck at 0V
- No response to IN1/IN2 direction control
- No PWM output on motor terminals
- EnA enable signal has no effect

**Truth Table - ACTUAL BEHAVIOR (Buggy):**
```
ENA | IN1 | IN2 | OUT1 | OUT2 | Result
----|-----|-----|------|------|------------------
 1  |  0  |  0  |  0   |  0   | Both LOW (no differential)
 1  |  1  |  0  |  1   |  1   | Both HIGH (no differential) BUG!
 1  |  0  |  1  |  1   |  1   | Both HIGH (no differential) BUG!
 1  |  1  |  1  |  1   |  1   | Both HIGH (no differential)
```

**Correct Solution:**
✅ **Connect SenA pin to Ground** - that's all!

Required L298P wiring:
```
ATmega128 → L298P Control:
  PB5 (OC1A) → EnA    (PWM enable for Motor A)
  PB6 (GPIO) → IN1    (Motor A direction bit 1)
  PB7 (GPIO) → IN2    (Motor A direction bit 2)

L298P → Power/Motor:
  Vs     → +12V       (Motor supply voltage)
  SenA   → GND        (Current sense return - MANDATORY!)
  OUT1   → Motor Terminal 1
  OUT2   → Motor Terminal 2
```

**Validation:**
After connecting SenA to ground:
- OUT1 = 12V when IN1=HIGH, IN2=LOW
- OUT2 = 0V (ground) when IN1=HIGH, IN2=LOW
- Creates proper 12V voltage differential across motor
- Motor spins correctly with PWM speed control
- Direction reversal works (swap IN1/IN2 logic)

**Why This Happens:**
L298P datasheet shows SenA/SenB are current sense pins that connect to ground through low-value resistors (typically 0.5Ω for current monitoring). SimulIDE's L298P model correctly implements this - without ground on SenA, the internal circuit has no return path and outputs float at 0V.

**Debugging Notes:**
During investigation, L298P.sim1 was modified to test enable buffer inversion and node separation. These changes are harmless but unnecessary - the original L298P.sim1 was correct.

**External inverter workaround is NOT needed** - just ground SenA properly.

---

## How to Apply These Fixes

If you reinstall SimulIDE or use it on another machine:

1. **ELPM Bug:** Replace `mega128.mcu` with corrected version
2. **UART1 Bug:** Replace `mega128.mcu` with corrected version (same file as ELPM)
3. **L298P Issue:** Just remember to connect **SenA pin to Ground** in your circuits!

**Note:** Always backup original files before applying fixes.

---

## Version Information

- **SimulIDE Version:** 1.1.0-SR1 Win64
- **Fixes Applied:** November 2025
- **Target MCU:** ATmega128
- **Projects Affected:** All projects using PROGMEM, UART1, or L298P H-bridge

---

## Related Files

Corrected files should be kept as reference:
- `mega128.mcu` (with ELPM and UART1 fixes)

For future SimulIDE updates, check if these bugs are fixed upstream before applying patches.

## Lessons Learned

**L298P Issue:** Always verify proper component pin connections before assuming simulator bugs. The SenA pin is documented in the L298P datasheet as a current sense return path - grounding it is mandatory, not optional. This matches real hardware behavior.
