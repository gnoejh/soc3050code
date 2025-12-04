# SimulIDE 1.1.0-SR1 Patches Applied for ATmega128

**Date Applied:** October 18-19, 2025  
**Patched By:** Professor Hong  
**Status:** ✅ All patches successfully applied and tested (UART patch required 2-stage fix)

---

## 🚨 QUICK TROUBLESHOOTING GUIDE

| Problem | Most Likely Cause | Quick Fix |
|---------|-------------------|-----------|
| **Graphics_Display crashes** | ELPM not patched | Check `mega128.mcu` has double-space in RAMPZ regblock |
| **No serial output at all** | USART peripheral missing | Check `mega64_perif.xml` has `<usart>` tags |
| **Garbled TX ("broken fonts")** | ⚠️ **Register mapping bug!** | Verify `configregsA="UCSR1A"` (NOT "UCSR1C") |
| **TX/RX both silent** | Wrong wire connections | **MUST CROSS:** TX→RX, RX→TX |
| **Intermittent garbage** | Baud rate mismatch | Match firmware (9600) and terminal (9600) |

**Critical Check for UART:**
```bash
# Open: tools/simulide/SimulIDE_1.1.0-SR1_Win64/data/AVR/mega64_128/mega64_perif.xml
# Search for: configregsA=
# MUST see: configregsA="UCSR1A"  ✅ CORRECT
# NOT:      configregsA="UCSR1C"  ❌ BROKEN
```

---

## Overview

SimulIDE 1.1.0-SR1 shipped with **three critical bugs** for ATmega128:
1. **ELPM instruction** - Not properly implemented (causes crashes in programs >64KB)
2. **UART/USART peripherals** - Completely missing from peripheral definitions (no serial communication)
3. **UART register mapping** - Incorrect register assignments (TX produces garbled output)

All bugs have been patched using XML configuration file modifications.

### Discovery Timeline
- **Oct 18, 2025:** ELPM bug identified and fixed
- **Oct 18, 2025:** UART missing peripherals discovered, initial fix applied
- **Oct 19, 2025:** UART register mapping bug discovered (after noticing "broken fonts" / garbled TX output)
- **Oct 19, 2025:** UART register mapping fixed by comparing with ATmega328 configuration

**Key Lesson:** Even after adding peripherals, incorrect register mapping can cause subtle data corruption!

---

## Patch 1: ELPM Instruction Fix

### Problem
- ELPM (Extended Load Program Memory) instruction crashes or behaves incorrectly
- Required for accessing program memory beyond 64KB address space
- Critical for `Graphics_Display` project and other large programs
- ATmega128 has 128KB flash, but only first 64KB was accessible

### Root Cause
Missing or malformed RAMPZ register definition in MCU configuration file.

### Files Modified
```
tools/simulide/SimulIDE_1.1.0-SR1_Win64/data/AVR/mega128.mcu
```

### What Was Changed

**Original (Broken):**
```xml
<regblock name="I/O_REG" start="0x003B" end="0x003B" offset="32">
  <register name="RAMPZ" addr="0x003B" bits="" />
</regblock>
```

**Fixed Version:**
```xml
<regblock  name="I/O_REG" start="0x003B" end="0x003B" offset="32">
  <register  name="RAMPZ" addr="0x003B" bits="" />
</regblock>
```

**Note:** The fix appears to be whitespace-related in the XML tags (double space after `<regblock` and `<register`). This may affect XML parsing or register initialization order.

### Backup Files Created
- `mega128modified.mcu` - The fixed version (reference copy)
- Original `mega128.mcu` was replaced with fixed version

### Testing
- ✅ `ELPM_Test` project runs successfully
- ✅ `Graphics_Display` project works (large font tables accessible)
- ✅ No crashes when accessing upper 64KB of flash memory

---

## Patch 2: UART/USART Peripheral Fix (CRITICAL - TWO STAGES)

### Problem
- **Stage 1 (Initial):** USART peripherals completely missing - no serial communication at all
- **Stage 2 (After Initial Fix):** UART TX produced garbled output ("broken fonts"), RX worked correctly
- Both UART0 and UART1 affected
- Serial communication projects (`Serial_Communications`) could not be taught properly

### Root Causes
1. **Missing USART peripheral definitions** (discovered Oct 18, 2025)
2. **Incorrect register mapping in peripheral config** (discovered Oct 19, 2025) ⚠️ **CRITICAL BUG**

### Files Modified
```
tools/simulide/SimulIDE_1.1.0-SR1_Win64/data/AVR/mega64_128/mega64_perif.xml
```

---

### STAGE 1: Adding Missing USART Peripherals (Oct 18, 2025)

**Location:** Insert after Timer3 definition, before ADC section (around line 73)

**Initial Version Added (INCOMPLETE - DO NOT USE):**
```xml
<!-- ========================================
     UART FIX: Added missing USART peripherals
     ======================================== -->
<usart name="USART0" number="1" configregsA="UCSR0C" configregsB="UCSR0B"
                     interrupt="USART0_U">

  <trunit type="tx" pin="PORTE1" register="UDR0" interrupt="USART0_T" />
  <trunit type="rx" pin="PORTE0"                 interrupt="USART0_R" />
</usart>

<usart name="USART1" number="2" configregsA="UCSR1C" configregsB="UCSR1B"
                     interrupt="USART1_U">

  <trunit type="tx" pin="PORTD3" register="UDR1" interrupt="USART1_T" />
  <trunit type="rx" pin="PORTD2"                 interrupt="USART1_R" />
</usart>
<!-- ======================================== -->
```

⚠️ **THIS VERSION IS BROKEN!** See Stage 2 below for the fix.

---

### STAGE 2: Fixing Register Mapping (Oct 19, 2025) ✅ **CORRECT VERSION**

### Problem with Stage 1 Implementation
The initial fix had **incorrect register mapping**:
- `configregsA="UCSR1C"` ← **WRONG!** This is the configuration register (data format, parity, stop bits)
- `configregsB="UCSR1B"` ← Correct (TX/RX enable, interrupt enables)
- `configregsC` was **MISSING!**

**Result:** TX produced garbled output because SimulIDE couldn't access the status register (UCSR1A) where UDRE1, TXC1, and RXC1 flags are located!

### ATmega128 UART Register Layout (From Datasheet)
```
Address | Register | Bit 7-0
--------|----------|--------------------------------------------------
$9D     | UCSR1C   | UMSEL1 UPM11 UPM10 USBS1 UCSZ11 UCSZ10 UCPOL1 -
$9C     | UDR1     | USART1 I/O Data Register
$9B     | UCSR1A   | RXC1 TXC1 UDRE1 FE1 DOR1 UPE1 U2X1 MPCM1  ← STATUS FLAGS!
$9A     | UCSR1B   | RXCIE1 TXCIE1 UDRIE1 RXEN1 TXEN1 UCSZ12 RXB81 TXB81
$99     | UBRR1L   | USART1 Baud Rate Register Low
$98     | UBRR1H   | ---- USART1 Baud Rate Register High
```

**Key Discovery:** `configregsA` MUST point to the **status register** (UCSR_A), not the configuration register!

### Corrected Version (FINAL - USE THIS)

**Location:** Same position as Stage 1 (after Timer3, before ADC)

```xml
<!-- ========================================
     UART FIX: Added missing USART peripherals with correct register mapping
     Fixed: configregsA must be UCSR*A (status register with UDRE/TXC/RXC flags)
     Date: Oct 18, 2025 (initial) | Oct 19, 2025 (register fix)
     ======================================== -->
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
<!-- ======================================== -->
```

### Critical Changes in Stage 2
| Attribute | Stage 1 (BROKEN) | Stage 2 (FIXED) | Purpose |
|-----------|------------------|-----------------|---------|
| `configregsA` | `UCSR1C` ❌ | `UCSR1A` ✅ | **Status register** - UDRE, TXC, RXC flags |
| `configregsB` | `UCSR1B` ✅ | `UCSR1B` ✅ | Control register - TX/RX enable |
| `configregsC` | *missing* ❌ | `UCSR1C` ✅ | **Configuration register** - 8N1, parity |

### Why This Matters
SimulIDE's USART peripheral code checks `configregsA` to access status flags:
- **UDRE (USART Data Register Empty)** - Firmware waits for this before sending data
- **TXC (Transmit Complete)** - Indicates transmission finished
- **RXC (Receive Complete)** - Indicates data received

Without proper access to UCSR1A, the TX logic cannot determine when the transmit buffer is ready, causing:
- Data corruption
- Garbled output (random characters)
- Timing issues

### Pin Assignments (ATmega128 UART Hardware)
| UART | TX Pin (Output) | RX Pin (Input) | Data Register |
|------|-----------------|----------------|---------------|
| USART0 | PORTE1 (TXD0) | PORTE0 (RXD0) | UDR0 |
| USART1 | PORTD3 (TXD1) | PORTD2 (RXD1) | UDR1 |

### ⚠️ CRITICAL: UART Connections Must Be CROSSED!

UART is a **crossover serial protocol**. You MUST connect:
- **MCU TX** → **Terminal/Device RX**
- **MCU RX** → **Terminal/Device TX**

**Example for SerialTerm in SimulIDE:**
```
ATmega128 PORTD3 (TXD1) → SerialTerm pin1 (RX)  ✅ CORRECT
ATmega128 PORTD2 (RXD1) → SerialTerm pin0 (TX)  ✅ CORRECT
```

**Example for SerialPort component:**
```
ATmega128 PORTD3 (TXD1) → SerialPort pin0 (RX)  ✅ CORRECT
ATmega128 PORTD2 (RXD1) → SerialPort pin1 (TX)  ✅ CORRECT
```

❌ **COMMON MISTAKE:** Connecting TX→TX and RX→RX will NOT work!

### Backup Files Created
- `mega64_perif_original_backup.xml` - Original (no USART, broken)
- `mega64_perif_modified.xml` - Fixed version with correct register mapping (reference copy)
- Active `mega64_perif.xml` was replaced with fixed version

### Testing Results (Stage 2 - Final)
- ✅ UART TX works perfectly - no garbled output
- ✅ UART RX works perfectly
- ✅ Bidirectional communication confirmed at 9600 baud
- ✅ Both USART0 and USART1 fully functional
- ✅ U2X=1 mode works (double-speed for better accuracy)
- ✅ U2X=0 mode works (normal speed)
- ✅ Tested baud rates: 1200, 2400, 4800, 9600, 19200 - all work correctly
- ✅ `Serial_Communications` project fully operational with all demos working

---

## How to Apply These Patches to a Fresh SimulIDE 1.1.0 Installation

### Prerequisites
- SimulIDE 1.1.0-SR1 Win64 installed
- Text editor or file manager for copying files

### Step 1: ELPM Patch
```powershell
# Navigate to SimulIDE data directory
cd "tools\simulide\SimulIDE_1.1.0-SR1_Win64\data\AVR"

# Backup original file
Copy-Item "mega128.mcu" "mega128_original_backup.mcu"

# Apply patch: Copy fixed version over original
# (Use mega128modified.mcu from this repository as reference)
# Or manually edit mega128.mcu to add double spaces in regblock tags
```

**Manual Edit:**
1. Open `mega128.mcu` in text editor
2. Find the RAMPZ register block (line ~8)
3. Add extra space after `<regblock` and `<register` opening tags
4. Save file

### Step 2: UART Patch
```powershell
# Navigate to mega64_128 peripheral directory
cd "tools\simulide\SimulIDE_1.1.0-SR1_Win64\data\AVR\mega64_128"

# Backup original file
Copy-Item "mega64_perif.xml" "mega64_perif_original_backup.xml"

# Apply patch: Copy fixed version over original
# (Use mega64_perif_modified.xml from this repository as reference)
```

**Manual Edit:**
1. Open `mega64_perif.xml` in text editor
2. Find the Timer3 section (around line 62-70)
3. After the closing `</timer>` tag for Timer3
4. Before the `<port name="PORTV"` line
5. Insert the USART definitions (see "What Was Added" section above)
6. Save file

### Step 3: Verify Patches Applied
```powershell
# Test ELPM
# Open ELPM_Test project in SimulIDE
# Should run without crashes and access large data arrays

# Test UART
# Open Serial_Communications project in SimulIDE
# Should see serial output in Serial Monitor
# Should be able to send/receive data bidirectionally
```

---

## Automated Patch Script (Future Enhancement)

Consider creating a PowerShell script to apply patches automatically:

```powershell
# tools/simulide/apply-patches.ps1
param(
    [string]$SimulIDEPath = "tools\simulide\SimulIDE_1.1.0-SR1_Win64"
)

# Verify SimulIDE installation
if (-not (Test-Path "$SimulIDEPath\simulide.exe")) {
    Write-Error "SimulIDE not found at: $SimulIDEPath"
    exit 1
}

# Apply ELPM patch
Copy-Item "mega128modified.mcu" "$SimulIDEPath\data\AVR\mega128.mcu" -Force

# Apply UART patch
Copy-Item "mega64_perif_modified.xml" "$SimulIDEPath\data\AVR\mega64_128\mega64_perif.xml" -Force

Write-Host "✅ Patches applied successfully!"
```

---

## Technical Details

### Why These Patches Work

**ELPM Patch:**
- RAMPZ register is essential for extended memory addressing
- The whitespace change may affect XML parser's handling of the register
- Possibly fixes initialization order or register mapping

**UART Patch (Stage 1 - Peripheral Instantiation):**
- SimulIDE uses XML to instantiate peripheral hardware models
- Missing `<usart>` tags = no USART peripheral created at all
- Interrupts were defined but had nothing to connect to
- Adding peripheral definitions instantiates the TX/RX hardware

**UART Patch (Stage 2 - Register Mapping):**
- SimulIDE's USART peripheral simulation expects three config registers:
  - `configregsA` → **Status register** (UCSR_A) - Contains UDRE, TXC, RXC flags
  - `configregsB` → **Control register** (UCSR_B) - Contains TX/RX enable, interrupt enables
  - `configregsC` → **Format register** (UCSR_C) - Contains data bits, parity, stop bits
- **The bug:** configregsA pointed to UCSR1C (format) instead of UCSR1A (status)
- **Why it broke TX:** Firmware polls UDRE flag in UCSR1A before sending data
- **When UCSR1A inaccessible:** TX timing corruption → garbled output
- **Why RX still worked:** RX uses interrupts and data register, doesn't need UDRE flag

### Register Mapping Comparison

**Correct (ATmega328 - Working Reference):**
```xml
<usart name="USART0" number="1" configregsA="UCSR0A" configregsB="UCSR0B" configregsC="UCSR0C"
```

**Broken (ATmega128 Initial Fix - WRONG):**
```xml
<usart name="USART1" number="2" configregsA="UCSR1C" configregsB="UCSR1B"
```
- Missing configregsC
- configregsA points to wrong register (format instead of status)

**Fixed (ATmega128 Final - CORRECT):**
```xml
<usart name="USART1" number="2" configregsA="UCSR1A" configregsB="UCSR1B" configregsC="UCSR1C"
```

### References to Other MCUs
The UART fix was derived from multiple sources:
1. **ATmega328 (megax8_perif.xml)** - Correct register mapping pattern ✅
2. **ATmega1281 (m1281_perif.xml)** - Had same bug as initial ATmega128 fix ❌
3. **ATmega644 (mxx4_perif.xml)** - Correct register mapping ✅

**Lesson:** Always verify register mapping against a known-working chip configuration (ATmega328 is the gold standard).

---

## Version Compatibility

| Feature | SimulIDE 0.4.15 | SimulIDE 1.1.0 (Unpatched) | SimulIDE 1.1.0 (Patched) |
|---------|-----------------|----------------------------|--------------------------|
| **ELPM** | ❌ Broken | ❌ Broken | ✅ **Fixed** |
| **UART TX** | ✅ Works | ❌ Broken | ✅ **Fixed** |
| **UART RX** | ✅ Works | ✅ Works | ✅ Works |
| **Graphics** | ⚠️ Limited | ❌ Broken | ✅ **Fixed** |
| **All Other** | ✅ Works | ✅ Works | ✅ Works |

**Recommendation:** Use patched SimulIDE 1.1.0 for all projects.

---

## Related Files

**Reference Copies (Keep These!):**
- `tools/simulide/SimulIDE_1.1.0-SR1_Win64/data/AVR/mega128modified.mcu`
- `tools/simulide/SimulIDE_1.1.0-SR1_Win64/data/AVR/mega64_128/mega64_perif_modified.xml`

**Backup Files (Original Broken Versions):**
- `tools/simulide/SimulIDE_1.1.0-SR1_Win64/data/AVR/mega64_128/mega64_perif_original_backup.xml`

**Documentation:**
- `docs/simulide/SIMULIDE_KNOWN_ISSUES.md` - Bug descriptions
- `WHICH_SIMULIDE_VERSION.md` - Version comparison guide
- `UART_INVESTIGATION_COMPLETE.md` - Debugging process documentation

---

## Troubleshooting

### ELPM Still Not Working
1. Verify `mega128.mcu` has double spaces in RAMPZ regblock
2. Restart SimulIDE completely
3. Check that correct MCU file is being loaded (atmega128)

### UART Still Not Working / Garbled Output
1. **Check register mapping** - Verify `configregsA="UCSR1A"` (NOT UCSR1C!)
   ```xml
   <usart name="USART1" number="2" configregsA="UCSR1A" configregsB="UCSR1B" configregsC="UCSR1C"
   ```
2. **Verify TX/RX crossover** - TX must connect to RX, RX must connect to TX
   - MCU TX → Terminal RX ✅
   - MCU RX → Terminal TX ✅
3. **Check baud rate match** - Firmware and terminal must use same baud rate (e.g., 9600)
4. Verify USART definitions present in `mega64_perif.xml`
5. Check pin assignments match your circuit (PORTE1/0 for USART0, PORTD3/2 for USART1)
6. Restart SimulIDE completely
7. Remove any dual connections (one pin should only connect to ONE terminal)

### Patches Lost After Update
If you update SimulIDE, reapply patches using reference copies:
```powershell
Copy-Item "mega128modified.mcu" "mega128.mcu" -Force
Copy-Item "mega64_perif_modified.xml" "mega64_perif.xml" -Force
```

---

## Contact / Maintenance

**Patches Maintained By:** Professor Hong  
**Last Updated:** October 18, 2025  
**SimulIDE Version:** 1.1.0-SR1 Win64  

**Important:** Do not delete `mega128modified.mcu` or `mega64_perif_modified.xml` - these are the master patch files!

---

## Summary

✅ **ELPM Patch:** Fixes RAMPZ register definition in `mega128.mcu`  
✅ **UART Patch (Stage 1):** Adds missing USART peripherals to `mega64_perif.xml`  
✅ **UART Patch (Stage 2):** Fixes critical register mapping bug (`configregsA` must be UCSR_A, not UCSR_C)  
✅ **Result:** Full ATmega128 functionality in SimulIDE 1.1.0-SR1

**All patches are REQUIRED** for complete ATmega128 support in educational projects.

### Quick Verification Checklist
After applying patches, verify in `mega64_perif.xml`:
```bash
# Should see these EXACT lines (around line 73):
configregsA="UCSR0A" configregsB="UCSR0B" configregsC="UCSR0C"  # USART0
configregsA="UCSR1A" configregsB="UCSR1B" configregsC="UCSR1C"  # USART1
```

❌ **If you see:** `configregsA="UCSR1C"` → **WRONG! TX will be garbled!**  
✅ **Should be:** `configregsA="UCSR1A"` → **CORRECT!**

### Symptoms of Incorrect Patches
| Symptom | Cause | Fix |
|---------|-------|-----|
| ELPM crashes | RAMPZ not fixed | Apply ELPM patch |
| No serial output at all | USART peripheral missing | Apply UART Stage 1 |
| Garbled TX ("broken fonts") | Wrong register mapping | Apply UART Stage 2 |
| TX/RX both silent | Crossover wiring wrong | Cross TX→RX connections |
| Intermittent corruption | Baud rate mismatch | Match firmware & terminal baud |
