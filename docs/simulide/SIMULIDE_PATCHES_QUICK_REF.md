# SimulIDE 1.1.0 Patch Quick Reference

**Last Updated:** October 18, 2025  
**Status:** ✅ All patches applied and tested

---

## What's Been Patched

1. ✅ **ELPM Instruction** - Fixed RAMPZ register definition
2. ✅ **UART/USART** - Added missing peripheral definitions

**Result:** SimulIDE 1.1.0-SR1 now fully supports ATmega128!

---

## Files That Were Modified

```
tools/simulide/SimulIDE_1.1.0-SR1_Win64/
├── data/AVR/
│   ├── mega128.mcu                     ← ELPM patch applied
│   ├── mega128modified.mcu             ← Reference copy (DO NOT DELETE)
│   └── mega64_128/
│       ├── mega64_perif.xml            ← UART patch applied
│       ├── mega64_perif_modified.xml   ← Reference copy (DO NOT DELETE)
│       └── mega64_perif_original_backup.xml
```

---

## If Patches Are Lost

**Symptom:** ELPM or UART stops working after update/reinstall

**Solution:** Re-copy the reference files:

```powershell
# ELPM Patch
Copy-Item "tools\simulide\SimulIDE_1.1.0-SR1_Win64\data\AVR\mega128modified.mcu" `
          "tools\simulide\SimulIDE_1.1.0-SR1_Win64\data\AVR\mega128.mcu" -Force

# UART Patch
Copy-Item "tools\simulide\SimulIDE_1.1.0-SR1_Win64\data\AVR\mega64_128\mega64_perif_modified.xml" `
          "tools\simulide\SimulIDE_1.1.0-SR1_Win64\data\AVR\mega64_128\mega64_perif.xml" -Force
```

---

## Full Documentation

See: **`docs/simulide/SIMULIDE_1.1.0_PATCHES_APPLIED.md`**

Contains complete technical details, step-by-step instructions, and troubleshooting guide.

---

## Testing Verification

**Test ELPM:**
```
Open: projects/ELPM_Test
Run in SimulIDE 1.1.0
Should: Access large data arrays without crashes
```

**Test UART:**
```
Open: projects/Serial_Communications  
Run in SimulIDE 1.1.0
Should: See bidirectional serial communication
```

---

## Important Notes

⚠️ **DO NOT DELETE** these reference files:
- `mega128modified.mcu`
- `mega64_perif_modified.xml`

These are your master patch copies!

---

**For detailed patch information, see:**
- `docs/simulide/SIMULIDE_1.1.0_PATCHES_APPLIED.md` (Complete guide)
- `docs/simulide/SIMULIDE_KNOWN_ISSUES.md` (Bug descriptions)
