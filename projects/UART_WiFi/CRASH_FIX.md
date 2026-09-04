# Quick Fix for Simulator Crash

## Problem
The simulator crashes because the full state machine tries to communicate with ESP-01 which isn't connected in the circuit.

## Solution: Use Simple Test Version

### Option 1: Use the Fixed Main.c (Already Done)
The Main.c now automatically falls back to bridge mode if ESP-01 doesn't respond:

```powershell
# Just rebuild and run
Ctrl+Shift+B

# Then simulate
.\tools\simulide\cli-simulide.ps1 -ProjectDir "projects\Bluethooth"
```

**What you'll see:**
```
[STATE] Testing AT...
[CMD] AT
[TIMEOUT] No ESP-01 detected
[INFO] Entering bridge mode for manual testing

=== BRIDGE MODE ACTIVE ===
```

Now you can type commands directly!

### Option 2: Use Ultra-Simple Version (No State Machine)

If it still crashes, use the minimal test:

**Step 1:** Temporarily rename files:
```powershell
cd projects\Bluethooth
mv Main.c Main_Full.c
mv Main_Simple.c Main.c
```

**Step 2:** Build and simulate:
```powershell
# Build
Ctrl+Shift+B

# Simulate
.\tools\simulide\cli-simulide.ps1 -ProjectDir "projects\Bluethooth"
```

**Step 3:** This version just echoes characters - no crashes!

---

## Why Did It Crash?

The original code had these issues:
1. **Infinite retry loop** - kept trying AT command forever
2. **No timeout escape** - `esp_wait_for_response()` blocked
3. **Empty UART buffers** - simulator may not like unconnected UARTs

## Now Fixed

✅ **Timeout detection** - stops after first timeout  
✅ **Fallback to bridge mode** - lets you test manually  
✅ **Simple version available** - minimal code for testing

## Test Now

```powershell
# Rebuild with fixed version
Ctrl+Shift+B

# Run simulator
.\tools\simulide\cli-simulide.ps1 -ProjectDir "projects\Bluethooth"
```

Should now show debug output on UART1 and wait in bridge mode!

## Next: Add Python Emulator

Once simulator runs, add the Python emulator for realistic testing:

```powershell
# Terminal 1: Start emulator
python tools\test_esp_emulator.py --port COM10

# Terminal 2: Start simulator  
.\tools\simulide\cli-simulide.ps1 -ProjectDir "projects\Bluethooth"
```

The emulator will respond to AT commands and you'll see the full state machine work!
