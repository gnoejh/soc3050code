# 🚀 Quick Start: Build and Simulate with TCP Bridge

## ✨ One-Click Solution

The "Build and Simulate Current Project" task now automatically:
1. ✅ Builds the firmware
2. ✅ Programs SimulIDE
3. ✅ Starts SimulIDE simulation
4. ✅ Starts TCP Serial Bridge (COM4 ↔ SimulIDE)

## 📋 How to Use

### Step 0: Remove com0com Port Pair (One-Time Setup)

**IMPORTANT**: If you were using com0com before, you must remove the COM4-COM5 pair first!

**Right-click and "Run as Administrator":**
```
tools\simulide\remove-com0com-pair.bat
```

Or manually:
1. Open "Setup for com0com" from Start Menu
2. Select the COM4-COM5 pair (Virtual Port Pair 0)
3. Click "Remove Pair"
4. Click "Apply"

### Step 1: Run the Task
Press `Ctrl+Shift+P` → Type "Run Task" → Select:
```
Build and Simulate Current Project
```

**Or** use the keyboard shortcut:
- Press `Ctrl+Shift+B`
- Select "Build and Simulate Current Project"

### Step 2: Wait for Everything to Start
You'll see three terminals:
1. **Build Output** - Compilation results
2. **SimulIDE** - Simulator window opens
3. **TCP Bridge** - Shows connection status

Wait for this message:
```
=== Bridge Running ===
✓ COM4 (Serial) ↔ localhost:1234 (TCP) ↔ SimulIDE
```

### Step 3: Open VS Code Serial Monitor
1. Click the **Serial Monitor** icon in VS Code status bar
2. Select Port: **COM4**
3. Set Baud rate: **9600**
4. Click "Start Monitoring"

### Step 4: Test Communication! 🎉
- Type messages in VS Code Serial Monitor
- They appear in SimulIDE Serial Monitor
- ATmega128 can send data back
- Bidirectional communication works!

## 🎯 What Happens Behind the Scenes

```
┌─────────────────────────────────────────────────────────┐
│ VS Code: "Build and Simulate Current Project"          │
└─────────────────────────────────────────────────────────┘
                        ↓
        ┌───────────────┴───────────────┐
        ↓                               ↓
┌───────────────┐              ┌────────────────┐
│ Build Project │              │ Start SimulIDE │
│ (AVR-GCC)     │              │ (Simulator)    │
└───────────────┘              └────────────────┘
        ↓                               ↓
┌───────────────┐              ┌────────────────┐
│ Main.hex      │──────────→   │ Load firmware  │
└───────────────┘              └────────────────┘
                                        ↓
                               ┌────────────────┐
                               │ Power ON       │
                               │ TCP Port: 1234 │
                               └────────────────┘
                                        ↓
                        ┌───────────────────────────┐
                        │ Start TCP Serial Bridge   │
                        │ COM4 ↔ localhost:1234    │
                        └───────────────────────────┘
                                        ↓
                        ┌───────────────────────────┐
                        │ VS Code Serial Monitor    │
                        │ Connect to COM4           │
                        └───────────────────────────┘
```

## 🔧 Configuration

### Current Settings
- **Firmware Baud Rate**: 9600 (with U2X=1)
- **TCP Port**: localhost:1234
- **Serial Port**: COM4
- **VS Code Monitor**: COM4 @ 9600

### Change Baud Rate
1. Edit `projects/Serial_Communications/config.h`:
   ```c
   #define BAUD 115200  // Change to desired rate
   ```
2. Rebuild project
3. No need to change TCP bridge (virtual, no baud limit)

### Change TCP Port
1. Edit `tools/simulide/Simulator110.simu`:
   ```xml
   Port="tcp://localhost:5000"
   ```
2. Edit `tools/simulide/tcp-serial-bridge.ps1`:
   ```powershell
   param([int]$TcpPort = 5000)
   ```

### Change Serial Port
Edit `tools/simulide/tcp-serial-bridge.ps1`:
```powershell
param([string]$SerialPort = "COM3")
```

## ❌ Troubleshooting

### "Failed to open serial port: COM4"
**Cause**: COM4 is already in use
**Solution**: 
1. Close VS Code Serial Monitor
2. Run the task again
3. Then open Serial Monitor

### "Failed to connect to TCP"
**Cause**: SimulIDE not running or simulation not started
**Solution**:
1. Wait for SimulIDE window to appear
2. Wait for simulation to power on
3. Bridge will retry automatically (10 attempts)

### Still seeing garbled text?
**Cause**: Baud rate mismatch
**Solution**:
1. Verify firmware: 9600 baud (check `config.h`)
2. Verify VS Code Monitor: 9600 baud
3. Rebuild if you changed the baud rate

### Bridge crashes
**Cause**: SimulIDE closed unexpectedly
**Solution**:
1. Stop the task (Ctrl+C in TCP Bridge terminal)
2. Restart SimulIDE
3. Run "Build and Simulate" task again

## 🎓 Manual Mode (Advanced)

If you want more control, run each step manually:

### 1. Build Only
```bash
# From VS Code
Ctrl+Shift+P → "Run Task" → "Build Current Project"

# Or from terminal
tools\batch\build-project.bat
```

### 2. Start SimulIDE
```bash
# From VS Code
Ctrl+Shift+P → "Run Task" → "Simulate in SimulIDE (Current Project)"

# Or from terminal
tools\simulide\cli-simulide.ps1 -ProjectDir projects\Serial_Communications
```

### 3. Start TCP Bridge
```bash
# From VS Code
Ctrl+Shift+P → "Run Task" → "Start TCP Serial Bridge"

# Or from terminal
powershell -ExecutionPolicy Bypass -File tools\simulide\tcp-serial-bridge.ps1
```

### 4. Open Serial Monitor
Click Serial Monitor icon in VS Code → Select COM4 @ 9600

## 📚 Related Documentation

- **TCP_BRIDGE_GUIDE.md** - Detailed TCP bridge explanation
- **docs/simulide/SIMULIDE_PATCHES_QUICK_REF.md** - SimulIDE UART fixes
- **docs/CLI_GUIDE.md** - Command line tools reference

## ✅ Benefits of This Setup

| Feature | Old (com0com) | New (TCP Bridge) |
|---------|---------------|------------------|
| Reliability | ⚠️ Poor | ✅ Excellent |
| Setup | Manual, complex | 1-click automated |
| Baud Rate Issues | Frequent | None |
| Error Messages | Cryptic | Clear, helpful |
| Debugging | Difficult | Easy (see all data) |
| Performance | Limited | Fast |

## 🎉 Success Indicators

You know everything is working when:
- ✅ SimulIDE window is open
- ✅ Circuit is powered (green power indicator)
- ✅ TCP Bridge terminal shows "Bridge Running"
- ✅ VS Code Serial Monitor is connected to COM4
- ✅ Messages sent from VS Code appear in SimulIDE
- ✅ Messages sent from SimulIDE appear in VS Code
- ✅ **No garbled text!**

Enjoy your clean, reliable UART communication! 🚀
