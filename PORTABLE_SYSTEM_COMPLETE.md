# Self-Sufficient Portable Development Environment - Complete

## Overview

This repository is now **100% self-sufficient and portable**. All required tools are bundled - no external installations needed.

## What's Included

### ✅ Bundled Tools (No Installation Required)

1. **AVR-GCC Toolchain** (`tools/avr-toolchain/`)
   - Complete C/C++ compiler for AVR microcontrollers
   - Includes: avr-gcc, avr-g++, avr-objcopy, avr-size, etc.
   - Version: 8.1.0 (Windows portable)

2. **AVRDUDE with GUI** (`tools/avrdudess/`)
   - Hardware programmer for ATmega128
   - Graphical interface for easy programming
   - Supports: Arduino, USBasp, AVRISP mkII, etc.

3. **SimulIDE 1.1.0-SR1** (`tools/simulide/`)
   - ATmega128 circuit simulator
   - **PATCHED** for ELPM instruction support
   - **PATCHED** for UART register mapping fix
   - Includes: Complete component library, oscilloscope, serial terminal

4. **Build Scripts** (`tools/cli/`)
   - PowerShell automation for build/program/simulate
   - VS Code task integration
   - Project templates and generators

## Quick Start for Students

### First Time Setup (5 minutes)

#### Step 1: Extract Repository
```
Extract to: W:\soc3050code  (recommended)
Or any folder WITHOUT spaces in the path
```

#### Step 2: Open in VS Code
```
1. Open VS Code
2. File → Open Folder
3. Select: W:\soc3050code
```

#### Step 3: Start Coding!

That's it! All tools are ready to use.

### Daily Usage

**No installation needed!** Just:

1. Open VS Code to `W:\soc3050code`
2. Open a project (e.g., `projects/Port_Basic/Main.c`)
3. Press `Ctrl+Shift+B` to build
4. Press `Ctrl+Shift+P` → `Tasks: Run Task` → `Build and Simulate Current Project`
5. Watch your code run in SimulIDE!

## VS Code Tasks Available

### Build Tasks
- **Build Current Project** - Compile current project (`Ctrl+Shift+B`)
- **Clean Project** - Remove build artifacts
- **Show Memory Usage** - Display flash/RAM usage

### Programming Tasks
- **Program Hardware** - Upload to physical ATmega128 board
- **Program Simulator** - Build .hex for SimulIDE
- **Build and Program Current Project** - Compile + upload to hardware

### Simulation Tasks
- **Simulate in SimulIDE (Current Project)** - Start simulation
- **Build and Simulate Current Project** - Build + auto-launch simulator

**Data flow:**
```
ATmega128 (Simulated)
    ↓ UART1 (PORTD2/PORTD3)
SimulIDE SerialPort (COM10)
    ↓ com0com driver
VS Code Serial Monitor (COM11)
    ↓
Your screen!
```

## Portability Features

### ✅ What Makes This Portable

1. **Relative Paths**: All scripts use `${workspaceFolder}` - works anywhere
2. **Bundled Toolchain**: No dependency on system-installed compilers
3. **No Registry Dependencies**: Everything in folder structure
4. **Isolated Environment**: Won't conflict with other AVR installations
5. **Cross-Machine**: Copy entire folder to USB drive → works on any Windows PC

### ✅ What Students Can Do

- **Copy to USB drive** - Entire workspace is portable
- **Work from any computer** - No installation needed
- **Share projects** - Copy `projects/YourProject/` folder to classmates
- **Backup easily** - Zip entire `soc3050code` folder
- **No admin needed** - All tools are portable

### ⚠️ Small Dependencies

| Dependency | Why Needed | Portable Alternative? |
|------------|------------|----------------------|
| VS Code | Code editor and task runner | Can use command-line scripts instead |
| PowerShell | Automation scripts | Built into Windows 10/11 |

## Directory Structure

```
soc3050code/
├── tools/
│   ├── avr-toolchain/          # ✅ Bundled AVR-GCC compiler
│   ├── avrdudess/              # ✅ Bundled programmer GUI
│   ├── simulide/               # ✅ Bundled simulator (patched)
│   └── cli/                    # ✅ Build automation scripts
│
├── projects/                   # Student lab projects
│   ├── Port_Basic/            # GPIO and LED control
│   ├── ADC_Basic/             # Analog-to-Digital conversion
│   ├── LCD_Character_Basic/   # Character LCD display
│   └── ... (30+ example projects)
│
├── shared_libs/                # Reusable libraries
│   ├── _port.h/.c             # GPIO functions
│   ├── _adc.h/.c              # ADC functions
│   └── ... (educational libraries)
│
├── .vscode/
│   └── tasks.json             # ✅ All automation tasks
│
└── docs/
    ├── QUICK_SETUP.md         # Setup guide
    └── ... (course documentation)
```

## Educational Philosophy

### Why Portable?

1. **Student Convenience**: One folder, works anywhere
2. **Lab Consistency**: Same environment on all computers
3. **Home Practice**: USB drive → home computer → same experience
4. **No IT Dependency**: Students can setup themselves
5. **Troubleshooting**: Easy to delete and re-extract if broken

### Why com0com?

1. **No Hardware Needed**: Students can practice at home without Arduino boards
2. **Reliable Simulation**: Virtual ports are faster and more reliable than physical
3. **Free and Open Source**: No licensing costs
4. **Industry Standard**: Used in professional embedded development
5. **One-Time Setup**: Install once, use forever

## Troubleshooting

### Problem: "avr-gcc is not recognized"

**Solution**: Use VS Code tasks, not manual command line. Tasks set up paths automatically.

### Problem: "COM10 not found"

**Solution**: Run `com0com: Verify Installation` task. If missing, run `com0com: Setup COM10-COM11 Pair`.

### Problem: SimulIDE shows garbled serial output

**Solution**: This was fixed! Your SimulIDE is patched. Make sure:
- Baud rate: 9600 in both firmware and Serial Monitor
- Format: 8N1 (8 data bits, No parity, 1 stop bit)
- U2X mode: Enabled in firmware

### Problem: "Cannot find project directory"

**Solution**: Make sure you're editing a file inside a project folder (e.g., `projects/Serial_Communications/Main.c`).

## Version History

### October 19, 2025 - Complete Portable System
- ✅ Added com0com with automated VS Code tasks
- ✅ Fixed COM port configuration to COM10-COM11
- ✅ Created student quick start guide
- ✅ Verified all scripts work without external dependencies

### October 18, 2025 - SimulIDE Patches
- ✅ Fixed ELPM instruction bug in ATmega128 simulation
- ✅ Fixed UART register mapping bug (TX garbled output)
- ✅ Documented all patches in `SIMULIDE_1.1.0_PATCHES_APPLIED.md`

### October 2025 - Initial Portable Setup
- ✅ Bundled AVR-GCC toolchain
- ✅ Bundled AVRDUDE with GUI
- ✅ Created PowerShell automation scripts
- ✅ Organized 30+ example projects

## Next Steps for Students

1. ✅ **Read**: `tools/com0com/QUICK_START.md`
2. ✅ **Setup**: Install com0com (10 minutes, one time)
3. ✅ **Verify**: Run verification task
4. ✅ **Test**: Build and simulate `Serial_Communications` project
5. ✅ **Learn**: Explore other projects in `projects/` folder

## For Instructors

### Deploying to Student Computers

**Option 1: Network Drive (Recommended)**
```
1. Extract to: \\server\share\soc3050code
2. Students map drive: net use W: \\server\share\soc3050code
3. Students run com0com setup (once per machine)
4. Ready to use!
```

**Option 2: Local Installation**
```
1. Extract to: C:\soc3050code or W:\soc3050code
2. Run: tools\com0com\install-com0com.ps1 (as Admin)
3. Run: tools\com0com\setup-com-pair.ps1 (as Admin)
4. Open VS Code to soc3050code folder
```

**Option 3: USB Drive (Lab Computers)**
```
1. Extract to USB drive root
2. On each computer: Run com0com setup (once)
3. Open VS Code to USB drive path
```

### Mass Deployment Script

See: `tools/setup/deploy-to-lab-computers.ps1` (if needed)

## Support

### For Students
- Check: `tools/com0com/QUICK_START.md`
- Check: `docs/QUICK_SETUP.md` (this file)
- Ask: Your instructor or TA
- Verify: Run `com0com: Verify Installation` task

### For Instructors
- Check: `tools/com0com/README.md` (detailed technical docs)
- Check: `docs/simulide/SIMULIDE_1.1.0_PATCHES_APPLIED.md`
- Check: `REORGANIZATION_SUMMARY.md`

## License

- **soc3050code**: Educational use, SOC 3050 course materials
- **AVR-GCC**: GPL v3
- **AVRDUDE**: GPL v2
- **SimulIDE**: GPL v3
- **com0com**: GPL v2

---

**Course**: SOC 3050 - Embedded Systems and Applications  
**Instructor**: Professor Hong Jeong  
**Year**: 2025  
**Status**: ✅ **COMPLETE PORTABLE SYSTEM**

🎉 **Students can now work anywhere with zero external dependencies (except com0com one-time setup)!**
