# 🚀 Quick Setup Guide - Self-Sufficient Repository

## ✅ No Installation Required!

This repository includes everything you need. Just clone and start coding!

### Step 1: Clone Repository
```bash
git clone <your-repository-url>
cd soc3050code
```

### Step 2: Verify Environment

**Option A: Simple Check (No PowerShell issues)**
```bash
# Double-click this file or run from command prompt
tools\batch\simple-check.bat
```

**Option B: Full Verification with PowerShell**
```bash
# Double-click this file (handles PowerShell policies automatically)
setup.bat

# Or run the full check directly:
tools\batch\setup-check.bat

# Or run manually with bypass:
powershell -ExecutionPolicy Bypass -File verify-environment.ps1
```

**Option C: Manual Check**
```bash
# If scripts don't work, manually verify:
# 1. Check if tools/avr-toolchain/bin/avr-gcc.exe exists
# 2. Check if tools/simulide/SimulIDE_1.1.0-SR1_Win64/simulide.exe exists
# 3. Open VS Code with: code .
```

### Step 3: Open in VS Code
```bash
code .
```

### Step 4: Start Coding!
1. Navigate to `projects/` folder
2. Open any project (try `Port_Basic` first)
3. Press `Ctrl+Shift+P` → "Tasks: Run Task" → "Build Current Project"
4. Connect your ATmega128 and program it!

## 🛠️ What's Bundled

- ✅ Complete AVR-GCC toolchain
- ✅ SimulIDE circuit simulator  
- ✅ ATmega128 device definitions
- ✅ VS Code configuration
- ✅ PowerShell build scripts + Batch file wrappers
- ✅ 35+ educational projects

## 🔧 Development Options

### **VS Code (Recommended)**
- **Build Current Project** - Compile your code
- **Build and Simulate** - Test in SimulIDE
- **Program Current Project** - Upload to microcontroller
- **Open Serial Monitor** - Debug via UART

### **Command Line (Easy)**
```bash
build.bat              # Build current project (wrapper)
tools\batch\build-project.bat      # Build specific project
tools\batch\program-project.bat    # Upload to MCU
tools\batch\simulate-project.bat   # Test in SimulIDE
```

### **Advanced CLI**
See [CLI_GUIDE.md](CLI_GUIDE.md) for complete reference

## 📁 Repository Structure

```
soc3050code/
├── projects/           # 35 Educational projects
├── tools/             # Complete development toolchain
│   ├── avr-toolchain/ # GCC compiler and tools
│   ├── simulide/      # Circuit simulator
│   └── cli/           # Build scripts
├── shared_libs/       # Reusable library functions
├── .vscode/          # Pre-configured VS Code settings
└── verify-environment.ps1  # Setup verification
```

## 🎯 Ready to Learn!

No more setup hassles - everything is included. Start with `projects/Port_Basic` and progress through the 35 educational projects.

**Happy coding! 🚀**