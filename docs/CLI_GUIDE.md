# 🛠️ SOC3050 Command Line Interface Guide

## 🎯 Easy Access - No PowerShell Issues!

All functionality is available through both PowerShell scripts and batch file wrappers.

### 📋 Quick Reference

| Task | Batch File (Easy) | PowerShell (Advanced) |
|------|-------------------|------------------------|
| **Verify Environment** | `tools\batch\simple-check.bat` | `powershell -ExecutionPolicy Bypass -File verify-environment.ps1` |
| **Build Project** | `tools\batch\build-project.bat` | `powershell -ExecutionPolicy Bypass -File tools\cli\cli-build-project.ps1 -ProjectDir .` |
| **Program MCU** | `tools\batch\program-project.bat` | `powershell -ExecutionPolicy Bypass -File tools\cli\cli-program-project.ps1 -ProjectDir .` |
| **Simulate Circuit** | `tools\batch\simulate-project.bat` | `powershell -ExecutionPolicy Bypass -File tools\cli\cli-build-simulide.ps1 -ProjectDir .` |
| **Build All** | `tools\batch\build-all.bat` | `powershell -ExecutionPolicy Bypass -File tools\cli\cli-build-all.ps1` |
| **New Project** | `tools\batch\new-project.bat` | `powershell -ExecutionPolicy Bypass -File tools\cli\cli-new-project.ps1` |

## 🚀 Basic Usage

### 1. Verify Environment
```bash
# Simple check (recommended)
tools\batch\simple-check.bat

# Full verification
setup.bat
```

### 2. Build a Project
```bash
# Build current directory project (easy wrapper)
build.bat

# Build specific project
tools\batch\build-project.bat "projects\Port_Basic"
```

### 3. Program Microcontroller
```bash
# Program with default settings (Arduino programmer, COM3)
tools\batch\program-project.bat

# Program with specific settings
tools\batch\program-project.bat "projects\Port_Basic" arduino COM5
tools\batch\program-project.bat "projects\Timer_Basic" usbasp
```

### 4. Simulate in SimulIDE
```bash
# Simulate current project
tools\batch\simulate-project.bat

# Simulate specific project
tools\batch\simulate-project.bat "projects\Graphics_Display"
```

### 5. Build All Projects
```bash
# Test build all 35+ projects
tools\batch\build-all.bat
```

### 6. Create New Project
```bash
# Interactive project creation
tools\batch\new-project.bat
```

## 🎮 VS Code Integration (Recommended)

For the best experience, use VS Code tasks:

1. Open VS Code: `code .`
2. Press `Ctrl+Shift+P`
3. Type "Tasks: Run Task"
4. Choose from available tasks:
   - **Build Current Project**
   - **Program Current Project**
   - **Build and Simulate**
   - **Open Serial Monitor**

## 🔧 Advanced PowerShell Usage

If you prefer PowerShell or need advanced features:

### Environment & Testing
```powershell
# Comprehensive environment check
powershell -ExecutionPolicy Bypass -File verify-environment.ps1 -Verbose

# Test system components
powershell -ExecutionPolicy Bypass -File tools\cli\cli-test-system.ps1 -Interactive

# Analyze code quality
powershell -ExecutionPolicy Bypass -File tools\cli\cli-analyze-code.ps1 -ProjectPath "projects\Port_Basic"
```

### Build Operations
```powershell
# Build with specific options
powershell -ExecutionPolicy Bypass -File tools\cli\cli-build-project.ps1 -ProjectDir "projects\Graphics_Display"

# Build for simulation (optimized)
powershell -ExecutionPolicy Bypass -File tools\cli\cli-build-simulide.ps1 -ProjectDir "projects\Serial_Communications"

# Build all projects with testing
powershell -ExecutionPolicy Bypass -File tools\cli\cli-build-all.ps1 -TestAll
```

### Programming Options
```powershell
# Program with Arduino as ISP
powershell -ExecutionPolicy Bypass -File tools\cli\cli-program-project.ps1 -ProjectDir "projects\Port_Basic" -Programmer arduino -Port COM3

# Program with USBasp
powershell -ExecutionPolicy Bypass -File tools\cli\cli-program-project.ps1 -ProjectDir "projects\Timer_Basic" -Programmer usbasp

# Dry run (test without programming)
powershell -ExecutionPolicy Bypass -File tools\cli\cli-program-project.ps1 -ProjectDir "projects\ADC_Basic" -DryRun
```

## 🆘 Troubleshooting

### PowerShell Execution Policy Issues
If you get "cannot be loaded because running scripts is disabled":

1. **Use batch files** (easiest solution)
2. **Run with bypass**: `powershell -ExecutionPolicy Bypass -File script.ps1`
3. **Fix policy**: Run `fix-powershell.ps1` for guided setup

### Common Issues
- **"Command not found"**: Use `.\command.bat` instead of `command.bat`
- **"Access denied"**: Run Command Prompt as Administrator
- **"Port not found"**: Check Device Manager for correct COM port
- **"Programmer not responding"**: Verify connections and power

## 📁 Project Structure

Understanding the CLI tools location:
```
soc3050code/
├── *.bat                    # Easy batch file wrappers
├── verify-environment.ps1   # Environment verification
├── tools/cli/              # Advanced PowerShell tools
│   ├── cli-build-project.ps1
│   ├── cli-program-project.ps1
│   ├── cli-build-simulide.ps1
│   └── ...
└── projects/               # Your coding projects
    ├── Port_Basic/
    ├── Timer_Basic/
    └── ...
```

## 💡 Tips for Success

1. **Start Simple**: Use batch files initially
2. **Use VS Code**: Best integrated experience
3. **Check Environment**: Run verification before starting
4. **Read Error Messages**: They usually point to the solution
5. **Test in Simulation**: Verify logic before programming hardware

---

**🎯 Ready to code? Start with `setup.bat` wrapper and then `code .`**