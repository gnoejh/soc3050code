# SOC3050 Portable Embedded Systems Development Environment

## Overview

This is a **completely portable** embedded systems development environment for SOC3050. Students can download this repository and start working immediately - **no external installations required**.

## ✅ What's Included (No External Dependencies)

### 🔧 Complete AVR Toolchain
- **AVR-GCC Compiler** (15.1.0) - Compile C/Assembly code
- **AVR-Objcopy** - Generate HEX files for programming
- **AVR-Size** - Memory usage analysis
- **Complete toolchain** in `tools/avr-toolchain/`

### 🎮 SimulIDE Simulators (2 Versions)
- **SimulIDE 1.1.0-SR1** - Latest version with enhanced features
- **SimulIDE 0.4.15-SR10** - Legacy version optimized for Graphics_Display
- **Pre-configured circuits** - Ready-to-use ATmega128 setups

### 📚 Comprehensive Libraries
- **Graphics LCD** (`_glcd.c/.h`) - KS0108 controller support
- **System Init** (`_init.c/.h`) - Hardware initialization
- **Port Control** (`_port.c/.h`) - Digital I/O operations
- **UART** (`_uart.c/.h`) - Serial communication
- **Timers, ADC, Buzzer, EEPROM** - Complete peripheral support

### 🎯 30+ Sample Projects
Ready-to-compile educational projects covering:
- Basic I/O and LED control
- Serial communication and debugging
- Graphics display programming
- Sensor interfacing (accelerometer, light sensor)
- Interrupt handling
- Game development examples

## 🚀 Quick Start (3 Steps)

### 1. Download & Verify
```powershell
# Clone or download this repository
git clone https://github.com/gnoejh/soc3050code.git
cd soc3050code

# Verify everything is working
.\verify-portable-system.ps1
```

### 2. Build a Project
```powershell
# Navigate to any project
cd projects\Graphics_Display

# Build for hardware
..\..\tools\cli\cli-build-project.ps1

# Build for simulation
..\..\tools\cli\cli-build-simulide.ps1
```

### 3. Run Simulation
```powershell
# Launch SimulIDE with your project
..\..\tools\simulide\cli-simulide.ps1
```

## 📖 For Students

### Daily Workflow
1. **Open project** - Choose from `projects/` directory
2. **Edit code** - Modify `Main.c` or create new files
3. **Build** - Use build scripts to compile
4. **Simulate** - Test in SimulIDE before hardware
5. **Program hardware** - Use programming scripts

### Key Commands
```powershell
# From any project directory:
..\..\tools\cli\cli-build-project.ps1        # Build for hardware
..\..\tools\cli\cli-build-simulide.ps1       # Build for simulation
..\..\tools\simulide\cli-simulide.ps1        # Launch SimulIDE

# From repository root:
.\verify-portable-system.ps1                 # Check system health
```

### Project Structure
```
projects/
├── Graphics_Display/     # LCD graphics programming
├── Serial_Communications/ # UART and debugging
├── Port_Basic/          # Digital I/O basics
├── Interrupt_Basic/     # Interrupt handling
├── ADC_Basic/          # Analog-to-digital conversion
└── ...                 # 30+ more projects
```

## 🔧 For Instructors

### System Features
- **Zero setup time** - Students download and start immediately
- **Cross-platform** - Works on any Windows machine
- **Consistent environment** - All students use identical tools
- **Educational focus** - Examples progress from basic to advanced
- **Hardware/simulation** - Test code before expensive hardware

### Customization
- **Add projects** - Copy template structure in `projects/`
- **Modify libraries** - Edit files in `shared_libs/`
- **Update tools** - Replace toolchain versions as needed
- **Circuit customization** - Modify `.simu` files for different setups

### Verification & Maintenance
```powershell
# Check system completeness
.\verify-portable-system.ps1 -Detailed

# Test all projects build correctly
.\tools\cli\cli-build-all.ps1 -TestAll
```

## 🏗️ Technical Details

### Build System
- **Unified compilation** - Same flags for hardware and simulation
- **Relative paths** - Works regardless of installation location
- **Smart detection** - Automatically finds tools and projects
- **Error handling** - Clear messages for common issues

### Portability Features
- **No absolute paths** - Everything uses relative references
- **Included dependencies** - No external tool requirements
- **Self-contained** - Complete development environment
- **Windows optimized** - PowerShell scripts with proper encoding

### Hardware Support
- **ATmega128** microcontroller (16MHz)
- **KS0108 Graphics LCD** (128x64 pixels)
- **Standard peripherals** (UART, ADC, Timers, I2C)
- **Educational kit compatibility** - Works with common learning boards

## 🛠️ Troubleshooting

### Common Issues

**Build fails with "AVR toolchain not found"**
```powershell
# Verify toolchain is present
.\verify-portable-system.ps1
# Check tools/avr-toolchain/bin/ contains executables
```

**SimulIDE won't launch**
```powershell
# Check SimulIDE versions are installed
ls tools\simulide\SimulIDE*\*\simulide.exe
# Run verification to check paths
.\verify-portable-system.ps1 -Detailed
```

**Project won't build**
```powershell
# Ensure you're in a project directory with Main.c
ls Main.c
# Check shared libraries are present
ls ..\..\shared_libs\*.c
```

### System Requirements
- **Windows 10/11** (PowerShell 5.1+)
- **500MB disk space** minimum
- **4GB RAM** recommended
- **No admin rights** required

## 📋 System Verification

Run the verification script to ensure everything is working:

```powershell
.\verify-portable-system.ps1
```

**Expected output:**
- ✅ All critical components present
- ✅ Build system functional  
- ✅ Students can download and start immediately

## 🎓 Educational Objectives

This portable system enables students to:

1. **Learn embedded programming** without setup barriers
2. **Compare hardware vs simulation** behavior
3. **Understand toolchain operations** (compile, link, program)
4. **Develop debugging skills** using serial output
5. **Progress systematically** through complexity levels
6. **Work anywhere** with consistent environment

## 📞 Support

- **System verification**: Run `.\verify-portable-system.ps1`
- **Build issues**: Check project has `Main.c` and use relative paths
- **SimulIDE issues**: Ensure correct version for your project type
- **General help**: All scripts have `-?` help parameters

---

**Ready to start?** Run `.\verify-portable-system.ps1` and begin with `projects/Port_Basic/`!