# SOC3050 ATmega128 Educational Framework

**🎯 Self-Sufficient Embedded Systems Learning Environment**

**35 Progressive Projects • Complete Toolchain Included • No Setup Required**

> **✅ NEW: SimulIDE 1.1.0 Fully Patched!** ELPM and UART bugs fixed for ATmega128. See [`docs/simulide/SIMULIDE_PATCHES_QUICK_REF.md`](docs/simulide/SIMULIDE_PATCHES_QUICK_REF.md)

## 🚀 Quick Start (No Installation Required!)

This repository is completely self-sufficient! Everything you need is included.

### Step 1: Clone and Verify
```bash
git clone <repository-url>
cd soc3050code
pwsh ./verify-environment.ps1
```

### Step 2: Open in VS Code
```bash
code .
```

### Step 3: Start Learning!
1. Navigate to `projects/` folder
2. Choose any project (start with `Port_Basic`)
3. Press `Ctrl+Shift+P` → "Tasks: Run Task" → "Build Current Project"
4. Your code is ready to upload to ATmega128!

## 🛠️ What's Included (No External Downloads!)

### Complete Development Environment
- ✅ **AVR-GCC Compiler** - Latest toolchain for ATmega128
- ✅ **ATmega128 Device Pack** - Device-specific optimizations
- ✅ **SimulIDE Simulator** - Test circuits without hardware
- ✅ **VS Code Configuration** - Pre-configured tasks and IntelliSense
- ✅ **Build Scripts** - PowerShell automation for all tasks

### Ready-to-Use Tools
```
tools/
├── avr-toolchain/          # Complete AVR-GCC compiler
├── packs/                  # ATmega device definitions
├── simulide/               # Circuit simulator
├── cli/                    # Build and programming scripts
└── setup/                  # Environment verification
```

## 📚 Projects Overview (35 Progressive Lessons)

### **Assembly Fundamentals (Projects 1-6)**
- Port control and digital I/O
- Assembly language basics
- Register manipulation

### **C Programming Basics (Projects 7-14)**
- UART serial communication  
- ADC analog input
- Timer and interrupt systems

### **Advanced Interfaces (Projects 15-21)**
- Motor control and PWM
- Sound generation
- EEPROM data storage
- I2C sensor communication

### **Complete Applications (Projects 22-35)**
- Interactive games (Pong, Hangman, Puzzles)
- IoT data logging
- Graphical LCD displays
- Real-time control systems

# SOC3050 ATmega128 Educational Framework

**🎯 Self-Sufficient Embedded Systems Learning Environment**

**35 Progressive Projects • Complete Toolchain Included • No Setup Required**

## � Quick Start (No Installation Required!)

This repository is completely self-sufficient! Everything you need is included.

### Step 1: Clone and Verify
```bash
git clone <repository-url>
cd soc3050code
pwsh ./verify-environment.ps1
```

### Step 2: Open in VS Code
```bash
code .
```

### Step 3: Start Learning!
1. Navigate to `projects/` folder
2. Choose any project (start with `Port_Basic`)
3. Press `Ctrl+Shift+P` → "Tasks: Run Task" → "Build Current Project"
4. Your code is ready to upload to ATmega128!

## 🛠️ What's Included (No External Downloads!)

### Complete Development Environment
- ✅ **AVR-GCC Compiler** - Latest toolchain for ATmega128
- ✅ **ATmega128 Device Pack** - Device-specific optimizations
- ✅ **SimulIDE Simulator** - Test circuits without hardware
- ✅ **VS Code Configuration** - Pre-configured tasks and IntelliSense
- ✅ **Build Scripts** - PowerShell automation for all tasks

### Ready-to-Use Tools
```
tools/
├── avr-toolchain/          # Complete AVR-GCC compiler
├── packs/                  # ATmega device definitions
├── simulide/               # Circuit simulator
├── cli/                    # Build and programming scripts
└── setup/                  # Environment verification
```

## 📚 Projects Overview (35 Progressive Lessons)

### **Assembly Fundamentals (Projects 1-6)**
- Port control and digital I/O
- Assembly language basics
- Register manipulation

### **C Programming Basics (Projects 7-14)**
- UART serial communication  
- ADC analog input
- Timer and interrupt systems

### **Advanced Interfaces (Projects 15-21)**
- Motor control and PWM
- Sound generation
- EEPROM data storage
- I2C sensor communication

### **Complete Applications (Projects 22-35)**
- Interactive games (Pong, Hangman, Puzzles)
- IoT data logging
- Graphical LCD displays
- Real-time control systems

## �🔧 Development Workflow

### Using VS Code (Recommended)
```
Ctrl+Shift+P → "Tasks: Run Task" → Choose:
├── "Build Current Project"      # Compile to .elf and .hex
├── "Build and Simulate"         # Test in SimulIDE
├── "Program Current Project"    # Upload to hardware
└── "Open Serial Monitor"        # Debug via UART
```

### Using Command Line

**Easy Batch Files (No PowerShell Issues):**
```bash
# Build and program projects
build.bat                            # Build current project (wrapper)
tools\batch\build-project.bat        # Build specific project  
tools\batch\program-project.bat      # Program microcontroller
tools\batch\simulate-project.bat     # Test in SimulIDE
tools\batch\build-all.bat            # Build all projects

# Environment verification
setup.bat                            # Quick setup check (wrapper)
tools\batch\simple-check.bat         # Quick environment check
tools\batch\setup-check.bat          # Full verification
```

**Advanced PowerShell (for power users):**
```powershell
# Use -ExecutionPolicy Bypass to avoid policy issues
powershell -ExecutionPolicy Bypass -File tools/cli/cli-build-project.ps1 -ProjectDir "projects/Port_Basic"
powershell -ExecutionPolicy Bypass -File tools/cli/cli-program-project.ps1 -ProjectDir "projects/Port_Basic"
powershell -ExecutionPolicy Bypass -File tools/cli/cli-build-simulide.ps1 -ProjectDir "projects/Port_Basic"
powershell -ExecutionPolicy Bypass -File verify-environment.ps1
```

📖 **See [CLI_GUIDE.md](CLI_GUIDE.md) for complete command line reference**

## 📋 Hardware Requirements

### Minimum Setup
- **ATmega128 development board** (any compatible board)
- **USB programmer** (Arduino as ISP, USBasp, etc.)
- **Serial connection** for debugging (USB-Serial adapter)

### Recommended Components
- LEDs connected to PORTB (pins 0-7)
- Switches/buttons on PORTC (pins 0-7)
- Breadboard for circuit experiments
- Basic components (resistors, capacitors, sensors)

## 🎓 Learning Path

### **Beginner Track**
1. Start with `Port_Basic` - Learn digital I/O
2. Progress to `Timer_Basic` - Understand timing
3. Try `Serial_Communications` - Master UART

### **Intermediate Track**
4. `ADC_Basic` - Analog input processing
5. `Interrupt_Basic` - Event-driven programming
6. `Graphics_Display` - Visual output systems

### **Advanced Track**
7. `I2C_Sensors_Multi` - Complex communication
8. Game projects - Apply all concepts
9. IoT projects - Real-world applications

## 🔍 Key Features

### **Self-Sufficient Design**
- ✅ No external software installation required
- ✅ Portable across Windows computers
- ✅ Consistent build environment for all students
- ✅ Works offline without internet connection

### **Educational Focus**
- 📖 Progressive difficulty with clear explanations
- 🧪 Interactive UART commands for testing
- 🎮 Fun game projects to maintain engagement
- 🔬 Practical applications in IoT and control

### **Professional Tools**
- 🛠️ Industry-standard AVR-GCC compiler
- 📊 Memory usage analysis and optimization
- 🔍 Integrated debugging and simulation
- 📈 Code quality analysis tools

## 📖 Documentation

- `docs/LAB_GUIDE.md` - Complete lab manual
- `docs/LIBRARY_REFERENCE.md` - Shared library functions
- `FRAMEWORK_GUIDE.md` - Architecture and design
- `PROJECT_CATALOG.md` - All projects with descriptions

## 🆘 Troubleshooting

### PowerShell Execution Policy Issues
If you get "cannot be loaded because running scripts is disabled":

```bash
# Option 1: Use batch file (easiest)
setup.bat

# Option 2: Run with bypass flag
powershell -ExecutionPolicy Bypass -File verify-environment.ps1

# Option 3: Fix PowerShell policy
powershell -ExecutionPolicy Bypass -File fix-powershell.ps1
```

### Environment Issues
```bash
# Verify everything is working (use bypass if needed)
powershell -ExecutionPolicy Bypass -File verify-environment.ps1 -Verbose

# Check specific tools
powershell -ExecutionPolicy Bypass -File tools/setup/check-simulide-environment.ps1
```

### Build Problems
```bash
# Clean and rebuild
./tools/cli/cli-build-project.ps1 -Clean

# Check memory usage
./tools/cli/cli-analyze-code.ps1
```

### Programming Issues
- Ensure correct COM port in tasks.json
- Check programmer type (arduino, usbasp, etc.)
- Verify hardware connections

## 🤝 Contributing

This repository is designed for educational use. Students can:
- Create new projects using provided templates
- Extend existing projects with additional features
- Share improvements via pull requests

## 📄 License

Educational use license. See individual source files for specific licensing terms.

---

**🎯 Ready to start? Run `./verify-environment.ps1` and begin your embedded systems journey!**

.\cli-new-project.ps1          # Create new project template

```- **Projects 1-6:** Assembly fundamentals and port control



## Hardware Requirements- **Projects 7-14:** C programming with UART and ADC2. **Open:** `projects/01_Port_Assembly/Main.cproj`2. **Open:** `projects/01_Port_Assembly/Main.cproj`



- ATmega128 development board- **Projects 15-21:** Advanced interfaces (motors, sound, memory)

- LEDs connected to PORTB

- Switches on PORTC  - **Projects 22-35:** Complete applications (games, IoT)3. **Build:** Press F73. **Build:** Press F7

- UART connection for debugging (9600 baud)



## Learning Path

## Development Tools4. **Program:** Connect ATmega128 and press Ctrl+Alt+F54. **Program:** Connect ATmega128 and press Ctrl+Alt+F5

Each project includes educational comments and interactive UART commands for testing. Progress sequentially as each project builds on previous concepts.



For detailed information, see `FRAMEWORK_GUIDE.md`.
```powershell

.\build-all.ps1                # Build all projects

.\build-project.ps1            # Build current project and program to ATmega128## Projects Overview## Projects Overview

.\test-system.ps1              # Test and debug projects

.\analyze-code.ps1             # Check code quality

.\new-project.ps1              # Create new project template

```- **Projects 1-6:** Assembly fundamentals and port control- **Projects 1-6:** Assembly fundamentals and port control



## Hardware Requirements- **Projects 7-14:** C programming with UART and ADC- **Projects 7-14:** C programming with UART and ADC



- ATmega128 development board- **Projects 15-21:** Advanced interfaces (motors, sound, memory)- **Projects 15-21:** Advanced interfaces (motors, sound, memory)

- LEDs connected to PORTB

- Switches on PORTC  - **Projects 22-35:** Complete applications (games, IoT)- **Projects 22-35:** Complete applications (games, IoT)

- UART connection for debugging (9600 baud)



## Learning Path

## Development Tools## Development Tools

Each project includes educational comments and interactive UART commands for testing. Progress sequentially as each project builds on previous concepts.



For detailed information, see `FRAMEWORK_GUIDE.md`.
```powershell```powershell

.\enhanced_build_system.ps1 -BuildAll    # Build all projects.\enhanced_build_system.ps1 -BuildAll    # Build all projects

.\system_debugger.ps1 -Interactive       # Test and debug.\system_debugger.ps1 -Interactive       # Test and debug

``````

## Hardware Requirements

## Hardware Requirements

- ATmega128 development board

- ATmega128 development board- LEDs connected to PORTB

- LEDs connected to PORTB- Switches on PORTC  

- Switches on PORTC  - UART connection for debugging (9600 baud)

- UART connection for debugging (9600 baud)

## Learning Path

## Learning Path

Each project includes educational comments and interactive UART commands for testing. Progress sequentially as each project builds on previous concepts.

Each project includes educational comments and interactive UART commands for testing. Progress sequentially as each project builds on previous concepts.

For detailed information, see `FRAMEWORK_GUIDE.md`.

For detailed information, see `FRAMEWORK_GUIDE.md`.├── projects/                    # 35 Educational Projects
│   ├── 01_Assembly_Blink_Basic/     # Basic LED blinking
│   ├── 02_Assembly_Blink_Pattern/   # Pattern-based blinking
│   ├── 05_Assembly_Button_Simple/   # Button input handling
│   ├── 09_ADC_Basic_Reading/        # Analog-to-digital conversion
│   ├── 12_Graphics_Display/         # GLCD graphics programming
│   ├── 15_Joystick_Control/         # Joystick input processing
│   ├── 25_IoT_Basic/                # IoT communication basics
│   └── 35_Game_Word_Puzzle_V2/      # Advanced game programming
├── shared_libs/                 # Reusable Library Functions
│   ├── _init.c/.h              # System initialization
│   ├── _port.c/.h              # Port configuration
│   ├── _uart.c/.h              # Serial communication
│   ├── _adc.c/.h               # ADC functions
│   └── _glcd.c/.h              # Graphics LCD functions
├── .vscode/                     # VS Code Configuration
│   └── tasks.json              # Build and programming tasks
├── program.ps1                 # Universal programming script
└── Main.atsln                  # Microchip Studio solution
```

---

## 🛠️ Available VS Code Tasks

| Task | Description | Shortcut |
|------|-------------|----------|
| **Build Current Project** | Compile current project | `F7` |
| **Program Current Project** | Program ATmega128 (Arduino/COM3 default) | `Ctrl+Alt+F5` |
| **Build and Program Current Project** | Build then program in sequence | `Ctrl+Alt+F7` |
| **Generate HEX File** | Create .hex file for programming | Command Palette |
| **Program with Custom Programmer** | Choose programmer type (USBasp/Arduino/etc) | Command Palette |
| **Show Memory Usage** | Display flash/RAM usage statistics | Command Palette |
| **Build All Projects** | Build all 35 projects in sequence | Command Palette |
| **List COM Ports** | Show available serial ports | Command Palette |

---

## 📋 Programming Options

### **Universal Programming Script**
```powershell
# Auto-detect project from current folder
.\program.ps1

# Specify project explicitly
.\program.ps1 -ProjectFolder "05_Assembly_Button_Simple"

# Use different programmer
.\program.ps1 -Programmer usbasp

# List all projects
.\program.ps1 -ListProjects

# List available COM ports
.\program.ps1 -ListPorts
```

### **Supported Programmers**
- `arduino` - Arduino as ISP (default)
- `usbasp` - USBasp programmer
- `avrisp` - AVRISP mkII
- `stk500v2` - STK500 v2
- `dragon_isp` - AVR Dragon

---

## 📚 Learning Progression (35 Projects)

### **Phase 1: Assembly Fundamentals (Projects 1-4)**
- Basic LED control and timing
- Register manipulation and bit operations
- Assembly language integration with C

### **Phase 2: Input/Output Systems (Projects 5-8)**
- Button handling and debouncing
- Serial communication basics
- Polling vs interrupt-driven programming

### **Phase 3: Analog & Sensors (Projects 9-11)**
- ADC reading and calibration
- Sensor interfacing (CDS, accelerometer)
- Data acquisition and processing

### **Phase 4: Advanced Peripherals (Projects 12-17)**
- Graphics LCD programming
- Timer and interrupt systems
- Joystick and motor control

### **Phase 5: Audio & Memory (Projects 18-22)**
- Sound generation and music
- Memory management and testing
- Inline assembly optimization

### **Phase 6: Communication (Projects 23-27)**
- Advanced serial protocols
- IoT communication basics
- Data transmission and reception

### **Phase 7: Game Development (Projects 28-35)**
- Game engine fundamentals
- Interactive entertainment systems
- Advanced user interfaces

---

## ⚙️ Hardware Configuration

### **ATmega128 Settings**
- **Clock Frequency**: 7.3728 MHz crystal
- **Fuse Settings**: Default (consult your development board manual)
- **Programming Interface**: ISP (In-System Programming)

### **Peripheral Connections**
- **LEDs**: Connected to appropriate ports (see individual project documentation)
- **Buttons**: Pull-up/pull-down configured per project
- **LCD Display**: Standard HD44780 or GLCD interface
- **Serial**: Hardware UART pins for communication

---

## 🔧 Troubleshooting

### **Build Issues**
- Ensure shared_libs folder is properly referenced
- Check F_CPU definition matches your crystal frequency
- Verify avr-gcc toolchain is properly installed

### **Programming Issues**
- Verify COM port in Device Manager
- Check programmer connections and power
- Try different programmer types with `-Programmer` parameter
- Ensure ATmega128 is properly powered and connected

### **VS Code Issues**
- Install C/C++ extension for better syntax highlighting
- Check .vscode/tasks.json for correct toolchain paths
- Verify PowerShell execution policy allows scripts

---

## � Documentation

**All documentation has been organized in the `docs/` folder:**

- 📘 **[Complete Documentation Index](docs/README.md)** - Start here for all guides
- 🎮 **[SimulIDE Guide](docs/simulide/SIMULIDE_GUIDE.md)** - Hardware-free simulation (91+ pages)
- 🧪 **[Lab Exercise Guide](docs/labs/LAB_EXERCISE_GUIDE.md)** - 65 hands-on exercises (138+ pages)
- 🛠️ **[Framework Guide](FRAMEWORK_GUIDE.md)** - Development framework (kept in root)
- 📜 **[Scripts Guide](SCRIPTS.md)** - PowerShell automation tools

**Quick Links:**
- For Students: [Lab Quick Start](docs/labs/LAB_QUICK_START.md)
- For Setup: [SimulIDE Quick Reference](docs/simulide/SIMULIDE_QUICK_REFERENCE.md)
- For Development: [Library Enhancements](docs/development/LIBRARY_ENHANCEMENTS_SUMMARY.md)

**Total Documentation: 289+ pages across 23 organized files**

---

## �📖 Educational Objectives

This framework teaches:
- **Low-level Programming**: Direct register manipulation and assembly integration
- **Embedded C Development**: Structured programming for microcontrollers
- **Hardware Interfacing**: Sensors, displays, communication protocols
- **Real-time Systems**: Interrupt handling and timing-critical applications
- **Software Engineering**: Modular design with shared libraries
- **Development Tools**: Professional IDE usage and build systems

Perfect for computer science students learning embedded systems, microcontroller programming, and hardware-software integration.

---

## 📝 License

Educational use - designed for SOC3050 course and similar embedded systems education.

---

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)  
All rights reserved for educational purposes.

**Contact:** [linkedin.com/in/gnoejh53](https://linkedin.com/in/gnoejh53)