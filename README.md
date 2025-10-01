# 🎓 ATmega128 Educational Development Environment

**Complete VS Code and Microchip Studio Development Environment for ATmega128**

A comprehensive learning environment with 35 educational projects progressing from basic assembly to advanced embedded C programming. Includes both VS Code and Microchip Studio support for flexible development.

## 🚀 Quick Start

### **Prerequisites**
- **Microchip Studio** (formerly Atmel Studio 7.0) - Primary development environment
- **VS Code** - Alternative development environment with custom tasks
- **ATmega128 Development Board** with programmer (Arduino, USBasp, AVRISP mkII, etc.)

### **Development Workflow**

#### **Option 1: VS Code Development (Recommended for Quick Development)**
1. Open project folder in VS Code: `projects/[project_name]/`
2. **Build**: `Ctrl+Shift+B` or VS Code Command Palette → "Build Current Project"
3. **Generate HEX**: Command Palette → "Generate HEX File"
4. **Program MCU**: Command Palette → "Program Any Project"

#### **Option 2: Microchip Studio (Recommended for Advanced Features)**
1. Open `Main.atsln` in Microchip Studio
2. Select desired project as startup project
3. Build with `F7` or Build menu
4. Program using Tools → Device Programming

---

## 📁 Project Structure

```
soc3050code/
├── projects/                    # 35 Educational Projects
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
| **Build Current Project** | Compile current project | `Ctrl+Shift+B` |
| **Generate HEX File** | Create .hex file for programming | Command Palette |
| **Program Any Project** | Program ATmega128 with auto-detection | Command Palette |
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

## 📖 Educational Objectives

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