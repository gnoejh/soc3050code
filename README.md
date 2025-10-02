# ATmega128 Educational Framework# ATmega128 Educational Framework# ATmega128 Educational Framework# ATmega128 Educational Framework



**35 Progressive Projects for Embedded Systems Learning**



## Quick Start**35 Progressive Projects for Embedded Systems Learning**



1. **Install:** [Microchip Studio](https://www.microchip.com/en-us/tools-resources/develop/microchip-studio)

2. **Open:** `projects/01_Port_Assembly/Main.cproj`

3. **Build:** Press F7## Quick Start**35 Progressive Projects for Embedded Systems Learning****35 Progressive Projects for Embedded Systems Learning**

4. **Program:** Connect ATmega128 and press Ctrl+Alt+F5



## Projects Overview

1. **Install:** [Microchip Studio](https://www.microchip.com/en-us/tools-resources/develop/microchip-studio)

- **Projects 1-6:** Assembly fundamentals and port control

- **Projects 7-14:** C programming with UART and ADC2. **Open:** `projects/01_Port_Assembly/Main.cproj`

- **Projects 15-21:** Advanced interfaces (motors, sound, memory)

- **Projects 22-35:** Complete applications (games, IoT)3. **Build:** Press F7## Quick Start## Quick Start



## Development Tools4. **Program:** Connect ATmega128 and press Ctrl+Alt+F5



```powershell

.\cli-build-all.ps1            # Build all projects

.\cli-build-project.ps1        # Build current project (generates .hex file)## Projects Overview

.\cli-test-system.ps1          # Test and debug projects

.\cli-analyze-code.ps1         # Check code quality1. **Install:** [Microchip Studio](https://www.microchip.com/en-us/tools-resources/develop/microchip-studio)1. **Install:** [Microchip Studio](https://www.microchip.com/en-us/tools-resources/develop/microchip-studio)

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