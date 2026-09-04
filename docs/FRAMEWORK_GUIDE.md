# ATmega128 Educational Framework

**Complete Embedded Systems Learning Environment**

## Quick Start

### Prerequisites
- Microchip Studio 7.0+
- ATmega128 development board
- AVR programmer (USBasp/Dragon)

### First Project
1. Open `projects/01_Port_Assembly/Main.cproj` in Microchip Studio
2. Build project (F7)
3. Program to ATmega128 (Ctrl+Alt+F5)
4. Observe LED behavior on PORTB

## Project Structure

**35 Progressive Projects** organized by learning complexity:

### Assembly Foundations (Projects 1-6)
- Basic port control and LED manipulation
- Assembly language fundamentals
- Direct hardware register access

### C Programming Transition (Projects 7-14)
- Serial communication and UART
- ADC readings and sensor interfaces
- Timer control and interrupts

### Advanced Applications (Projects 15-35)
- Motor control and actuators
- Game development
- IoT applications and communication

## Development Tools

### Build System
```powershell
.\enhanced_build_system.ps1 -BuildAll
```

### Code Quality
```powershell
.\code_quality_optimizer.ps1 -Analyze
```

### Project Templates
```powershell
.\project_template_generator.ps1 -ProjectName "MyProject" -ProjectType "uart"
```

## Learning Path

1. **Start with Project 01** - Basic port control
2. **Progress sequentially** - Each project builds on previous concepts
3. **Use UART output** - Most projects include debugging information
4. **Experiment** - Modify code to understand behavior
5. **Build projects** - Hands-on hardware experience

## Hardware Setup

- **ATmega128-16AU** with 7.3728MHz crystal
- **LEDs** on PORTB with current-limiting resistors
- **Switches** on PORTC with pull-up resistors
- **UART** connection for debugging (9600 baud, 8N1)

## Support

Each project includes:
- Detailed code comments explaining functionality
- Learning objectives and hardware requirements
- Interactive UART commands for testing
- Troubleshooting guidance

For technical issues, use the system debugger:
```powershell
.\system_debugger.ps1 -Interactive
```

---

**Educational framework optimized for progressive learning from assembly basics to advanced embedded applications.**

© 2025 Prof. Hong Jeaong, IUT (Inha University in Tashkent)  
All rights reserved for educational purposes.  
**Contact:** [linkedin.com/in/gnoejh53](https://linkedin.com/in/gnoejh53)