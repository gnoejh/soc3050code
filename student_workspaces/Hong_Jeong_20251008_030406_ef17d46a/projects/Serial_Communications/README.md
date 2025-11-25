# Serial_interrupt Project - Module Selection System

## Overview

The Serial_interrupt project has been successfully restructured to follow the standard pattern used throughout the ATmega128 educational framework. Students can now easily select which demonstration to run by uncommenting function calls in the main() function.

## Project Structure

### Main.c Organization
```c
// Project header and usage instructions
// Global variables and interrupt handlers  
// Demo function implementations (always available)
// Main entry point with commented demo calls
```

### Available Demonstrations

#### Demo 1: Basic RX Interrupt (`demo_basic_rx_interrupt()`)
- **Purpose**: Character echo using RX interrupts
- **Learning**: Basic interrupt service routine handling
- **Features**: Immediate character echo, simple ISR implementation

#### Demo 2: TX Interrupt Queue (`demo_tx_interrupt_queue()`)  
- **Purpose**: Buffered transmission system
- **Learning**: Non-blocking I/O, interrupt-driven output
- **Features**: Message queuing, automatic transmission management

#### Demo 3: Bidirectional Interrupts (`demo_bidirectional_interrupts()`)
- **Purpose**: Full-duplex communication
- **Learning**: Simultaneous RX/TX interrupt handling
- **Features**: Real-time response system, concurrent I/O

#### Demo 4: Command Processing (`demo_command_processing()`)
- **Purpose**: Interactive command interpreter
- **Learning**: Real-time command parsing and execution
- **Features**: LED control, status reporting, command recognition

#### Demo 5: Advanced Buffering (`demo_advanced_buffering()`)
- **Purpose**: Buffer management and statistics
- **Learning**: Circular buffers, overflow handling, performance monitoring
- **Features**: Real-time statistics, buffer visualization

## Usage Instructions

### Selecting a Demo
1. Open `w:\soc3050code\projects\Serial_interrupt\Main.c`
2. Locate the main() function (around line 700)
3. Find the demo selection section:
   ```c
   // ========================================
   // SELECT DEMO: Uncomment ONE line below
   // ========================================
   
   //demo_basic_rx_interrupt();        // Demo 1: Basic RX interrupt with echo
   //demo_tx_interrupt_queue();        // Demo 2: TX interrupt queue system
   //demo_bidirectional_interrupts();  // Demo 3: Full duplex communication
   //demo_command_processing();        // Demo 4: Command processing system
   demo_advanced_buffering();          // Demo 5: Advanced buffering (currently enabled)
   ```
4. Comment out the currently active demo (prefix with `//`)
5. Uncomment the desired demo (remove `//`)
6. Build and program the project

### Building and Programming
```powershell
# Build the project
Ctrl+Shift+P -> "Tasks: Run Task" -> "Build Current Project"

# Program to device
Ctrl+Shift+P -> "Tasks: Run Task" -> "Program Current Project"
```

### Communication Setup
- **Baud Rate**: 9600
- **Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Port**: COM3 (or as configured)
- **Tool**: VS Code Serial Monitor extension

## WebDash Compatibility

The project now follows the standard pattern expected by the webdash system:
- ✅ Standard main() entry point
- ✅ Modular demo selection via commenting/uncommenting
- ✅ Consistent project structure
- ✅ Compatible with automated build systems
- ✅ Educational progression from basic to advanced concepts

## Technical Features

### Centralized Interrupt Management
- Uses the new `_interrupt_manager` system
- No ISR conflicts between libraries
- Priority-based callback registration
- Scalable architecture for complex projects

### Memory Optimization
- Reduced buffer sizes (32 bytes each) for ATmega128 constraints
- Efficient circular buffer implementation
- Minimal memory footprint while maintaining functionality

### Educational Value
- Progressive complexity from basic echo to advanced buffering
- Real-world interrupt programming techniques
- Professional embedded systems architecture
- Hands-on experience with serial communication protocols

## Testing Results

- ✅ Build system: All demos compile successfully
- ✅ Programming: HEX files generated and uploaded correctly  
- ✅ Communication: Serial interface working at 9600 baud
- ✅ Interrupts: Centralized management system operational
- ✅ Compatibility: WebDash integration confirmed

## Next Steps

Students can now:
1. Select any demo by uncommenting the function call
2. Build and test each demonstration independently
3. Study the source code for each interrupt technique
4. Modify and extend the demos for custom projects
5. Use this as a foundation for more complex interrupt-driven applications

The project provides a solid foundation for understanding interrupt-driven serial communication in embedded systems while maintaining compatibility with the broader educational framework.