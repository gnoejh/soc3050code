# Project Modularity & Independence Guide

## Overview
The `soc3050code` repository is designed for **modular teaching** where each project demonstrates a specific peripheral or concept independently. This guide explains how to maintain modularity and handle peripheral conflicts when combining modules.

## Current Architecture

### Directory Structure
```
soc3050code/
├── projects/          # Independent teaching modules
│   ├── Timer_Programming/
│   ├── Serial_Communications/
│   ├── Graphics_Display/
│   └── [32 other projects]
├── shared_libs/       # Reusable peripheral libraries
│   ├── _init.h/.c     # Initialization functions
│   ├── _uart.h/.c     # UART library
│   ├── _glcd.h/.c     # Graphics LCD library
│   └── [other libraries]
└── tools/             # Build and simulation tools
    ├── cli/           # Command-line build scripts
    └── simulide/      # SimulIDE integration
```

### Design Principles

1. **Each Project is Standalone**
   - Every project in `projects/` can build and run independently
   - No dependencies between projects
   - Students work on ONE project at a time

2. **Shared Libraries are Modular**
   - Libraries in `shared_libs/` are reusable
   - Multiple projects can include the same library
   - Libraries don't have global state conflicts

3. **Portable Paths Only**
   - All includes use relative paths: `#include "../../shared_libs/_uart.h"`
   - Build system uses `-I../../shared_libs` flag
   - Works regardless of installation directory

## ISR (Interrupt Service Routine) Usage

### Current ISR Allocation by Project

| ISR Vector | Projects Using It |
|------------|------------------|
| `TIMER0_OVF_vect` | Timer_Programming, Power_Wakeup_Optimization |
| `TIMER1_COMPA_vect` | Timer_Programming |
| `TIMER2_OVF_vect` | Power_LowPower_Sensors, Power_Sleep_Modes, Power_Wakeup_Optimization |
| `INT0_vect` | Interrupt, Power_Sleep_Modes, Power_Wakeup_Optimization |
| `INT1_vect` | Power_Wakeup_Optimization |
| `USART1_RX_vect` | Serial_Communications, Power_Wakeup_Optimization |
| `USART1_UDRE_vect` | Serial_Communications |
| `ADC_vect` | Power_LowPower_Sensors, Power_Wakeup_Optimization |

### Why ISR "Conflicts" Are Not a Problem

**ISRs are defined in Main.c (application layer), not in shared libraries:**
- ✅ Each project defines its own ISRs
- ✅ Projects never link together
- ✅ Students compile one Main.c at a time
- ✅ No linker errors occur

**Example:**
- `Timer_Programming/Main.c` defines `ISR(TIMER0_OVF_vect)` 
- `Power_Wakeup_Optimization/Main.c` also defines `ISR(TIMER0_OVF_vect)`
- **No conflict** because they are in different executables

## Combining Modules (Advanced Students)

When students want to combine multiple peripherals (e.g., Timer + UART + LCD), they should:

### Option 1: Copy from Multiple Projects (Recommended for Learning)

1. **Create a new project folder**
   ```
   projects/My_Combined_Project/
   ```

2. **Copy relevant code sections from each module:**
   - Timer initialization from `Timer_Programming/Main.c`
   - UART setup from `Serial_Communications/Main.c`
   - LCD functions from `Graphics_Display/Main.c`

3. **Merge ISRs carefully:**
   ```c
   // From Timer_Programming
   ISR(TIMER0_OVF_vect) {
       // Timer logic
   }
   
   // From Serial_Communications
   ISR(USART1_RX_vect) {
       // UART receive logic
   }
   
   // No conflict - different vectors
   ```

4. **Resolve ISR conflicts manually:**
   If two projects use the same ISR (e.g., both use `TIMER0_OVF_vect`):
   ```c
   // Combined ISR - merge logic from both sources
   ISR(TIMER0_OVF_vect) {
       // Logic from project A
       if (mode_A_active) {
           // ...
       }
       
       // Logic from project B
       if (mode_B_active) {
           // ...
       }
   }
   ```

### Option 2: Use Shared Libraries Only

For production code, use the modular shared libraries:

```c
#include "config.h"

int main(void) {
    // Use shared library initialization
    init_basic_io();     // From _init.h
    uart_init(9600);     // From _uart.h
    glcd_init();         // From _glcd.h
    
    sei();  // Enable global interrupts
    
    // Your application logic
    while(1) {
        // ...
    }
}
```

**Advantages:**
- ✅ No ISR conflicts (libraries don't define ISRs)
- ✅ Tested, reusable code
- ✅ Clean separation of concerns

**Disadvantages:**
- ❌ Less educational (hides implementation details)
- ❌ Students don't see low-level register manipulation

## Adding New Projects

When creating new teaching modules:

### 1. Choose Unique ISR Vectors When Possible

**Available ISR vectors (not heavily used):**
- `TIMER1_COMPB_vect` (Timer1 Compare Match B)
- `TIMER3_*` vectors (Timer3 family)
- `INT2_vect` through `INT7_vect` (External interrupts 2-7)
- `SPI_STC_vect` (SPI Transfer Complete)
- `TWI_vect` (I2C/TWI)
- `USART0_*` vectors (USART0 family)
- `ANALOG_COMP_vect` (Analog Comparator)

### 2. Document ISR Usage in config.h

```c
/*
 * Configuration Header - My New Project
 * 
 * ISR VECTORS USED:
 * - TIMER3_COMPA_vect: 1ms tick for scheduling
 * - INT2_vect: Button interrupt
 * - TWI_vect: I2C communication
 */
```

### 3. Keep Libraries ISR-Free

**Shared libraries should NOT define ISRs:**
```c
// ❌ BAD - Don't do this in shared libraries
ISR(USART1_RX_vect) {
    // Library-defined ISR creates conflicts
}

// ✅ GOOD - Provide callback mechanism instead
void uart_set_rx_callback(void (*callback)(uint8_t)) {
    user_rx_callback = callback;
}

// Application code defines the ISR
ISR(USART1_RX_vect) {
    uint8_t data = UDR1;
    if (user_rx_callback) {
        user_rx_callback(data);
    }
}
```

## Configuration Header (config.h) Best Practices

Each project should have its own `config.h`:

```c
#ifndef CONFIG_H_
#define CONFIG_H_

// Clock frequency (ATmega128 @ 16MHz)
#define F_CPU 16000000UL

// Standard AVR headers
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Include ONLY the shared libraries this project needs
#include "_init.h"      // If using shared initialization
#include "_uart.h"      // If using UART
// #include "_glcd.h"   // Commented out - not used in this project

// Forward declarations for application functions
void my_setup(void);
void my_loop(void);

#endif /* CONFIG_H_ */
```

**Key points:**
- ✅ Each project has its own `config.h`
- ✅ Include only needed libraries (reduces compile time & memory)
- ✅ Use relative paths or compiler search paths (`-I../../shared_libs`)
- ✅ Document peripheral usage

## Build System

The build system (`tools/cli/cli-build-project.ps1`) ensures modularity:

```powershell
# Portable include paths (relative to project directory)
$CommonFlags = "-I. -I../../shared_libs"

# Each project builds independently
& avr-gcc $CommonFlags Main.c -o Main.elf
```

**Features:**
- ✅ Portable (works on any Windows system)
- ✅ Each project compiles in isolation
- ✅ No global dependencies
- ✅ Includes only needed `.c` files from `shared_libs/`

## Testing Modularity

To verify a project is truly independent:

1. **Build Test:**
   ```powershell
   cd projects/Your_Project
   powershell -File ..\..\tools\cli\cli-build-project.ps1 -ProjectDir . -SourceFile Main.c
   ```

2. **Check includes:**
   ```powershell
   # Should only show relative paths or filenames
   Select-String -Path "*.c","*.h" -Pattern '#include'
   ```

3. **Verify no absolute paths:**
   ```powershell
   # Should return nothing
   Select-String -Path "*.c","*.h" -Pattern '[A-Z]:\\'
   ```

## Summary

✅ **Current Status: Fully Modular**
- Each project is independent
- ISRs are project-specific (in Main.c)
- Shared libraries are reusable
- All paths are portable

✅ **For Advanced Users:**
- Combine modules by copying code
- Merge ISRs manually when needed
- Use shared libraries for production

✅ **For Educators:**
- Each module teaches ONE concept
- Students can explore internals
- No hidden magic in libraries
- Easy to add new modules

---

**Last Updated:** November 3, 2025  
**Maintained by:** Professor Hong Jeong
