# INTEGRATED LAB DESIGN GUIDE

**Comprehensive Hands-On Experience for SOC 3050 Embedded Systems**

---

## ⚠️ IMPORTANT: Separate Projects Approach

**NEW STRATEGY (2025-11-03):**

Integrated labs should be **separate standalone projects**, not embedded within the existing educational demo projects.

**Directory Structure:**
```
soc3050code/
├── projects/                    # Educational demos (Main.c only)
│   ├── Timer_Programming/      # Focused timer demos
│   ├── ADC_Basic/              # Focused ADC demos
│   └── ...                     # Each project: ONE concept, clearly
│
└── integrated_labs/            # Comprehensive integrated systems
    ├── Timer_Dashboard/        # Timer + ISR + GLCD + Serial + Buttons
    ├── Sensor_Monitor/         # I2C + ADC + GLCD + Serial + Data logging
    ├── Motor_Controller/       # PWM + Encoder + GLCD + PID + Serial
    └── ...                     # Each lab: FULL system integration
```

**Rationale:**
- ✅ **Modularity**: Educational demos stay focused and simple
- ✅ **Clarity**: Students learn fundamentals first (Main.c), integration later (integrated_labs)
- ✅ **Flexibility**: Integrated labs can combine ANY modules without cluttering base projects
- ✅ **Maintenance**: Easier to update demos without breaking integrated systems

**Migration:**
- All `Lab.c` files removed from `projects/*/`
- Integrated lab concepts documented here for future implementation
- Create `integrated_labs/` folder structure when ready

---

## Philosophy: Integration Over Isolation

### Traditional Approach (OLD - Isolated Exercises)
```
Lab.c Structure (Old):
├── Exercise 1: LED blink (only Timer)
├── Exercise 2: PWM generation (only Timer)
├── Exercise 3: Musical tones (only Timer)
└── Exercise 4: Task scheduler (only Timer)

Problem:
- Students learn components in isolation
- No understanding of real-world system integration
- Lacks motivation (individual concepts are boring)
- Doesn't reflect actual embedded development
```

### New Approach (NEW - Integrated System)
```
Separate Integrated Lab Project Structure:
┌─────────────────────────────────────────────────┐
│   INTEGRATED LAB: TIMER DASHBOARD SYSTEM        │
│   (Standalone project in integrated_labs/)      │
├─────────────────────────────────────────────────┤
│  Main.c:                                        │
│    ├─ Timer (Timer0/1/2)                        │
│    ├─ Interrupt Service Routines (ISR)         │
│    ├─ Graphics LCD Display (User Interface)    │
│    ├─ Serial Communication (Debug/Control)     │
│    ├─ Button Handling (User Input)             │
│    └─ Task Scheduler (Integration)             │
│                                                 │
│  Includes from shared_libs:                     │
│    #include "_init.h"                           │
│    #include "_uart.h"                           │
│    #include "_glcd.h"                           │
│    #include "_port.h"                           │
│                                                 │
│  = COMPLETE EMBEDDED SYSTEM                     │
└─────────────────────────────────────────────────┘

Educational projects/ folder:
┌─────────────────────────────────────────────────┐
│   EDUCATIONAL PROJECT: TIMER_PROGRAMMING        │
│   (Focused demos in projects/ folder)           │
├─────────────────────────────────────────────────┤
│  Main.c only:                                   │
│    ├─ demo1_normal_polling()                    │
│    ├─ demo2_normal_interrupt()                  │
│    ├─ demo3_ctc_polling()                       │
│    ├─ demo4_ctc_interrupt()                     │
│    ├─ demo5_fast_pwm()                          │
│    └─ ...                                       │
│                                                 │
│  Focus: ONE concept demonstrated clearly        │
│  Minimal includes (local simple_init only)      │
│  = EDUCATIONAL CLARITY                          │
└─────────────────────────────────────────────────┘

Benefits:
✓ Students learn fundamentals FIRST (projects/)
✓ Students tackle integration SECOND (integrated_labs/)
✓ No mixing of simple demos with complex systems
✓ Each project stays focused and maintainable
✓ Integration labs can combine ANY modules freely
```

## The Integration Pattern

### Core Formula
```
Integrated Lab Project = Major Topic + Graphics + Interrupts + Serial + UI

Location: integrated_labs/[ProjectName]/
Structure: Main.c (all-in-one integrated system)
          config.h (includes all shared libraries)
          README.md (learning objectives, setup instructions)
```

### Example: integrated_labs/Timer_Dashboard/

**Major Topic**: Timer/Counter Programming
**Integration Components**:
1. **Timer2** → Real-Time Clock (hours:minutes:seconds)
2. **Timer1** → Stopwatch with lap times (millisecond precision)
3. **Timer0** → PWM LED brightness control
4. **ISRs** → TIMER2_OVF_vect, TIMER1_COMPA_vect
5. **Graphics** → 128x64 GLCD with 4 display modes
6. **Serial** → UART commands (+, -, 0-9, S, L, R, M, ?)
7. **Buttons** → MODE, START/STOP, LAP/RESET

**Result**: Students build a multi-function timer dashboard that demonstrates:
- Timer configuration for different purposes (RTC vs Stopwatch vs PWM)
- ISR-based event handling (non-blocking updates)
- Graphics rendering with real-time data
- Serial command parsing and response
- Cooperative multitasking
- Responsive user interface

## Application to Other Projects

**IMPORTANT**: These are designs for future **separate integrated lab projects**, NOT additions to existing projects/*/Main.c files.

Create these as standalone projects in `integrated_labs/` folder.

### 1. integrated_labs/Serial_Terminal/

**Major Topic**: UART Communication

**Integration**:
```c
Serial Communication (UART)
  + Graphics LCD (display incoming data, buffer status)
  + Timer (periodic transmission, timeout detection)
  + Interrupts (RX/TX ISRs)
  + Buttons (send commands)
  
Real-World Application:
  → Serial Terminal with graphical monitor
  → Data logging with timestamps
  → Command interpreter with visual feedback
```

**Feature Ideas**:
- Display incoming serial data on GLCD
- Show TX/RX buffer status (circular buffer visualization)
- Implement AT command parser (like modems)
- Display transmission statistics (bytes sent/received, errors)
- Timer-based auto-send at intervals
- Button shortcuts for common commands

### 2. integrated_labs/I2C_Sensor_Dashboard/

**Major Topic**: I2C Communication

**Integration**:
```c
I2C Communication
  + Graphics LCD (I2C device scanner, data display)
  + Serial (log I2C transactions, debug output)
  + Timer (periodic sensor polling)
  + Interrupts (I2C state machine)
  
Real-World Application:
  → Multi-sensor dashboard
  → I2C bus analyzer
  → Sensor data logger with visualization
```

**Feature Ideas**:
- I2C bus scanner showing detected devices
- Real-time sensor readings on GLCD (temp, accel, RTC)
- Graph sensor trends over time
- Serial logging of all I2C transactions
- Timer-triggered multi-sensor polling
- Error detection and recovery visualization

### 3. integrated_labs/Motor_Speed_Controller/

**Major Topic**: Motor Control

**Integration**:
```c
PWM Motor Control
  + Graphics LCD (speed display, control panel)
  + Serial (speed commands, PID tuning)
  + Timer (encoder reading, speed calculation)
  + Interrupts (encoder ISRs, control loop)
  
Real-World Application:
  → Motor speed controller with tachometer
  → PID tuning interface
  → Performance monitoring dashboard
```

**Feature Ideas**:
- Real-time speed display with bar graph
- Target vs actual speed comparison
- Encoder pulse counting via ISR
- Serial commands to set speed (0-100%)
- PID parameter tuning via serial
- Acceleration/deceleration curves visualization
- Button controls: START, STOP, SPEED_UP, SPEED_DOWN

### 4. integrated_labs/Multi_Channel_Oscilloscope/

**Major Topic**: Analog-to-Digital Conversion

**Integration**:
```c
ADC Conversion
  + Graphics LCD (voltage bar graph, waveform)
  + Serial (data stream, CSV export)
  + Timer (sampling rate control)
  + Interrupts (ADC conversion complete)
  
Real-World Application:
  → Multi-channel oscilloscope
  → Voltage monitor with alerts
  → Data acquisition system
```

**Feature Ideas**:
- 4-channel ADC monitoring on GLCD
- Bar graphs for voltage levels
- Min/Max/Avg statistics per channel
- Serial output: CSV format for plotting
- Timer-controlled sampling (adjustable rate)
- Threshold alerts (visual + serial)
- Waveform display (simple oscilloscope mode)

### 5. integrated_labs/Event_Logger/

**Major Topic**: Interrupt Programming

**Integration**:
```c
External Interrupts
  + Graphics LCD (event log, count display)
  + Serial (interrupt statistics)
  + Timer (debouncing, event timestamps)
  + Multiple ISRs (INT0-7, PCINT, Timer)
  
Real-World Application:
  → Event logger with timestamps
  → Interrupt priority demonstration
  → Multi-source event monitoring
```

**Feature Ideas**:
- Event counter for each interrupt source
- Last 10 events displayed on GLCD with timestamps
- Interrupt latency measurement
- Priority demonstration (nested interrupts)
- Debounce implementation with Timer
- Serial log of all events with microsecond precision

### 6. integrated_labs/SPI_Data_Recorder/

**Major Topic**: SPI Communication

**Integration**:
```c
SPI Communication
  + Graphics LCD (SPI device status, data)
  + Serial (transaction logging)
  + Timer (periodic polling)
  + Interrupts (SPI transfer complete)
  
Real-World Application:
  → SPI EEPROM programmer
  → SD card file browser
  → Multi-device SPI bus manager
```

**Feature Ideas**:
- SPI device enumeration display
- Read/write EEPROM with progress bar
- File listing from SD card on GLCD
- Serial commands: READ, WRITE, ERASE
- Transaction statistics (speed, errors)
- Chip select management for multiple devices

## Implementation Guidelines

### 1. Project Structure (Standalone Integrated Labs)

**Create new folder**: `integrated_labs/[ProjectName]/`

```
integrated_labs/Timer_Dashboard/
├── Main.c              # Complete integrated system
├── config.h            # Include all needed libraries
├── README.md           # Learning objectives, setup, usage
├── circuit.simu        # SimulIDE circuit file
└── Makefile            # Build configuration (optional)
```

**Main.c Template:**

```c
/* ========================================
 * INTEGRATED [PROJECT_NAME] LAB
 * ========================================
 * Learning Objectives:
 *   ✓ Major Topic (detailed)
 *   ✓ ISR implementation
 *   ✓ Graphics LCD integration
 *   ✓ Serial communication
 *   ✓ User interface design
 * 
 * Real-World Application:
 *   Brief description of what students build
 * ========================================
 */

#include "config.h"
#include <stdio.h>
#include <string.h>

// System configuration
#define CONSTANTS

// Global state
volatile uint8_t variables;

// ISRs
ISR(INTERRUPT_VECTOR) {
    // Real-time event handling
}

// Major topic functions
void major_topic_init(void);
void major_topic_function(void);

// Graphics LCD functions
void draw_screen_mode1(void);
void draw_screen_mode2(void);
void update_screen(void);

// Serial interface
void handle_serial_command(char cmd);
void print_help(void);

// User interface
void handle_buttons(void);

// Main program
int main(void) {
    // Initialize all subsystems
    init_devices();
    Uart1_init();
    lcd_init();
    // ... other inits
    
    sei();  // Enable interrupts
    
    // Welcome screen + serial
    // Main loop integrating all systems
    while(1) {
        handle_buttons();
        if (serial_available()) handle_serial_command();
        update_screen();
        _delay_ms(10);
    }
}
```

### 2. User Interface Modes

Each Lab.c should have **3-5 display modes** switchable via button or serial:

```c
typedef enum {
    MODE_OVERVIEW,    // Dashboard with all info
    MODE_DETAIL_1,    // Deep dive into aspect 1
    MODE_DETAIL_2,    // Deep dive into aspect 2
    MODE_SETTINGS,    // Configuration interface
    MODE_DEBUG        // Debug/diagnostic info
} SystemMode;
```

### 3. Serial Commands

Standard command set across all labs:

```
Common Commands:
  ?  - Help (list all commands)
  M  - Next Mode (cycle through display modes)
  I  - Info (system status)
  R  - Reset (restart system)

Project-Specific:
  [Custom commands relevant to major topic]
  
Example (Timer Lab):
  S - Stopwatch Start/Stop
  L - Lap time
  + - Increase brightness
  - - Decrease brightness
  0-9 - Quick level setting
```

### 4. Graphics LCD Layout

**Standard Layout** (128x64 pixels, 8 rows × 20 columns):

```
Row 0: ====================
Row 1:   [SCREEN TITLE]
Row 2: ====================
Row 3: [Main Data Display]
Row 4: [Secondary Info]
Row 5: [Status/Progress]
Row 6: [Contextual Info]
Row 7: [Instructions/Help]
```

**Available Functions** (from _glcd.h):
- `lcd_clear()` - Clear entire screen
- `lcd_init()` - Initialize GLCD
- `lcd_xy(row, col)` - Position cursor (0-7, 0-19)
- `lcd_char(ch)` - Print single character
- `lcd_string(row, col, str)` - Print string
- `GLCD_Rectangle(x1, y1, x2, y2)` - Draw rectangle (pixel coords 0-127, 0-63)
- `GLCD_Line(x1, y1, x2, y2)` - Draw line
- `GLCD_Circle(x, y, r)` - Draw circle
- `GLCD_Dot(x, y)` - Draw pixel
- `GLCD_2DigitDecimal(num)` - Print 2-digit number

### 5. Common Patterns

#### Real-Time Updates (ISR Pattern)
```c
volatile uint16_t counter = 0;
volatile uint8_t update_flag = 0;

ISR(TIMER_OVF_vect) {
    counter++;
    if (counter % 100 == 0) {
        update_flag = 1;  // Signal main loop
    }
}

int main(void) {
    while(1) {
        if (update_flag) {
            update_flag = 0;
            update_screen();  // Update GLCD
        }
    }
}
```

#### Serial Command Parser
```c
void handle_serial_command(char cmd) {
    char msg[30];
    
    switch (cmd) {
        case '?':
            print_help();
            break;
        case 'M':
            current_mode = (current_mode + 1) % MODE_COUNT;
            break;
        case '+':
            value++;
            sprintf(msg, "Value: %d\r\n", value);
            puts_USART1(msg);
            break;
        // ...
    }
}
```

#### Multi-Mode Display
```c
void update_screen(void) {
    switch (current_mode) {
        case MODE_OVERVIEW:
            draw_overview_screen();
            break;
        case MODE_DETAIL:
            draw_detail_screen();
            break;
        // ...
    }
}
```

## Educational Progression

### Complexity Ladder

**Early Labs** (Simple Integration):
- 2-3 display modes
- Basic serial commands (5-10 commands)
- 1-2 ISRs
- Simple button handling
- Text-based GLCD (no graphics)

**Example**: ADC_Basic/Lab.c
- MODE_VOLTAGE: Show voltage readings
- MODE_BAR_GRAPH: Bar graph visualization
- Serial: Read channel, set sampling rate

**Mid Labs** (Moderate Integration):
- 4 display modes
- Advanced serial commands (10-15 commands)
- 3-4 ISRs
- Button debouncing
- Mixed text + simple graphics (bars, boxes)

**Example**: Serial_Communications/Lab.c
- MODE_TERMINAL: Scrolling text display
- MODE_BUFFER: Circular buffer visualization
- MODE_STATS: TX/RX statistics
- MODE_SETTINGS: Baud rate, parity config

**Advanced Labs** (Full Integration):
- 5 display modes
- Comprehensive serial interface (15+ commands)
- Complex ISR interactions
- Advanced UI (menus, settings)
- Full graphics (graphs, waveforms, animations)

**Example**: I2C_Sensors_Multi/Lab.c
- MODE_DASHBOARD: All sensors at once
- MODE_ACCELEROMETER: 3-axis graph
- MODE_TEMPERATURE: Trend plot
- MODE_RTC: Clock with calendar
- MODE_I2C_SCANNER: Bus enumeration

## Checklist: Is This a Proper Integrated Lab?

✅ **Structure Criteria**:
- [ ] Located in `integrated_labs/` folder (NOT in projects/)
- [ ] Standalone project (can build independently)
- [ ] Has README.md with clear learning objectives
- [ ] Includes SimulIDE circuit file for testing

✅ **Integration Criteria**:
- [ ] Major topic demonstrated comprehensively
- [ ] At least 1 ISR actively used
- [ ] Graphics LCD displays real-time data
- [ ] Serial commands control system
- [ ] Buttons provide user input
- [ ] Multiple subsystems interact (not just coexist)
- [ ] Real-world application is clear
- [ ] Students motivated by functionality
- [ ] Debugging requires multi-domain knowledge

❌ **NOT an Integrated Lab** (These belong in projects/):
- Single-concept demos ("blink LED")
- Only one subsystem used
- No user interaction
- No real-world application
- Isolated exercises (Exercise 1, Exercise 2, ...)

❌ **DON'T Mix**:
- Don't add Lab.c to existing projects/* folders
- Don't complicate educational Main.c demos
- Don't blur the line between "learning fundamentals" and "integration"

## Benefits for Students

### Technical Skills
1. **System Thinking**: Understand how subsystems interact
2. **Debugging**: Trace bugs across Timer→ISR→GLCD→Serial chain
3. **Real-Time**: Experience timing challenges and solutions
4. **UI Design**: Make embedded systems user-friendly
5. **Documentation**: Code clarity matters when integrating complex systems

### Soft Skills
1. **Motivation**: Building a "real product" is exciting
2. **Creativity**: Students experiment with features
3. **Problem Solving**: Integration bugs require deeper thinking
4. **Communication**: Explaining complex systems builds understanding
5. **Project Management**: Larger codebases need organization

### Industry Readiness
1. **Professional Pattern**: Multi-subsystem integration is standard
2. **Portfolio Piece**: Integrated labs become impressive demo projects
3. **Interview Prep**: "Tell me about a complex embedded system you built"
4. **Confidence**: "I can build complete embedded products"

## Migration Strategy

### Creating New Integrated Labs

**IMPORTANT**: Don't modify existing `projects/` - create new `integrated_labs/` structure.

1. **Create Folder Structure**:
   ```powershell
   mkdir integrated_labs
   mkdir integrated_labs/Timer_Dashboard
   ```

2. **Design Integrated Application**:
   - What real-world device uses this major topic?
   - How can GLCD visualize it?
   - What serial commands make sense?
   - What ISRs are needed?

3. **Implement From Scratch**:
   - Create Main.c (not Lab.c!)
   - Create config.h with all includes
   - Write README.md with objectives
   - Create SimulIDE circuit
   - Test integration thoroughly

4. **Document**:
   - Clear learning objectives
   - Setup instructions
   - User guide (button/serial commands)
   - Expected behavior
   - Troubleshooting tips

### Example: Creating Timer_Dashboard

1. **Create project**:
   ```powershell
   mkdir integrated_labs/Timer_Dashboard
   cd integrated_labs/Timer_Dashboard
   ```

2. **Create Main.c** (600+ lines - comprehensive dashboard)

3. **Create config.h**:
   ```c
   #include <avr/io.h>
   #include <avr/interrupt.h>
   #include <util/delay.h>
   #include <stdio.h>
   
   #include "_init.h"
   #include "_uart.h"
   #include "_glcd.h"
   #include "_port.h"
   ```

4. **Create README.md**:
   ```markdown
   # Timer Dashboard - Integrated Lab
   
   ## Learning Objectives
   - Master Timer0/1/2 for different purposes
   - ISR-based real-time event handling
   - Graphics LCD user interface design
   - Serial command parsing
   - Multi-mode system architecture
   
   ## Features
   - Real-time clock (Timer2 ISR)
   - Stopwatch with laps (Timer1 ISR)
   - PWM LED control (Timer0)
   - 4 display modes
   - Serial command interface
   - Button controls
   
   ## Hardware
   - ATmega128 @ 16MHz
   - KS0108 GLCD
   - 3 push buttons (PD4, PD5, PD6)
   - 8 LEDs (PORTB)
   - UART1 @ 9600 baud
   
   ## Commands
   ...
   ```

5. **Build and test**:
   ```powershell
   # Use existing build system
   ../../tools/cli/cli-build-project.ps1 -ProjectDir . -SourceFile Main.c
   
   # Simulate
   ../../tools/simulide/cli-simulide.ps1 -ProjectDir .
   ```

### Reusing Code from Old Lab.c Files

The Timer_Programming/Lab.c that was created (before deletion) can be:
1. Retrieved from git history: `git show 4aed867:projects/Timer_Programming/Lab.c`
2. Saved as template for `integrated_labs/Timer_Dashboard/Main.c`
3. Adapted for standalone project structure
4. Enhanced with better documentation

**Git command to recover**:
```powershell
git show 4aed867:projects/Timer_Programming/Lab.c > integrated_labs/Timer_Dashboard/Main.c
```

**Current**: Isolated PWM speed demos (10%, 50%, 100%)

**Integrated Design** (for `integrated_labs/Motor_Speed_Controller/`):
```
Application: Motor Speed Controller Dashboard

Features:
- GLCD Display:
  * Current speed (RPM) with bar graph
  * Target speed setting
  * Encoder pulse count
  * Motor direction (CW/CCW)
  * Duty cycle percentage
  
- Serial Commands:
  * S<speed>  - Set target speed (S75 = 75%)
  * D<dir>    - Set direction (D1=CW, D0=CCW)
  * P<kp>     - Set PID Kp parameter
  * I<ki>     - Set PID Ki parameter
  * +/-       - Inc/dec speed by 10%
  * R         - Reset (stop motor)
  * ?         - Help
  
- ISRs:
  * TIMER1_COMPA - PWM generation
  * TIMER2_OVF - Encoder reading
  * INT0 - Encoder pulse counting
  
- Buttons:
  * START/STOP
  * SPEED_UP (+10%)
  * SPEED_DOWN (-10%)
  
Integration:
  User sets speed via button or serial
    → PWM duty cycle updated (Timer1)
    → Encoder counts pulses (INT0 ISR)
    → Speed calculated (Timer2 ISR, periodic)
    → GLCD shows target vs actual
    → Serial logs RPM every second
```

## Resources

### Existing Code
- **Git History**: Retrieve old Lab.c files
  - Timer_Programming/Lab.c: `git show 4aed867:projects/Timer_Programming/Lab.c`
  - Use as templates for new integrated labs

### Code Templates
- `projects/*/Main.c` - Educational demo patterns
- `shared_libs/_glcd.h` - Graphics LCD API reference
- `shared_libs/_uart.h` - Serial communication API
- `shared_libs/_port.h` - Button handling functions

### Documentation
- `docs/LIBRARY_REFERENCE.md` - Shared library function catalog
- `docs/HARDWARE_REFERENCE.md` - Pin assignments, peripherals
- `docs/PROJECT_CATALOG.md` - Overview of all 33 projects
- This document - Integration design patterns

### Tools
- SimulIDE - Hardware simulation (test without physical board)
- Serial Monitor - Test UART communication
- Build system - `tools/cli/cli-build-project.ps1`

## Conclusion

**Integrated labs transform learning, but they need proper structure**:

✅ **DO**:
- Create separate `integrated_labs/` folder
- Design comprehensive real-world applications
- Combine Timer+ISR+Graphics+Serial+Buttons
- Write clear documentation and READMEs
- Test thoroughly in SimulIDE and hardware

❌ **DON'T**:
- Mix Lab.c into projects/* folders
- Complicate educational Main.c demos
- Create half-integrated systems
- Sacrifice educational clarity for features

**Educational Flow**:
1. **projects/**: Students learn fundamentals (ONE concept at a time)
2. **integrated_labs/**: Students build systems (ALL concepts together)
3. **Result**: Deep understanding of both components AND integration

**This approach prepares students for industry**, where embedded systems are always integrated, always real-time, and always user-facing.

**Next Steps**:
1. Create `integrated_labs/` folder structure
2. Start with Timer_Dashboard (retrieve from git history)
3. Develop 2-3 more integrated labs as templates
4. Test with students
5. Iterate based on feedback
6. Eventually create 10-15 integrated labs covering all major topics

---

**Document Version**: 2.0 (Updated for separate projects approach)  
**Last Updated**: 2025-11-03  
**Author**: SOC 3050 Teaching Team  
**Related**: `COMBINING_MODULES_GUIDE.md`, `PROJECT_MODULARITY_GUIDE.md`
