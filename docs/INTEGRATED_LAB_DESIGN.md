# INTEGRATED LAB DESIGN GUIDE

**Comprehensive Hands-On Experience for SOC 3050 Embedded Systems**

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
Lab.c Structure (New):
┌─────────────────────────────────────────────────┐
│          INTEGRATED DASHBOARD SYSTEM             │
├─────────────────────────────────────────────────┤
│  Major Topic (e.g., Timer)                      │
│     ↓                                            │
│  + Interrupt Service Routines (ISR)             │
│     ↓                                            │
│  + Graphics LCD Display (User Interface)        │
│     ↓                                            │
│  + Serial Communication (Debug/Control)         │
│     ↓                                            │
│  + Button Handling (User Input)                 │
│     ↓                                            │
│  = COMPLETE EMBEDDED SYSTEM                     │
└─────────────────────────────────────────────────┘

Benefits:
✓ Students see how subsystems integrate
✓ Real-world application builds motivation
✓ Comprehensive understanding emerges
✓ Debugging skills across multiple domains
✓ Industry-standard development patterns
```

## The Integration Pattern

### Core Formula
```
Lab.c = Major Topic + Graphics + Interrupts + Serial + UI
```

### Example: Timer_Programming/Lab.c

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

### 1. Serial_Communications/Lab.c

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

### 2. I2C_Master_Basic/Lab.c

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

### 3. PWM_Motor_DC/Lab.c

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

### 4. ADC_Basic/Lab.c

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

### 5. Interrupt/Lab.c

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

### 6. SPI_Master_Basic/Lab.c

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

### 1. File Structure

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

## Checklist: Is This an Integrated Lab?

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

❌ **NOT Integrated** (Avoid):
- Isolated exercises (Exercise 1, Exercise 2, ...)
- Serial output only (no GLCD)
- No interrupts (all polling)
- Hardcoded behavior (no user input)
- Trivial demos ("blink LED with serial command")

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

### Industry Readiness
1. **Professional Pattern**: Multi-subsystem integration is standard
2. **Portfolio Piece**: Labs become impressive demo projects
3. **Interview Prep**: "Tell me about a complex embedded system you built"
4. **Confidence**: "I can build complete embedded products"

## Migration Strategy

### Rewriting Existing Labs

1. **Audit Current Lab.c**:
   - Identify major topic
   - Note any existing integration
   - List missing subsystems (GLCD? Serial? ISR?)

2. **Design Integrated Application**:
   - What real-world device uses this major topic?
   - How can GLCD visualize it?
   - What serial commands make sense?
   - What ISRs are needed?

3. **Implement Incrementally**:
   - Start with major topic (ensure it works)
   - Add ISRs (non-blocking updates)
   - Add basic GLCD display (one mode)
   - Add serial commands (minimal set)
   - Add more display modes
   - Add button handling
   - Polish UI and help text

4. **Test Integration**:
   - Build and upload to hardware
   - Verify all subsystems work together
   - Test edge cases (button during serial command, etc.)
   - Have student beta-test for clarity

5. **Document**:
   - Update Lab.c header comments
   - Add serial help (? command)
   - Update project README if exists

### Example: Rewriting PWM_Motor_DC/Lab.c

**Current**: Isolated PWM speed demos (10%, 50%, 100%)

**Integrated Design**:
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

### Code Templates
- `projects/Timer_Programming/Lab.c` - Reference implementation
- `shared_libs/_glcd.h` - Graphics LCD API reference
- `shared_libs/_uart.h` - Serial communication API
- `shared_libs/_port.h` - Button handling functions

### Documentation
- `docs/LIBRARY_REFERENCE.md` - Shared library function catalog
- `docs/HARDWARE_REFERENCE.md` - Pin assignments, peripherals
- `docs/PROJECT_CATALOG.md` - Overview of all 33 projects

### Tools
- SimulIDE - Hardware simulation (test without physical board)
- Serial Monitor - Test UART communication
- Build system - `tools/cli/cli-build-project.ps1`

## Conclusion

**Integrated labs transform learning**:
- Students don't just understand timers—they build timer-based systems
- Students don't just know I2C—they create sensor dashboards
- Students don't just learn interrupts—they master real-time event handling

**This approach prepares students for industry**, where embedded systems are always integrated, always real-time, and always user-facing.

**Next Steps**:
1. Review `Timer_Programming/Lab.c` as reference
2. Identify 3-5 projects to rewrite next
3. Apply integration pattern iteratively
4. Gather student feedback
5. Iterate and improve

---

**Document Version**: 1.0  
**Last Updated**: 2025-01-15  
**Author**: SOC 3050 Teaching Team  
**Related**: `COMBINING_MODULES_GUIDE.md`, `PROJECT_MODULARITY_GUIDE.md`
