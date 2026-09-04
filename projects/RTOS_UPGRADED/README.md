# Enhanced RTOS v2.0 for ATmega128

## 🎓 Educational RTOS Project

This is a Real-Time Operating System (RTOS) designed for **teaching embedded systems** and concurrent programming on the ATmega128 microcontroller.

### 📚 For Students

**Start here:** Read [STUDENT_GUIDE.md](STUDENT_GUIDE.md) to learn how to add your own tasks!

**Task Template:** See [TASK_TEMPLATE.c](TASK_TEMPLATE.c) for ready-to-use code examples.

---

## Overview

This RTOS implementation features cooperative multitasking with up to 10 concurrent tasks, fully compatible with the Simulator110.simu hardware configuration.

## Key Features

### RTOS Core

- **8 Concurrent Tasks**: Increased from 5 to 8 tasks
- **Enhanced Priority Scheduling**: 5 priority levels with intelligent round-robin
- **Task Statistics**: Execution count, runtime tracking, and performance metrics
- **Auto-Restart Tasks**: Configurable task restart behavior
- **System Health Monitoring**: Watchdog, RAM monitoring, context switch tracking
- **Larger Stack Space**: 256 bytes per task (up from 128 bytes)

### Hardware Integration

- **Shared Library Support**: Leverages all shared libraries for hardware abstraction
  - `_init.h` - System initialization
  - `_uart.h` - UART communication
  - `_port.h` - GPIO management
  - `_timer.h` - Timer control
  - `_adc.h` - ADC operations
  - `_buzzer.h` - Buzzer control
  - `_pwm.h` - PWM motor control

### Task Definitions

1. **LED Sequence Task** (Normal Priority)
   - Sequentially lights up 8 LEDs on Port B
   - Creates visual feedback pattern
   - Period: 500ms per LED

2. **UART Status Reporter** (Normal Priority)
   - Reports system status every 2 seconds
   - Shows: tick count, task switches, report counter
   - Provides system health information

3. **ADC Monitor** (Normal Priority)
   - Reads potentiometer on PORTF0
   - Reports value and percentage
   - Sample rate: Every 1.5 seconds

4. **Button Monitor** (High Priority)
   - Monitors all 8 buttons on Port D
   - Debounced input handling
   - Counts button presses per pin
   - Provides buzzer feedback

5. **Motor Control** (Normal Priority)
   - PWM motor speed control
   - Automatic duty cycle sweep
   - Range: 0-100% with smooth transitions
   - Update rate: 300ms

6. **System Watchdog** (Low Priority)
   - Monitors system health
   - Reports free RAM
   - Ensures system stability
   - Check interval: 5 seconds

7. **Heartbeat LED** (High Priority)
   - System alive indicator on PB7
   - 1Hz blink rate
   - Visual confirmation of operation

8. **Statistics Reporter** (Low Priority)
   - Comprehensive RTOS statistics
   - Task execution counts
   - Task states
   - Report interval: 10 seconds

## Hardware Configuration (Simulator110.simu)

### ATmega128 Connections

**Port B (LEDs):**

- PB0-PB6: Sequence LEDs with 100Ω resistors
- PB7: Heartbeat LED with 100Ω resistor

**Port D (Buttons):**

- PD0-PD7: Push buttons (active low with pull-ups)
- Internal pull-ups enabled

**Port F (Analog):**

- PF0: Potentiometer input (0-5V)
- Additional ADC channels available

**UART1:**

- TX (PD3): Serial output
- RX (PD2): Serial input
- Baud rate: 9600
- Format: 8N1

**PWM (Motor Control):**

- PB5 (OC1A): PWM output for motor control
- PB6 (OC1B): Secondary PWM output

**Additional Components:**

- Buzzer on designated GPIO
- Stepper motor interface (Port B 0-3)
- DC motor with L298P driver
- Servo motor control

## Building the Project

### Using Build Script (Recommended)

```batch
cd projects\RTOS_UPGRADED
build.bat
```

This will:

1. Compile all source files with shared libraries
2. Link everything together
3. Generate Main.elf and Main.hex
4. Display memory usage statistics
5. Show next steps for simulation

### Manual Build

```batch
avr-gcc -mmcu=atmega128 -DF_CPU=16000000UL -DBAUD=9600 -Os -Wall -std=gnu99 ^
  -I../../shared_libs ^
  Main.c ^
  ../../shared_libs/_init.c ^
  ../../shared_libs/_uart.c ^
  ../../shared_libs/_port.c ^
  ../../shared_libs/_timer.c ^
  ../../shared_libs/_adc.c ^
  ../../shared_libs/_buzzer.c ^
  ../../shared_libs/_pwm.c ^
  -o Main.elf

avr-objcopy -O ihex -R .eeprom Main.elf Main.hex
```

## Running in SimulIDE

### Quick Start

1. **Open SimulIDE**

   ```
   tools\simulide\SimulIDE_1.1.0-SR1_Win64\simulide.exe
   ```

2. **Load Circuit**
   - File → Open → `tools\simulide\Simulator110.simu`
   - Circuit contains all required components already connected

3. **Load Firmware**
   - Right-click on ATmega128
   - Select "Load Firmware"
   - Navigate to `projects\RTOS_UPGRADED\Main.hex`

4. **Connect Serial Monitor**
   - SerialPort component already in circuit
   - Double-click to open Serial Monitor
   - Verify: 9600 baud, 8N1

5. **Start Simulation**
   - Click Play button or press F5
   - Observe LED sequences, UART output, button responses

### Expected Behavior

**Visual:**

- LEDs on PB0-PB6 sequence in order
- Heartbeat LED (PB7) blinks at 1Hz
- Button presses show immediate response

**Serial Output:**

```
========================================
  Enhanced RTOS v2.0 for ATmega128
  Hardware-Integrated System
========================================

Initializing tasks...
[OK] Task 1: LED Sequence (Normal Priority)
[OK] Task 2: UART Status Reporter
[OK] Task 3: ADC Monitor
[OK] Task 4: Button Handler (High Priority)
[OK] Task 5: PWM Motor Control
[OK] Task 6: System Watchdog
[OK] Task 7: Heartbeat LED
[OK] Task 8: Statistics Reporter

All tasks initialized successfully!

========================================
  Enhanced RTOS v2.0 for ATmega128
  Integrated with Shared Libraries
========================================
Active Tasks: 8

[Status] Report #0 | Ticks: 200 | Task Switches: 160
[ADC] Channel 0: 512 (50%)
[PWM] Motor duty: 50/255
[Button] PD0 pressed! Count: 1
[Watchdog] System health: OK | Free RAM: 2048 bytes

=== RTOS Statistics ===
System Ticks: 1000
Context Switches: 800
Active Tasks: 8
Task 0: LED Sequence [READY] Exec: 200
Task 1: UART Status [READY] Exec: 100
...
```

## Configuration

### RTOS Parameters

Edit in `Main.c`:

```c
#define MAX_TASKS 8           // Maximum concurrent tasks
#define STACK_SIZE 256        // Stack per task (bytes)
#define TIMER_TICK_MS 10      // Scheduler tick (ms)
```

### Task Creation

```c
uint8_t task_id = rtos_create_task(
    task_function,           // Function pointer
    "Task Name",            // Task name (max 19 chars)
    PRIORITY_NORMAL,        // Priority level
    true                    // Auto-restart
);
```

### Task Priority Levels

```c
PRIORITY_IDLE      = 0  // Lowest
PRIORITY_LOW       = 1
PRIORITY_NORMAL    = 2
PRIORITY_HIGH      = 3
PRIORITY_CRITICAL  = 4  // Highest
```

## Advanced Features

### Task Statistics

Each task maintains:

- Execution count
- Total runtime
- Current state
- Delay ticks remaining

Access via `rtos_print_stats()` or individual TCB inspection.

### Task Suspension/Resume

```c
rtos_suspend_task(task_id);  // Pause task
rtos_resume_task(task_id);   // Resume task
```

### Task Delay

```c
task_delay(100);  // Delay 100 ticks (1 second at 10ms tick)
```

### System Ticks

```c
uint32_t ticks = rtos_get_ticks();  // Get current system time
```

## Memory Usage

**Program Size:** ~6-8 KB (Flash)
**RAM Usage:** ~2.5-3 KB

- Task stacks: 8 × 256 = 2048 bytes
- TCBs: 8 × ~180 = 1440 bytes
- Globals and stack: ~512 bytes

**Total RAM:** ~4 KB of 8 KB (50%)
**Flash:** ~8 KB of 128 KB (6%)

## Troubleshooting

### Build Errors

**"avr-gcc not found"**

- Check AVR toolchain installation
- Verify paths in `build.bat`

**"Shared library not found"**

- Ensure shared_libs directory exists
- Check include paths

### Runtime Issues

**Tasks not running**

- Verify task creation succeeded
- Check task priorities
- Ensure scheduler started (`rtos_start()`)

**No UART output**

- Verify UART initialization
- Check baud rate (9600)
- Connect Serial Monitor in SimulIDE

**LEDs not working**

- Check Port B configuration (all outputs)
- Verify circuit connections in SimulIDE
- Check LED component polarities

### Performance Issues

**Task overruns**

- Increase `TIMER_TICK_MS`
- Reduce task complexity
- Check `sys_stats.task_overruns`

**Stack overflow**

- Increase `STACK_SIZE`
- Reduce local variables
- Minimize recursion

## Comparison with Basic RTOS

| Feature | Basic RTOS | Enhanced RTOS v2.0 |
|---------|------------|-------------------|
| Max Tasks | 5 | 8 |
| Stack Size | 128 bytes | 256 bytes |
| Shared Libraries | No | Yes |
| Task Statistics | No | Yes |
| Auto-Restart | No | Yes |
| RAM Usage | ~800 bytes | ~4000 bytes |
| Hardware Integration | Basic | Full |
| Documentation | Basic | Comprehensive |

## Educational Concepts

### Demonstrated Concepts

1. **Hardware Abstraction**
   - Shared library usage
   - Portable code design
   - Modular architecture

2. **Real-Time Scheduling**
   - Priority-based scheduling
   - Round-robin for equal priority
   - Context switching

3. **Resource Management**
   - Stack allocation
   - Memory monitoring
   - Task lifecycle

4. **System Integration**
   - Multi-peripheral coordination
   - UART communication
   - ADC sampling
   - PWM motor control

## Future Enhancements

Possible additions:

- Inter-task communication (queues, semaphores)
- Dynamic task creation/deletion
- Full preemptive context switching
- CPU usage calculation
- Task deadlock detection
- Power management integration

## References

- ATmega128 Datasheet
- Shared Library Documentation (`shared_libs/`)
- SimulIDE User Manual
- FreeRTOS (comparison reference)

## License

Educational use - SOC3050 Course Material

## Author

Enhanced RTOS v2.0 - ATmega128 Educational Project
Integrated with Shared Library Ecosystem
