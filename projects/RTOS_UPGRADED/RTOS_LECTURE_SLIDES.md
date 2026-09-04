# Real-Time Operating Systems (RTOS)

## ATmega128 Educational Project

---

## Course Overview

### What You Will Learn

- Real-Time Operating System concepts
- Cooperative multitasking
- Task scheduling and management
- Concurrent programming on embedded systems
- ATmega128 hardware integration

---

## Slide 1: What is an Operating System?

### Traditional OS Functions

- **Resource Management**: CPU, Memory, I/O
- **Process Scheduling**: Run multiple programs
- **Abstraction**: Hide hardware complexity
- **Services**: File system, networking, UI

### Examples

- Windows, Linux, macOS (Desktop)
- Android, iOS (Mobile)
- Embedded Linux (Raspberry Pi)

---

## Slide 2: What is Real-Time?

### Real-Time Definition

**"A system that must respond to events within a specific time constraint"**

### Examples of Real-Time Systems

- ✈️ **Aircraft control**: Must respond in milliseconds
- 🚗 **Airbag deployment**: Must trigger in microseconds
- 🏥 **Heart monitor**: Must detect within heartbeat cycle
- 🏭 **Industrial robots**: Must move with precision timing

### Not Real-Time

- Web browsing, Word processing, Gaming (some delay is OK)

---

## Slide 3: Types of Real-Time Systems

### Hard Real-Time

- Missing deadline = **SYSTEM FAILURE**
- Examples: Airbags, pacemakers, anti-lock brakes
- Consequences: Injury, death, damage

### Soft Real-Time

- Missing deadline = **DEGRADED PERFORMANCE**
- Examples: Video streaming, audio playback
- Consequences: Glitches, but system continues

### Our RTOS

- **Soft real-time** system
- Educational focus on concepts
- Tolerates some timing variations

---

## Slide 4: Why RTOS on Microcontrollers?

### Without RTOS (Super Loop)

```c
void main() {
    while(1) {
        read_sensors();
        update_display();
        check_buttons();
        control_motor();
        // All tasks run sequentially
        // Hard to manage timing!
    }
}
```

### Problems

- ❌ Hard to add new features
- ❌ Difficult timing coordination
- ❌ No priority management
- ❌ Code becomes messy

---

## Slide 5: With RTOS - Clean Structure

### RTOS Approach

```c
void task_sensors() {
    if (time_to_run) {
        read_sensors();
    }
}

void task_display() {
    if (time_to_run) {
        update_display();
    }
}

// Each task is independent!
// Scheduler manages execution
```

### Benefits

- ✅ Modular and organized
- ✅ Easy to add new tasks
- ✅ Clear timing for each task
- ✅ Independent development

---

## Slide 6: RTOS Key Concepts

### 1. Tasks (Threads)

- Independent units of work
- Each has its own execution flow
- Like mini-programs

### 2. Scheduler

- Decides which task runs when
- Manages task priorities
- Ensures fair execution

### 3. Timing

- Tasks run at specific intervals
- Coordinated by system tick (1ms)

### 4. Shared Resources

- Multiple tasks access same hardware
- Must coordinate access (LEDs, UART, etc.)

---

## Slide 7: Our RTOS Architecture

```
┌─────────────────────────────────────┐
│         Application Tasks           │
│  (LED, UART, ADC, Buttons, etc.)   │
├─────────────────────────────────────┤
│         RTOS Scheduler              │
│    (Round-Robin Task Selection)     │
├─────────────────────────────────────┤
│      Hardware Abstraction           │
│   (UART, ADC, GPIO, Timer, GLCD)   │
├─────────────────────────────────────┤
│         ATmega128 Hardware          │
│  (CPU, Memory, Peripherals, I/O)   │
└─────────────────────────────────────┘
```

---

## Slide 8: Task States

### Task State Diagram

```
    ┌─────────────┐
    │   READY     │ ◄──┐
    └──────┬──────┘    │
           │           │
     [Scheduler        │
      selects]         │
           │           │
           ▼           │
    ┌─────────────┐    │
    │   RUNNING   │────┤
    └──────┬──────┘    │
           │           │
     [Task            │
      completes]      │
           │           │
           └───────────┘
```

### States in Our RTOS

- **READY**: Task waiting to run
- **RUNNING**: Task currently executing
- **BLOCKED**: Task waiting (not used in current system)
- **TERMINATED**: Task finished

---

## Slide 9: Cooperative vs Preemptive

### Preemptive Multitasking

- Scheduler **interrupts** running task
- Forces task switch
- Used in Windows, Linux
- More complex, needs context saving

### Cooperative Multitasking (Our System)

- Tasks **voluntarily** give up control
- Task runs until it returns
- Simpler to implement
- Less overhead

### Why Cooperative for Learning?

- ✅ Easier to understand
- ✅ Simpler debugging
- ✅ No complex context switching
- ✅ Sufficient for many embedded systems

---

## Slide 10: System Tick - The Heartbeat

### Timer0 Configuration

```c
// Timer0 generates interrupt every 1ms
TCCR0 = (1 << CS01) | (1 << CS00);  // Prescaler 64
TCNT0 = 6;                          // Preload value
TIMSK |= (1 << TOIE0);              // Enable interrupt

// In ISR:
ISR(TIMER0_OVF_vect) {
    TCNT0 = 6;
    system_ticks++;  // Global time keeper
}
```

### System Ticks

- Increments every **1 millisecond**
- Used by all tasks for timing
- Foundation of RTOS timing

---

## Slide 11: Task Structure

### Anatomy of a Task

```c
void task_example(void)
{
  // 1. Static variables (keep state)
  static uint32_t last_time = 0;
  static uint8_t counter = 0;
  
  // 2. Timing check
  if ((system_ticks - last_time) >= 100) {
    
    // 3. Do work
    uart_puts("Count: ");
    uart_print_num(counter++);
    
    // 4. Update timing
    last_time = system_ticks;
  }
  
  // 5. Return (give control back)
}
```

### Key Points

1. Fast execution (no delays!)
2. Static variables preserve state
3. Time-based triggering
4. Clean exit

---

## Slide 12: The Scheduler

### Round-Robin Scheduling

```c
uint8_t rtos_scheduler(void) {
  // Find next ready task
  next_task = (current_task + 1) % task_count;
  
  while (next_task != start) {
    if (task_list[next_task].state == TASK_READY)
      return next_task;
    next_task = (next_task + 1) % task_count;
  }
}
```

### How It Works

1. Starts after current task
2. Finds next READY task
3. Wraps around (circular)
4. Fair distribution of CPU time

---

## Slide 13: Main Scheduler Loop

### The Control Loop

```c
while (1) {
  // 1. Find next ready task
  current_task = rtos_scheduler();
  
  // 2. Check if task is ready
  if (task_list[current_task].state == TASK_READY) {
    
    // 3. Mark as running
    task_list[current_task].state = TASK_RUNNING;
    
    // 4. Execute task
    task_list[current_task].task_function();
    
    // 5. Mark as ready again (auto-restart)
    task_list[current_task].state = TASK_READY;
  }
  
  _delay_us(100);  // Small delay
}
```

---

## Slide 14: Task Control Block (TCB)

### What is a TCB?

**Data structure storing task information**

```c
typedef struct {
  uint8_t task_id;              // Unique ID
  char task_name[20];           // Name for debugging
  uint16_t stack_pointer;       // (Not used in simple system)
  uint8_t stack[STACK_SIZE];    // Reserved memory
  TaskState state;              // Current state
  TaskPriority priority;        // Importance level
  uint32_t delay_ticks;         // For blocking
  uint32_t execution_count;     // Statistics
  void (*task_function)(void);  // Function pointer
  bool auto_restart;            // Restart after completion
} TCB;
```

---

## Slide 15: Memory Organization

### Memory Layout

```
┌─────────────────────┐ 0x0000
│   Code (Flash)      │ Program memory
│   - RTOS code       │ (64KB - 128KB)
│   - Task functions  │
│   - Libraries       │
├─────────────────────┤
│   Data (RAM)        │ 
│   - Global vars     │ (4KB)
│   - Task stacks     │ 10 × 128 = 1280 bytes
│   - TCB array       │ 
│   - Heap            │
└─────────────────────┘ 0x10FF (4KB end)
```

### Resource Limits

- **RAM**: 4KB total
- **Stack per task**: 128 bytes
- **Max tasks**: 10
- Must manage carefully!

---

## Slide 16: Example - LED Sequence Task

### Task Implementation

```c
void task_led_sequence(void) {
  static uint8_t led_pattern = 0;
  static uint32_t last_update = 0;

  if ((system_ticks - last_update) >= 50) {
    // Create pattern (active-LOW)
    uint8_t pattern = ~(1 << led_pattern);
    led_sequence_bits = pattern | 0x80;
    
    // Update hardware
    cli();
    PORTB = led_sequence_bits & led_heartbeat_bit;
    sei();
    
    // Next LED
    led_pattern = (led_pattern + 1) % 7;
    last_update = system_ticks;
  }
}
```

### Features

- Runs every 50ms
- Cycles through 7 LEDs
- Coordinates with heartbeat task

---

## Slide 17: Synchronization - Shared Resources

### Problem: Multiple Tasks Access PORTB

```c
// Task 1: LED Sequence
PORTB = led_sequence_bits;

// Task 2: Heartbeat
PORTB = heartbeat_value;

// CONFLICT! They overwrite each other!
```

### Solution: Shadow Registers

```c
// Each task updates its shadow
led_sequence_bits = pattern;
led_heartbeat_bit = mask;

// Combine atomically
cli();  // Disable interrupts
PORTB = led_sequence_bits & led_heartbeat_bit;
sei();  // Enable interrupts
```

---

## Slide 18: Critical Sections

### What is a Critical Section?

**Code that must execute without interruption**

### Example

```c
// UNSAFE - Can be interrupted
uint32_t value = system_ticks;  // May change mid-read!

// SAFE - Protected
cli();                          // Disable interrupts
uint32_t value = system_ticks;  // Atomic read
sei();                          // Enable interrupts
```

### When to Use

- Accessing shared variables
- Multi-byte operations
- Hardware register updates
- **Keep SHORT** - blocks all interrupts!

---

## Slide 19: Real-World RTOS Example

### Industrial Control System

```c
// High Priority - Safety
task_emergency_stop()      // 1ms   - Monitor E-stop
task_limit_switches()      // 5ms   - Check boundaries

// Normal Priority - Control
task_motor_control()       // 10ms  - Update motors
task_sensor_reading()      // 50ms  - Read sensors

// Low Priority - Interface
task_display_update()      // 100ms - Update screen
task_log_data()           // 1000ms - Save logs
```

### Why RTOS?

- Clear separation of concerns
- Guaranteed response times
- Easy to add features
- Maintainable code

---

## Slide 20: Current System - 9 Tasks

| Task | Priority | Rate | Function |
|------|----------|------|----------|
| LED Sequence | Normal | 50ms | Visual feedback |
| UART Status | Normal | 200ms | System reports |
| ADC Monitor | Normal | 150ms | Sensor reading |
| Button Handler | High | Continuous | User input |
| Motor Control | Normal | 30ms | PWM output |
| Watchdog | Low | 500ms | Health check |
| Heartbeat | High | 100ms | Status LED |
| Statistics | Low | 1000ms | RTOS stats |
| GLCD Display | Normal | 500ms | Visual dashboard |

---

## Slide 21: Task Timing Diagram

```
Time (ms) →
0    50   100  150  200  250  300  350  400  450  500
|----|----|----|----|----|----|----|----|----|----|
LED  *         *         *         *         *      
UART      *              *              *           
ADC       *         *         *         *           
BTN  **********************************************  
PWM  **********************************************  
WDG                                            *    
HB        *         *         *         *         *
STAT                                           *    
GLCD                                           *    
```

### Observations

- Some tasks run frequently (Buttons, PWM)
- Some run periodically (LED, UART)
- Some run rarely (Watchdog, Stats)
- Scheduler coordinates all!

---

## Slide 22: Performance Metrics

### What to Monitor

1. **CPU Utilization**: % of time tasks are running
2. **Context Switches**: How often tasks change
3. **Task Execution Count**: Verify all tasks run
4. **Free RAM**: Memory available
5. **Response Time**: Time from event to reaction

### In Our System

```c
typedef struct {
  uint32_t context_switches;  // Total switches
  uint16_t free_ram_bytes;    // Available RAM
} SystemStats;

// View in Statistics task output
// Or on GLCD display
```

---

## Slide 23: Common RTOS Mistakes

### ❌ Mistake 1: Using Delays in Tasks

```c
void task_bad(void) {
  _delay_ms(1000);  // BLOCKS EVERYTHING!
  do_something();
}
```

### ✅ Correct: Time-Based Execution

```c
void task_good(void) {
  static uint32_t last = 0;
  if ((system_ticks - last) >= 1000) {
    do_something();
    last = system_ticks;
  }
}
```

---

## Slide 24: Common Mistakes (cont'd)

### ❌ Mistake 2: Infinite Loop in Task

```c
void task_bad(void) {
  while (1) {  // NEVER DO THIS!
    check_sensor();
  }
}
```

### ❌ Mistake 3: Forgetting to Update Time

```c
void task_bad(void) {
  static uint32_t last = 0;
  if ((system_ticks - last) >= 100) {
    do_work();
    // FORGOT: last = system_ticks;
    // Will run every cycle!
  }
}
```

---

## Slide 25: Debugging RTOS Applications

### Debugging Techniques

#### 1. UART Output

```c
uart_puts("[DEBUG] Task started\r\n");
uart_puts("Value: ");
uart_print_num(sensor_value);
```

#### 2. Task Statistics

- Check execution counts (should increase)
- Watch context switches (should be active)
- Monitor free RAM (shouldn't decrease)

#### 3. GLCD Status Display

- Visual feedback (task indicators)
- Real-time system state

#### 4. LED Indicators

- Heartbeat shows system alive
- Task-specific LEDs show activity

---

## Slide 26: Design Patterns for Tasks

### Pattern 1: Sensor Reading

```c
void task_sensor(void) {
  static uint32_t last = 0;
  if ((system_ticks - last) >= sample_rate) {
    raw_value = adc_read(channel);
    filtered_value = filter(raw_value);
    last = system_ticks;
  }
}
```

### Pattern 2: State Machine

```c
void task_control(void) {
  switch (state) {
    case IDLE:   /* wait for trigger */; break;
    case ACTIVE: /* do work */; break;
    case ERROR:  /* handle error */; break;
  }
}
```

---

## Slide 27: Adding Your Own Task

### Step-by-Step Process

#### Step 1: Write Task Function

```c
void task_myled(void) {
  static uint32_t last = 0;
  if ((system_ticks - last) >= 500) {
    PORTC ^= (1 << PC0);  // Toggle LED
    last = system_ticks;
  }
}
```

#### Step 2: Register Task

```c
rtos_create_task(task_myled, "MyLED", PRIORITY_NORMAL, true);
```

#### Step 3: Test

- Build project
- Run in simulator
- Check UART output
- Verify GLCD shows active task

---

## Slide 28: Exercise Ideas

### Beginner Level

1. **Blinking Pattern**: Create custom LED pattern
2. **Button Counter**: Count button presses
3. **Sensor Display**: Show ADC value on GLCD

### Intermediate Level

4. **Temperature Monitor**: Read sensor, alarm if too high
5. **Menu System**: Button-driven menu on GLCD
6. **PWM Controller**: Adjust motor speed with buttons

### Advanced Level

7. **Data Logger**: Store sensor readings over time
8. **Communication Protocol**: Send/receive commands
9. **PID Controller**: Closed-loop control system

---

## Slide 29: RTOS vs Bare Metal

### Bare Metal (Super Loop)

**Pros:**

- Simple, direct control
- Low overhead
- Full predictability

**Cons:**

- Hard to manage complexity
- Difficult timing coordination
- Messy code as project grows

### RTOS

**Pros:**

- Modular, organized
- Easy to add features
- Clear task responsibilities
- Better maintainability

**Cons:**

- Learning curve
- Slight overhead
- Need to understand concepts

---

## Slide 30: Real RTOS Systems

### Commercial RTOS Examples

#### FreeRTOS

- Most popular embedded RTOS
- Open source, free
- Used in millions of devices
- Preemptive scheduler

#### RTX (Keil)

- ARM Cortex-M optimized
- Commercial support
- Automotive qualified

#### VxWorks

- Used in Mars rovers!
- Industrial, aerospace
- Hard real-time capable

### Our Educational RTOS

- Simplified for learning
- Core concepts remain same
- Foundation for understanding advanced RTOS

---

## Slide 31: Career Applications

### Industries Using RTOS

🚗 **Automotive**

- Engine control, ABS, airbags
- Infotainment systems

✈️ **Aerospace**

- Flight control systems
- Avionics, navigation

🏥 **Medical**

- Patient monitors
- Infusion pumps, ventilators

🏭 **Industrial**

- PLCs, factory automation
- Robotics

📱 **Consumer**

- IoT devices, smart home
- Wearables, drones

---

## Slide 32: Learning Outcomes

### After This Course, You Can

✅ **Understand:**

- RTOS concepts and terminology
- Task scheduling and states
- Cooperative vs preemptive multitasking

✅ **Implement:**

- Custom tasks for any function
- Timing-based task execution
- Multi-task applications

✅ **Debug:**

- RTOS timing issues
- Resource conflicts
- Task synchronization

✅ **Design:**

- Modular embedded systems
- Real-time control applications
- Complex multi-function devices

---

## Slide 33: Next Steps

### Continue Learning

1. **Practice**: Add 3-5 custom tasks
2. **Experiment**: Try different timing intervals
3. **Challenge**: Build a complete application
4. **Study**: Read FreeRTOS documentation
5. **Explore**: Other RTOS features (queues, semaphores, mutexes)

### Resources

- `STUDENT_GUIDE.md` - Detailed tutorials
- `TASK_TEMPLATE.c` - Code examples
- `QUICK_REFERENCE.md` - Quick lookup
- ATmega128 Datasheet - Hardware details

---

## Slide 34: Summary

### Key Takeaways

🎯 **RTOS = Organized Multitasking**

- Multiple tasks run "simultaneously"
- Scheduler coordinates execution

⏰ **Timing is Everything**

- System tick provides time base
- Tasks use timing checks

🔧 **Cooperative = Simple**

- Tasks must return quickly
- No blocking code allowed

📊 **Monitor and Debug**

- Use UART for debugging
- GLCD shows system state
- Statistics show health

🚀 **Foundation for Career**

- Real-world systems use RTOS
- Concepts apply to all RTOS

---

## Slide 35: Questions & Discussion

### Common Questions

**Q: Why not just use Arduino?**
A: Arduino hides complexity. You learn more by building from scratch!

**Q: When do I need an RTOS?**
A: When you have 3+ concurrent activities with different timing requirements.

**Q: Is this a "real" RTOS?**
A: It's simplified but teaches core concepts used in commercial RTOS.

**Q: Can I use this in production?**
A: It's educational. For production, use FreeRTOS or commercial RTOS.

**Q: How do I learn more?**
A: Build projects! Add tasks, experiment, break things, fix them!

---

## Slide 36: Lab Time

### Hands-On Exercise

**Task: Create a Traffic Light Controller**

Requirements:

1. 3 LEDs (Red, Yellow, Green)
2. Sequence: Green(5s) → Yellow(2s) → Red(5s) → repeat
3. Button to force Red (emergency)
4. Display current state on GLCD
5. Log state changes to UART

**Challenge:** Add pedestrian crossing button!

### Get Started

1. Open `TASK_TEMPLATE.c`
2. Copy state machine example
3. Modify for traffic light logic
4. Build and test!

---

## End of Lecture

### Thank You

**Remember:**

- Start simple
- Test frequently  
- Use debugging tools
- Read the guides
- Ask questions
- Have fun coding! 🚀

### Good Luck with Your RTOS Projects

---

## Additional Resources

### Files in This Project

- `Main.c` - RTOS implementation
- `STUDENT_GUIDE.md` - Learning guide
- `TASK_TEMPLATE.c` - Code examples
- `QUICK_REFERENCE.md` - Cheat sheet
- `README.md` - Project overview

### External Resources

- FreeRTOS: <www.freertos.org>
- ATmega128 Datasheet: microchip.com
- AVR Libc Documentation
- Embedded Systems Textbooks

**Questions? Check the documentation or ask instructor!**
