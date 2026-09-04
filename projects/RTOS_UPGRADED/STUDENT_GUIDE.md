# RTOS Student Guide - ATmega128 Task Development

## 📚 Learning Objectives

This RTOS project teaches you:

- Real-time operating system concepts
- Cooperative multitasking
- Task scheduling and management
- Hardware abstraction with tasks
- Concurrent programming on embedded systems

---

## 🎯 Current System Overview

The RTOS currently runs **9 concurrent tasks**:

| Task # | Name | Function | Hardware | Update Rate |
|--------|------|----------|----------|-------------|
| 1 | LED Sequence | Cycles through LEDs 0-6 | Port B (PB0-PB6) | 50ms |
| 2 | UART Status | Reports system status | UART1 | 200ms |
| 3 | ADC Monitor | Reads potentiometer | ADC Ch0 (PF0) | 150ms |
| 4 | Button Handler | Monitors button presses | Port D (PD0-PD7) | Continuous |
| 5 | Motor Control | PWM motor control | Timer1/PB5 | 30ms |
| 6 | Watchdog | System health check | - | 500ms |
| 7 | Heartbeat | Status LED blink | PB7 | 100ms |
| 8 | Statistics | RTOS statistics | UART1 | 1000ms |
| 9 | GLCD Display | System status display | KS0108 GLCD | 500ms |

---

## 🔧 How to Add Your Own Task

### Step 1: Write the Task Function

Add your task function **before** the `main()` function:

```c
// Task 10: Your Custom Task
void task_your_name(void)
{
  static uint32_t last_update = 0;
  static uint8_t counter = 0;

  // Run every 100ms (100 ticks)
  if ((system_ticks - last_update) >= 100)
  {
    // Your task code here
    uart_puts("[MyTask] Counter: ");
    uart_print_num(counter++);
    uart_puts("\r\n");
    
    last_update = system_ticks;
  }
}
```

### Step 2: Register the Task in `main()`

Find the task initialization section in `main()` and add:

```c
rtos_create_task(task_your_name, "Your Task", PRIORITY_NORMAL, true);
uart_puts("[OK] Task 10: Your Task\r\n");
```

### Step 3: Increase MAX_TASKS (if needed)

If you have more than 10 tasks, change:

```c
#define MAX_TASKS 10  // Increase to 12, 15, etc.
```

### Step 4: Build and Test

```bash
# Build the project
Build Current Project (Ctrl+Shift+B)

# Simulate
Simulate in SimulIDE (Current Project)
```

---

## 📝 Task Programming Guidelines

### ✅ DO

- Use `static` variables to maintain state between calls
- Use `system_ticks` for timing
- Keep tasks **short and fast** (cooperative multitasking)
- Use timing checks: `if ((system_ticks - last_time) >= delay)`
- Return quickly from the task function

### ❌ DON'T

- Use `_delay_ms()` or blocking delays inside tasks
- Use infinite loops inside tasks
- Hog the CPU - let other tasks run
- Forget to update your timing variable

---

## 🎓 Student Exercise Ideas

### Exercise 1: Temperature Display Task

**Hardware:** ADC channel (simulated temperature sensor)
**Goal:** Read ADC, display temperature on GLCD

```c
void task_temperature(void)
{
  static uint32_t last_read = 0;
  
  if ((system_ticks - last_read) >= 1000)  // Every 1 second
  {
    uint16_t adc_val = adc_read(1);  // Read ADC channel 1
    uint16_t temp = (adc_val * 100) / 1023;  // Convert to 0-100°C
    
    // Display on GLCD
    ks0108_set_cursor(6, 0);
    ks0108_puts("Temp:");
    ks0108_putchar('0' + (temp / 10));
    ks0108_putchar('0' + (temp % 10));
    ks0108_putchar('C');
    
    last_read = system_ticks;
  }
}
```

### Exercise 2: Pattern Generator Task

**Hardware:** Port C LEDs
**Goal:** Create custom LED patterns

```c
void task_pattern(void)
{
  static uint32_t last_update = 0;
  static uint8_t pattern[] = {0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF};
  static uint8_t index = 0;
  
  if ((system_ticks - last_update) >= 200)
  {
    PORTC = ~pattern[index];  // Active-LOW LEDs
    index = (index + 1) % 8;
    last_update = system_ticks;
  }
}
```

### Exercise 3: UART Command Task

**Hardware:** UART1
**Goal:** Read commands from serial, control LEDs

```c
void task_command(void)
{
  // Check if data available (implement UART receive)
  if (uart_available())
  {
    char cmd = uart_getchar();
    
    if (cmd == '1') PORTB &= ~(1 << 0);  // LED0 ON
    if (cmd == '0') PORTB |= (1 << 0);   // LED0 OFF
    
    uart_puts("Command received\r\n");
  }
}
```

### Exercise 4: Alarm System Task

**Hardware:** Buzzer, Button, LED
**Goal:** Create a simple alarm system

```c
void task_alarm(void)
{
  static bool armed = false;
  static uint32_t alarm_time = 0;
  
  // Check arm button (PD0)
  if (!(PIND & (1 << PD0)))
  {
    armed = !armed;
    uart_puts(armed ? "Armed\r\n" : "Disarmed\r\n");
    _delay_ms(200);  // Debounce
  }
  
  // Check sensor (PD1)
  if (armed && !(PIND & (1 << PD1)))
  {
    alarm_time = system_ticks;
    buzzer_beep(100);
  }
  
  // Flash LED when alarm active
  if (alarm_time > 0 && (system_ticks - alarm_time) < 5000)
  {
    if ((system_ticks % 200) < 100)
      PORTB &= ~(1 << 1);  // LED1 ON
    else
      PORTB |= (1 << 1);   // LED1 OFF
  }
}
```

---

## 🛠️ Available Hardware Functions

### UART (Serial Communication)

```c
uart_init();                    // Initialize UART
uart_putchar(char c);           // Send single character
uart_puts(const char *str);     // Send string
uart_print_num(uint32_t num);   // Print number
```

### ADC (Analog Input)

```c
adc_init();                     // Initialize ADC
uint16_t val = adc_read(0);     // Read channel 0-7 (returns 0-1023)
```

### GPIO (Digital I/O)

```c
DDRB = 0xFF;                    // Set Port B as output
PORTB = 0x00;                   // Write to Port B
uint8_t input = PIND;           // Read Port D
```

### Buzzer

```c
buzzer_init();                  // Initialize buzzer
buzzer_beep(50);                // Beep for 50ms
```

### GLCD (Graphical LCD)

```c
ks0108_init();                  // Initialize display
ks0108_clear_screen();          // Clear display
ks0108_set_cursor(line, col);   // Set cursor (line 0-7, col 0-20)
ks0108_puts("Hello");           // Print string
ks0108_putchar('A');            // Print character
ks0108_draw_line(x1,y1,x2,y2,KS0108_PIXEL_ON);  // Draw line
```

### Timing

```c
system_ticks                    // Global tick counter (1ms per tick)
rtos_get_ticks()               // Get current tick count
```

---

## 🎮 Task Priority Levels

```c
PRIORITY_IDLE = 0       // Lowest priority
PRIORITY_LOW = 1        // Low priority
PRIORITY_NORMAL = 2     // Normal priority (most tasks)
PRIORITY_HIGH = 3       // High priority (time-critical)
PRIORITY_CRITICAL = 4   // Highest priority
```

**Note:** Current scheduler uses round-robin, so priority doesn't affect order.

---

## 🐛 Debugging Tips

1. **Use UART for debugging:**

   ```c
   uart_puts("[DEBUG] My value: ");
   uart_print_num(my_variable);
   uart_puts("\r\n");
   ```

2. **Check task execution count:**
   - View in Statistics task output (every 1 second)
   - Shows how many times each task has executed

3. **Monitor system health:**
   - Watch the Heartbeat LED (PB7) - should blink steadily
   - Check GLCD status indicators - all should show '*'
   - Watch free RAM in Watchdog task output

4. **Common issues:**
   - Task not running? Check if MAX_TASKS is sufficient
   - Strange behavior? Check for blocking code in tasks
   - Out of memory? Reduce STACK_SIZE or number of tasks

---

## 📊 Understanding the RTOS

### Cooperative Multitasking

- Tasks voluntarily yield control
- No preemption - tasks run to completion
- Scheduler cycles through all READY tasks

### Task States

- **READY** - Task is ready to run
- **RUNNING** - Task is currently executing
- **BLOCKED** - Task is waiting (not used in current system)
- **SUSPENDED** - Task is paused
- **TERMINATED** - Task has finished

### Scheduler Operation

1. Timer0 ISR increments `system_ticks` every 1ms
2. Main loop calls scheduler to find next ready task
3. Scheduler uses round-robin selection
4. Selected task executes once
5. Task returns to READY state (if auto_restart = true)
6. Repeat

---

## 🚀 Advanced Challenges

1. **Multi-Sensor Dashboard**: Read 3+ sensors, display on GLCD with graphs
2. **Menu System**: Create button-driven menu on GLCD
3. **Data Logger**: Store ADC readings, display history
4. **PID Controller**: Implement PID control using motor and sensor
5. **Communication Protocol**: Create master-slave communication between tasks

---

## 📖 Learning Resources

- **ATmega128 Datasheet**: Full hardware documentation
- **AVR GCC Manual**: C programming for AVR
- **Simulator110.simu**: Circuit reference
- **Shared Libraries**: Check `shared_libs/` for available functions

---

## 💡 Tips for Success

1. Start simple - add one task at a time
2. Test each task independently
3. Use UART output to verify task execution
4. Keep timing intervals reasonable (not too fast)
5. Monitor system resources (RAM, CPU time)
6. Comment your code for future reference

---

## ✅ Task Development Checklist

- [ ] Define task purpose and hardware requirements
- [ ] Write task function with proper timing
- [ ] Use static variables for state
- [ ] Add debug UART output
- [ ] Register task in main()
- [ ] Increase MAX_TASKS if needed
- [ ] Build and test
- [ ] Verify task appears in Statistics output
- [ ] Document your task (what it does, update rate)

---

**Happy Coding! 🎓**

*This RTOS project is designed to teach real-time embedded systems programming. Experiment, learn, and build amazing applications!*
