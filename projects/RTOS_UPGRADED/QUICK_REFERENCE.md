# RTOS Quick Reference Card

## 🚀 Quick Start

### Add a New Task (3 Steps)

1. **Write task function** (before `main()`):

```c
void task_myname(void) {
  static uint32_t last = 0;
  if ((system_ticks - last) >= 100) {
    // Your code here
    last = system_ticks;
  }
}
```

2. **Register in `main()`**:

```c
rtos_create_task(task_myname, "MyTask", PRIORITY_NORMAL, true);
```

3. **Build and test!**

---

## 📌 Essential Functions

### Timing

```c
system_ticks              // Current time in ms
rtos_get_ticks()         // Get tick count
```

### UART

```c
uart_puts("Hello\r\n");           // Print string
uart_print_num(123);              // Print number
uart_putchar('A');                // Print char
```

### GPIO

```c
PORTB &= ~(1 << PB0);     // Set pin LOW (LED ON)
PORTB |= (1 << PB0);      // Set pin HIGH (LED OFF)
uint8_t val = PIND;       // Read port
```

### ADC

```c
uint16_t val = adc_read(0);       // Read channel 0-7
uint16_t percent = (val * 100) / 1023;  // To percent
```

### GLCD

```c
ks0108_set_cursor(line, col);     // Line 0-7, Col 0-20
ks0108_puts("Text");              // Print string
ks0108_putchar('A');              // Print char
```

### Buzzer

```c
buzzer_beep(50);          // Beep for 50ms
```

---

## 🎯 Task Timing Patterns

### Run every 100ms

```c
static uint32_t last = 0;
if ((system_ticks - last) >= 100) {
  // Code here
  last = system_ticks;
}
```

### Run every second

```c
if ((system_ticks - last) >= 1000) {
  // Code here
  last = system_ticks;
}
```

### Toggle every 500ms

```c
if ((system_ticks % 500) < 250) {
  // First half
} else {
  // Second half
}
```

---

## 🔌 Hardware Map (Simulator110.simu)

### Port B - LEDs (Active-LOW)

```
PB0-PB6: Sequence LEDs
PB7:     Heartbeat LED
PB5:     PWM output (Timer1)
```

### Port D - Buttons (Active-LOW)

```
PD0-PD7: Push buttons
PD2:     UART RX
PD3:     UART TX
```

### Port F - ADC

```
PF0:     Potentiometer (Channel 0)
PF1-PF7: ADC channels 1-7
```

### Other

```
PG4:     Buzzer
GLCD:    KS0108 128x64
```

---

## 💡 Common Code Snippets

### Button with Debounce

```c
static uint8_t last_state = 0xFF;
uint8_t current = PIND;
if ((last_state & (1 << PD0)) && !(current & (1 << PD0))) {
  // Button 0 pressed
}
last_state = current;
```

### LED Blink

```c
static bool state = false;
if ((system_ticks - last) >= 500) {
  state = !state;
  if (state)
    PORTB &= ~(1 << PB0);  // ON
  else
    PORTB |= (1 << PB0);   // OFF
  last = system_ticks;
}
```

### Read Sensor

```c
if ((system_ticks - last) >= 1000) {
  uint16_t raw = adc_read(0);
  uart_puts("Value: ");
  uart_print_num(raw);
  uart_puts("\r\n");
  last = system_ticks;
}
```

---

## 🐛 Debugging

### Print Debug Info

```c
uart_puts("[DEBUG] x=");
uart_print_num(x);
uart_puts("\r\n");
```

### Check Task Running

- Watch GLCD status line (should show *)
- Check UART for task messages
- Statistics task shows execution counts

### Common Issues

- **Task not running?** Check MAX_TASKS
- **Out of memory?** Reduce STACK_SIZE
- **Weird behavior?** Remove blocking code

---

## ⚙️ Configuration

```c
#define MAX_TASKS 10      // Max number of tasks
#define STACK_SIZE 128    // Bytes per task
```

---

## 📝 Task Rules

✅ **DO:**

- Use `static` for variables
- Use `system_ticks` for timing
- Keep tasks short
- Return quickly

❌ **DON'T:**

- Use `_delay_ms()` in tasks
- Use infinite loops
- Block other tasks
- Forget timing updates

---

## 🎓 Priority Levels

```c
PRIORITY_IDLE     = 0
PRIORITY_LOW      = 1
PRIORITY_NORMAL   = 2  // ← Use this
PRIORITY_HIGH     = 3
PRIORITY_CRITICAL = 4
```

---

## 📊 Task Template

```c
void task_name(void)
{
  static uint32_t last_time = 0;
  static uint8_t counter = 0;
  
  if ((system_ticks - last_time) >= 100)  // 100ms
  {
    // ===== YOUR CODE =====
    
    uart_puts("Count: ");
    uart_print_num(counter++);
    uart_puts("\r\n");
    
    // ===== END CODE =====
    
    last_time = system_ticks;
  }
}
```

---

**Need more help?** See `STUDENT_GUIDE.md` for detailed tutorials and examples!
