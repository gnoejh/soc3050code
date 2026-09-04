# ATmega128 Interrupt Quick Reference

**COURSE:** SOC 3050 - Embedded Systems and Applications  
**TOPIC:** Polling vs Interrupt-Driven Programming  
**PURPOSE:** Student reference for ATmega128 external interrupts

---

## TABLE OF CONTENTS

1. [Polling vs Interrupt Comparison](#polling-vs-interrupt-comparison)
2. [ATmega128 Interrupt System](#atmega128-interrupt-system)
3. [Control Registers](#control-registers)
4. [Configuration Steps](#configuration-steps)
5. [ISR Programming Rules](#isr-programming-rules)
6. [Code Examples](#code-examples)
7. [Common Patterns](#common-patterns)
8. [Debugging Checklist](#debugging-checklist)
9. [Register Values Reference](#register-values-reference)

---

## POLLING VS INTERRUPT COMPARISON

### Quick Decision Guide

**Use POLLING when:**
- Simple application with few inputs
- Predictable, periodic checking is acceptable
- Learning basics (easier to understand)
- No strict timing requirements
- System cannot sleep (other reasons)

**Use INTERRUPTS when:**
- Fast response time required (< 1ms)
- Multiple event sources
- Power efficiency important (CPU can sleep)
- Events are rare but critical
- Cannot afford to miss events

### Detailed Comparison Table

| Aspect              | POLLING                        | INTERRUPT                     |
|---------------------|--------------------------------|-------------------------------|
| **Response Time**   | Variable (loop-dependent)      | Fixed (2-4 clock cycles)     |
|                     | Typical: 1-100ms               | Typical: 0.125-0.25 µs @ 16MHz|
| **CPU Efficiency**  | Low (100% busy checking)       | High (event-driven)          |
| **Power Usage**     | High (CPU always active)       | Low (CPU can sleep)          |
| **Code Complexity** | Simple (linear flow)           | Moderate (ISR management)    |
| **Event Detection** | May miss if faster than loop   | Guaranteed (hardware detects)|
| **Multiple Events** | Sequential checking            | Priority-based handling      |
| **Debugging**       | Easy (step through loop)       | Complex (async, ISR context) |
| **Predictability**  | Deterministic timing           | Asynchronous events          |
| **Best For**        | Simple sensors, slow inputs    | Buttons, communication, timers|

### Timing Analysis

**Polling Example (50ms loop delay):**
```
Check → Process → Delay 50ms → Check → Process → Delay 50ms → ...
```
- Minimum event duration to detect: ~50ms
- Faster events may be missed
- Response time: 0-50ms (depends on when event occurs in loop)

**Interrupt Example:**
```
Main task... ──→ [EVENT!] ──→ ISR (instant) ──→ Resume main task
```
- Any event duration detected (even 1 microsecond pulse)
- Response time: 2-4 clock cycles = 0.125-0.25 µs @ 16MHz
- No missed events (hardware queues flags)

---

## ATMEGA128 INTERRUPT SYSTEM

### External Interrupt Vector Table

| Vector | Name    | Pin | Address | Priority | Description              |
|--------|---------|-----|---------|----------|--------------------------|
| 1      | RESET   | -   | 0x0000  | Highest  | System reset             |
| 2      | **INT0**| PD0 | 0x0004  | 2        | External Interrupt 0     |
| 3      | **INT1**| PD1 | 0x0006  | 3        | External Interrupt 1     |
| 4      | **INT2**| PD2 | 0x0008  | 4        | External Interrupt 2     |
| 5      | **INT3**| PD3 | 0x000A  | 5        | External Interrupt 3     |
| 6      | **INT4**| PE4 | 0x000C  | 6        | External Interrupt 4     |
| 7      | **INT5**| PE5 | 0x000E  | 7        | External Interrupt 5     |
| 8      | **INT6**| PE6 | 0x0010  | 8        | External Interrupt 6     |
| 9      | **INT7**| PE7 | 0x0012  | 9        | External Interrupt 7     |

**Priority Rule:** Lower vector number = higher priority  
**Example:** If INT0 and INT3 trigger simultaneously, INT0 ISR runs first

### Pin Configuration

| Interrupt | Physical Pin | Port | Bit | Alternative Functions         |
|-----------|--------------|------|-----|-------------------------------|
| INT0      | PD0          | D    | 0   | SCL (I2C), RXD1 (UART1)      |
| INT1      | PD1          | D    | 1   | SDA (I2C), TXD1 (UART1)      |
| INT2      | PD2          | D    | 2   | RXD1, INT2                    |
| INT3      | PD3          | D    | 3   | TXD1, INT3                    |
| INT4      | PE4          | E    | 4   | OC3B (PWM), INT4              |
| INT5      | PE5          | E    | 5   | OC3C (PWM), INT5              |
| INT6      | PE6          | E    | 6   | T3 (Timer3 ext clock), INT6   |
| INT7      | PE7          | E    | 7   | IC3 (Input Capture), INT7     |

**NOTE:** INT0-INT3 are on PORTD, INT4-INT7 are on PORTE

---

## CONTROL REGISTERS

### EICRA - External Interrupt Control Register A (Address: 0x6A)

Controls INT0-INT3 edge/level sensing.

```
Bit:  7     6     5     4     3     2     1     0
    ISC31 ISC30 ISC21 ISC20 ISC11 ISC10 ISC01 ISC00
    └──────────┴──────────┴──────────┴──────────┘
        INT3       INT2       INT1       INT0
```

### EICRB - External Interrupt Control Register B (Address: 0x5A)

Controls INT4-INT7 edge/level sensing.

```
Bit:  7     6     5     4     3     2     1     0
    ISC71 ISC70 ISC61 ISC60 ISC51 ISC50 ISC41 ISC40
    └──────────┴──────────┴──────────┴──────────┘
        INT7       INT6       INT5       INT4
```

### EIMSK - External Interrupt Mask Register (Address: 0x59)

Enable/disable individual interrupts.

```
Bit:  7     6     5     4     3     2     1     0
     INT7  INT6  INT5  INT4  INT3  INT2  INT1  INT0
```
- **0 = Disabled:** Interrupt will not trigger ISR
- **1 = Enabled:** Interrupt can trigger ISR (if global enabled)

### EIFR - External Interrupt Flag Register (Address: 0x58)

Interrupt flags (set by hardware, cleared by ISR or writing 1).

```
Bit:  7     6     5     4     3     2     1     0
    INTF7 INTF6 INTF5 INTF4 INTF3 INTF2 INTF1 INTF0
```
- **Flag set:** Edge/level condition detected
- **Cleared:** Automatically by entering ISR, or manually by writing 1

### SREG - Status Register (Global Interrupt Control)

```
Bit 7: I (Global Interrupt Enable)
  0 = All interrupts disabled
  1 = Interrupts enabled (if individual interrupt also enabled)
```

**Functions:**
- `sei()` - Set I-bit (enable global interrupts)
- `cli()` - Clear I-bit (disable global interrupts)

---

## INTERRUPT SENSE CONTROL (ISC) MODES

Each interrupt has 2 control bits: ISCn1 and ISCn0

| ISCn1 | ISCn0 | Mode         | Trigger Condition                    | Use Case                  |
|-------|-------|--------------|--------------------------------------|---------------------------|
| 0     | 0     | Low Level    | Continuously while pin is LOW        | Rarely used (flood risk)  |
| 0     | 1     | Any Change   | Rising edge OR falling edge          | Encoder, bidirectional    |
| 1     | 0     | Falling Edge | HIGH → LOW transition                | **Button press (most common)** |
| 1     | 1     | Rising Edge  | LOW → HIGH transition                | Button release            |

### Visual Representation

```
Button Signal (active-low with pull-up):
         ┌─────┐                    ┌─────┐
Released │     │ Pressed       │     │
    HIGH │     └────────────────┘     │ HIGH
         │                          │
         A                          B

A = Falling Edge (button pressed)  → Most common for button detection
B = Rising Edge (button released)  → For release detection
```

### Mode Selection Examples

```c
// Falling Edge (button press detection)
EICRA |= (1 << ISC01);   // ISC01 = 1
EICRA &= ~(1 << ISC00);  // ISC00 = 0

// Rising Edge (button release detection)
EICRA |= (1 << ISC01) | (1 << ISC00);  // Both bits = 1

// Any Change (press + release)
EICRA &= ~(1 << ISC01);  // ISC01 = 0
EICRA |= (1 << ISC00);   // ISC00 = 1

// Low Level (continuous while pressed) - USE CAREFULLY!
EICRA &= ~((1 << ISC01) | (1 << ISC00));  // Both bits = 0
```

---

## CONFIGURATION STEPS

### Step-by-Step INT0 Setup (Falling Edge)

```c
// STEP 1: Configure pin as input with pull-up
DDRD &= ~(1 << PD0);     // PD0 as input
PORTD |= (1 << PD0);     // Enable internal pull-up resistor

// STEP 2: Configure interrupt sense control (falling edge)
EICRA |= (1 << ISC01);   // Set ISC01 bit
EICRA &= ~(1 << ISC00);  // Clear ISC00 bit
// Result: ISC01=1, ISC00=0 → Falling edge mode

// STEP 3: Enable INT0 in mask register
EIMSK |= (1 << INT0);    // Enable INT0 interrupt

// STEP 4: Enable global interrupts
sei();                   // Set I-bit in SREG

// STEP 5: Implement ISR
ISR(INT0_vect) {
    // Your interrupt handling code here
    // Keep SHORT and FAST!
}
```

### Quick Configuration for Other Interrupts

**INT1 (PD1):**
```c
EICRA |= (1 << ISC11);    // Falling edge
EICRA &= ~(1 << ISC10);
EIMSK |= (1 << INT1);
sei();
```

**INT4 (PE4):**
```c
EICRB |= (1 << ISC41);    // Falling edge
EICRB &= ~(1 << ISC40);
EIMSK |= (1 << INT4);
sei();
```

---

## ISR PROGRAMMING RULES

### Golden Rules (Follow These!)

1. **KEEP ISRs SHORT AND FAST**
   - Target: < 10 microseconds execution time
   - Avoid: delays, loops, heavy calculations
   - Do: Set flags, toggle LEDs, read sensors quickly

2. **USE VOLATILE FOR SHARED VARIABLES**
   ```c
   volatile uint8_t flag;   // CORRECT - compiler won't optimize
   uint8_t flag;            // WRONG - may be optimized away
   ```

3. **NO BLOCKING OPERATIONS**
   - No `_delay_ms()` or `_delay_us()`
   - No `while()` loops waiting for conditions
   - No Serial/UART transmit (may block)
   - No complex GLCD operations

4. **MINIMIZE ISR WORK**
   - Set flags for main loop processing
   - Defer complex work to main context
   - Example: ISR sets flag → main checks flag → main does work

5. **PROTECT CRITICAL SECTIONS** (when needed)
   ```c
   cli();                      // Disable interrupts
   multi_byte_variable = 0;    // Atomic operation
   sei();                      // Re-enable interrupts
   ```

### ISR Template Pattern

```c
// Global variables (with volatile!)
volatile uint8_t event_flag = 0;
volatile uint16_t event_count = 0;

// ISR: Minimal work, set flags
ISR(INT0_vect) {
    event_flag = 1;        // Signal main loop
    event_count++;         // Track events
    led_toggle(0);         // Quick visual feedback (OK)
    // DON'T do complex processing here!
}

// Main loop: Check flag and process
int main(void) {
    setup_interrupt();
    while (1) {
        if (event_flag) {
            event_flag = 0;       // Clear flag
            process_event();      // Do complex work HERE
        }
        // Other main loop tasks
    }
}
```

---

## CODE EXAMPLES

### Example 1: Simple Button Interrupt

```c
#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t button_pressed = 0;

ISR(INT0_vect) {
    button_pressed = 1;
    PORTB ^= (1 << PB0);  // Toggle LED0
}

int main(void) {
    // Setup LED
    DDRB |= (1 << PB0);   // PB0 as output
    
    // Setup button
    DDRD &= ~(1 << PD0);  // PD0 as input
    PORTD |= (1 << PD0);  // Enable pull-up
    
    // Configure INT0 for falling edge
    EICRA |= (1 << ISC01);
    EICRA &= ~(1 << ISC00);
    EIMSK |= (1 << INT0);
    sei();
    
    while (1) {
        if (button_pressed) {
            button_pressed = 0;
            // Do something when button pressed
        }
    }
    return 0;
}
```

### Example 2: Multiple Interrupts with Priority

```c
volatile uint8_t int0_flag = 0;
volatile uint8_t int1_flag = 0;

ISR(INT0_vect) {
    int0_flag = 1;  // Higher priority (vector 2)
}

ISR(INT1_vect) {
    int1_flag = 1;  // Lower priority (vector 3)
}

int main(void) {
    // Setup both interrupts
    DDRD &= ~((1 << PD0) | (1 << PD1));
    PORTD |= (1 << PD0) | (1 << PD1);
    
    // Both falling edge
    EICRA |= (1 << ISC01) | (1 << ISC11);
    EICRA &= ~((1 << ISC00) | (1 << ISC10));
    
    // Enable both
    EIMSK |= (1 << INT0) | (1 << INT1);
    sei();
    
    while (1) {
        if (int0_flag) {
            int0_flag = 0;
            handle_button0();
        }
        if (int1_flag) {
            int1_flag = 0;
            handle_button1();
        }
    }
    return 0;
}
```

### Example 3: Event Counter

```c
volatile uint16_t pulse_count = 0;

ISR(INT2_vect) {
    pulse_count++;  // Count every interrupt
}

int main(void) {
    // Setup INT2 on PD2
    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);
    
    // Rising edge (count pulses)
    EICRA |= (1 << ISC21) | (1 << ISC20);
    EIMSK |= (1 << INT2);
    sei();
    
    while (1) {
        // Display pulse count
        display_number(pulse_count);
        _delay_ms(100);
    }
    return 0;
}
```

---

## COMMON PATTERNS

### Pattern 1: Debounced Button Interrupt

```c
#include <util/delay.h>

volatile uint8_t button_flag = 0;

ISR(INT0_vect) {
    _delay_ms(20);  // Simple debounce (NOT RECOMMENDED - blocks ISR!)
    if (!(PIND & (1 << PD0))) {  // Check if still pressed
        button_flag = 1;
    }
}
```

**BETTER APPROACH (debounce in main):**
```c
ISR(INT0_vect) {
    button_flag = 1;  // Just set flag
}

int main(void) {
    while (1) {
        if (button_flag) {
            _delay_ms(20);  // Debounce delay in main (OK!)
            if (!(PIND & (1 << PD0))) {  // Confirm press
                // Handle button press
            }
            button_flag = 0;
        }
    }
}
```

### Pattern 2: State Machine with Interrupts

```c
typedef enum {
    STATE_IDLE,
    STATE_ACTIVE,
    STATE_DONE
} state_t;

volatile state_t current_state = STATE_IDLE;

ISR(INT0_vect) {
    // Change state on interrupt
    if (current_state == STATE_IDLE) {
        current_state = STATE_ACTIVE;
    }
}

int main(void) {
    while (1) {
        switch (current_state) {
            case STATE_IDLE:
                // Do idle tasks
                break;
            case STATE_ACTIVE:
                // Do active tasks
                current_state = STATE_DONE;
                break;
            case STATE_DONE:
                // Cleanup
                current_state = STATE_IDLE;
                break;
        }
    }
}
```

### Pattern 3: Timestamp Events

```c
volatile uint32_t last_interrupt_time = 0;
uint32_t system_ticks = 0;  // Updated by timer interrupt

ISR(INT0_vect) {
    last_interrupt_time = system_ticks;  // Record when event occurred
}

int main(void) {
    while (1) {
        uint32_t time_since = system_ticks - last_interrupt_time;
        if (time_since > 5000) {  // 5 seconds without interrupt
            // Timeout action
        }
    }
}
```

---

## DEBUGGING CHECKLIST

### Interrupt Not Triggering?

- [ ] **Pin configured as input?** `DDRD &= ~(1 << PD0);`
- [ ] **Pull-up enabled?** `PORTD |= (1 << PD0);`
- [ ] **ISC bits set correctly?** Check EICRA/EICRB
- [ ] **Interrupt enabled in EIMSK?** `EIMSK |= (1 << INT0);`
- [ ] **Global interrupts enabled?** `sei();` called?
- [ ] **ISR function defined?** `ISR(INT0_vect) { ... }`
- [ ] **Check EIFR flag:** Is INTF0 being set? (indicates edge detected)
- [ ] **Hardware issue?** Button working? Wiring correct?

### ISR Runs Too Often?

- [ ] **Bouncing contacts:** Add debounce (hardware or software)
- [ ] **Wrong edge mode:** Check if "any change" when you want "falling"
- [ ] **Low level mode:** Triggers continuously - use edge mode instead
- [ ] **Noise on line:** Add hardware filtering (capacitor)

### Variables Not Updating?

- [ ] **Missing volatile?** Must use `volatile` for ISR-shared variables
- [ ] **Compiler optimization:** `-Os` or `-O2` may optimize away checks
- [ ] **Multi-byte variable:** May need cli()/sei() protection

### ISR Seems Slow?

- [ ] **Too much work in ISR:** Move processing to main loop
- [ ] **Blocking calls:** Remove delays, UART, GLCD operations
- [ ] **Other interrupts:** Higher-priority interrupts may delay this one

---

## REGISTER VALUES REFERENCE

### PORTB (LED States - Active Low)

| Decimal | Hex  | Binary    | LEDs ON                    |
|---------|------|-----------|----------------------------|
| 255     | 0xFF | 11111111  | None (all off)            |
| 254     | 0xFE | 11111110  | LED0                       |
| 253     | 0xFD | 11111101  | LED1                       |
| 252     | 0xFC | 11111100  | LED0, LED1                 |
| 247     | 0xF7 | 11110111  | LED3                       |
| 240     | 0xF0 | 11110000  | LED0-3                     |
| 170     | 0xAA | 10101010  | LED0, 2, 4, 6 (even)      |
| 85      | 0x55 | 01010101  | LED1, 3, 5, 7 (odd)       |
| 0       | 0x00 | 00000000  | All LEDs on                |

### PIND (Button States - Active Low with Pull-ups)

| Decimal | Hex  | Binary    | Buttons Pressed            |
|---------|------|-----------|----------------------------|
| 255     | 0xFF | 11111111  | None                       |
| 254     | 0xFE | 11111110  | PD0                        |
| 253     | 0xFD | 11111101  | PD1                        |
| 252     | 0xFC | 11111100  | PD0, PD1                   |
| 251     | 0xFB | 11111011  | PD2                        |
| 247     | 0xF7 | 11110111  | PD3                        |

### EICRA Values for Common Configurations

| EICRA Value | Binary    | INT3      | INT2      | INT1      | INT0      |
|-------------|-----------|-----------|-----------|-----------|-----------|
| 0x00        | 00000000  | Low       | Low       | Low       | Low       |
| 0x02        | 00000010  | Low       | Low       | Low       | **Fall**  |
| 0x03        | 00000011  | Low       | Low       | Low       | **Rise**  |
| 0x0A        | 00001010  | Low       | Low       | **Fall**  | **Fall**  |
| 0x2A        | 00101010  | Low       | **Fall**  | **Fall**  | **Fall**  |
| 0xAA        | 10101010  | **Fall**  | **Fall**  | **Fall**  | **Fall**  |

---

## PERFORMANCE NOTES

### Interrupt Latency

**Time from event to ISR execution:**
- Minimum: 2 clock cycles (if instruction just finished)
- Maximum: 4 clock cycles (if multi-cycle instruction running)
- At 16MHz: 0.125 - 0.25 microseconds

**ISR Overhead:**
- Enter ISR: ~10-15 cycles (save PC, SREG, etc.)
- Exit ISR: ~10-15 cycles (restore registers, RETI)
- Total overhead: ~20-30 cycles (~2 microseconds @ 16MHz)

### Best Practices for Performance

1. **Keep ISR under 50 instructions** (target: < 10 µs)
2. **Disable interrupts briefly** for critical 16-bit reads/writes
3. **Use edge-triggered** (not level) for most applications
4. **Prioritize interrupts** by vector number (lower = higher priority)
5. **Minimize ISR nesting** (usually disabled by default)

---

## RESOURCES

- **ATmega128 Datasheet:** Section on External Interrupts
- **AVR Libc Manual:** `<avr/interrupt.h>` documentation
- **Main.c:** Working examples in this project
- **INTERRUPT_FLOW_DIAGRAMS.md:** Visual learning aids
- **DOCUMENTATION_SUMMARY.md:** Instructor teaching guide

---

**Last Updated:** 2025  
**For:** SOC 3050 Students  
**Print this for lab reference!**
