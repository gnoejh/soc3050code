# Timer0 Overflow LED Blink

## Learning Objectives
- Understand Timer0 8-bit counter operation
- Master overflow interrupt mechanism  
- Learn non-blocking LED control without delay()
- Practice prescaler calculations

## Hardware Requirements
- ATmega128 development board
- LED on PB0 (built-in, active low)
- UART connection for monitoring (9600 baud)

## Theory

### Timer0 Overview
Timer0 is an 8-bit counter that counts from 0 to 255. When it reaches 255 and increments, it **overflows** back to 0, triggering an interrupt.

**Overflow Frequency Calculation:**
```
F_overflow = F_CPU / (Prescaler × 256)
           = 7372800 / (1024 × 256)  
           = 28.125 Hz
           = One overflow every 35.6ms
```

### Why This Matters
Traditional `_delay_ms()` **blocks** the CPU - nothing else can run during the delay.  
Timer interrupts allow **non-blocking** operation - LED blinks automatically while the main loop does other work!

## Demos

### Demo 1: Basic Overflow Blink
- LED blinks at 2 Hz using Timer0 overflow
- Demonstrates ISR-driven LED toggle
- Main loop remains free

### Demo 2: Variable Speed Blinking
- Change blink rates: 1 Hz, 2 Hz, 4 Hz, 7 Hz
- Shows how to adjust timing by changing overflow counts
- Interactive UART control

### Demo 3: Timing Accuracy Test
- Measures actual overflow frequency
- Compares with calculated 28.125 Hz
- Demonstrates timer precision

### Demo 4: Multitasking Demo
- LED blinks while main loop counts
- Proves non-blocking advantage
- Shows true parallel operation

## Key Concepts

### Prescaler Values
The prescaler divides the CPU clock before feeding Timer0:
- **1**: No prescaling (very fast)
- **8**: Clock / 8
- **64**: Clock / 64
- **256**: Clock / 256
- **1024**: Clock / 1024 (slowest, longest period)

### Interrupt Service Routine (ISR)
```c
ISR(TIMER0_OVF_vect) {
    // Runs automatically every overflow
    overflow_count++;
    if (overflow_count >= blink_rate) {
        LED_TOGGLE();
        overflow_count = 0;
    }
}
```

## Register Configuration

### TCCR0 - Timer/Counter Control Register
- **CS02:CS00** - Clock Select (prescaler)
  - `111` = 1024 prescaler
- **WGM01:WGM00** - Waveform Generation Mode
  - `00` = Normal mode (counts 0→255)

### TIMSK - Timer Interrupt Mask Register
- **TOIE0** - Timer0 Overflow Interrupt Enable
  - `1` = Enable overflow interrupt

### TCNT0 - Timer/Counter Register
- Current counter value (0-255)
- Read to check current count
- Write to reset or adjust timing

## Educational Value

This project teaches:
1. ✅ **Timer fundamentals** - How 8-bit timers work
2. ✅ **Interrupt handling** - ISR-based programming
3. ✅ **Non-blocking code** - Alternative to delay()
4. ✅ **Prescaler math** - Frequency calculations
5. ✅ **Multitasking basics** - True parallel operation

## Building & Running

```bash
cd projects/Timer0_Overflow_Blink
./build.bat
```

Connect via UART (9600 baud) and select demos from the menu.

## Expected Output

```
╔════════════════════════════════════════════╗
║   TIMER0 OVERFLOW INTERRUPT LED BLINK     ║
║   Educational ATmega128 Project           ║
╚════════════════════════════════════════════╝

Timer0 initialized:
  Mode: Normal (0-255)
  Prescaler: 1024
  Overflow Rate: 28.125 Hz (every 35.6ms)

  [1] Basic Overflow Blink
  [2] Variable Speed Blinking
  [3] Timing Accuracy Test
  [4] Multitasking Demo
```

## Common Student Questions

**Q: Why use interrupts instead of delay()?**  
A: Interrupts free the CPU for other tasks. delay() blocks everything!

**Q: Can I use other prescalers?**  
A: Yes! Change CS02:CS00 bits. Smaller prescalers = faster overflows.

**Q: What if I need longer delays?**  
A: Count multiple overflows (as shown in variable speed demo).

## Next Steps

After mastering Timer0 overflow:
- Try **Timer1_CTC_Precision** for exact timing
- Learn **Timer1_Input_Capture** for frequency measurement
- Explore **Timer_Stopwatch** for practical applications

---

**Difficulty:** ⭐⭐ Intermediate  
**Prerequisites:** Port I/O, UART communication  
**Estimated Time:** 1-2 hours
