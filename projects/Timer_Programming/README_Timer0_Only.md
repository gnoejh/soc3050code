# Timer0 Programming - Polling vs Interrupt Comparison

## Educational Project - ATmega128 Timer0 Demonstrations

**Course:** SOC 3050 - Embedded Systems and Applications  
**Year:** 2025  
**Professor:** Hong Jeong

---

## Overview

This simplified version focuses **exclusively on Timer0** to provide clear comparisons between **POLLING** and **INTERRUPT** programming methods. All timer modes (Normal, CTC, PWM) are demonstrated using only Timer0.

### Key Learning Objectives

1. **Master Timer0 control registers** (TCCR0, TCNT0, OCR0, TIFR, TIMSK)
2. **Compare polling vs interrupt methods** - understand CPU efficiency
3. **Understand timer modes** - Normal, CTC, Fast PWM, Phase Correct PWM
4. **Learn prescaler effects** on timing
5. **Practice ISR programming** with Timer0 overflow and compare match interrupts

---

## Timer0 Specifications

| Feature | Details |
|---------|---------|
| **Timer Type** | 8-bit timer (counts 0-255) |
| **Control Register** | TCCR0 (mode, prescaler, output compare) |
| **Counter Register** | TCNT0 (current value, readable/writable) |
| **Compare Register** | OCR0 (compare match value for CTC/PWM) |
| **Interrupt Vectors** | ISR(TIMER0_OVF_vect), ISR(TIMER0_COMP_vect) |
| **Prescaler Options** | 1, 8, 64, 256, 1024 |

---

## Demonstration Programs

### **Demo 1: Normal Mode - POLLING** 🔴
- **Method:** CPU checks TOV0 flag in loop
- **Prescaler:** 256
- **Overflow Period:** 4.096 ms (244 Hz)
- **Educational Point:** CPU is **BLOCKED** waiting for overflow
- **Limitation:** Cannot do other work while polling

```c
// Polling loop - CPU is stuck here!
while(1) {
    if (TIFR & (1 << TOV0)) {
        TIFR = (1 << TOV0);  // Clear flag
        // Do work
    }
}
```

### **Demo 2: Normal Mode - INTERRUPT** 🟢
- **Method:** ISR called automatically on overflow
- **Prescaler:** 256
- **Overflow Period:** 4.096 ms (244 Hz)
- **Educational Point:** CPU is **FREE** to do other work
- **Advantage:** Main loop can perform other tasks

```c
// ISR executes automatically
ISR(TIMER0_OVF_vect) {
    // Work happens here, CPU is free elsewhere
}

// Main loop can do other work!
while(1) {
    other_task_1();
    other_task_2();
}
```

### **Demo 3: CTC Mode - POLLING** 🔴
- **Method:** CPU checks OCF0 flag
- **Prescaler:** 64, **OCR0:** 249
- **Compare Frequency:** 1000 Hz (1 ms period)
- **Educational Point:** Precise timing with polling
- **Limitation:** CPU blocked again

```c
OCR0 = 249;  // TOP value for 1ms
TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);  // CTC + prescaler 64

while(1) {
    if (TIFR & (1 << OCF0)) {
        TIFR = (1 << OCF0);
        // Do work every 1ms
    }
}
```

### **Demo 4: CTC Mode - INTERRUPT** 🟢
- **Method:** ISR called on compare match
- **Prescaler:** 64, **OCR0:** 249
- **Compare Frequency:** 1000 Hz (1 ms period)
- **Educational Point:** Precise interrupt-driven timing
- **Advantage:** CPU free, same precision as polling

```c
ISR(TIMER0_COMP_vect) {
    // Executes every 1ms automatically
}
```

### **Demo 5: Fast PWM Mode** 🌊
- **Mode:** Fast PWM (0→255, reset)
- **Prescaler:** 64
- **PWM Frequency:** 976.5 Hz
- **Duty Cycle:** 0% to 100% sweep
- **Output:** OC0 pin (PB4)
- **Educational Point:** PWM generation for motor control, LED dimming

```c
// Fast PWM with non-inverting mode
TCCR0 = (1 << WGM01) | (1 << WGM00) | (1 << COM01) | 
        (1 << CS01) | (1 << CS00);
OCR0 = 128;  // 50% duty cycle
```

### **Demo 6: Phase Correct PWM** 🌊
- **Mode:** Phase Correct PWM (0→255→0, symmetric)
- **Prescaler:** 64
- **PWM Frequency:** ~490 Hz (half of Fast PWM)
- **Educational Point:** Symmetric waveform for motor control
- **Advantage:** Better for precise motor applications

```c
// Phase Correct PWM
TCCR0 = (1 << WGM00) | (1 << COM01) | 
        (1 << CS01) | (1 << CS00);
```

### **Demo 7: Prescaler Comparison** ⏱️
- **Demonstrates:** All 5 prescaler values (1, 8, 64, 256, 1024)
- **Educational Point:** Effect of prescaler on overflow frequency
- **Visualization:** LED toggle speed for each prescaler

| Prescaler | Timer Freq | Overflow Period |
|-----------|-----------|-----------------|
| 1         | 16 MHz    | 16 µs           |
| 8         | 2 MHz     | 128 µs          |
| 64        | 250 kHz   | 1.024 ms        |
| 256       | 62.5 kHz  | 4.096 ms        |
| 1024      | 15.625 kHz| 16.384 ms       |

### **Demo 8: Multi-tasking with ISR** 🚀
- **Method:** ISR-based task scheduling
- **Prescaler:** 64 with timer reload for 1ms ticks
- **Tasks:** 3 tasks at different rates (1ms, 100ms, 500ms)
- **Educational Point:** RTOS-like scheduling with timer interrupts
- **Advantage:** Multiple concurrent tasks from single timer

```c
ISR(TIMER0_OVF_vect) {
    TCNT0 = 6;  // Reload for 1ms
    timer_ticks++;
    
    if (timer_ticks % 1 == 0) task1_flag = 1;      // 1ms
    if (timer_ticks % 100 == 0) task2_flag = 1;    // 100ms
    if (timer_ticks % 500 == 0) task3_flag = 1;    // 500ms
}
```

---

## ISR Architecture - Important Notes

### Single ISR per Vector

AVR-GCC allows **only ONE ISR definition per interrupt vector**. This code solves this by using a **mode selector**:

```c
volatile uint8_t demo_mode = 0;  // 2=demo2, 8=demo8

ISR(TIMER0_OVF_vect) {
    if (demo_mode == 2) {
        // Demo 2 logic
        overflow_count++;
    }
    else if (demo_mode == 8) {
        // Demo 8 logic (multi-tasking)
        TCNT0 = 6;  // Reload timer
        timer_ticks++;
        // Set task flags...
    }
}
```

Each demo sets `demo_mode` before enabling interrupts, so the ISR knows which behavior to execute.

---

## Polling vs Interrupt Comparison Table

| Aspect | **POLLING** | **INTERRUPT** |
|--------|------------|--------------|
| **CPU Usage** | 🔴 Blocked in loop | 🟢 Free for other work |
| **Complexity** | 🟢 Simple logic | 🟡 Requires ISR knowledge |
| **Response Time** | 🟡 Depends on loop | 🟢 Immediate |
| **Precision** | 🟡 Can have jitter | 🟢 Hardware-driven |
| **Multitasking** | 🔴 Difficult | 🟢 Natural |
| **Debugging** | 🟢 Easy to trace | 🟡 More complex |
| **Power Efficiency** | 🔴 CPU always active | 🟢 Can sleep between ISRs |

**Recommendation:** Use **interrupts** for production code, **polling** for initial learning/debugging.

---

## Building the Project

### Using VS Code Tasks

Press `Ctrl+Shift+B` and select:
- **"Build Current Project"** - Compile Main.c
- **"Build and Simulate Current Project"** - Build and run in SimulIDE
- **"Program Hardware"** - Upload to physical ATmega128

### Manual Build

```powershell
cd w:\soc3050code\projects\Timer_Programming

# Build
w:\soc3050code\tools\avr-toolchain\bin\avr-gcc.exe `
  -mmcu=atmega128 -DF_CPU=16000000UL -Os -Wall -Werror `
  -I../../shared_libs -o Main.elf Main.c `
  ../../shared_libs/_uart.c ../../shared_libs/_timer2.c

# Generate HEX file
w:\soc3050code\tools\avr-toolchain\bin\avr-objcopy.exe `
  -O ihex Main.elf Main.hex

# Check memory usage
w:\soc3050code\tools\avr-toolchain\bin\avr-size.exe `
  --format=berkeley Main.elf
```

**Expected Output:**
```
   text    data     bss     dec     hex filename
   6686    2502      49    9237    2415 Main.elf
```

---

## Hardware Connections

### LED Indicators (PORTB)
- **PB0** - Demo 1 (Normal Polling)
- **PB1** - Demo 2 (Normal Interrupt)
- **PB2** - Demo 3 (CTC Polling)
- **PB3** - Demo 4 (CTC Interrupt)
- **PB4** - OC0 pin (PWM output for demos 5-6)
- **PB5** - Demo 7 (Prescaler comparison)
- **PB6** - Demo 8 Task 2 (100ms)
- **PB7** - Demo 8 Task 3 (500ms)

### Additional Outputs (PORTA)
- **PA0** - Demo 8 Task 1 (1ms, toggled every 500ms)
- **PA7-PA0** - PWM duty cycle display (Demo 5)

### Serial Connection
- **RXD1/TXD1** - UART communication @ 9600 baud
- Use serial terminal to view timing measurements

---

## Selecting a Demo

Change the `demo_choice` variable in `main()`:

```c
int demo_choice = 1;  // Change this to 1-8
```

Rebuild and upload to run different demonstrations.

---

## Educational Exercises

### Exercise 1: Measure Polling Overhead
1. Run Demo 1 and Demo 2 side-by-side
2. Add a counter in Demo 1's main loop
3. Compare how much "other work" can be done in Demo 2 vs Demo 1
4. **Question:** How much CPU time is wasted in polling?

### Exercise 2: Prescaler Effects
1. Run Demo 7
2. Use oscilloscope on PORTB pins
3. Measure actual frequencies
4. **Question:** Do they match theoretical calculations?

### Exercise 3: PWM Duty Cycle
1. Run Demo 5 (Fast PWM)
2. Connect oscilloscope to PB4 (OC0)
3. Measure frequency and duty cycle at different OCR0 values
4. **Question:** What OCR0 value gives exactly 25% duty cycle?

### Exercise 4: Multi-tasking
1. Run Demo 8
2. Modify task rates (change modulo values in ISR)
3. Add a 4th task at 250ms rate
4. **Question:** What's the maximum number of tasks you can schedule?

### Exercise 5: Compare CTC vs Normal
1. Generate 1kHz square wave using Demo 3 (CTC polling)
2. Generate 1kHz square wave using Demo 1 (Normal polling)
3. Measure jitter with oscilloscope
4. **Question:** Which method is more precise?

---

## Timer0 Register Reference (Quick Sheet)

### TCCR0 - Timer/Counter Control Register

```
Bit:   7      6      5      4      3      2      1      0
     FOC0  WGM00  COM01  COM00  WGM01   CS02   CS01   CS00
```

**Waveform Generation Mode (WGM01:00):**
- `00` = Normal (0→255, overflow)
- `01` = Phase Correct PWM
- `10` = CTC (Clear Timer on Compare)
- `11` = Fast PWM

**Clock Select (CS02:00):**
- `000` = No clock (stopped)
- `001` = clk/1
- `010` = clk/8
- `011` = clk/64
- `100` = clk/256
- `101` = clk/1024

### TIMSK - Timer Interrupt Mask

```
Bit 1: OCIE0 - Output Compare Match Interrupt Enable
Bit 0: TOIE0 - Timer Overflow Interrupt Enable
```

### TIFR - Timer Interrupt Flag Register

```
Bit 1: OCF0 - Output Compare Flag (clear by writing 1)
Bit 0: TOV0 - Timer Overflow Flag (clear by writing 1)
```

---

## Files in This Project

| File | Purpose |
|------|---------|
| `Main.c` | Complete Timer0 demonstration program |
| `README_Timer0_Only.md` | This documentation file |
| `Main.hex` | Compiled firmware for upload |
| `Main_original.c` | Backup of previous version (Timer0/1/2) |
| `Main_Timer0_only.c` | Source backup |

---

## Troubleshooting

### Problem: ISR Not Triggering
1. Check `sei()` is called after enabling interrupts
2. Verify TIMSK register is set correctly
3. Ensure global interrupts are enabled

### Problem: Wrong Timing
1. Verify F_CPU is 16000000UL
2. Check prescaler settings in TCCR0
3. Confirm OCR0 value for CTC mode

### Problem: Build Errors
1. Ensure config.h is included
2. Check all paths in build command
3. Verify AVR toolchain is in correct location

---

## Why Timer0 Only?

**Simplified Learning Path:**
- Focus on ONE timer deeply rather than three timers superficially
- Clear comparison of polling vs interrupt on same hardware
- Easier to understand ISR architecture
- All timer modes demonstrated without confusion
- Students master Timer0, then can apply knowledge to Timer1/Timer2

**Timer0 is ideal for learning:**
- Simple 8-bit design
- All common timer modes supported
- Both overflow and compare match interrupts
- Sufficient for most embedded timing tasks

---

## Next Steps

After mastering Timer0:
1. Study Timer1 (16-bit timer with input capture)
2. Learn Timer2 (async operation with RTC crystal)
3. Explore Timer3 (16-bit, similar to Timer1)
4. Apply timers to real applications (motor control, sensor sampling, etc.)

---

## Summary

This Timer0-focused project provides:
✅ Clear polling vs interrupt comparison  
✅ All timer modes on single timer  
✅ Practical ISR programming examples  
✅ Real-world timing demonstrations  
✅ Foundation for advanced timer usage  

**Master Timer0, master all timers!** 🎓

---

*Last Updated: January 2025*  
*For questions: Contact Professor Hong Jeong*
