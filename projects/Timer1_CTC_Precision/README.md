# Timer1 CTC Mode - Precision Timing

## Learning Objectives

1. **Understand CTC Mode**: Clear Timer on Compare match for exact frequencies
2. **Calculate OCR1A Values**: Generate precise timing signals using formulas
3. **Compare vs Overflow**: Learn advantages of CTC over overflow interrupts
4. **Millisecond Timing**: Create accurate millis() function for delays

## What is CTC Mode?

**CTC** = **Clear Timer on Compare Match**

In CTC mode, the timer automatically resets to zero when it matches the OCR1A value. This allows you to generate EXACT frequencies, not limited to powers of 2 like overflow mode.

### Formula:
```
F_output = F_CPU / (Prescaler × (OCR1A + 1))

Or rearranged:
OCR1A = (F_CPU / (Prescaler × F_output)) - 1
```

### Example Calculation for 1 Hz:
```
F_CPU = 7372800 Hz
Prescaler = 1024
F_output = 1 Hz (1 second period)

OCR1A = (7372800 / (1024 × 1)) - 1
OCR1A = 7200 - 1
OCR1A = 7199
```

## Demos Included

### Demo 1: 1-Second Precision Blink
- Uses 1 Hz CTC interrupt
- LED toggles every second exactly
- Counts toggles to verify accuracy
- **Learning**: Basic CTC configuration

### Demo 2: Variable Frequencies
- Switch between 1 Hz, 2 Hz, 5 Hz, 10 Hz
- Real-time OCR1A reconfiguration
- No timer restart needed
- **Learning**: Dynamic frequency changes

### Demo 3: Millisecond Counter
- 100 Hz CTC (every 10ms)
- Implements millis() function
- Displays elapsed time
- **Learning**: Practical timing application

### Demo 4: Accuracy Test
- Runs for 10 seconds
- Counts actual toggles vs expected
- Calculates error percentage
- **Learning**: Timer precision verification

## Key Registers

### TCCR1B - Timer Control Register B
```c
TCCR1B = (1 << WGM12);  // Enable CTC mode (WGM13:12 = 01)
```

### OCR1A - Output Compare Register A
```c
OCR1A = 7199;  // Top value for comparison
```

### TIMSK - Timer Interrupt Mask
```c
TIMSK |= (1 << OCIE1A);  // Enable compare match A interrupt
```

### ISR - Interrupt Service Routine
```c
ISR(TIMER1_COMPA_vect) {
    // Called when TCNT1 == OCR1A
    // Timer automatically resets to 0
}
```

## Prescaler Selection Guide

| Prescaler | Lowest Frequency | Highest Frequency | Best For |
|-----------|------------------|-------------------|----------|
| 1         | 112 Hz           | 7.37 MHz          | High frequency signals |
| 8         | 14 Hz            | 921 kHz           | Audio range |
| 64        | 1.75 Hz          | 115 kHz           | **Millisecond timing** |
| 256       | 0.44 Hz          | 28.8 kHz          | Sub-second timing |
| 1024      | 0.11 Hz          | **7.2 kHz**       | **Second-range timing** |

## Common OCR1A Values

### For 1 Hz (Prescaler 1024):
```c
OCR1A = 7199;  // (7372800 / (1024 × 1)) - 1
```

### For 2 Hz (Prescaler 1024):
```c
OCR1A = 3599;  // (7372800 / (1024 × 2)) - 1
```

### For 10 Hz (Prescaler 1024):
```c
OCR1A = 719;   // (7372800 / (1024 × 10)) - 1
```

### For 100 Hz (Prescaler 64):
```c
OCR1A = 1151;  // (7372800 / (64 × 100)) - 1
```

## Why CTC is Better Than Overflow

### Overflow Mode Limitations:
- Timer counts 0 → 65535, then overflows
- Frequency = F_CPU / (Prescaler × 65536)
- Can only get specific frequencies (e.g., 1.74 Hz with prescaler 64)
- **Cannot get exact 1 Hz, 2 Hz, etc.**

### CTC Mode Advantages:
- ✅ Set ANY frequency (within limits)
- ✅ Exact 1 Hz, 2 Hz, 10 Hz, 100 Hz, etc.
- ✅ Easy to calculate OCR1A
- ✅ Timer auto-resets (no manual TCNT1 = 0)
- ✅ More accurate for long periods

## Common Student Questions

**Q: Why does OCR1A = 7199 give 1 Hz, not 7200?**
A: The timer counts from 0 to OCR1A (inclusive). That's actually 7200 counts (0, 1, 2, ... 7199). The formula uses (OCR1A + 1) to account for this.

**Q: Can I change OCR1A while the timer is running?**
A: Yes! Just write a new value. The change takes effect on the next comparison.

**Q: What happens if I write OCR1A smaller than TCNT1?**
A: The timer will continue counting to MAX (65535), overflow to 0, then match OCR1A. Best to reset TCNT1 when changing modes.

**Q: Why use prescaler 64 for milliseconds?**
A: 100 Hz with prescaler 1024 needs OCR1A = 71 (very small, less precise). Prescaler 64 gives OCR1A = 1151 (better resolution).

**Q: How accurate is this?**
A: With a crystal oscillator, typically ±50 ppm (0.005%). That's less than 5 seconds error per day!

## Building and Running

### Build:
```batch
.\build.bat
```

### Program Hardware:
```batch
cd w:\soc3050code\tools\cli
.\cli-program-project.ps1 -ProjectDir w:\soc3050code\projects\Timer1_CTC_Precision -Programmer arduino -Port COM3
```

### Expected Output:
```
text:  7698 bytes
data:  2482 bytes
bss:     31 bytes
Total: 10211 bytes (0x27e3)
```

## Testing Checklist

- [ ] Demo 1: LED blinks exactly once per second
- [ ] Demo 1: Toggle count displayed on LCD/UART
- [ ] Demo 2: Frequency changes work (1/2/5/10 Hz)
- [ ] Demo 2: Visual speed differences obvious
- [ ] Demo 3: Millisecond counter increments smoothly
- [ ] Demo 3: Matches stopwatch timing
- [ ] Demo 4: Error less than 1% over 10 seconds
- [ ] Demo 4: Expected vs actual counts match

## Next Steps

After mastering CTC mode:

1. **Timer1_Input_Capture** - Measure external frequencies
2. **Timer_Stopwatch** - Build a practical stopwatch application
3. **Timer_Software_RTC** - Create a real-time clock
4. **PWM_Motor_Control** - Use CTC with PWM for motor speed control

## Key Takeaways

- CTC mode gives **exact frequencies** (overflow mode doesn't)
- Formula: `OCR1A = (F_CPU / (Prescaler × F_output)) - 1`
- Choose prescaler based on desired frequency range
- CTC is perfect for **timing applications** (clocks, delays, metronomes)
- Millisecond timing needs ~100 Hz (prescaler 64, OCR1A = 1151)
