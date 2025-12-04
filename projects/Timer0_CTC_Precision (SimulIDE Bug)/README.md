# Timer0 CTC Mode - Precision Timing

## Learning Objectives

1. **Understand Timer0 CTC Mode**: 8-bit Clear Timer on Compare match for kHz-range frequencies
2. **Calculate OCR0 Values**: Generate precise high-frequency timing signals
3. **Compare Timer0 vs Timer1**: Learn when to use 8-bit vs 16-bit timers
4. **Master 8-bit Limitations**: Work within 0-255 range effectively

## What is Timer0 CTC Mode?

**CTC** = **Clear Timer on Compare Match**

Timer0 is an **8-bit timer** that automatically resets to zero when it matches the OCR0 value. This allows you to generate EXACT frequencies in the kHz range.

### Formula:
```
F_output = F_CPU / (Prescaler × (OCR0 + 1))

Or rearranged:
OCR0 = (F_CPU / (Prescaler × F_output)) - 1
```

### Example Calculation for 1 kHz:
```
F_CPU = 16000000 Hz
Prescaler = 64
F_output = 1000 Hz (1 kHz = 1ms period)

OCR0 = (16000000 / (64 × 1000)) - 1
OCR0 = 250 - 1
OCR0 = 249
```

## Timer0 vs Timer1

| Feature | Timer0 | Timer1 |
|---------|--------|--------|
| **Bit Width** | 8-bit (0-255) | 16-bit (0-65535) |
| **Compare Register** | OCR0 | OCR1A, OCR1B |
| **Prescaler Options** | /8, /64, /256, /1024 | /1, /8, /64, /256, /1024 |
| **Best For** | kHz range (1-100 kHz) | Hz range (0.1-1000 Hz) |
| **Interrupt** | TIMER0_COMP_vect | TIMER1_COMPA_vect |
| **Use Cases** | Fast timing, PWM | Seconds, RTC, slow timing |

## Demos Included

### Demo 1: 1 kHz Polling
- Uses 1 kHz CTC (1ms period)
- Polls compare flag manually
- LED toggles every 500ms
- **Learning**: Basic Timer0 CTC configuration

### Demo 2: 1 kHz Interrupt
- Interrupt-driven millisecond counter
- ISR handles compare match
- Main loop stays free
- **Learning**: Timer0 ISR implementation

### Demo 3: Multiple Frequencies
- Switch between 1 kHz, 2 kHz, 4 kHz, 8 kHz
- Real-time OCR0 reconfiguration
- Visual frequency differences
- **Learning**: Dynamic frequency changes

### Demo 4: Precision Test
- Runs for 10 seconds
- Counts 10,000 matches (1 kHz × 10s)
- Calculates timing accuracy
- **Learning**: Timer precision verification

## Key Registers

### TCCR0 - Timer Control Register
```c
TCCR0 = (1 << WGM01);  // Enable CTC mode (WGM01=1, WGM00=0)
```

### OCR0 - Output Compare Register (8-bit)
```c
OCR0 = 249;  // Top value for comparison (max 255)
```

### TIMSK - Timer Interrupt Mask
```c
TIMSK |= (1 << OCIE0);  // Enable compare match interrupt
```

### ISR - Interrupt Service Routine
```c
ISR(TIMER0_COMP_vect) {
    // Called when TCNT0 == OCR0
    // Timer automatically resets to 0
}
```

## Prescaler Selection Guide (Timer0)

| Prescaler | Timer Freq | Min Freq | Max Freq | OCR0 Range | Best For |
|-----------|------------|----------|----------|------------|----------|
| 8         | 2 MHz      | 7.8 kHz  | 2 MHz    | 0-255      | Very high freq |
| **64**    | **250 kHz**| **976 Hz** | **250 kHz** | **0-255** | **kHz range** ⭐ |
| 256       | 62.5 kHz   | 244 Hz   | 62.5 kHz | 0-255      | Medium freq |
| 1024      | 15.6 kHz   | 61 Hz    | 15.6 kHz | 0-255      | Lower freq |

**Note**: Prescaler 64 is ideal for kHz-range frequencies (1-10 kHz)

## Common OCR0 Values @ 16 MHz

### With Prescaler 64:
```c
OCR0 = 249;  // 1 kHz   (16MHz / 64 / 1000)  - 1
OCR0 = 124;  // 2 kHz   (16MHz / 64 / 2000)  - 1
OCR0 = 62;   // 4 kHz   (16MHz / 64 / 4000)  - 1
OCR0 = 31;   // 8 kHz   (16MHz / 64 / 8000)  - 1
OCR0 = 15;   // 16 kHz  (16MHz / 64 / 16000) - 1
```

### With Prescaler 256:
```c
OCR0 = 255;  // ~244 Hz (16MHz / 256 / 244)  - 1
OCR0 = 155;  // 400 Hz  (16MHz / 256 / 400)  - 1
OCR0 = 77;   // 800 Hz  (16MHz / 256 / 800)  - 1
```

## 8-Bit Limitations

### Maximum OCR0 Value: 255
If your calculation exceeds 255, you have two options:
1. **Use larger prescaler** (e.g., 256 or 1024 instead of 64)
2. **Use Timer1** for lower frequencies

### Example - 100 Hz FAILS with prescaler 64:
```c
OCR0 = (16000000 / (64 × 100)) - 1 = 2499  // ERROR! > 255
```

**Solution**: Use prescaler 256:
```c
OCR0 = (16000000 / (256 × 100)) - 1 = 624  // Still > 255!
```

**Best Solution**: Use Timer1 for frequencies below ~1 kHz!

## When to Use Timer0 vs Timer1

### Use Timer0 When:
✓ Frequency is in **kHz range** (1-100 kHz)  
✓ Need **fast ISR** (8-bit comparison)  
✓ Timer1 is busy with other tasks  
✓ Millisecond precision timing  
✓ High-speed sampling or PWM  

### Use Timer1 When:
✓ Frequency is in **Hz range** (0.1-1000 Hz)  
✓ Need **second-range** periods  
✓ Require **16-bit resolution**  
✓ Building real-time clocks  
✓ Low-frequency signal generation  

## Practical Applications

### Timer0 CTC Applications:
- **millis()** function (1 kHz interrupt)
- **High-speed ADC** sampling (10 kHz)
- **Audio tone** generation (440 Hz = A note)
- **Fast LED PWM** (20 kHz for flicker-free)
- **UART bit timing** helpers
- **SPI/I2C** timeout monitoring

### Example: millis() Implementation
```c
volatile uint32_t milliseconds = 0;

ISR(TIMER0_COMP_vect) {
    milliseconds++;  // Increment every 1ms
}

void setup_millis(void) {
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);  // CTC, /64
    OCR0 = 249;                                         // 1 kHz
    TIMSK |= (1 << OCIE0);                             // Enable interrupt
    sei();
}

uint32_t millis(void) {
    uint32_t m;
    cli();
    m = milliseconds;
    sei();
    return m;
}
```

## Why CTC is Better Than Overflow

### Overflow Mode Limitations:
- Timer counts 0 → 255, then overflows
- Frequency = F_CPU / (Prescaler × 256)
- Can only get specific frequencies
- **Cannot get exact 1 kHz, 2 kHz, etc.**

### CTC Mode Advantages:
- ✅ Set ANY frequency (within 8-bit limits)
- ✅ Exact 1 kHz, 2 kHz, 4 kHz, etc.
- ✅ Easy to calculate OCR0
- ✅ Timer auto-resets (no manual TCNT0 = 0)
- ✅ More accurate for timing applications

## Common Student Questions

**Q: Why use Timer0 when Timer1 has more resolution?**
A: Timer0 is perfect for kHz-range frequencies and has faster ISR execution. Save Timer1 for low-frequency tasks.

**Q: What if my OCR0 calculation is > 255?**
A: Either use a larger prescaler or switch to Timer1 for that frequency range.

**Q: Can Timer0 generate 1 Hz like Timer1?**
A: Not efficiently. With max prescaler (1024) and max OCR0 (255):
   Min freq = 16MHz / (1024 × 256) = 61 Hz
   Use Timer1 for Hz-range frequencies!

**Q: Why doesn't Timer0 have /1 prescaler?**
A: Design choice by Atmel. Timer0 is meant for slower applications. Use Timer1 for /1 prescaler.

**Q: How accurate is Timer0 CTC?**
A: With a crystal oscillator: ±50 ppm (0.005%). Very precise for embedded timing.

## Building and Running

### Build:
```batch
.\build.bat
```

### Program Hardware:
```powershell
cd w:\soc3050code\tools\cli
.\cli-program-project.ps1 -ProjectDir w:\soc3050code\projects\Timer0_CTC_Precision -Programmer arduino -Port COM3
```

### Simulate in SimulIDE:
```powershell
cd w:\soc3050code\tools\simulide
.\cli-simulide.ps1 -ProjectDir w:\soc3050code\projects\Timer0_CTC_Precision
```

## Testing Checklist

- [ ] Demo 1: LED blinks every 500ms (0.5 seconds)
- [ ] Demo 1: Timing is stable and consistent
- [ ] Demo 2: Millisecond counter increments correctly
- [ ] Demo 2: Main loop remains responsive
- [ ] Demo 3: All 4 frequencies visibly different
- [ ] Demo 3: Transitions smooth between frequencies
- [ ] Demo 4: Completes 10-second test
- [ ] Demo 4: Match count equals 10,000 (±1%)

## Next Steps

After mastering Timer0 CTC mode:

1. **Timer1_CTC_Precision** - Learn 16-bit CTC for Hz range
2. **Timer0_CTC_PWM** - Combine CTC with PWM output
3. **Timer_Stopwatch** - Build millisecond-precision stopwatch
4. **ADC_Triggered_Timer** - Use Timer0 to trigger ADC sampling
5. **PWM_Fast_Mode** - High-frequency PWM for motors/LEDs

## Key Takeaways

- Timer0 is **8-bit** (0-255), Timer1 is **16-bit** (0-65535)
- Timer0 best for **kHz range** (1-100 kHz), Timer1 for **Hz range** (0.1-1000 Hz)
- Formula: `OCR0 = (F_CPU / (Prescaler × F_output)) - 1`
- Prescaler **64** is ideal for kHz frequencies
- If OCR0 > 255, use **larger prescaler** or **Timer1**
- Perfect for **millis()**, **fast PWM**, and **high-speed sampling**
