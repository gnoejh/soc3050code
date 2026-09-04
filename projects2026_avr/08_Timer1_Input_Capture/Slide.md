# Timer1 Input Capture - Frequency and Pulse Measurement

## ATmega128 Embedded Systems Course

**Reference**: [ATmega128 Datasheet - Section 16.8](https://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf)  
**Target**: ATmega128 @ 16 MHz  
**Last Updated**: December 1, 2025

---

## Slide 1: Introduction to Input Capture

### What is Input Capture?

- **Hardware feature** that captures timer value on external event
- **ICP1 pin** (PD4 on ATmega128) triggers capture
- **Saves TCNT1** value to ICR1 register automatically
- Used for **precise timing** of external events

**Demo Setup**: ICP1 is connected to Push Button SW4 (PD4). Press SW4 to test!

### Why Use Input Capture?

✓ **Hardware precision** - no software delay  
✓ **Microsecond accuracy** - captures exact timer value  
✓ **Frequency measurement** - measure signal periods  
✓ **Pulse width measurement** - measure duty cycles  
✓ **RPM sensing** - rotary encoders, wheel speed  

### Applications

- Frequency counter
- Tachometer (RPM measurement)
- Ultrasonic distance (echo timing)
- IR remote decoding
- Pulse width measurement

---

## Slide 2: Input Capture Hardware

### SimulIDE Compatibility

| Version | Status | Notes |
|---------|--------|-------|
| **SimulIDE 1.1.0-SR1** | ✅ **Fully Functional** | Timer1 Type 160 works perfectly |
| **SimulIDE 0.4.15** | ✅ **Fully Functional** | All features work |
| **Hardware** | ✅ **Fully Functional** | Reference implementation |

**Note**: Timer1 Input Capture (Type 160) works in ALL SimulIDE versions!

### Timer1 Input Capture Architecture

```mermaid
graph TD
    A[ICP1 Pin PD4<br/>Push Button SW4] --> B[Noise Canceler Optional]
    B --> C[Edge Detector]
    C --> D{Edge Match?}
    D -->|Yes| E[Capture TCNT1]
    E --> F[Store in ICR1]
    F --> G[Set ICF1 Flag]
    G --> H[Trigger TIMER1_CAPT ISR]
    
    I[TCNT1 Counter] --> E
```

### Input Capture Resources

| Register | Description |
|----------|-------------|
| **ICR1H/L** | Input Capture Register (captured timer value) |
| **TCNT1H/L** | Timer Counter (free-running) |
| **TCCR1B** | Control register (edge select, noise canceler) |
| **TIMSK** | Interrupt mask (TICIE1) |
| **TIFR** | Interrupt flag (ICF1) |

### Pin Assignment

| Timer | ICP Pin | ATmega128 Port | Hardware Connection |
|-------|---------|----------------|---------------------|
| Timer1 | ICP1 | PD4 | Push Button SW4 |
| Timer3 | ICP3 | PE4 | Available |

---

## Slide 3: Input Capture Configuration

### TCCR1B - Control Register (IC bits)

```
Bit     7      6      5      4      3      2      1      0
      ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
TCCR1B│ ICNC1│ ICES1│  -   │ WGM13│ WGM12│ CS12 │ CS11 │ CS10 │
      └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
```

| Bit | Name | Description |
|-----|------|-------------|
| **ICES1** | Input Capture Edge Select | 1=Rising, 0=Falling |
| **ICNC1** | Input Capture Noise Canceler | 1=Enable (4-clock delay) |
| **CS12:10** | Clock Select | Prescaler setting |

### Basic Configuration

```c
// Configure ICP1 as input (PD4 = Push Button SW4)
DDRD &= ~(1 << PD4);   // PD4 as input
PORTD |= (1 << PD4);   // Enable pull-up (required for button)

// Configure Timer1 for input capture
TCCR1B = (1 << ICES1);  // Capture on rising edge
TCCR1B |= (1 << CS11);  // Prescaler /8 (for 16MHz: 2MHz timer clock)

// Enable input capture interrupt
TIMSK |= (1 << TICIE1);
sei();
```

### Edge Selection

```c
// Rising edge
TCCR1B |= (1 << ICES1);

// Falling edge
TCCR1B &= ~(1 << ICES1);

// Toggle edge (for both edges)
TCCR1B ^= (1 << ICES1);
```

---

## Slide 4: Frequency Measurement Principles

### Method: Period Measurement

```
Signal:  ┌─────┐     ┌─────┐     ┌─────┐
         │     │     │     │     │     │
    ─────┘     └─────┘     └─────┘     └─────
         ↑           ↑           ↑
      Capture1   Capture2   Capture3
         │←─ Period ─→│
         
Period = Capture2 - Capture1
Frequency = 1 / Period
```

### Calculation Formula @ 16 MHz

```
Timer frequency after prescaler:
  F_timer = F_CPU / Prescaler
  F_timer = 16,000,000 / Prescaler
  
Ticks between edges:
  Ticks = ICR1_new - ICR1_old
  
Signal frequency:
  F_signal = F_timer / Ticks
  
Example 1 - Prescaler /8, 1 kHz signal:
  F_timer = 16,000,000 / 8 = 2,000,000 Hz
  Ticks = 2000
  F_signal = 2,000,000 / 2000 = 1,000 Hz ✓
  
Example 2 - Prescaler /64, 100 Hz signal:
  F_timer = 16,000,000 / 64 = 250,000 Hz
  Ticks = 2500
  F_signal = 250,000 / 2500 = 100 Hz ✓
```

---

## Slide 5: Frequency Measurement Implementation

### ISR-based Frequency Counter @ 16 MHz

```c
#define F_CPU 16000000UL
#define PRESCALER 64                    // For 10 Hz - 10 kHz range
#define TIMER_FREQ (F_CPU / PRESCALER)  // 250 kHz

volatile uint16_t last_capture = 0;
volatile uint32_t frequency = 0;

ISR(TIMER1_CAPT_vect) {
    uint16_t current_capture = ICR1;
    
    // Calculate ticks (handles single overflow)
    uint16_t ticks = current_capture - last_capture;
    last_capture = current_capture;
    
    // Calculate frequency: 250000 / ticks
    if (ticks > 0) {
        frequency = (uint32_t)TIMER_FREQ / ticks;
    }
}

int main(void) {
    // Setup ICP1 pin (PD4 = Button SW4)
    DDRD &= ~(1 << PD4);   // PD4 as input
    PORTD |= (1 << PD4);   // Enable pull-up (required)
    
    // Timer1: Normal mode, /64 prescaler, rising edge
    TCCR1A = 0x00;
    TCCR1B = (1 << ICES1) | (1 << CS11) | (1 << CS10);  // /64
    
    // Enable capture interrupt
    TIMSK |= (1 << TICIE1);
    sei();
    
    while(1) {
        // Display frequency (250000/ticks Hz)
        printf("Frequency: %lu Hz\r\n", frequency);
        _delay_ms(500);
    }
}
```

### Handling Timer Overflow

```c
volatile uint8_t overflow_count = 0;

ISR(TIMER1_OVF_vect) {
    overflow_count++;
}

ISR(TIMER1_CAPT_vect) {
    uint32_t current_capture = ((uint32_t)overflow_count << 16) | ICR1;
    static uint32_t last_capture = 0;
    
    uint32_t ticks = current_capture - last_capture;
    last_capture = current_capture;
    
    if (ticks > 0) {
        frequency = TIMER_FREQ / ticks;
    }
    overflow_count = 0;
}
```

---

## Slide 6: Pulse Width Measurement

### Method: Capture Both Edges

```
Signal:  ┌───────────────┐
         │               │
    ─────┘               └─────────────
         ↑               ↑
    Rising Edge    Falling Edge
    Capture1       Capture2
         │← Pulse Width →│
         
Pulse Width = Capture2 - Capture1
Duty Cycle = (Pulse Width / Period) × 100%
```

### Implementation

```c
volatile uint16_t pulse_start = 0;
volatile uint16_t pulse_width = 0;
volatile uint8_t edge_state = 0;  // 0=waiting for rising, 1=waiting for falling

ISR(TIMER1_CAPT_vect) {
    if (edge_state == 0) {
        // Rising edge - start of pulse
        pulse_start = ICR1;
        edge_state = 1;
        
        // Switch to falling edge detection
        TCCR1B &= ~(1 << ICES1);
    }
    else {
        // Falling edge - end of pulse
        uint16_t pulse_end = ICR1;
        pulse_width = pulse_end - pulse_start;
        edge_state = 0;
        
        // Switch back to rising edge
        TCCR1B |= (1 << ICES1);
    }
}

// Calculate pulse width in microseconds
uint32_t get_pulse_width_us(void) {
    // For prescaler /8: each tick = 0.5 µs
    return (pulse_width * PRESCALER) / (F_CPU / 1000000UL);
}
```

---

## Slide 7: Prescaler Selection and Range

### Frequency Measurement Ranges @ 16 MHz

| Prescaler | Timer Freq | Min Freq | Max Freq | Resolution | Best Range |
|-----------|------------|----------|----------|------------|------------|
| **/1** | 16.000 MHz | 244 Hz | 8.0 MHz | 62.5 ns | > 10 kHz |
| **/8** | 2.000 MHz | 31 Hz | 1.0 MHz | 500 ns | 1 kHz - 100 kHz |
| **/64** | 250 kHz | 4 Hz | 125 kHz | 4.0 µs | **10 Hz - 10 kHz** ⭐ |
| **/256** | 62.5 kHz | 1 Hz | 31.25 kHz | 16 µs | 1 Hz - 1 kHz |
| **/1024** | 15.625 kHz | 0.24 Hz | 7.8 kHz | 64 µs | < 1 Hz |

**Calculation Formulas:**

```
Timer_Freq = F_CPU / Prescaler = 16,000,000 / Prescaler
Min_Freq = Timer_Freq / 65536  (16-bit counter max)
Max_Freq = Timer_Freq / 2      (minimum 2 ticks)
Resolution = 1 / Timer_Freq
```

### Prescaler Selection Guide

```
Signal > 10 kHz:        Use /1 or /8
Signal 100 Hz - 10 kHz: Use /64 (recommended default)
Signal 1 - 100 Hz:      Use /256
Signal < 1 Hz:          Use /1024
```

**Configuration Bits (CS12:10)**:

| CS12 | CS11 | CS10 | Prescaler | Code |
|------|------|------|-----------|------|
| 0 | 0 | 1 | /1 | `(1 << CS10)` |
| 0 | 1 | 0 | /8 | `(1 << CS11)` |
| 0 | 1 | 1 | /64 | `(1 << CS11) \| (1 << CS10)` |
| 1 | 0 | 0 | /256 | `(1 << CS12)` |
| 1 | 0 | 1 | /1024 | `(1 << CS12) \| (1 << CS10)` |

---

## Slide 8: Noise Canceler

### Purpose

- Filters out **short glitches** on ICP pin
- Uses **4-clock sampling** window
- Reduces **false triggers**

### Configuration

```c
// Enable noise canceler
TCCR1B |= (1 << ICNC1);

// Disable noise canceler (more responsive)
TCCR1B &= ~(1 << ICNC1);
```

### Trade-offs

| Feature | ICNC1 = 0 (Off) | ICNC1 = 1 (On) |
|---------|-----------------|----------------|
| **Response** | Immediate | 4 clocks delay |
| **Noise** | Sensitive | Filtered |
| **Use Case** | Clean signals | Noisy environments |

### When to Use

✓ **Enable** for: Long cables, motors, switches  
✗ **Disable** for: High-speed signals, short pulses  

---

## Slide 9: Polling vs Interrupt Methods

### Method 1: Polling

```c
uint16_t measure_frequency_polling(void) {
    uint16_t capture1, capture2;
    
    // Wait for first edge
    while (!(TIFR & (1 << ICF1)));
    TIFR |= (1 << ICF1);  // Clear flag
    capture1 = ICR1;
    
    // Wait for second edge
    while (!(TIFR & (1 << ICF1)));
    TIFR |= (1 << ICF1);
    capture2 = ICR1;
    
    // Calculate frequency
    uint16_t ticks = capture2 - capture1;
    return (uint32_t)TIMER_FREQ / ticks;
}
```

**Pros:** Simple, blocking until measurement complete  
**Cons:** CPU locked, can't do other work

### Method 2: Interrupt

```c
volatile uint16_t frequency = 0;
volatile uint8_t measurement_ready = 0;

ISR(TIMER1_CAPT_vect) {
    static uint16_t last_capture = 0;
    uint16_t current = ICR1;
    
    uint16_t ticks = current - last_capture;
    last_capture = current;
    
    if (ticks > 0) {
        frequency = (uint32_t)TIMER_FREQ / ticks;
        measurement_ready = 1;
    }
}

int main(void) {
    // ... setup ...
    
    while(1) {
        if (measurement_ready) {
            measurement_ready = 0;
            printf("Freq: %u Hz\r\n", frequency);
        }
        
        // Can do other work here
    }
}
```

**Pros:** CPU free, multitasking  
**Cons:** Slightly more complex

---

## Slide 10: Advanced Applications

### Application 1: Tachometer (RPM Measurement)

```c
// Sensor: 1 pulse per revolution
ISR(TIMER1_CAPT_vect) {
    static uint16_t last_capture = 0;
    uint16_t current = ICR1;
    
    uint16_t ticks = current - last_capture;
    last_capture = current;
    
    // RPM = (60 × Timer_Freq) / (Ticks × Pulses_per_Rev)
    uint32_t rpm = (60UL * TIMER_FREQ) / ticks;
    
    display_rpm(rpm);
}
```

### Application 2: Ultrasonic Distance Sensor (HC-SR04)

```c
// Measure echo pulse width @ 16 MHz
// Prescaler /8 → Each tick = 0.5 µs
volatile uint16_t distance_cm = 0;

ISR(TIMER1_CAPT_vect) {
    static uint16_t pulse_start = 0;
    static uint8_t state = 0;
    
    if (state == 0) {  // Rising edge - echo start
        pulse_start = ICR1;
        TCCR1B &= ~(1 << ICES1);  // Switch to falling
        state = 1;
    }
    else {  // Falling edge - echo end
        uint16_t pulse_width = ICR1 - pulse_start;
        
        // Distance calculation:
        // Speed of sound = 343 m/s = 0.0343 cm/µs
        // Round trip, so divide by 2
        // Each tick @ /8 = 0.5 µs
        // Distance = (ticks × 0.5 µs × 0.0343 cm/µs) / 2
        // Simplified: Distance ≈ ticks / 58
        
        distance_cm = pulse_width / 58;
        
        TCCR1B |= (1 << ICES1);  // Switch to rising
        state = 0;
    }
}
```

### Application 3: IR Remote Decoder

```c
// Decode IR pulses (e.g., NEC protocol)
ISR(TIMER1_CAPT_vect) {
    static uint16_t last_time = 0;
    uint16_t current_time = ICR1;
    
    uint16_t pulse_length = current_time - last_time;
    last_time = current_time;
    
    // Decode based on pulse length
    if (pulse_length > 13500 && pulse_length < 13900) {
        // Start bit detected
    }
    else if (pulse_length > 1000 && pulse_length < 1300) {
        // Logic '0'
    }
    else if (pulse_length > 2000 && pulse_length < 2300) {
        // Logic '1'
    }
    
    TCCR1B ^= (1 << ICES1);  // Toggle edge
}
```

---

## Slide 11: Common Pitfalls and Solutions

### Pitfall 1: Division by Zero

```c
// ❌ WRONG - may crash if ticks = 0
frequency = TIMER_FREQ / ticks;

// ✅ CORRECT - check first
if (ticks > 0) {
    frequency = TIMER_FREQ / ticks;
}
```

### Pitfall 2: Integer Overflow

```c
// ❌ WRONG - overflow if frequency high
uint16_t freq = TIMER_FREQ / ticks;  // Max 65535

// ✅ CORRECT - use 32-bit
uint32_t freq = (uint32_t)TIMER_FREQ / ticks;
```

### Pitfall 3: Forgetting Edge Toggle

```c
// ❌ WRONG - stuck on one edge
ISR(TIMER1_CAPT_vect) {
    // Measure pulse, but never toggle edge!
}

// ✅ CORRECT - alternate edges
ISR(TIMER1_CAPT_vect) {
    // ... measurement ...
    TCCR1B ^= (1 << ICES1);  // Toggle edge
}
```

### Best Practices

✓ **Choose prescaler** for expected frequency range  
✓ **Check for zero** before division  
✓ **Use 32-bit** for frequency calculations  
✓ **Toggle edge** for pulse width measurement  
✓ **Enable noise canceler** for noisy signals  

---

## Slide 12: Summary and Key Takeaways

### Input Capture Key Concepts

✓ **Hardware precision** - captures exact timer value (zero CPU overhead)  
✓ **ICP1 pin** (PD4 / SW4) triggers capture on edge  
✓ **ICR1 register** stores captured TCNT1 value automatically  
✓ **Edge selection** - rising, falling, or toggle both  
✓ **Noise canceler** - optional 4-clock filter for noisy signals  
✓ **Wide range** - 0.24 Hz to 8 MHz (depending on prescaler)

### Critical Formulas @ 16 MHz

```
Timer frequency:
  F_timer = 16,000,000 / Prescaler
  
Signal frequency:
  F_signal = F_timer / (Capture₂ - Capture₁)
  
Example (Prescaler /64):
  F_timer = 250,000 Hz
  Ticks = 2500
  F_signal = 250,000 / 2500 = 100 Hz
  
Pulse width:
  Pulse_Width = Falling_Capture - Rising_Capture
  Time_µs = Pulse_Width × (Prescaler / 16)
```

### Register Summary

| Register | Purpose | Key Bits |
|----------|---------|----------|
| **ICR1H/L** | Captured timer value | Read captured TCNT1 |
| **TCCR1B** | Control register | ICES1 (edge), ICNC1 (noise), CS12:10 (prescaler) |
| **TIMSK** | Interrupt enable | TICIE1 (enable ISR) |
| **TIFR** | Interrupt flag | ICF1 (capture occurred) |

### Common Applications

| Application | Prescaler | Typical Range | Example |
|-------------|-----------|---------------|---------|
| **Frequency Counter** | /64 | 10 Hz - 10 kHz | Audio, PWM signals |
| **Tachometer (RPM)** | /256 | 1-100 Hz | Motor speed sensing |
| **Ultrasonic Sensor** | /8 | 150 µs - 25 ms | Distance measurement |
| **IR Remote Decoder** | /8 | 560 µs - 2.25 ms | Remote control |
| **Servo PWM Reader** | /64 | 1-2 ms pulses | RC servo feedback |

### SimulIDE Compatibility

✅ **Timer1 Input Capture works in ALL SimulIDE versions**  

- SimulIDE 1.1.0-SR1: ✅ Fully functional  
- SimulIDE 0.4.15: ✅ Fully functional  
- Hardware: ✅ Reference implementation

### Best Practices

✓ **Check for zero** before division (avoid crash)  
✓ **Use 32-bit** for frequency calculations (avoid overflow)  
✓ **Toggle edge** for pulse width measurement  
✓ **Enable noise canceler** for noisy signals/long cables  
✓ **Choose prescaler** based on expected frequency range  
✓ **Handle overflow** for very slow signals (< 4 Hz @ /64)

### Next Steps

- Combine with **USART** for data logging
- Use **multiple captures** for averaging (reduce jitter)
- Implement **timeout detection** (detect signal loss)
- Build **complete measurement system** (frequency + duty cycle)
- Explore **Timer3 ICP3** for additional channel

---

## References and Resources

### Documentation

- [ATmega128 Datasheet - Section 16.8: Input Capture Unit](https://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf)
- Input Capture pages: 134-136
- Noise canceler: Page 135

### Related Projects

- `Timer_Programming` - Basic timer examples
- `Timer1_CTC_Precision` - CTC mode
- `Timer_Stopwatch` - Timer applications

### Application Notes

- AVR Frequency meter design
- Ultrasonic sensor interfacing
- IR remote decoding techniques
