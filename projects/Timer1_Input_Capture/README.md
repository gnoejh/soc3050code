# Timer1 Input Capture - Frequency and Pulse Width Measurement

**Project**: Timer1_Input_Capture  
**Target**: ATmega128 @ 16 MHz  
**Features**: Hardware-based frequency/pulse measurement, zero CPU overhead during capture

---

## Overview

This project demonstrates Timer1's **Input Capture Unit (ICU)** for precise timing measurements of external signals. The hardware automatically captures the timer value when an edge is detected on the ICP1 pin, enabling accurate frequency and pulse width measurements without CPU intervention during the capture event.

### Key Features

- ✅ **Hardware precision** - Automatic timer value capture
- ✅ **Zero CPU overhead** - No polling during capture
- ✅ **Edge selection** - Rising, falling, or both edges
- ✅ **Noise filtering** - Optional 4-clock noise canceler
- ✅ **Wide range** - 0.24 Hz to 8 MHz (depending on prescaler)

---

## SimulIDE Compatibility

| Version | Status | Notes |
|---------|--------|-------|
| **SimulIDE 1.1.0-SR1** | ✅ **Fully Functional** | Timer1 Type 160 works perfectly |
| **SimulIDE 0.4.15** | ✅ **Fully Functional** | All input capture features work |
| **Hardware (ATmega128)** | ✅ **Fully Functional** | Reference implementation |

**Note**: Unlike Timer0 (Type 821), Timer1 Input Capture (Type 160) works flawlessly in all SimulIDE versions.

---

## Hardware Requirements

### Pin Assignment

- **ICP1**: Port D, Pin 4 (PD4) - Input Capture Pin (Connected to Push Button SW4)
- **LED**: Port B, Pin 0 (PB0) - Status indicator (active LOW)

**Testing Note**: Press SW4 (button on PD4) to generate edge events for testing input capture!

### Configuration

```c
// Configure ICP1 as input (PD4 = Push Button SW4)
DDRD &= ~(1 << PD4);   // PD4 as input
PORTD |= (1 << PD4);   // Enable pull-up (required for button)

// Configure LED output
DDRB = 0xFF;           // Port B as output
PORTB = 0xFF;          // LEDs OFF (active LOW)
```

---

## Theory of Operation

### Input Capture Mechanism

When a selected edge (rising or falling) occurs on the ICP1 pin:

1. **Hardware detects edge** on ICP1 (PD4 / SW4)
2. **TCNT1 value automatically copied** to ICR1 register
3. **ICF1 flag set** in TIFR register
4. **Optional interrupt triggered** if TICIE1 enabled

**Key Advantage**: No CPU cycles wasted during capture - hardware handles everything!

### Frequency Measurement Formula

```
F_signal = F_timer / (Capture₂ - Capture₁)

Where:
  F_timer = F_CPU / Prescaler
  
Example @ 16 MHz, Prescaler /64:
  F_timer = 16,000,000 / 64 = 250,000 Hz
  Ticks = 2500
  F_signal = 250,000 / 2500 = 100 Hz
```

### Pulse Width Measurement

Toggle edge detection to measure HIGH pulse width:

1. Capture on **rising edge** (start of pulse)
2. Switch to **falling edge** detection
3. Capture on **falling edge** (end of pulse)
4. Calculate: `Pulse_Width = Capture_Fall - Capture_Rise`
5. Convert to time: `Time_µs = Pulse_Width × (Prescaler / 16)`

---

## Prescaler Selection Guide

### Frequency Measurement Ranges @ 16 MHz

| Prescaler | Timer Freq | Resolution | Min Freq | Max Freq | Best For |
|-----------|------------|------------|----------|----------|----------|
| **/1** | 16.000 MHz | 62.5 ns | 244 Hz | 8.0 MHz | High-speed signals (>10 kHz) |
| **/8** | 2.000 MHz | 500 ns | 31 Hz | 1.0 MHz | Medium signals (1 kHz - 100 kHz) |
| **/64** | 250 kHz | 4.0 µs | 4 Hz | 125 kHz | **General purpose (10 Hz - 10 kHz)** |
| **/256** | 62.5 kHz | 16 µs | 1 Hz | 31.25 kHz | Low frequency (1 Hz - 1 kHz) |
| **/1024** | 15.625 kHz | 64 µs | 0.24 Hz | 7.8 kHz | Very slow signals (<1 Hz) |

**Calculation**:

- Min Freq = Timer_Freq / 65536 (max counter value)
- Max Freq = Timer_Freq / 2 (minimum 2 ticks required)

### Selection Rules

```
Signal > 10 kHz:      Use prescaler /1 or /8
Signal 100 Hz - 10 kHz: Use prescaler /64 (default)
Signal 1 - 100 Hz:    Use prescaler /256
Signal < 1 Hz:        Use prescaler /1024
```

---

## Common Use Cases

### 1. Frequency Counter (100 Hz signal)

**Prescaler**: /64  
**Timer Freq**: 250 kHz  
**Expected Ticks**: 2500

```c
// Example: Measure 100 Hz signal
TCCR1B = (1 << ICES1) | (1 << CS11) | (1 << CS10);  // Rising, /64

ISR(TIMER1_CAPT_vect) {
    static uint16_t last = 0;
    uint16_t current = ICR1;
    uint16_t ticks = current - last;
    last = current;
    
    // F = 250000 / ticks
    uint32_t freq = 250000UL / ticks;  // = 100 Hz
}
```

### 2. Servo PWM Decoder (1-2 ms pulses, 50 Hz)

**Prescaler**: /64  
**Timer Freq**: 250 kHz  
**Pulse Width**: 250-500 ticks (1-2 ms)

```c
// Measure servo pulse width
ISR(TIMER1_CAPT_vect) {
    static uint16_t rise_time = 0;
    static uint8_t edge = 1;
    
    if (edge) {
        rise_time = ICR1;
        TCCR1B &= ~(1 << ICES1);  // Switch to falling
        edge = 0;
    } else {
        uint16_t pulse = ICR1 - rise_time;
        // pulse = 250-500 ticks = 1-2 ms
        uint16_t pulse_us = pulse * 4;  // 4 µs per tick
        TCCR1B |= (1 << ICES1);   // Switch to rising
        edge = 1;
    }
}
```

### 3. Ultrasonic Distance Sensor (Echo Timing)

**Prescaler**: /8  
**Timer Freq**: 2 MHz (0.5 µs per tick)  
**Echo Time**: 150 µs - 25 ms (300-50000 ticks)

```c
// Measure echo pulse width
uint16_t distance_cm = 0;

ISR(TIMER1_CAPT_vect) {
    static uint16_t start = 0;
    static uint8_t state = 0;
    
    if (state == 0) {  // Rising - echo start
        start = ICR1;
        TCCR1B &= ~(1 << ICES1);
        state = 1;
    } else {  // Falling - echo end
        uint16_t pulse = ICR1 - start;
        // Distance = (pulse × 0.5 µs × 343 m/s) / 2
        distance_cm = (pulse / 58);  // Simplified: pulse/58 ≈ cm
        TCCR1B |= (1 << ICES1);
        state = 0;
    }
}
```

### 4. RPM Measurement (Tachometer)

**Prescaler**: /256  
**Timer Freq**: 62.5 kHz  
**1 pulse per revolution**

```c
// Calculate RPM from pulse period
ISR(TIMER1_CAPT_vect) {
    static uint16_t last = 0;
    uint16_t current = ICR1;
    uint16_t ticks = current - last;
    last = current;
    
    // RPM = (60 × Timer_Freq) / Ticks
    uint32_t rpm = (60UL * 62500) / ticks;
}
```

### 5. IR Remote Decoder (38 kHz carrier)

**Prescaler**: /8  
**Timer Freq**: 2 MHz  
**NEC Protocol**: 560 µs, 1.69 ms pulses

```c
// Decode NEC protocol IR pulses
ISR(TIMER1_CAPT_vect) {
    static uint16_t last = 0;
    uint16_t current = ICR1;
    uint16_t pulse = current - last;
    last = current;
    
    // Each tick = 0.5 µs
    if (pulse > 1100 && pulse < 1200)      // ~560 µs = '0'
        ir_bit = 0;
    else if (pulse > 3300 && pulse < 3500) // ~1.69 ms = '1'
        ir_bit = 1;
    
    TCCR1B ^= (1 << ICES1);  // Toggle edge
}
```

---

## Demos Included

### Demo 1: Polling Method

**Method**: Check ICF1 flag manually  
**Use Case**: Simple applications, blocking operation

```c
while (1) {
    if (TIFR & (1 << ICF1)) {
        uint16_t capture = ICR1;
        TIFR = (1 << ICF1);  // Clear flag
        // Process capture
    }
}
```

### Demo 2: Interrupt Method

**Method**: ISR handles captures automatically  
**Use Case**: Multitasking, non-blocking operation

```c
ISR(TIMER1_CAPT_vect) {
    capture_value = ICR1;
    // Automatic flag clearing
}
```

### Demo 3: Frequency Meter

**Method**: Calculate input signal frequency  
**Display**: LED pattern indicates frequency range

### Demo 4: Pulse Width Measurement

**Method**: Toggle edge detection  
**Display**: Pulse width shown on LEDs

---

## Register Configuration

### Edge Selection

```c
// Capture on RISING edge (LOW → HIGH)
TCCR1B |= (1 << ICES1);

// Capture on FALLING edge (HIGH → LOW)
TCCR1B &= ~(1 << ICES1);

// Toggle edge (for both edges)
TCCR1B ^= (1 << ICES1);
```

### Noise Canceler

```c
// Enable 4-clock noise filter (reduces glitches)
TCCR1B |= (1 << ICNC1);

// Disable (faster response, sensitive to noise)
TCCR1B &= ~(1 << ICNC1);
```

**Trade-offs**:

- **ICNC1 = 1**: Filters noise, 4-clock delay, use with long cables/motors
- **ICNC1 = 0**: Immediate response, sensitive to glitches, use with clean signals

### Interrupt Configuration

```c
// Enable input capture interrupt
TIMSK |= (1 << TICIE1);
sei();  // Global interrupt enable

// ISR automatically triggered on capture
ISR(TIMER1_CAPT_vect) {
    uint16_t captured_value = ICR1;
    // Flag cleared automatically
}
```

---

## Common Pitfalls and Solutions

### ❌ Pitfall 1: Division by Zero

```c
// WRONG - may crash if ticks = 0
frequency = TIMER_FREQ / ticks;

// CORRECT - check first
if (ticks > 0) {
    frequency = TIMER_FREQ / ticks;
}
```

### ❌ Pitfall 2: Integer Overflow

```c
// WRONG - overflow if frequency > 65535
uint16_t freq = TIMER_FREQ / ticks;

// CORRECT - use 32-bit
uint32_t freq = (uint32_t)TIMER_FREQ / ticks;
```

### ❌ Pitfall 3: Forgetting Edge Toggle

```c
// WRONG - stuck on one edge for pulse measurement
ISR(TIMER1_CAPT_vect) {
    // Measure pulse, but never toggle edge!
}

// CORRECT - alternate edges
ISR(TIMER1_CAPT_vect) {
    if (measuring_high_time) {
        // ... measure ...
        TCCR1B ^= (1 << ICES1);  // Toggle edge
    }
}
```

### ❌ Pitfall 4: Timer Overflow (Long Periods)

```c
// WRONG - doesn't handle overflow
ticks = current - previous;

// CORRECT - handle overflow
volatile uint8_t overflow_count = 0;

ISR(TIMER1_OVF_vect) {
    overflow_count++;
}

ISR(TIMER1_CAPT_vect) {
    uint32_t total_ticks = ((uint32_t)overflow_count << 16) | ICR1;
    // Calculate with total_ticks
    overflow_count = 0;
}
```

---

## Comparison: Timer1 vs Timer0 Input Capture

| Feature | Timer1 | Timer0 |
|---------|--------|--------|
| **Bit Width** | 16-bit (0-65535) | 8-bit (0-255) |
| **Resolution** | High | Low |
| **Input Capture** | ✅ Yes (ICP1/PD4) | ❌ No |
| **Max Period** | 65535 ticks | 255 ticks |
| **Frequency Range** | 0.24 Hz - 8 MHz | Limited |
| **SimulIDE 1.1.0-SR1** | ✅ Works | ⚠️ CTC broken |
| **Best For** | Frequency/pulse measurement | Simple delays |

**Recommendation**: Always use **Timer1** for input capture applications.

---

## Building and Running

### Build for Hardware

```powershell
# Build project
.\build.bat

# Program to ATmega128
avrdude -c arduino -p m128 -P COM3 -U flash:w:Main.hex:i
```

### Build for SimulIDE

```powershell
# Build and load to simulator
.\build.bat simulide
```

### Run Tests

1. Press **Push Button SW4 (PD4/ICP1)** to generate edge events
2. Observe **LED on PB0** for status
3. Monitor **frequency/pulse width** output

**Testing Tip**: Each button press generates rising/falling edges captured by ICP1!

---

## Learning Outcomes

After completing this project, you will understand:

- ✅ How Input Capture Unit works (hardware timing)
- ✅ Frequency measurement techniques (period method)
- ✅ Pulse width measurement (edge toggling)
- ✅ Prescaler selection for different frequency ranges
- ✅ Noise filtering and edge selection
- ✅ Handling timer overflow for long periods
- ✅ Polling vs interrupt-driven measurement

---

## References

- **ATmega128 Datasheet**: [Section 16.8 - Input Capture Unit](https://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf) (Pages 134-136)
- **Related Projects**:
  - `Timer1_CTC_Precision` - CTC mode frequency generation
  - `Timer_Programming` - Basic timer concepts
  - `INT_External_Pins` - External interrupt comparison

---

## Troubleshooting

### Issue: No captures detected

**Solution**:

- Check ICP1 pin configuration (DDRD, PORTD)
- Verify button SW4 connected to PD4
- Check edge selection (ICES1 bit)
- Press button SW4 to generate test edges

### Issue: Incorrect frequency readings

**Solution**:

- Verify F_CPU = 16000000UL
- Check prescaler setting matches calculation
- Use 32-bit variables for frequency
- Check for timer overflow on slow signals

### Issue: Glitches causing false triggers

**Solution**:

- Enable noise canceler: `TCCR1B |= (1 << ICNC1);`
- Add external hardware filter (capacitor)
- Use shielded cable for long connections

### Issue: Pulse width always zero

**Solution**:

- Verify edge toggling in ISR
- Check that both edges are being captured
- Ensure pulse width longer than 2 timer ticks

---

**Last Updated**: December 1, 2025  
**Author**: SOC 3050 Embedded Systems Course  
**License**: Educational Use
