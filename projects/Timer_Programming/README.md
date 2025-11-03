# Timer Programming - Educational Demonstration

## Project Overview

This educational project demonstrates comprehensive timer/counter programming for the ATmega128 microcontroller. Students learn about timer control registers, various timer modes, prescaler settings, and interrupt-driven programming.

## Educational Objectives

1. **Master timer/counter configuration** - Understanding TCCR, TCNT, OCR, and TIMSK registers
2. **Learn prescaler settings** - Control timing scales from microseconds to seconds
3. **Understand timer modes** - Normal, CTC, Fast PWM, Phase Correct PWM
4. **Practice interrupt-driven programming** - Real-time task scheduling
5. **Generate precise timing** - Delays, frequencies, and PWM signals
6. **Compare timer types** - 8-bit vs 16-bit timers, features comparison

## Hardware Requirements

- **ATmega128 microcontroller** @ 16MHz crystal
- **LEDs on PORTB** for visual timing indication
- **Serial connection** (9600 baud) for timing measurements and status reports
- **Optional**: Oscilloscope on OC0 (PB4), OC1A, OC2 pins for PWM waveform observation

## Demonstration Programs

### Demo 1: Normal Mode - Basic Timer Overflow
**Learning Focus**: Timer normal mode, overflow flags, timing calculations

- **Configuration**: Timer0, Normal mode, Prescaler 256
- **Overflow Period**: ~4.096 ms
- **Operation**: Polls TOV0 flag, toggles LEDs on overflow
- **Key Concepts**: 
  - TCCR0 control register configuration
  - TIFR flag polling
  - Prescaler timing calculations
  
**Timing Calculation**:
```
Overflow frequency = F_CPU / (prescaler * 256)
                   = 16MHz / (256 * 256)
                   = 244.14 Hz
Overflow period = 4.096 ms
```

### Demo 2: CTC Mode - Clear Timer on Compare Match
**Learning Focus**: CTC mode, compare match operations, precise frequency generation

- **Configuration**: Timer0, CTC mode, Prescaler 64, OCR0 = 249
- **Match Frequency**: 1000 Hz (1 ms period)
- **Operation**: Timer clears on compare match, sets OCF0 flag
- **Key Concepts**:
  - WGM01:00 = 10 for CTC mode
  - OCR0 as TOP value
  - Precise frequency control
  
**Frequency Calculation**:
```
Compare match frequency = F_CPU / (prescaler * (OCR0 + 1))
                        = 16MHz / (64 * 250)
                        = 1000 Hz
```

### Demo 3: Fast PWM Mode - High-Frequency PWM Generation
**Learning Focus**: Fast PWM, duty cycle control, PWM waveform characteristics

- **Configuration**: Timer0, Fast PWM mode, Prescaler 64
- **PWM Frequency**: ~977 Hz
- **Duty Cycle**: Sweeps from 0% to 100%
- **Output**: OC0 pin (PB4)
- **Key Concepts**:
  - WGM01:00 = 11 for Fast PWM
  - COM01:00 = 10 for non-inverting PWM
  - Duty cycle = (OCR0 / 256) * 100%
  
**PWM Calculations**:
```
PWM frequency = F_CPU / (prescaler * 256)
              = 16MHz / (64 * 256)
              = 976.56 Hz
Duty cycle = (OCR0 / 256) * 100%
```

### Demo 4: Phase Correct PWM - Symmetric PWM Waveforms
**Learning Focus**: Phase correct vs Fast PWM, motor control applications

- **Configuration**: Timer0, Phase Correct PWM, Prescaler 64
- **PWM Frequency**: ~488 Hz (half of Fast PWM)
- **Advantage**: Symmetric waveform, reduced motor noise
- **Key Concepts**:
  - WGM01:00 = 01 for Phase Correct PWM
  - Up/down counting (0→255→0)
  - Better for motor control applications

### Demo 5: Prescaler Comparison - Different Timing Scales
**Learning Focus**: Prescaler selection, timing trade-offs, optimal prescaler choice

Demonstrates all five prescaler options:
- **Prescaler 1**: 16 μs overflow (very fast)
- **Prescaler 8**: 128 μs overflow
- **Prescaler 64**: 1.024 ms overflow (millisecond timing)
- **Prescaler 256**: 4.096 ms overflow (general timing)
- **Prescaler 1024**: 16.384 ms overflow (slow timing, power saving)

### Demo 6: Interrupt-Based Multitasking
**Learning Focus**: Timer interrupts, real-time scheduling, multi-task programming

- **Configuration**: Timer2 with 1ms interrupt
- **Tasks**:
  - Task 1: 1000ms interval (1 Hz heartbeat)
  - Task 2: 500ms interval (2 Hz status)
  - Task 3: 100ms interval (10 Hz fast blink)
- **Key Concepts**:
  - ISR (TIMER2_OVF_vect)
  - Non-blocking task scheduling
  - System uptime tracking

### Demo 7: Timer1 Advanced - 16-bit Timer Features
**Learning Focus**: 16-bit timer advantages, extended timing ranges

- **Configuration**: Timer1, Normal mode, Prescaler 256
- **Overflow Period**: ~1.048 seconds
- **Range**: 0 to 65535 (vs 0 to 255 for 8-bit)
- **Key Concepts**:
  - TCCR1A and TCCR1B registers
  - 16-bit TCNT1 register
  - Longer overflow periods
  - Input Capture capability

## ATmega128 Timer Overview

### Available Timers

| Timer | Type | Features |
|-------|------|----------|
| Timer0 | 8-bit | Simple PWM, overflow, compare match |
| Timer1 | 16-bit | Advanced PWM, input capture, two compare channels |
| Timer2 | 8-bit | Asynchronous operation (RTC crystal capable) |
| Timer3 | 16-bit | Similar to Timer1, dual output compare |

### Timer Control Registers

#### TCCR0 (Timer/Counter Control Register 0)
```
Bit 7    6     5     4     3     2     1     0
   FOC0  WGM00 COM01 COM00 WGM01 CS02  CS01  CS00
```

- **WGM01:00**: Waveform Generation Mode (00=Normal, 01=PWM, 10=CTC, 11=Fast PWM)
- **COM01:00**: Compare Output Mode (controls OC0 pin behavior)
- **CS02:00**: Clock Select (prescaler: 000=stop, 001=1, 010=8, 011=64, 100=256, 101=1024)

#### TCCR1A/B (Timer1 Control Registers)
- **TCCR1A**: Waveform generation and compare output modes
- **TCCR1B**: Clock select and remaining WGM bits

#### TIMSK (Timer Interrupt Mask Register)
- **TOIE0/1/2**: Timer Overflow Interrupt Enable
- **OCIE0/1/2**: Output Compare Match Interrupt Enable

#### TIFR (Timer Interrupt Flag Register)
- **TOV0/1/2**: Timer Overflow Flag
- **OCF0/1/2**: Output Compare Match Flag

### Prescaler Options

| CS02 CS01 CS00 | Prescaler | Timer Freq @ 16MHz | Overflow Period (8-bit) |
|----------------|-----------|-------------------|------------------------|
| 0    0    0    | Stopped   | -                 | -                      |
| 0    0    1    | 1         | 16 MHz            | 16 μs                  |
| 0    1    0    | 8         | 2 MHz             | 128 μs                 |
| 0    1    1    | 64        | 250 kHz           | 1.024 ms               |
| 1    0    0    | 256       | 62.5 kHz          | 4.096 ms               |
| 1    0    1    | 1024      | 15.625 kHz        | 16.384 ms              |

## Building and Running

### Method 1: Custom Build Script
```batch
cd projects\Timer_Programming
.\build.bat
```

### Method 2: VS Code Task
1. Open `Main.c` in VS Code
2. Press `Ctrl+Shift+B` (Build Current Project)
3. View output in terminal

### Method 3: SimulIDE Simulation
```batch
# Build and simulate
.\build.bat
# Then open Timer_Programming.simu in SimulIDE
```

## Selecting a Demo

Edit `Main.c` and change the `SELECTED_DEMO` define:

```c
#define SELECTED_DEMO 1  // Change this value to select demo (1-7)
```

Then rebuild the project.

## Serial Monitor Output

Connect to the ATmega128's UART1 at 9600 baud to see:
- Program header and menu
- Selected demo configuration
- Real-time timing measurements
- Task execution reports
- Register values and calculations

Example output:
```
=================================================================
  ATmega128 Timer Programming - Educational Demonstration
  SOC 3050 - Embedded Systems and Applications
=================================================================

SELECTED DEMO: 1

Available Demonstrations:
1. Normal Mode - Basic overflow timing with prescalers
2. CTC Mode - Clear Timer on Compare Match
...
```

## Key Learning Points

### Timer Modes Summary

| Mode | WGM01:00 | Behavior | Use Case |
|------|----------|----------|----------|
| Normal | 00 | Count 0→255, overflow | Basic timing, delays |
| CTC | 10 | Count 0→OCR0, clear | Precise frequencies |
| Fast PWM | 11 | Count 0→255, PWM output | LED dimming, DAC |
| Phase Correct PWM | 01 | Count 0→255→0, symmetric PWM | Motor control |

### Timing Formulas

**Overflow Period** (Normal mode):
```
Period = (256 - TCNT_start) * Prescaler / F_CPU
```

**Compare Match Frequency** (CTC mode):
```
Frequency = F_CPU / (Prescaler * (OCR + 1))
```

**PWM Frequency**:
```
Fast PWM: F_PWM = F_CPU / (Prescaler * 256)
Phase Correct PWM: F_PWM = F_CPU / (Prescaler * 510)
```

**PWM Duty Cycle**:
```
Duty% = (OCR / 256) * 100%
```

## Common Applications

1. **LED Blinking** - Basic overflow timing
2. **Tone Generation** - CTC mode for precise frequencies
3. **LED Dimming** - Fast PWM with variable duty cycle
4. **Motor Speed Control** - Phase Correct PWM
5. **Real-Time Clock** - Timer2 with 32.768kHz crystal
6. **Task Scheduling** - Interrupt-based multitasking
7. **Frequency Measurement** - Input Capture mode

## Troubleshooting

### Build Issues
- Ensure portable AVR toolchain is installed
- Check `w:\soc3050code\tools\avr-toolchain\` exists
- Verify shared libraries are present

### Serial Output Not Working
- Check baud rate (9600)
- Verify TX pin connection (PD3 for UART1)
- Ensure Uart1_init() is called

### LEDs Not Blinking
- Verify PORTB connections (active LOW)
- Check selected demo number
- Confirm timer is started (CSx bits set)

## Additional Resources

- **ATmega128 Datasheet**: Chapter 15 (8-bit Timer/Counter0), Chapter 16 (16-bit Timer/Counter1), Chapter 17 (8-bit Timer/Counter2)
- **AVR GCC Documentation**: Timer/Counter programming
- **Course Materials**: SOC 3050 lecture notes on timers
- **Related Projects**: 
  - `PWM_Motor_*` - Motor control with PWM
  - `Serial_Communications` - UART and timer integration
  - `LCD_*` - Display timing considerations

## Author

Professor Hong Jeong  
SOC 3050 - Embedded Systems and Applications  
2025

---

**Note**: This project is designed for educational purposes. All demos include extensive comments explaining register operations, timing calculations, and programming concepts.
