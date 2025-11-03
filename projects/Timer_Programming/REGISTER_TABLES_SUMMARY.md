# Timer Programming - Register Tables Summary

## Build Status: ✅ SUCCESS

**Build Output:**
- Text: 6224 bytes
- Data: 2678 bytes  
- BSS: 40 bytes
- Total: 8942 bytes

## File Created

A completely clean `Main.c` file with comprehensive timer programming demonstrations and detailed control register documentation.

## Key Features

### 1. **Detailed Control Register Documentation**
All major ATmega128 timer registers documented in compiler-safe format:

- **TCCR0** - Timer/Counter Control Register 0 (8-bit)
  - Bit layout with WGM, COM, CS fields
  - Prescaler values (1, 8, 64, 256, 1024)
  - Waveform modes (Normal, CTC, Fast PWM, Phase Correct PWM)

- **TCNT0** - Timer/Counter Register 0
  - Current counter value (0-255)

- **OCR0** - Output Compare Register 0
  - Compare/TOP value for CTC and PWM modes

- **TIFR** - Timer Interrupt Flag Register
  - TOV0, OCF0, TOV1, OCF1A, OCF1B, etc.
  - Flag polling and clearing instructions

- **TIMSK** - Timer Interrupt Mask Register
  - TOIE0, OCIE0, TOIE1, OCIE1A, TOIE2, OCIE2
  - Interrupt enable documentation

- **TCCR1A/B** - Timer1 Control Registers (16-bit)
  - 4-bit WGM field for advanced modes
  - Compare output modes
  - Input capture features

- **TCNT1** - 16-bit Timer/Counter 1
  - Range: 0-65535

### 2. **Seven Complete Demo Programs**

All demos include **INTERRUPT-DRIVEN** versions:

#### Demo 1: Normal Mode - Overflow Polling
- Basic overflow timing with flag polling
- TOV0 flag demonstration
- LED toggling based on overflows

#### Demo 2: CTC Mode - Clear Timer on Compare
- OCR0 register usage
- Precise frequency generation (1000 Hz)
- OCF0 flag polling

#### Demo 3: Fast PWM Mode
- PWM generation at 976.5 Hz
- Dynamic duty cycle sweeping (0-100%)
- OC0 pin output demonstration

#### Demo 4: Phase Correct PWM
- Symmetric PWM waveforms
- 490 Hz frequency (half of Fast PWM)
- Motor control applications

#### Demo 5: Prescaler Comparison
- All 5 prescaler values demonstrated
- Real-time prescaler switching
- Timing calculation verification

#### Demo 6: **Interrupt Multi-tasking - Timer2 ISR** ⭐
- **ISR(TIMER2_OVF_vect)** - 1ms periodic interrupt
- Multi-rate task scheduling
- Volatile variable usage
- Three concurrent tasks (1ms, 100ms, 500ms rates)
- **sei()** global interrupt enable

#### Demo 7: **Timer1 Advanced - 16-bit with Interrupts** ⭐
- **ISR(TIMER1_COMPA_vect)** - Compare match interrupt
- 16-bit CTC mode
- Precise 1Hz frequency generation
- OCR1A compare value (31250 counts)

### 3. **Educational Features**

- **Compiler-Safe Comments**: All register documentation uses simple text formatting (no Unicode, no pipe characters, no problematic syntax)
- **Prescaler Timing Table**: Quick reference for all timing calculations at 16MHz
- **ISR Examples**: Real interrupt service routines for Timer1 and Timer2
- **Volatile Variables**: Proper ISR communication patterns
- **Detailed Inline Comments**: Every register write explained with bit patterns

### 4. **Interrupt Methods Included**

All three major interrupt methods are demonstrated:

1. **Polling Method** (Demo 1-5)
   - Manual flag checking
   - Software-controlled timing
   - Blocking operations

2. **Timer Overflow Interrupt** (Demo 6)
   - `ISR(TIMER2_OVF_vect)`
   - Periodic execution every ~1ms
   - Task scheduling based on tick counter

3. **Compare Match Interrupt** (Demo 7)
   - `ISR(TIMER1_COMPA_vect)`  
   - Triggered when TCNT1 equals OCR1A
   - Precise frequency control

## Register Documentation Format

The register tables use a simple, compiler-safe format:

```
Bit:   7      6      5      4      3      2      1      0
Name: FOC0  WGM00  COM01  COM00  WGM01   CS02   CS01   CS00
```

No Unicode characters, no pipe symbols, no problematic C syntax that confuses avr-gcc.

## Usage

Change the `demo_choice` variable in `main()` to select which demo to run (1-7).

Each demo is self-contained and demonstrates specific timer features with detailed serial output.

## Compilation

The project successfully compiles with:
- AVR-GCC 15.1.0
- Target: ATmega128 @ 16MHz
- Optimization: -O2
- Dependencies: _uart.c, _timer2.c only

All interrupt service routines compile correctly and use proper ISR syntax for ATmega128.
