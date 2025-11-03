# ATmega128 Peripherals — Assembly vs C (Hands‑On Guide)

This guide teaches the ATmega128 on‑chip peripherals through practical, side‑by‑side Assembly and C examples. For instruction set basics (LDI, IN/OUT, SBI/CBI, branches, etc.), first read `Instructions.md`. Here we focus on the peripherals: what they do, the key registers, and working init/use patterns in both Assembly and C.

Target: ATmega128 @ 16 MHz, toolchain: avr-gcc/avr-libc

---

## Table of contents

1. Digital I/O (GPIO)
2. External Interrupts (INT0–INT7)
3. Timers/Counters overview (0, 1, 2, 3)
	 - 3.1 Timer0 (8‑bit) — CTC example
	 - 3.2 Timer1 (16‑bit) — Input Capture example
	 - 3.3 Timer2 (8‑bit, async) — RTC tick
	 - 3.4 Timer3 (16‑bit) — Fast PWM (motor/servo)
4. ADC (10‑bit) — single + interrupt modes
5. USART0/USART1 — 8N1 @ 9600 bps
6. SPI — Master transmit
7. TWI (I²C) — master read/write
8. EEPROM — read/write safely
9. Analog Comparator — polling + interrupt
10. Watchdog Timer — enable/disable safely
11. JTAG and PORTF pins — disabling JTAG
12. Sleep modes — idle and power‑save
13. Quick reference: registers and bits
14. Common pitfalls and debug tips

When “C (library)” appears, it refers to helpers in `shared_libs/` (e.g., `_port.h`, `_uart.h`, `_adc.h`, `_timer2.h`, `_interrupt.h`).

---

## 1) Digital I/O (GPIO)

Pins are grouped by ports (A–G). For each port x: `DDRx` sets direction (1=out), `PORTx` writes output or enables pull‑ups for inputs, `PINx` reads input.

Key registers: DDRB, PORTB, PINB (and similarly for A, C, D, E, F, G)

- Assembly patterns: `SBI/CBI` (set/clear bit), `IN/OUT` for registers
- C register-level: direct write to DDRx/PORTx/PINx
- C (library): see `_port.h` (e.g., `led_on`, `led_off`, patterns)

Example: PB0 LED toggle

Assembly:

```assembly
; Configure PB0 as output and toggle it
LDI  r16, (1<<0)
OUT  DDRB, r16            ; PB0 output

loop:
	SBI  PORTB, 0           ; LED off/on depending on board (active level)
	RCALL delay
	CBI  PORTB, 0
	RCALL delay
	RJMP loop
```

C (register-level):

```c
DDRB |= (1 << 0);
while (1) {
	PORTB |= (1 << 0);
	_delay_ms(500);
	PORTB &= ~(1 << 0);
	_delay_ms(500);
}
```

C (library):

```c
#include "shared_libs/_port.h"
Port_init();
while (1) { led_toggle(0); _delay_ms(250); }
```

---

## 2) External Interrupts (INT0–INT7)

ATmega128 provides eight external interrupts. Configure sense control, enable mask, clear flags, then enable global interrupts.

Key registers:

- EICRA: ISC01/ISC00 (INT0), ISC11/ISC10 (INT1), ISC21/ISC20 (INT2), ISC31/ISC30 (INT3)
- EICRB: ISC41/ISC40 (INT4) … ISC71/ISC70 (INT7)
- EIMSK: interrupt mask bits INT0..INT7
- EIFR: interrupt flags INTF0..INTF7 (write 1 to clear)
- SREG: I bit (global enable via `SEI`)

Sense control codes: 00=low-level, 01=any change, 10=falling edge, 11=rising edge

Example: INT0 on falling edge

Assembly:

```assembly
; Configure INT0 falling edge and enable
IN   r16, EICRA
ORI  r16, (1<<ISC01)         ; ISC01=1, ISC00=0 → falling edge
ANDI r16, ~(1<<ISC00)
OUT  EICRA, r16

SBI  EIMSK, INT0             ; Enable INT0
SBI  SREG, 7                 ; SEI (or use instruction SEI)
```

C (register-level):

```c
EICRA = (EICRA & ~((1<<ISC01)|(1<<ISC00))) | (1<<ISC01);
EIMSK |= (1 << INT0);
sei();

ISR(INT0_vect) { /* handle event */ }
```

C (library):

```c
#include "shared_libs/_interrupt.h"
Interrupt_init();
Interrupt_enable_global();
```

Bitfield reference (External Interrupts):

REGISTER: EICRA (External Interrupt Control Register A)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │ISC31│ISC30│ISC21│ISC20│ISC11│ISC10│ISC01│ISC00│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

REGISTER: EICRB (External Interrupt Control Register B)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │ISC71│ISC70│ISC61│ISC60│ISC51│ISC50│ISC41│ISC40│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

REGISTER: EIMSK (External Interrupt Mask Register)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │INT7│INT6 │INT5 │INT4 │INT3 │INT2 │INT1 │INT0 │
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

REGISTER: EIFR (External Interrupt Flag Register)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │INTF7│INTF6│INTF5│INTF4│INTF3│INTF2│INTF1│INTF0│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

---

## 3) Timers/Counters overview

- Timer0: 8‑bit, PWM, CTC, Overflow
- Timer1: 16‑bit, Input Capture (ICP1), 2×OC channels (A/B)
- Timer2: 8‑bit, optional asynchronous clock (32.768 kHz) for RTC
- Timer3: 16‑bit (like Timer1) with channels A/B/C

Interrupt mask/flags:

- TIMSK/TIFR for Timer0/1/2; ETIMSK/ETIFR for extended Timer3

Prescalers: CS bits in TCCRnB/TCCRn for each timer

### 3.1 Timer0 (8‑bit) — CTC 1 kHz example

Key registers: TCCR0, TCNT0, OCR0, TIMSK (OCIE0, TOIE0), TIFR

Goal: Toggle PB0 at 1 kHz interrupt (0.5 kHz square on PB0 inside ISR)

Assembly:

```assembly
; CTC: Clear on Compare Match, OCR0 sets period
LDI  r16, (1<<WGM01)            ; CTC mode (WGM01=1)
OUT  TCCR0, r16
LDI  r16, 249                   ; at 16 MHz, prescale 64: 16e6/64=250k → 1kHz when OCR0=249
OUT  OCR0, r16
IN   r17, TIMSK
ORI  r17, (1<<OCIE0)            ; enable compare match interrupt
OUT  TIMSK, r17
LDI  r16, (1<<CS01)|(1<<CS00)   ; prescaler 64
IN   r17, TCCR0
OR   r17, r16
OUT  TCCR0, r17
SEI
; ISR at vector TIMER0_COMP_vect must toggle PORTB0, then RETI
```

C (register-level):

```c
TCCR0 = (1<<WGM01);               // CTC
OCR0  = 249;                      // 1 kHz
TIMSK |= (1<<OCIE0);              // enable compare interrupt
TCCR0 |= (1<<CS01)|(1<<CS00);     // prescaler 64
sei();
ISR(TIMER0_COMP_vect) { PORTB ^= (1<<0); }
```

Bitfield reference (Timer0):

REGISTER: TCCR0 (Timer/Counter0 Control Register)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │FOC0│WGM00│COM01│COM00│WGM01│CS02│CS01│CS00│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

REGISTER: TIMSK (Timer/Counter Interrupt Mask Register) [Timer0 bits]
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────
│ Bit │ ... │ ... │ ... │ ... │ ... │  1  │  0  
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────
│Name │ ... │ ... │ ... │ ... │ ... │OCIE0│TOIE0
```

### 3.2 Timer1 (16‑bit) — Input Capture (frequency measurement)

Key registers: TCCR1A, TCCR1B, TCNT1, ICR1, TIMSK (TICIE1), TIFR (ICF1)

Setup: capture rising edges on ICP1 (PD4). Measure period between captures.

C (register-level) sketch:

```c
volatile uint16_t last_icr, period;

void timer1_input_capture_init(void) {
	TCCR1A = 0;
	TCCR1B = (1<<ICES1) | (1<<CS11); // capture rising edge, prescaler 8
	TIMSK |= (1<<TICIE1);            // enable input capture interrupt
}

ISR(TIMER1_CAPT_vect) {
	uint16_t now = ICR1;
	period = now - last_icr;
	last_icr = now;
}
```

Bitfield reference (Timer1):

REGISTER: TCCR1A (Timer/Counter1 Control Register A)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │COM1A1│COM1A0│COM1B1│COM1B0│ —  │ —  │WGM11│WGM10│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

REGISTER: TCCR1B (Timer/Counter1 Control Register B)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │ICNC1│ICES1│ —  │WGM13│WGM12│CS12│CS11│CS10│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

### 3.3 Timer2 (8‑bit, asynchronous) — RTC 1 Hz tick

Key registers: TCCR2, TCNT2, OCR2, ASSR (AS2), TIMSK (TOIE2/OCIE2)

Idea: Clock Timer2 from 32.768 kHz crystal (AS2=1), set OCR2 for 1 Hz.

C (register-level) sketch:

```c
ASSR |= (1<<AS2);                 // async clock
TCCR2 = (1<<WGM21) | (1<<CS22) | (1<<CS21) | (1<<CS20); // CTC, prescale 1024
OCR2  = 31;                       // 32768/1024=32 Hz, OCR2=31 → 1 Hz
while (ASSR & ((1<<TCN2UB)|(1<<OCR2UB)|(1<<TCR2UB))) { /* wait update */ }
TIMSK |= (1<<OCIE2);
```

Bitfield reference (Timer2/ASSR):

REGISTER: TCCR2 (Timer/Counter2 Control Register)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │FOC2│WGM20│COM21│COM20│WGM21│CS22│CS21│CS20│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

REGISTER: ASSR (Asynchronous Status Register)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │ —  │ —  │ —  │ —  │ AS2│ TCN2UB│ OCR2UB│ TCR2UB│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

### 3.4 Timer3 (16‑bit) — Fast PWM on OC3A

Key registers: TCCR3A, TCCR3B, OCR3A, ICR3, ETIMSK (OCIE3A), OC3A pin (PE3)

Example: 20 ms period (50 Hz) servo PWM on OC3A with variable duty.

C (register-level) sketch:

```c
DDRE |= (1<<3);                   // PE3 (OC3A) output
TCCR3A = (1<<COM3A1) | (1<<WGM31);
TCCR3B = (1<<WGM33)|(1<<WGM32) | (1<<CS31); // Fast PWM, ICR3 top, prescale 8
ICR3   = 40000 - 1;               // 16MHz/8=2MHz, 20ms → 40000 counts
OCR3A  = 3000;                    // ~1.5ms center (variable 2000..4000)
```

Bitfield reference (Timer3):

REGISTER: TCCR3A (Timer/Counter3 Control Register A)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │COM3A1│COM3A0│COM3B1│COM3B0│COM3C1│COM3C0│WGM31│WGM30│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

REGISTER: TCCR3B (Timer/Counter3 Control Register B)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │ICNC3│ICES3│ —  │WGM33│WGM32│CS32│CS31│CS30│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

Library alternative: see `_timer2.h` for 1ms system tick and scheduling patterns.

---

## 4) ADC (10‑bit)

8 channels (ADC0–ADC7), 10‑bit result in ADCL/ADCH. Reference via ADMUX (REFS1:0). Enable via ADCSRA.AD EN, start conversion ADSC, prescaler ADPS[2:0]. Interrupt optional with ADIE.

Key registers: ADMUX, ADCSRA, ADCL/ADCH

Single conversion: channel 0, AVCC ref, right‑adjust

Assembly:

```assembly
LDI  r16, (1<<REFS0)             ; AVCC ref, MUX=0
OUT  ADMUX, r16
LDI  r16, (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) ; enable, prescale 128
OUT  ADCSRA, r16
SBI  ADCSRA, ADSC                ; start
wait:
	SBIS ADCSRA, ADIF
	RJMP wait
SBI  ADCSRA, ADIF                ; clear flag by writing 1
IN   r18, ADCL                   ; read low first
IN   r19, ADCH                   ; then high
```

C (register-level):

```c
ADMUX  = (1<<REFS0) | 0;               // AVCC, ADC0
ADCSRA = (1<<ADEN) | (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
ADCSRA |= (1<<ADSC);
while (!(ADCSRA & (1<<ADIF))) {}
ADCSRA |= (1<<ADIF);                   // clear
uint16_t val = ADC;                    // combines ADCL/ADCH
```

C (library):

```c
#include "shared_libs/_adc.h"
Adc_init();
uint16_t val = Read_Adc_Data(ADC_CHANNEL_0);
```

Interrupt mode (sketch): set `ADIE`, handle `ISR(ADC_vect)`.

Bitfield reference (ADC):

REGISTER: ADMUX (ADC Multiplexer Selection Register)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │REFS1│REFS0│ADLAR│ MUX4│ MUX3│ MUX2│ MUX1│ MUX0│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

REGISTER: ADCSRA (ADC Control and Status Register A)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │ADEN│ADSC│ADATE│ADIF│ADIE│ADPS2│ADPS1│ADPS0│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

---

## 5) USART0/USART1 (Serial)

Two USARTs. Configure baud via UBRRnH/L, frame via UCSRnC, enable via UCSRnB. Use UDREn/RXCn flags in UCSRnA for polling; or enable RXCIE/TXCIE for interrupts.

Key registers: UBRR0H/L, UCSR0A/B/C, UDR0 and likewise for USART1

Init 9600 8N1 on USART1 (common on our boards):

C (register-level):

```c
UBRR1H = 0; UBRR1L = 103;             // 16MHz → 9600
UCSR1B = (1<<RXEN1)|(1<<TXEN1);       // enable RX/TX
UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);     // 8 data, 1 stop, no parity

void usart1_putc(char c){ while(!(UCSR1A&(1<<UDRE1))); UDR1=c; }
char usart1_getc(void){ while(!(UCSR1A&(1<<RXC1))); return UDR1; }
```

C (library):

```c
#include "shared_libs/_uart.h"
Uart1_init();
puts_USART1("Hello\r\n");
```

Interrupt RX example: enable `(1<<RXCIE1)` and define `ISR(USART1_RX_vect)`.

---

### USART Bitfield Reference (ATmega128)

REGISTER: UCSR1A (USART1 Control and Status Register A)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │RXC1 │TXC1 │UDRE1│ FE1 │DOR1 │UPE1 │U2X1 │MPCM1│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```
Legend: RXC1=Receive Complete, TXC1=Transmit Complete, UDRE1=Data Reg Empty, FE1=Frame Error, DOR1=Data Overrun, UPE1=Parity Error, U2X1=Double Speed, MPCM1=Multi-processor Mode

REGISTER: UCSR1B (USART1 Control and Status Register B)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │RXCIE1│TXCIE1│UDRIE1│RXEN1│TXEN1│UCSZ12│RXB81│TXB81│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```
Legend: RXCIE1=RX Complete Int En, TXCIE1=TX Complete Int En, UDRIE1=Data Reg Empty Int En, RXEN1=Receiver En, TXEN1=Transmitter En, UCSZ12=Char Size bit 2, RXB81/ TXB81=9th Data Bit

REGISTER: UCSR1C (USART1 Control and Status Register C)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │UMSEL│  UPM1 │ UPM0 │ —  │ USBS│ UCSZ1│ UCSZ0│ UCPOL│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```
Legend: UMSEL=Mode (0 async/1 sync), UPM1:0=Parity, USBS=Stop Bits, UCSZ1:0=Char Size bits, UCPOL=Clock Polarity (sync)

Note: On ATmega128, UBRR1H/L hold the baud setting and UDR1 is the data register for USART1.

REGISTER: UCSR0A (USART0 Control and Status Register A)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │RXC0 │TXC0 │UDRE0│ FE0 │DOR0 │UPE0 │U2X0 │MPCM0│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

REGISTER: UCSR0B (USART0 Control and Status Register B)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │RXCIE0│TXCIE0│UDRIE0│RXEN0│TXEN0│UCSZ02│RXB80│TXB80│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

REGISTER: UCSR0C (USART0 Control and Status Register C)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │UMSEL│  UPM1 │ UPM0 │ —  │ USBS│ UCSZ1│ UCSZ0│ UCPOL│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

Tip: Character size 8-bit is UCSZ1:0=11 with UCSZ12=0; 9-bit uses UCSZ12=1 and RXB8/TXB8.


## 6) SPI — Master mode

Pins: MOSI (PB2), MISO (PB3), SCK (PB1), SS (PB0). Set as output for MOSI/SCK/SS when master. Configure SPCR. SCK = F_CPU / (2, 4, 8, …)

Key registers: SPCR, SPSR, SPDR

C (register-level):

```c
DDRB |= (1<<PB0)|(1<<PB1)|(1<<PB2); // SS,SCK,MOSI outputs; MISO input
SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0); // enable, master, fosc/16
SPSR = 0;                             // no double speed

uint8_t spi_txrx(uint8_t v){ SPDR=v; while(!(SPSR&(1<<SPIF))); return SPDR; }
```

Assembly sketch:

```assembly
; SPDR load then wait SPIF
OUT  SPDR, r16
wait_spif:
	SBIS SPSR, SPIF
	RJMP wait_spif
IN   r16, SPDR
```

Bitfield reference (SPI):

REGISTER: SPCR (SPI Control Register)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │SPIE│SPE │DORD│MSTR│CPOL│CPHA│SPR1│SPR0│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

REGISTER: SPSR (SPI Status Register)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │SPIF│WCOL│ —  │ —  │ —  │ —  │ —  │SPI2X│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

---

## 7) TWI (I²C) — Master transactions

Key registers: TWBR (bit rate), TWSR (status+prescaler), TWCR (control), TWDR (data), TWAR (address)

Basic master write sequence (pseudocode C):

```c
// Set SCL ~100kHz: TWBR = ((F_CPU/SCL)-16)/2, prescaler=1
TWBR = 72; TWSR &= ~((1<<TWPS1)|(1<<TWPS0));

// START
TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
while(!(TWCR & (1<<TWINT)));

// SLA+W
TWDR = (dev_addr<<1) | 0;
TWCR = (1<<TWINT)|(1<<TWEN);
while(!(TWCR & (1<<TWINT)));

// DATA
TWDR = data;
TWCR = (1<<TWINT)|(1<<TWEN);
while(!(TWCR & (1<<TWINT)));

// STOP
TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
```

Bitfield reference (TWI):

REGISTER: TWCR (TWI Control Register)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │TWINT│TWEA│TWSTA│TWSTO│TWWC│TWEN│ —  │TWIE│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

REGISTER: TWSR (TWI Status Register)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │TWS7│TWS6│TWS5│TWS4│TWS3│ —  │TWPS1│TWPS0│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

For reads, after SLA+R set ACK/NACK via TWEA in TWCR before clearing TWINT.

---

## 8) EEPROM — safe read/write

Key registers: EEARH/L (address), EEDR (data), EECR (EERE, EEWE, EEMWE)

Write sequence (must set EEMWE, then EEWE within 4 cycles; wait until EEWE clears):

Assembly sketch:

```assembly
; Wait until not writing
wait_ee:
	SBIC EECR, EEWE
	RJMP wait_ee
OUT  EEARH, r17
OUT  EEARL, r16
OUT  EEDR,  r18
SBI  EECR, EEMWE
SBI  EECR, EEWE
```

C (register-level):

```c
while (EECR & (1<<EEWE)) {}
EEAR = address; EEDR = value;
EECR |= (1<<EEMWE); EECR |= (1<<EEWE);
```

avr-libc alternative: `#include <avr/eeprom.h>` then `eeprom_write_byte()`/`eeprom_read_byte()`.

Bitfield reference (EEPROM):

REGISTER: EECR (EEPROM Control Register)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │ —  │ —  │ —  │ —  │EERIE│EEMWE│EEWE│EERE│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

---

## 9) Analog Comparator

Compares AIN0 vs AIN1 (or AIN1 via ADC mux when ACME=1 and ADC disabled).

Key registers: ACSR (ACIE, ACI, ACIS1:0, ACD), SFIOR (ACME)

Polling:

```c
ACSR = 0;                    // default, comparator on
if (ACSR & (1<<ACO)) { /* AIN0 > AIN1 */ }
```

Interrupt on toggle:

```c
ACSR = (1<<ACIE);                // ACIS1:0 = 00 → interrupt on toggle
sei();
ISR(ANALOG_COMP_vect){ /* handle edge */ }
```

Bitfield reference (Comparator):

REGISTER: ACSR (Analog Comparator Control and Status Register)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │ACD │ACBG│ ACO│ ACI│ACIE│ACIC│ACIS1│ACIS0│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

REGISTER: SFIOR (Special Function IO Register) [ACME bit]
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │ —  │ —  │ —  │ —  │ACME│ —  │ —  │ —  │
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

---

## 10) Watchdog Timer

On ATmega128 the WDT control register is `WDTCR` (older naming). To change WDE, write the timed sequence using WDTOE.

Enable ~1s timeout (register-level):

```c
// Change enable with timed sequence
WDTCR = (1<<WDTOE)|(1<<WDE);
WDTCR = (1<<WDE) | (1<<WDP2) | (1<<WDP1); // ~1s @ 16MHz
```

Disable:

```c
WDTCR = (1<<WDTOE)|(1<<WDE);
WDTCR = 0; // within 4 cycles
```

avr-libc: `#include <avr/wdt.h>` then `wdt_enable(WDTO_1S);` / `wdt_disable();`

Bitfield reference (Watchdog):

REGISTER: WDTCR (Watchdog Timer Control Register)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │ —  │WDTOE│ —  │ —  │WDE │WDP2│WDP1│WDP0│
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

---

## 11) JTAG and PORTF pins

JTAG uses PF4–PF7. To reclaim as GPIO, disable JTAG by writing JTD in MCUCSR twice within 4 cycles.

```c
MCUCSR |= (1<<JTD);
MCUCSR |= (1<<JTD);   // write twice
// Now DDRF/PORTF PF4..PF7 are normal GPIO
```

Note: There is also a fuse that enables JTAG by default; ensure it matches your need.

---

## 12) Sleep modes

Select mode via MCUCR SM2..SM0 and enable with SE; then `SLEEP` instruction (or `sleep_mode()` in C). Common modes: Idle, Power‑save (good with async Timer2), Power‑down.

```c
#include <avr/sleep.h>
set_sleep_mode(SLEEP_MODE_IDLE);
sleep_enable();
sei();
sleep_cpu();
sleep_disable();
```

Bitfield reference (Sleep/MCU):

REGISTER: MCUCR (MCU Control Register)
```
┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
│ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
│Name │SE  │SM2 │SM1 │SM0 │ —  │ —  │ —  │ —  │
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
```

---

## 13) Quick reference: registers and bits (ATmega128)

- External interrupts: EICRA/EICRB, EIMSK, EIFR
- Timers: TCCR0/1A/1B/2/3A/3B, OCRx, ICRx, TIMSK/TIFR, ETIMSK/ETIFR, ASSR
- ADC: ADMUX, ADCSRA, ADCL/ADCH
- USART: UBRR0/1H/L, UCSR0/1A/B/C, UDR0/1
- SPI: SPCR, SPSR, SPDR
- TWI: TWBR, TWSR, TWCR, TWDR, TWAR, TWAMR
- EEPROM: EEARH/L, EEDR, EECR
- Comparator: ACSR, SFIOR.ACME
- JTAG/MCU: MCUCSR (JTD), SREG (I), MCUCR (SMx, SE)

Datasheet sections contain full bit definitions and timing; this guide focuses on working patterns.

---

## 14) Common pitfalls and debug tips

- Forgetting to set DDRx for outputs, or pull‑ups for inputs
- For interrupts: configure sense + enable mask + clear flags, then `sei()`
- Timer compare values depend on prescaler and clock; compute carefully
- Timer2 async: wait for ASSR update bits to clear after writes
- ADC: read ADCL first, then ADCH; clear ADIF by writing 1
- USART: UDRE before write, RXC before read; or use interrupts
- SPI: SS must be output in master, or it can force slave mode
- TWI: always check status codes (in TWSR) during debug
- EEPROM: wait for write complete (EEWE clear) before next op
- Watchdog: use correct timed sequence to change/disable
- JTAG: if PF4–PF7 don’t behave, JTAG might still be enabled

---

## Cross‑references in this repository

- Instruction basics: `projects/Assembly_C/Instructions.md`
- GPIO demos: `projects/Port_Basic/`
- Libraries: `shared_libs/_port.h`, `_uart.h`, `_adc.h`, `_timer2.h`, `_interrupt.h`
- Many complete projects under `projects/` show peripheral usage end‑to‑end

---

Version: 1.0 • Updated: October 2025 • Device: ATmega128

