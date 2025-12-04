# Interrupt Flow Diagrams - ATmega128

**COURSE:** SOC 3050 - Embedded Systems and Applications  
**TOPIC:** Polling vs Interrupt Visual Learning Aid  
**PURPOSE:** Flowcharts and diagrams for interrupt concepts

---

## DIAGRAM 1: Polling Flow

```
POLLING METHOD - CONTINUOUS CHECKING
====================================

Start Program
     |
     v
+------------------+
| Initialize GPIO  |
| - Set DDRx       |
| - Set PORTx      |
+------------------+
     |
     v
+------------------+
| while (1) {      |  ← INFINITE LOOP
+------------------+
     |
     v
+------------------+
| Read Input Pin   |
| val = PIND;      |
+------------------+
     |
     v
  [Button Pressed?]
     /    \
  YES      NO
   |        |
   v        v
[LED ON] [LED OFF]
   |        |
   +---+----+
       |
       v
+------------------+
| _delay_ms(50)    |  ← AFFECTS RESPONSE TIME
+------------------+
       |
       v
   (Back to Read)

KEY POINTS:
- CPU always busy checking
- Response time = loop time
- May miss fast events
- Simple to understand
```

---

## DIAGRAM 2: Interrupt Flow

```
INTERRUPT METHOD - EVENT-DRIVEN
================================

Start Program
     |
     v
+------------------+
| Initialize GPIO  |
+------------------+
     |
     v
+------------------+
| Configure EICRA  |
| Set edge mode    |
+------------------+
     |
     v
+------------------+
| Enable EIMSK     |
| Enable INT0      |
+------------------+
     |
     v
+------------------+
| Enable sei()     |
| Global interrupt |
+------------------+
     |
     v
+------------------+
| Main Loop:       |
| - Do other tasks |
| - Check flags    |
| - Process events |
+------------------+
     |     ^
     |     |
     |     +------+
     |            |
  [HARDWARE]      |
  DETECTS EDGE    |
     |            |
     v            |
+------------------+
| ISR(INT0_vect) { |  ← RUNS AUTOMATICALLY
| - Set flag       |
| - Toggle LED     |
| - Return         |
+------------------+
       |
       +------------+
                   (Resume main)

KEY POINTS:
- CPU does other work
- Response instant (2-4 cycles)
- Hardware guarantees detection
- More complex to implement
```

---

## DIAGRAM 3: ISR Execution Timeline

```
INTERRUPT EXECUTION SEQUENCE
=============================

MAIN CODE RUNNING:
─────────────────────────────────────────────────────
instruction 1 → instruction 2 → instruction 3 → ...
                    ↑
                 [BUTTON PRESS!]
                    |
              Hardware detects
              falling edge on PD0
                    |
                    v
              +-------------+
              | INTF0 = 1   |  Flag set in EIFR
              +-------------+
                    |
              Check: EIMSK[INT0]=1?
              Check: SREG[I]=1?
                    |
                 YES (both)
                    |
                    v
         +----------------------+
         | SAVE CONTEXT         |
         | - Push PC to stack   |  ~10-15 cycles
         | - Push SREG to stack |
         | - Clear I-bit (SREG) |
         +----------------------+
                    |
                    v
         +----------------------+
         | JUMP TO 0x0004       |  INT0 vector address
         +----------------------+
                    |
                    v
         +----------------------+
         | ISR(INT0_vect) {     |
         |   flag = 1;          |  User ISR code
         |   led_toggle(0);     |  (~5-50 cycles)
         | }                    |
         +----------------------+
                    |
                    v
         +----------------------+
         | RETI INSTRUCTION     |
         | - Pop SREG from stack|  ~10-15 cycles
         | - Pop PC from stack  |
         | - Set I-bit (SREG)   |
         +----------------------+
                    |
                    v
MAIN CODE RESUMES:
─────────────────────────────────────────────────────
instruction 3 (continues) → instruction 4 → ...

TOTAL LATENCY: 25-80 clock cycles typical
At 16MHz: 1.5 - 5 microseconds
```

---

## DIAGRAM 4: Edge Detection Modes

```
BUTTON SIGNAL (Active-Low with Pull-up)
========================================

          Released              Pressed          Released
            HIGH                  LOW              HIGH
             |                     |                |
    ┌────────┴─────────┐    ┌─────┴────┐    ┌─────┴────────┐
    │                  │    │          │    │              │
────┘                  └────┘          └────┘              └────
                       A    B          C    D
                       |    |          |    |
                       v    v          v    v
                    FALLING  RISING  FALLING  RISING
                     EDGE     EDGE    EDGE     EDGE

MODE 1: Falling Edge (ISC01=1, ISC00=0)
        Triggers at: A, C (button press)
        Use: Detect button press

MODE 2: Rising Edge (ISC01=1, ISC00=1)
        Triggers at: B, D (button release)
        Use: Detect button release

MODE 3: Any Change (ISC01=0, ISC00=1)
        Triggers at: A, B, C, D (all edges)
        Use: Count all transitions

MODE 4: Low Level (ISC01=0, ISC00=0)
        Triggers: Continuously while between A-B, C-D
        Use: RARELY (causes interrupt flood!)
```

---

## DIAGRAM 5: Register Configuration

```
INTERRUPT SETUP - REGISTER FLOW
================================

STEP 1: PIN CONFIGURATION
┌──────────────────────────┐
│ DDRD &= ~(1 << PD0);     │  Set PD0 as INPUT
│ PORTD |= (1 << PD0);     │  Enable pull-up resistor
└──────────────────────────┘
           |
           v
STEP 2: EDGE DETECTION
┌──────────────────────────┐
│ EICRA |= (1 << ISC01);   │  ISC01 = 1 ┐
│ EICRA &= ~(1 << ISC00);  │  ISC00 = 0 ├─ Falling Edge
└──────────────────────────┘            ┘
           |
           v
STEP 3: ENABLE INTERRUPT
┌──────────────────────────┐
│ EIMSK |= (1 << INT0);    │  Enable INT0 in mask
└──────────────────────────┘
           |
           v
STEP 4: GLOBAL ENABLE
┌──────────────────────────┐
│ sei();                   │  Set I-bit in SREG
└──────────────────────────┘
           |
           v
     [READY!]
   Interrupt system
   armed and waiting
```

---

## DIAGRAM 6: ISR Communication Pattern

```
ISR-TO-MAIN COMMUNICATION
=========================

┌─────────────────────────┐
│ GLOBAL VARIABLES        │
│ (in SRAM)               │
├─────────────────────────┤
│ volatile uint8_t flag;  │ ← MUST BE VOLATILE!
│ volatile uint16_t count;│
└─────────────────────────┘
      ↑               ↑
      |               |
      | WRITES        | READS
      |               |
┌─────┴────┐    ┌─────┴────────┐
│   ISR    │    │  MAIN LOOP   │
│          │    │              │
│ ISR() {  │    │ while(1) {   │
│  flag=1; │    │  if(flag) {  │
│  count++;│    │   flag=0;    │
│  ...     │    │   process(); │
│ }        │    │  }           │
└──────────┘    │ }            │
                └──────────────┘

CRITICAL RULES:
1. Variables shared between ISR and main MUST be volatile
2. ISR writes, main reads (typical pattern)
3. Multi-byte variables need cli()/sei() protection
4. Keep ISR short - set flag, let main do work
```

---

## DIAGRAM 7: Multiple Interrupt Priority

```
MULTIPLE INTERRUPTS - PRIORITY HANDLING
========================================

Scenario: INT0 and INT3 both trigger simultaneously

PRIORITY TABLE:
┌─────────┬──────────┬──────────┐
│ Vector  │ Interrupt│ Priority │
├─────────┼──────────┼──────────┤
│   2     │  INT0    │ HIGHEST  │ ← Runs first
│   3     │  INT1    │    ↓     │
│   4     │  INT2    │    ↓     │
│   5     │  INT3    │ Lower    │ ← Waits
└─────────┴──────────┴──────────┘

EXECUTION TIMELINE:
─────────────────────────────────────
Main running...
    ↓
[Both INT0 and INT3 triggered!]
    ↓
  ┌─────────────────┐
  │ ISR(INT0_vect)  │ ← Executes first (higher priority)
  │ {               │
  │   ...           │
  │ }               │
  └─────────────────┘
    ↓
  ┌─────────────────┐
  │ ISR(INT3_vect)  │ ← Executes second (lower priority)
  │ {               │
  │   ...           │
  │ }               │
  └─────────────────┘
    ↓
Resume main code

NOTE: By default, interrupts are disabled during ISR
      (prevents nesting unless specifically configured)
```

---

## DIAGRAM 8: Polling vs Interrupt Comparison

```
TIMING COMPARISON
=================

POLLING (50ms delay):
Time: 0ms    50ms   100ms  150ms  200ms  250ms
      |------|------|------|------|------|
Check ✓      ✓      ✓      ✓      ✓      ✓

Button press at 25ms:
      |------|------|
           ↓ PRESS
           └──────────→ NOT DETECTED until 50ms check!
                       Response delay: 25ms

Button press at 5ms (very quick 10ms pulse):
      |------|
       ↓↑
       └─ MISSED! Faster than 50ms check rate


INTERRUPT:
Time: 0ms    50ms   100ms  150ms  200ms  250ms
      |------|------|------|------|------|
              ↓ PRESS
              ↓ ISR RUNS INSTANTLY (0.2µs latency)
              ✓ DETECTED

Even 1µs pulse detected!

CONCLUSION:
- Polling: Response = 0 to loop_delay time
- Interrupt: Response = constant ~0.2µs (2-4 cycles @ 16MHz)
```

---

## DIAGRAM 9: Debounce in ISR vs Main

```
BUTTON BOUNCING PROBLEM
========================

Physical Button Press:
     Ideal:     │     │
                └─────┘

     Reality:   │  │ ││  │
                └──┘ └┘└──┘
                 ↑↑↑  ↑↑
                 Multiple transitions!

BAD APPROACH (debounce in ISR):
┌─────────────────────────────┐
│ ISR(INT0_vect) {            │
│   _delay_ms(20);  ← BLOCKS! │
│   if (button_pressed) {     │
│     flag = 1;               │
│   }                         │
│ }                           │
└─────────────────────────────┘
Problem: Delays block other interrupts!

GOOD APPROACH (debounce in main):
┌─────────────────────────────┐
│ ISR(INT0_vect) {            │
│   flag = 1;  ← FAST!        │
│ }                           │
│                             │
│ main() {                    │
│   if (flag) {               │
│     _delay_ms(20);  ← OK!   │
│     if (still_pressed) {    │
│       process();            │
│     }                       │
│     flag = 0;               │
│   }                         │
│ }                           │
└─────────────────────────────┘
Better: ISR is fast, main handles debounce
```

---

## DIAGRAM 10: Five Demo Progression

```
TEACHING PROGRESSION
====================

WEEK 1: POLLING FOUNDATION
    |
    +--→ Demo 1: Polling Basics
    |      - Simple while(1) loop
    |      - Check button, control LED
    |      - Understand _delay_ms() impact
    |
    +--→ Demo 2: Polling Limitations
           - Slow/fast polling phases
           - Demonstrate missed events
           - Motivate need for interrupts

WEEK 2: INTERRUPT INTRODUCTION
    |
    +--→ Demo 3: Interrupt Basics
    |      - Configure INT0
    |      - Write first ISR
    |      - Compare response time
    |
    +--→ Demo 4: ISR Communication
           - Use volatile flags
           - ISR-to-main messaging
           - Event counting

WEEK 3: ADVANCED TECHNIQUES
    |
    +--→ Demo 5: Edge Detection Modes
           - Mode 0: Falling edge
           - Mode 1: Rising edge
           - Mode 2: Any change
           - Mode 3: Low level
           
FINAL PROJECT IDEAS:
- Reaction time game
- Multi-button menu system
- Event logger with timestamps
- Interrupt-driven state machine
```

---

## RESOURCES

- **Main.c:** 5 working demos
- **INTERRUPT_QUICK_REFERENCE.md:** Complete register reference
- **DOCUMENTATION_SUMMARY.md:** Instructor teaching guide
- **ATmega128 Datasheet:** External Interrupts chapter

---

**Last Updated:** 2025  
**For:** SOC 3050 Students  
**Visual learning aid for lectures!**
