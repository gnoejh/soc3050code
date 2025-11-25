# GPIO Programming Flow Diagrams - ATmega128

**COURSE:** SOC 3050 - Embedded Systems and Applications  
**TOPIC:** Port I/O Visual Learning Aid  
**PURPOSE:** Flowcharts and diagrams for GPIO operations

---

## DIAGRAM 1: GPIO Configuration Flow

```
Start GPIO Configuration
         |
         v
+----------------------+
| Determine pin usage  |
| (Input or Output?)   |
+----------------------+
         |
         +------------------+
         |                  |
    [Output]           [Input]
         |                  |
         v                  v
+-------------------+  +-------------------+
| Set DDRx bit = 1  |  | Set DDRx bit = 0  |
| DDRx |= (1<<n);   |  | DDRx &= ~(1<<n);  |
+-------------------+  +-------------------+
         |                  |
         v                  |
+-------------------+       |
| Set initial state |       |
| PORTB = LOW/HIGH  |       |
+-------------------+       |
         |                  v
         |          +----------------------+
         |          | Enable pull-up?      |
         |          +----------------------+
         |                  |
         |           +------+------+
         |           |             |
         |         [YES]         [NO]
         |           |             |
         |           v             v
         |    +-------------+ +-------------+
         |    | PORTx |= 1; | | PORTx &= 0; |
         |    | (Pull-up ON)| | (Floating)  |
         |    +-------------+ +-------------+
         |           |             |
         +-----------+-------------+
                     |
                     v
            +----------------+
            | GPIO Ready!    |
            | Begin using    |
            +----------------+
```

---

## DIAGRAM 2: Output Pin State Machine

```
OUTPUT PIN STATE MACHINE
========================

         +--------+
    +--->|  LOW   |<---+
    |    | (0V)   |    |
    |    +--------+    |
    |         |        |
    |   [Set HIGH]     |
    |   PORTB |= bit   |
    |         |        |
    |         v        |
    |    +--------+    |
    |    |  HIGH  |    |
    |    | (5V)   |    |
    |    +--------+    |
    |         |        |
    |   [Set LOW]      |
    |   PORTB &= ~bit  |
    |         |        |
    +---------+        |
                       |
         [Toggle]      |
         PORTB ^= bit  |
         +-------------+
```

---

## DIAGRAM 3: Input Reading Flow (Button with Pull-up)

```
INPUT READING FLOW (Active LOW Button)
========================================

Start
  |
  v
+-------------------+
| Configure pin:    |
| DDRx &= ~(1<<n);  | <- Set as input
| PORTx |= (1<<n);  | <- Enable pull-up
+-------------------+
  |
  v
+-----------------------+
| Pin steady state:     |
| Physical: HIGH (5V)   |
| PINx reads: 1         |
| Button: NOT pressed   |
+-----------------------+
  |
  v
+-------------------+
| Main loop:        |
| if (!(PINx&bit))  |
+-------------------+
  |
  +------------------+
  |                  |
[FALSE]          [TRUE]
  |                  |
  v                  v
+------------+  +--------------------+
| No action  |  | Button PRESSED!    |
| Continue   |  | PINx reads: 0      |
| loop       |  | (Pulled to GND)    |
+------------+  +--------------------+
  |                  |
  |                  v
  |            +-----------------+
  |            | Execute action  |
  |            | (LED on, etc.)  |
  |            +-----------------+
  |                  |
  +------------------+
           |
           v
      Loop again
```

---

## DIAGRAM 4: Bit Manipulation Logic

```
BIT MANIPULATION OPERATIONS
============================

OPERATION 1: SET BIT
--------------------
Original: PORTB = 0b10110010  (0xB2)
Mask:     (1<<3)= 0b00001000  (0x08)

Step 1: Create mask    (1 << 3)
Step 2: OR operation   PORTB |= mask
Result: PORTB = 0b10111010  (0xBA)
                      ^--- Bit 3 now SET


OPERATION 2: CLEAR BIT
----------------------
Original: PORTB = 0b10111010  (0xBA)
Mask:     (1<<5)= 0b00100000  (0x20)
Inverted: ~mask = 0b11011111  (0xDF)

Step 1: Create mask      (1 << 5)
Step 2: Invert mask      ~(1 << 5)
Step 3: AND operation    PORTB &= ~mask
Result: PORTB = 0b10011010  (0x9A)
                     ^--- Bit 5 now CLEAR


OPERATION 3: TOGGLE BIT
-----------------------
Original: PORTB = 0b10011010  (0x9A)
Mask:     (1<<2)= 0b00000100  (0x04)

Step 1: Create mask    (1 << 2)
Step 2: XOR operation  PORTB ^= mask
Result: PORTB = 0b10011110  (0x9E)
                       ^--- Bit 2 TOGGLED (0→1)


OPERATION 4: TEST BIT
---------------------
Current: PIND = 0b11010110  (0xD6)
Mask:    (1<<7)=0b10000000  (0x80)

Step 1: Create mask      (1 << 7)
Step 2: AND operation    PIND & mask
Result: Non-zero (0x80) → Bit 7 is SET
        Zero (0x00)     → Bit 7 is CLEAR

Example:
  if (PIND & (1 << 7)) {  // Returns 0x80 (true)
      // Bit 7 is HIGH
  }
```

---

## DIAGRAM 5: Pull-up Resistor Behavior

```
PULL-UP RESISTOR OPERATION
===========================

Case 1: Pull-up ENABLED (PORTx = 1, DDRx = 0)
----------------------------------------------

    VCC (5V)
      |
      \
      / Internal ~20-50kΩ resistor
      \
      |
      +---> Pin reads HIGH (5V) when floating
      |
    [Switch/Button]
      |
     GND

When switch OPEN:  Pin = HIGH (pulled up to VCC)
When switch CLOSED: Pin = LOW  (pulled down to GND)


Case 2: Pull-up DISABLED (PORTx = 0, DDRx = 0)
-----------------------------------------------

    VCC (5V)
      |
      X  <- Pull-up disabled
      X
      |
      +---> Pin FLOATING (undefined state!)
      |     Susceptible to noise/interference
    [Optional external circuit]


RECOMMENDATION: Always enable pull-ups for input pins
                unless external pull-up/pull-down exists
```

---

## DIAGRAM 6: Read-Modify-Write Hazard

```
READ-MODIFY-WRITE PROBLEM IN INTERRUPTS
========================================

Time Line (Without Protection):
--------------------------------

Main Program:                    Interrupt:
    |                                |
    v                                |
Read PORTB (0b10110000)              |
    |                                |
Modify (set bit 3)                   |
(0b10111000)                         |
    |                            [Interrupt!]
    |<---------------------------Read PORTB (0b10110000)
    |                                |
    |                            Modify (set bit 5)
    |                            (0b10110000 | 0b00100000)
    |                                |
    |                            Write PORTB (0b10110000)
    |                                |
Write PORTB (0b10111000)         [Return]
    |                                |
    v                                v
RESULT: Bit 5 change LOST!
        Only bit 3 is set


SOLUTION 1: Use atomic assembly (SBI/CBI)
------------------------------------------
SBI PORTB, 3   <- Cannot be interrupted mid-operation


SOLUTION 2: Disable interrupts during critical section
-------------------------------------------------------
cli();                    // Disable interrupts
PORTB |= (1 << 3);        // Safe Read-Modify-Write
sei();                    // Re-enable interrupts


SOLUTION 3: Use PINx toggle feature
------------------------------------
PINB = (1 << 3);          // Atomic toggle (hardware feature)
```

---

## DIAGRAM 7: Seven Demo Functions Flow

```
PORT_BASIC PROJECT STRUCTURE
=============================

main()
  |
  +---> init_system()
  |        |
  |        +---> Initialize serial (9600 baud)
  |        +---> Configure PORTB as output (LEDs)
  |        +---> Configure PORTD.7 as input (button)
  |        +---> Enable PORTD.7 pull-up
  |
  +---> Print welcome message
  |
  +---> infinite loop
          |
          v
    +-----------------+
    | Choose ONE demo |
    | (comment/uncomment)
    +-----------------+
          |
          +---> demo_01_write_port()
          |       "Write entire port at once"
          |       PORTB = 0xFF; (all LEDs ON)
          |       PORTB = 0x00; (all LEDs OFF)
          |
          +---> demo_02_read_port()
          |       "Read entire port at once"
          |       val = PINB; (read all 8 pins)
          |
          +---> demo_03_set_bits()
          |       "Set individual bits"
          |       PORTB |= (1 << n); (turn on specific LED)
          |
          +---> demo_04_clear_bits()
          |       "Clear individual bits"
          |       PORTB &= ~(1 << n); (turn off specific LED)
          |
          +---> demo_05_test_bit_clear()
          |       "Test if bit is clear (button check)"
          |       if (!(PIND & (1<<7))) { /* pressed */ }
          |
          +---> demo_06_test_bit_set()
          |       "Test if bit is set (inverted logic)"
          |       if (PIND & (1<<7)) { /* released */ }
          |
          +---> demo_07_combined()
                  "Combined real-world operations"
                  Button controls LED patterns


TEACHING PROGRESSION:
=====================
Week 1: Demos 1-2 (Basic read/write)
Week 2: Demos 3-4 (Bit manipulation)
Week 3: Demos 5-6 (Input testing)
Week 4: Demo 7 (Real-world application)
```

---

## DIAGRAM 8: GPIO vs Other Peripherals

```
PIN FUNCTION PRIORITY
======================

Each pin can have multiple functions.
Priority determines which function controls the pin:

Highest Priority (overrides GPIO):
    |
    +---> [Special Function Active]
    |        |
    |        +---> USART (TXD/RXD pins)
    |        +---> SPI (MOSI/MISO/SCK pins)
    |        +---> I2C (SDA/SCL pins)
    |        +---> ADC (ADCx pins)
    |        +---> Timer PWM (OCxx pins)
    |        +---> External Interrupts (INTx pins)
    |
    v
Lower Priority:
    |
    +---> [GPIO Function]
            |
            +---> PORTx controls pin
            +---> DDRx configures direction
            +---> PINx reads state

RULE: Enabling a special function (e.g., USART)
      automatically overrides GPIO control for those pins.

Example:
  If USART is enabled → PD0 (RXD) and PD1 (TXD)
  no longer respond to PORTD/DDRD/PIND operations!
```

---

## DIAGRAM 9: Debouncing State Machine

```
BUTTON DEBOUNCING STATE MACHINE
================================

State 0: RELEASED (stable)
    |
    | PINx reads HIGH (pull-up)
    v
[Wait for press]
    |
    | PINx reads LOW (first time)
    v
State 1: DEBOUNCE_WAIT
    |
    | Start timer (50ms typical)
    v
[Wait for debounce period]
    |
    +---> Check PINx after delay
    |
    +---------------+---------------+
    |               |               |
[Still LOW]    [Went HIGH]    [Noise]
    |               |               |
    v               v               v
State 2:     Return to       Ignore
PRESSED      State 0         spurious
(stable)     (false alarm)   pulse
    |
    | Execute button action
    v
[Wait for release]
    |
    | PINx reads HIGH again
    v
State 3: RELEASE_DEBOUNCE
    |
    | Start timer (50ms)
    v
[Wait for debounce period]
    |
    +---> Check PINx after delay
    |
    +---------------+
    |               |
[Still HIGH]   [Went LOW]
    |               |
    v               v
Return to      Ignore
State 0        (still held)
(stable)
```

---

## DIAGRAM 10: Memory-Mapped I/O Concept

```
MEMORY-MAPPED I/O IN ATMEGA128
===============================

CPU Address Space:
    0x0000 +-------------------+
           |  Flash (Program)  |
           |   128 KB          |
    0x1FFFF+-------------------+
           |                   |
    ...    |                   |
           |                   |
    0x0020 +-------------------+
           |  32 Registers     | <- R0-R31
    0x0000 +-------------------+
           |                   |
    0x0020 +-------------------+
           |  64 I/O Registers | <- GPIO registers here!
           |                   |
           |  PORTA  = 0x1B    |
           |  DDRA   = 0x1A    |
           |  PINA   = 0x19    |
           |  ...              |
           |  PORTB  = 0x18    |
           |  DDRB   = 0x17    |
           |  PINB   = 0x16    |
           |  ...              |
    0x005F +-------------------+
           |                   |
    0x0060 +-------------------+
           |  Internal SRAM    |
           |   4 KB            |
    0x10FF +-------------------+
           |                   |
    0x1100 +-------------------+
           |  External SRAM    |
           |   (if enabled)    |
    0xFFFF +-------------------+

ACCESSING GPIO:
---------------
C Code:         PORTB = 0xFF;
Assembly:       LDI r16, 0xFF
                OUT 0x18, r16

The OUT instruction writes to I/O space (0x20-0x5F).
C compiler translates PORTB to address 0x18 automatically.
```

---

## RESOURCES

- **ATmega128 Datasheet:** Chapter 4 (I/O Ports) for detailed timing
- **Lab Manual:** Port_Basic exercises with step-by-step instructions
- **Quick Reference:** GPIO_REGISTER_QUICK_REFERENCE.md for register details
- **Code:** projects/Port_Basic/Main.c for working examples

---

**Last Updated:** 2025  
**For:** SOC 3050 Students  
**Print this for lab reference!**
