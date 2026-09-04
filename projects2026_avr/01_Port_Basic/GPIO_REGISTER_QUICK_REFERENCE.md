# GPIO Register Quick Reference - ATmega128

**COURSE:** SOC 3050 - Embedded Systems and Applications  
**TOPIC:** Port I/O and GPIO Programming  
**PURPOSE:** Student quick reference for GPIO register usage

---

## GPIO PORT REGISTERS (3 registers per port)

Each GPIO port (PORTA-PORTG) has three 8-bit registers:

1. **DDRx** - Data Direction Register (Input/Output configuration)
2. **PORTx** - Port Data Register (Output data or pull-up control)
3. **PINx** - Port Input Register (Read pin state)

---

## REGISTER 1: DDRx (Data Direction Register)

**Purpose:** Configure each pin as INPUT (0) or OUTPUT (1)

**Bit Layout:**
```
Bit:     7      6      5      4      3      2      1      0
Name:   DD7    DD6    DD5    DD4    DD3    DD2    DD1    DD0
```

**Bit Values:**
- `0` = Input  (default after reset)
- `1` = Output

**Examples:**
```c
// Set all pins as output
DDRB = 0xFF;                  // 0b11111111 - all output

// Set all pins as input
DDRB = 0x00;                  // 0b00000000 - all input

// Mixed: PB7-PB4 output, PB3-PB0 input
DDRB = 0xF0;                  // 0b11110000

// Set single pin as output (bit 3)
DDRB |= (1 << 3);             // Set bit 3 = output

// Set single pin as input (bit 5)
DDRB &= ~(1 << 5);            // Clear bit 5 = input
```

---

## REGISTER 2: PORTx (Port Data Register)

**Purpose:** DUAL PURPOSE depending on DDR setting!

### When DDR = 1 (OUTPUT mode):
PORTx controls output voltage level
- `PORTx = 0` → Output LOW  (0V, GND)
- `PORTx = 1` → Output HIGH (5V, VCC)

### When DDR = 0 (INPUT mode):
PORTx controls pull-up resistor
- `PORTx = 0` → Pull-up OFF (tri-state, floating)
- `PORTx = 1` → Pull-up ON  (internal ~20-50kΩ to VCC)

**Bit Layout:**
```
Bit:     7      6      5      4      3      2      1      0
Name:   P7     P6     P5     P4     P3     P2     P1     P0
```

**Examples for OUTPUT mode (DDR=1):**
```c
// Set all outputs HIGH
PORTB = 0xFF;                 // All pins = 5V

// Set all outputs LOW
PORTB = 0x00;                 // All pins = 0V

// Set specific pin HIGH (bit 4)
PORTB |= (1 << 4);            // OR operation - set bit

// Set specific pin LOW (bit 2)
PORTB &= ~(1 << 2);           // AND NOT operation - clear bit

// Toggle specific pin (bit 6)
PORTB ^= (1 << 6);            // XOR operation - toggle bit
```

**Examples for INPUT mode (DDR=0):**
```c
// Enable pull-up on input pin 7
DDRD &= ~(1 << 7);            // Step 1: Set as input
PORTD |= (1 << 7);            // Step 2: Enable pull-up

// Disable pull-up (tri-state / floating input)
DDRD &= ~(1 << 7);            // Step 1: Set as input
PORTD &= ~(1 << 7);           // Step 2: Disable pull-up
```

**Why Pull-ups Matter:**
- Buttons: Pull-up keeps input HIGH; button press pulls to LOW
- Prevents floating inputs (undefined state = noise sensitivity)
- Saves external resistor components

---

## REGISTER 3: PINx (Port Input Register)

**Purpose:** Read-only register showing ACTUAL pin voltage

**Bit Layout:**
```
Bit:     7      6      5      4      3      2      1      0
Name:   PIN7   PIN6   PIN5   PIN4   PIN3   PIN2   PIN1   PIN0
```

**Bit Values (Read):**
- `0` = Pin is LOW  (≈0V, GND)
- `1` = Pin is HIGH (≈5V, VCC)

**IMPORTANT:** PINx reads PHYSICAL state regardless of DDR!

**Examples:**
```c
// Read entire port
uint8_t value = PIND;         // Read all 8 pins

// Check if specific pin is HIGH (bit 7)
if (PIND & (1 << 7)) {
    // Pin 7 is HIGH
}

// Check if specific pin is LOW (bit 3)
if (!(PIND & (1 << 3))) {
    // Pin 3 is LOW
}

// Button example (active LOW with pull-up)
DDRD &= ~(1 << 7);            // Configure as input
PORTD |= (1 << 7);            // Enable pull-up

while (1) {
    if (!(PIND & (1 << 7))) { // Button pressed (reads LOW)
        // Do something
    }
}
```

**Special Feature:** Writing 1 to PINx toggles corresponding PORTx bit!
```c
PINB = (1 << 5);              // Toggle PORTB bit 5 (atomic operation)
```

---

## BIT MANIPULATION CHEAT SHEET

### Operation 1: SET a bit (make it 1)
```c
PORTB |= (1 << n);            // Set bit n to 1
// Assembly: SBI PORTB, n
// Example: PORTB |= (1 << 3);  // Set bit 3
```

### Operation 2: CLEAR a bit (make it 0)
```c
PORTB &= ~(1 << n);           // Clear bit n to 0
// Assembly: CBI PORTB, n
// Example: PORTB &= ~(1 << 5);  // Clear bit 5
```

### Operation 3: TOGGLE a bit (flip 0↔1)
```c
PORTB ^= (1 << n);            // Toggle bit n
// Assembly: IN + EOR + OUT (3 instructions)
// Example: PORTB ^= (1 << 2);  // Toggle bit 2
```

### Operation 4: TEST if bit is SET (check if 1)
```c
if (PIND & (1 << n)) {        // True if bit n is 1
    // Bit is HIGH
}
// Assembly: SBIS PIND, n  (skip if bit set)
// Example: if (PIND & (1 << 7)) { /* HIGH */ }
```

### Operation 5: TEST if bit is CLEAR (check if 0)
```c
if (!(PIND & (1 << n))) {     // True if bit n is 0
    // Bit is LOW
}
// Assembly: SBIC PIND, n  (skip if bit clear)
// Example: if (!(PIND & (1 << 7))) { /* LOW */ }
```

### Operation 6: SET multiple bits
```c
PORTB |= (1<<3) | (1<<5) | (1<<7);  // Set bits 3, 5, 7
```

### Operation 7: CLEAR multiple bits
```c
PORTB &= ~((1<<2) | (1<<4) | (1<<6));  // Clear bits 2, 4, 6
```

---

## COMMON GPIO PATTERNS

### Pattern 1: LED Control
```c
DDRB |= (1 << 3);             // Configure as output
PORTB |= (1 << 3);            // Turn ON (HIGH = LED on)
PORTB &= ~(1 << 3);           // Turn OFF (LOW = LED off)
PORTB ^= (1 << 3);            // Toggle (flip state)
```

### Pattern 2: Button Input (Active LOW with pull-up)
```c
DDRD &= ~(1 << 7);            // Configure as input
PORTD |= (1 << 7);            // Enable pull-up
if (!(PIND & (1 << 7))) {     // Check if pressed (LOW)
    // Button pressed
}
```

### Pattern 3: Sensor Input (Active HIGH)
```c
DDRC &= ~(1 << 5);            // Configure as input
PORTC &= ~(1 << 5);           // Disable pull-up (floating)
if (PINC & (1 << 5)) {        // Check if HIGH
    // Sensor active
}
```

### Pattern 4: Relay Control (Safe power-on)
```c
DDRA |= (1 << 2);             // Configure as output
PORTA &= ~(1 << 2);           // Initialize OFF (safe)
PORTA |= (1 << 2);            // Activate relay
PORTA &= ~(1 << 2);           // Deactivate relay
```

### Pattern 5: Shift Register Output
```c
uint8_t pattern = 1;          // Start with bit 0
for (int i = 0; i < 8; i++) {
    PORTB = pattern;          // Output current pattern
    _delay_ms(100);
    pattern <<= 1;            // Shift left (move to next LED)
}
```

### Pattern 6: Debounced Button
```c
#define BUTTON_PIN 7
#define DEBOUNCE_TIME 50      // milliseconds

uint8_t read_button(void) {
    if (!(PIND & (1 << BUTTON_PIN))) {  // Initial press detected
        _delay_ms(DEBOUNCE_TIME);       // Wait for bounce to settle
        if (!(PIND & (1 << BUTTON_PIN))) {  // Still pressed?
            return 1;                   // Valid press
        }
    }
    return 0;                           // Not pressed
}
```

---

## C VS ASSEMBLY COMPARISON

| **Operation**       | **C Code**             | **Assembly**      |
|---------------------|------------------------|-------------------|
| Write entire port   | `PORTB = 0xFF;`        | `LDI r16, 0xFF`   |
|                     |                        | `OUT PORTB, r16`  |
| Read entire port    | `val = PIND;`          | `IN r16, PIND`    |
| Set single bit      | `PORTB \|= (1 << 3);`  | `SBI PORTB, 3`    |
| Clear single bit    | `PORTB &= ~(1<<5);`    | `CBI PORTB, 5`    |
| Toggle bit          | `PORTB ^= (1 << 2);`   | `IN r16, PORTB`   |
|                     |                        | `LDI r17, (1<<2)` |
|                     |                        | `EOR r16, r17`    |
|                     |                        | `OUT PORTB, r16`  |
| Test if bit set     | `if (PIND & (1<<7))`   | `SBIS PIND, 7`    |
|                     |                        | `RJMP not_set`    |
| Test if bit clear   | `if (!(PIND&(1<<7)))`  | `SBIC PIND, 7`    |
|                     |                        | `RJMP not_clear`  |

**KEY DIFFERENCES:**
1. Assembly SBI/CBI are ATOMIC (cannot be interrupted)
2. C bit operations use Read-Modify-Write (can be interrupted)
3. For interrupt-safe C operations, use `cli()`/`sei()` around code
4. Assembly is more efficient for single-bit operations
5. C is more readable and maintainable

---

## ATMEGA128 PORT AVAILABILITY

| **Port**  | **Pins**    | **Special Functions / Notes**           |
|-----------|-------------|-----------------------------------------|
| PORTA     | PA0-PA7     | 8-bit, ADC inputs (alternate function)  |
| PORTB     | PB0-PB7     | 8-bit, SPI, Timer PWM                   |
| PORTC     | PC0-PC7     | 8-bit, External memory interface        |
| PORTD     | PD0-PD7     | 8-bit, USART, External interrupts       |
| PORTE     | PE0-PE7     | 8-bit, USART1, External interrupts      |
| PORTF     | PF0-PF7     | 8-bit, ADC inputs (analog)              |
| PORTG     | PG0-PG4     | 5-bit, External memory, JTAG            |

---

## DEBUGGING CHECKLIST

### LED Not Working?
- [ ] Is DDR set to output? (`DDRx |= (1 << n);`)
- [ ] Is PORT set HIGH for LED on? (`PORTx |= (1 << n);`)
- [ ] Check physical wiring (LED polarity, resistor)
- [ ] Verify pin not used by alternate function

### Button Not Working?
- [ ] Is DDR set to input? (`DDRx &= ~(1 << n);`)
- [ ] Is pull-up enabled? (`PORTx |= (1 << n);`)
- [ ] Reading from PINx, not PORTx?
- [ ] Is button logic inverted? (Active LOW vs HIGH)
- [ ] Add debounce delay (~50ms)

### Port Behaving Strangely?
- [ ] Check for alternate functions (ADC, SPI, USART)
- [ ] Verify no conflicting DDR settings
- [ ] Use volatile keyword for register access
- [ ] Check for race conditions in interrupts
- [ ] Verify VCC/GND connections

---

## RESOURCES

- **ATmega128 Datasheet:** Chapter 4 (I/O Ports)
- **AVR Instruction Set:** See SBI, CBI, SBIS, SBIC, IN, OUT
- **Lab Manual:** Port_Basic project exercises
- **Code Examples:** projects/Port_Basic/Main.c

---

**Last Updated:** 2025  
**For:** SOC 3050 Students  
**Keep this reference handy during labs and exams!**
