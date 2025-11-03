# Port Programming in C - Level 1

## Overview
This project teaches **port I/O programming in C language** for the ATmega128 microcontroller. Learn the fundamental concepts here, then progress to `Port_Assembly` to see how the same operations work in assembly language.

## Learning Objectives
By completing these demos, you will understand:
1. **Writing to ports** - Setting output values
2. **Reading from ports** - Getting input values
3. **Setting individual bits** - Turning on specific outputs
4. **Clearing individual bits** - Turning off specific outputs
5. **Testing bit conditions** - Checking input states

## Hardware Setup
- **Microcontroller:** ATmega128 @ 16MHz
- **LEDs:** 8 LEDs connected to PORTB (PB0-PB7)
- **Button:** 1 button on PORTD.7 (active LOW with pull-up resistor)

## Demo Progression

### Demo 1: Writing to Ports (`demo_01_write_port`)
**Concept:** Write entire byte to port register
- C Code: `PORTB = 0xFF;`
- Assembly Equivalent: `OUT PORTB, r16`
- **What you'll see:** All LEDs blink together, then alternating patterns
- **Key Learning:** Direct port assignment controls all 8 pins simultaneously

### Demo 2: Reading from Ports (`demo_02_read_port`)
**Concept:** Read entire byte from port register
- C Code: `value = PIND;`
- Assembly Equivalent: `IN r16, PIND`
- **What you'll see:** LED pattern changes when button is pressed
- **Key Learning:** Reading PIN register gets current input state

### Demo 3: Setting Individual Bits (`demo_03_set_bits`)
**Concept:** Turn ON one bit without affecting others
- C Code: `PORTB |= (1 << bit);`
- Assembly Equivalent: `SBI PORTB, bit`
- **What you'll see:** LEDs turn ON one by one from right to left
- **Key Learning:** OR operation sets specific bits

### Demo 4: Clearing Individual Bits (`demo_04_clear_bits`)
**Concept:** Turn OFF one bit without affecting others
- C Code: `PORTB &= ~(1 << bit);`
- Assembly Equivalent: `CBI PORTB, bit`
- **What you'll see:** LEDs turn OFF one by one from right to left
- **Key Learning:** AND with inverted mask clears specific bits

### Demo 5: Testing if Bit is Clear (`demo_05_test_bit_clear`)
**Concept:** Check if an input bit is 0 (LOW)
- C Code: `if (!(PIND & (1 << 7)))`
- Assembly Equivalent: `SBIC PIND, 7`
- **What you'll see:** All LEDs ON when button pressed, OFF when released
- **Key Learning:** Test for 0 using negated AND operation

### Demo 6: Testing if Bit is Set (`demo_06_test_bit_set`)
**Concept:** Check if an input bit is 1 (HIGH)
- C Code: `if (PIND & (1 << 7))`
- Assembly Equivalent: `SBIS PIND, 7`
- **What you'll see:** Pattern changes between alternating and opposite when button pressed
- **Key Learning:** Test for 1 using AND operation

### Demo 7: Combined Example (`demo_07_combined`)
**Concept:** All operations working together
- **What you'll see:** LEDs count up when button pressed, count down when released
- **Key Learning:** Combining read, write, set, clear, and test operations

## How to Use

1. **Open** `Main.c` in your editor

2. **Select a demo** by uncommenting it in `main()`:
   ```c
   int main(void)
   {
       demo_01_write_port();    // ← Active demo
       // demo_02_read_port();  // Commented out
       // demo_03_set_bits();   // Commented out
       // ...
   }
   ```

3. **Build the project:**
   ```powershell
   .\cli-build-project.ps1 -ProjectDir "projects\Port"
   ```

4. **Program to board:**
   ```powershell
   .\cli-program-project.ps1 -ProjectDir "projects\Port" -Programmer arduino -Port COM3
   ```

5. **Observe** the LED behavior and understand the code

6. **Move to next demo** by commenting current and uncommenting next

## Learning Path

```
Port (C Language)          ←── YOU ARE HERE
    ↓
Port_Assembly              ←── Same concepts in assembly
    ↓
Port_Configuration         ←── Advanced applications
```

## C vs Assembly Comparison

| Operation | C Code | Assembly |
|-----------|--------|----------|
| Write port | `PORTB = 0xFF;` | `OUT PORTB, r16` |
| Read port | `val = PIND;` | `IN r16, PIND` |
| Set bit | `PORTB \|= (1<<3);` | `SBI PORTB, 3` |
| Clear bit | `PORTB &= ~(1<<3);` | `CBI PORTB, 3` |
| Test bit clear | `if(!(PIND & (1<<7)))` | `SBIC PIND, 7` |
| Test bit set | `if(PIND & (1<<7))` | `SBIS PIND, 7` |

## Important Concepts

### Atomicity
- **Assembly:** SBI/CBI instructions are atomic (cannot be interrupted)
- **C:** Bit operations use read-modify-write (can be interrupted)
- **When it matters:** Interrupt service routines, concurrent access

### Performance
- **Assembly:** Faster, predictable cycle counts
- **C:** Slightly slower, but compiler optimizes well
- **When it matters:** Time-critical code, real-time systems

### Readability
- **C:** More readable, easier to understand
- **Assembly:** Shows exact hardware operations
- **Balance:** Use C for clarity, assembly for critical sections

## Next Steps

After mastering these C demos:
1. **Review** the code comments to understand each operation
2. **Experiment** by modifying delay times and patterns
3. **Progress** to `Port_Assembly` to see assembly equivalents
4. **Compare** how C compiles to assembly (use `-S` flag)
5. **Apply** knowledge in `Port_Configuration` advanced project

## Reference Files
- `Main.c` - All 7 demo functions with detailed comments
- `config.h` - System configuration (16MHz, 9600 baud)
- `../Port_Assembly/README_IO_INSTRUCTIONS.md` - Assembly instruction reference

## Tips for Learning
1. Start with Demo 1 and work sequentially
2. Read the comments carefully - they explain every line
3. Observe the hardware behavior before moving to next demo
4. Compare the C code with assembly version in `Port_Assembly`
5. Ask yourself: "What would this look like in assembly?"

---
**After completing all demos, you'll have a solid foundation in port I/O programming!**
