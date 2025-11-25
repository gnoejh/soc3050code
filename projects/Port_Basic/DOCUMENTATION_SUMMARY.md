# Port_Basic Documentation Summary

**PROJECT:** Port_Basic - GPIO Programming Teaching Module  
**COURSE:** SOC 3050 - Embedded Systems and Applications  
**YEAR:** 2025  
**INSTRUCTOR:** Professor Hong Jeong

---

## 1. PROJECT OVERVIEW

### Purpose
Educational demonstration of ATmega128 GPIO (General Purpose Input/Output) programming. This project teaches fundamental digital I/O concepts through seven progressive demonstrations.

### Target Audience
- **Level:** Sophomore/Junior embedded systems students
- **Prerequisites:** Basic C programming, binary number systems, logic gates
- **Duration:** 3-4 weeks (2 hours per lab session)

### Learning Objectives
By completing this module, students will be able to:
1. Configure GPIO pins as inputs or outputs using DDR registers
2. Control output pins using PORT registers
3. Read input pins using PIN registers
4. Apply bit manipulation techniques (set, clear, toggle, test)
5. Understand pull-up resistors and their applications
6. Compare C vs Assembly GPIO operations
7. Implement debounced button input
8. Build simple interactive embedded applications

---

## 2. PROJECT STRUCTURE

### Hardware Setup
```
ATmega128 Development Board:
- PORTB (PB0-PB7): 8 LEDs (active HIGH)
- PORTD.7: Push button (active LOW with internal pull-up)
- Serial port: UART1 @ 9600 baud for debugging messages
- Power: 5V (USB or external)
- Clock: 16MHz crystal
```

### Software Components
```
Port_Basic/
├── Main.c                           # 7 demo functions + main loop
├── config.h                         # Hardware definitions
├── GPIO_REGISTER_QUICK_REFERENCE.md # Student reference (printable)
├── GPIO_FLOW_DIAGRAMS.md            # Visual learning aids
└── DOCUMENTATION_SUMMARY.md         # This file (instructor guide)
```

### Seven Demonstration Functions

#### Demo 1: `demo_01_write_port()` - Write Entire Port
- **Concept:** Basic output using full 8-bit write
- **Code:** `PORTB = 0xFF; _delay_ms(1000); PORTB = 0x00;`
- **Learning:** Direct register assignment, all LEDs on/off
- **Assembly equivalent:** `LDI r16, 0xFF; OUT PORTB, r16`

#### Demo 2: `demo_02_read_port()` - Read Entire Port
- **Concept:** Basic input using full 8-bit read
- **Code:** `uint8_t value = PINB; printf("PORTB = 0x%02X\n", value);`
- **Learning:** Read all pins at once, hexadecimal display
- **Assembly equivalent:** `IN r16, PINB`

#### Demo 3: `demo_03_set_bits()` - Set Individual Bits
- **Concept:** Turn on specific LEDs without affecting others
- **Code:** `PORTB |= (1 << n);`
- **Learning:** OR operation for setting bits, bit shifting
- **Assembly equivalent:** `SBI PORTB, n`

#### Demo 4: `demo_04_clear_bits()` - Clear Individual Bits
- **Concept:** Turn off specific LEDs without affecting others
- **Code:** `PORTB &= ~(1 << n);`
- **Learning:** AND with inverted mask for clearing bits
- **Assembly equivalent:** `CBI PORTB, n`

#### Demo 5: `demo_05_test_bit_clear()` - Test if Bit is Clear
- **Concept:** Check button press (active LOW logic)
- **Code:** `if (!(PIND & (1 << 7))) { /* button pressed */ }`
- **Learning:** Inverted logic for active-LOW inputs
- **Assembly equivalent:** `SBIC PIND, 7; RJMP not_pressed`

#### Demo 6: `demo_06_test_bit_set()` - Test if Bit is Set
- **Concept:** Check button release (inverted from Demo 5)
- **Code:** `if (PIND & (1 << 7)) { /* button released */ }`
- **Learning:** Normal logic for active-HIGH detection
- **Assembly equivalent:** `SBIS PIND, 7; RJMP not_released`

#### Demo 7: `demo_07_combined()` - Real-World Application
- **Concept:** Combine all techniques in practical scenario
- **Code:** Button controls LED patterns with state machine
- **Learning:** Integration of concepts, real embedded application

---

## 3. TEACHING STRATEGY

### Week-by-Week Plan

#### Week 1: GPIO Basics (Demos 1-2)
**Lecture Topics:**
- Memory-mapped I/O concept
- GPIO register overview (DDR, PORT, PIN)
- Binary/hexadecimal number systems review
- Direct register read/write operations

**Lab Activities:**
1. Configure PORTB as all outputs
2. Write patterns to LEDs (0xFF, 0x00, 0xAA, 0x55)
3. Read PORTB value and display via serial
4. Measure timing with oscilloscope (optional)

**Assessment:**
- Lab quiz: "What value makes LEDs display 10101010?"
- Practical: "Make LEDs blink in specific pattern"

#### Week 2: Bit Manipulation (Demos 3-4)
**Lecture Topics:**
- Bit manipulation operators (OR, AND, NOT, XOR)
- Bit shifting (`1 << n` creates bit mask)
- Non-destructive bit operations
- C vs Assembly comparison

**Lab Activities:**
1. Set individual LED bits (turn on LED 3, LED 5)
2. Clear individual LED bits (turn off LED 2, LED 6)
3. Toggle multiple LEDs using XOR
4. Create "walking LED" pattern (shift register simulation)

**Assessment:**
- Lab quiz: "Write code to set bits 3 and 7, clear bit 2"
- Practical: "Implement binary counter on LEDs"

#### Week 3: Input Operations (Demos 5-6)
**Lecture Topics:**
- Pull-up/pull-down resistors
- Active LOW vs Active HIGH logic
- Button debouncing basics
- Polling vs interrupt-driven input (preview)

**Lab Activities:**
1. Configure PORTD.7 as input with pull-up
2. Read button state and display on serial
3. Control single LED with button
4. Implement simple debounce delay

**Assessment:**
- Lab quiz: "Why do we need pull-up resistors?"
- Practical: "Button toggles LED state on each press"

#### Week 4: Integration (Demo 7)
**Lecture Topics:**
- State machines in embedded systems
- Combining input and output
- Real-world application design
- Code organization and modularity

**Lab Activities:**
1. Button cycles through LED patterns
2. Multi-button input (if hardware available)
3. LED brightness control using timing (PWM preview)
4. Student mini-project: Design custom LED game

**Assessment:**
- Mini-project presentation
- Peer code review
- Written reflection on learning

---

## 4. COMMON STUDENT MISTAKES AND SOLUTIONS

### Mistake 1: Confusing DDR, PORT, and PIN
**Symptom:** LEDs don't light up despite code looking correct

**Common Error:**
```c
DDRB = 0xFF;   // Set as output - CORRECT
PIND = 0xFF;   // Trying to write to PINx - WRONG!
```

**Solution:**
```c
DDRB = 0xFF;   // Set as output
PORTB = 0xFF;  // Write to PORTx, not PINx
```

**Teaching Fix:**
- Emphasize the THREE registers mnemonic:
  - **DDR** = Direction (in or out)
  - **PORT** = Output data / pull-up control
  - **PIN** = Input reading only

### Mistake 2: Forgetting to Configure DDR
**Symptom:** Port doesn't respond to PORT writes

**Common Error:**
```c
PORTB = 0xFF;  // Trying to output without setting direction
```

**Solution:**
```c
DDRB = 0xFF;   // First set direction
PORTB = 0xFF;  // Then write data
```

**Teaching Fix:**
- Always show initialization code first
- Create checklist: "Did you set DDR before using PORT?"

### Mistake 3: Not Enabling Pull-up for Input
**Symptom:** Button input reads random values, unreliable

**Common Error:**
```c
DDRD &= ~(1 << 7);         // Set as input
// Missing: PORTD |= (1 << 7);  <- Enable pull-up!
if (!(PIND & (1 << 7))) {  // Reads floating value
```

**Solution:**
```c
DDRD &= ~(1 << 7);         // Set as input
PORTD |= (1 << 7);         // Enable pull-up
if (!(PIND & (1 << 7))) {  // Now reads stable value
```

**Teaching Fix:**
- Demonstrate floating input with oscilloscope
- Show noise pickup when pull-up is disabled

### Mistake 4: Bit Manipulation Logic Errors
**Symptom:** Multiple bits affected when trying to change one

**Common Error:**
```c
PORTB = (1 << 3);   // Accidentally clears all other bits!
```

**Solution:**
```c
PORTB |= (1 << 3);  // Only sets bit 3, preserves others
```

**Teaching Fix:**
- Draw truth tables showing OR vs assignment
- Practice on paper before coding

### Mistake 5: Button Debouncing Ignored
**Symptom:** One button press detected as multiple presses

**Common Error:**
```c
if (!(PIND & (1 << 7))) {
    counter++;  // Increments multiple times per press!
}
```

**Solution:**
```c
if (!(PIND & (1 << 7))) {
    _delay_ms(50);  // Debounce delay
    if (!(PIND & (1 << 7))) {  // Verify still pressed
        counter++;
        while (!(PIND & (1 << 7)));  // Wait for release
        _delay_ms(50);  // Debounce release too
    }
}
```

**Teaching Fix:**
- Show button bounce on oscilloscope
- Demonstrate problem with LED counter

### Mistake 6: Reading PORT Instead of PIN
**Symptom:** Input always reads what was last written, not actual pin state

**Common Error:**
```c
DDRD &= ~(1 << 7);         // Set as input
if (PORTD & (1 << 7)) {    // Reading PORTD - WRONG!
```

**Solution:**
```c
DDRD &= ~(1 << 7);         // Set as input
if (PIND & (1 << 7)) {     // Read PINx, not PORTx
```

**Teaching Fix:**
- Emphasize: "PIN for input, PORT for output"
- Show difference with serial debug prints

### Mistake 7: Active LOW Button Logic Confusion
**Symptom:** Button logic appears inverted

**Common Error:**
```c
if (PIND & (1 << 7)) {  // Checking for HIGH
    // This runs when button is RELEASED!
}
```

**Solution:**
```c
if (!(PIND & (1 << 7))) {  // Checking for LOW
    // This runs when button is PRESSED
}
```

**Teaching Fix:**
- Draw circuit diagram showing pull-up + switch to GND
- Use LED indicator to show button state visually

---

## 5. ASSESSMENT IDEAS

### Lab Quizzes (5-10 minutes)
1. **Register Identification:** "Which register configures pin direction?"
2. **Bit Masks:** "What is the result of `(1 << 5)`?"
3. **Code Reading:** "Does `PORTB |= 0x08` turn LED on bit 2 or bit 3?"
4. **Circuit Analysis:** "If pull-up is ON and button is pressed, pin reads ___?"
5. **Debugging:** "Why don't my LEDs light up? (Provide buggy code)"

### Practical Exams (30 minutes)
**Scenario 1: LED Control**
- "Make LEDs blink in pattern: 10101010, 01010101 (repeat)"
- "Control LED speed with button press (faster/slower)"

**Scenario 2: Input Processing**
- "Count button presses and display on LEDs as binary number"
- "Debounce button properly (no multiple counts per press)"

**Scenario 3: Integration Challenge**
- "Button 1 starts pattern, Button 2 stops pattern"
- "Create 'Simon Says' memory game with LEDs and buttons"

### Mini-Project Ideas (Week 4)
1. **LED Dice:** Button press generates random number (1-6) shown on LEDs
2. **Reaction Timer:** LED turns on randomly, measure button press delay
3. **Binary Quiz:** Display random binary number, user presses if even/odd
4. **Traffic Light:** Realistic traffic light simulation with button crossing
5. **Morse Code:** Button inputs Morse, LEDs display letters

### Written Exam Questions
1. **Conceptual:** "Explain the difference between DDRx, PORTx, and PINx registers."
2. **Code Analysis:** "What does this code do?" (Provide snippet)
3. **Code Writing:** "Write code to read PORTD.3 and control PORTB.7"
4. **Troubleshooting:** "Student reports button doesn't work. List 3 possible causes."
5. **Design:** "Draw circuit diagram for 4 LEDs + 2 buttons on ATmega128"

---

## 6. EXTENSIONS AND ADVANCED TOPICS

### For Fast Learners
1. **External Interrupts:** Convert button polling to interrupt-driven (INT7)
2. **PWM Simulation:** Use timing loops for LED brightness control
3. **Seven-Segment Display:** Drive 7-segment display with PORTB
4. **Matrix Keypad:** Scan 4x4 matrix keypad using ports
5. **LCD Interface:** Use GPIO to drive character LCD in 4-bit mode

### Research Projects
1. **Atomic Operations:** Investigate Read-Modify-Write hazards in multitasking
2. **GPIO Speed:** Measure maximum toggle frequency of pins
3. **Power Consumption:** Compare power usage with/without pull-ups
4. **ESD Protection:** Research electrostatic discharge protection on GPIO pins
5. **Hardware Comparison:** Compare ATmega128 GPIO with ARM Cortex-M

---

## 7. TROUBLESHOOTING GUIDE (For TAs and Students)

### Hardware Issues

**LEDs Not Lighting:**
- [ ] Check power supply (5V present?)
- [ ] Verify LED polarity (anode to VCC or pin, cathode to GND or pin)
- [ ] Test LED with multimeter in diode mode
- [ ] Check for damaged LEDs (try different LED)
- [ ] Verify current-limiting resistor (220Ω-1kΩ typical)

**Button Not Working:**
- [ ] Measure voltage at pin when button released (should be ~5V with pull-up)
- [ ] Measure voltage at pin when button pressed (should be ~0V)
- [ ] Check button mechanical contact (try different button)
- [ ] Verify wiring (button connects pin to GND)
- [ ] Test with multimeter continuity mode

**Serial Port Issues:**
- [ ] Verify baud rate (9600 in both code and terminal)
- [ ] Check TX/RX connections (TX → RX crossover)
- [ ] Test with USB-serial adapter (CP2102, FTDI, etc.)
- [ ] Try different terminal program (PuTTY, Tera Term, Arduino Serial Monitor)

### Software Issues

**Code Compiles But Doesn't Run:**
- [ ] Verify hex file uploaded to correct microcontroller
- [ ] Check programmer settings (AVRDUDE configuration)
- [ ] Ensure fuse bits set correctly (clock source, brown-out)
- [ ] Try simple blink program first (isolate problem)

**Unexpected Behavior:**
- [ ] Add serial debug prints to track execution
- [ ] Use simulator (SimulIDE) to step through code
- [ ] Check for infinite loops or blocking delays
- [ ] Verify variable types (uint8_t vs int)

**Compiler Errors:**
- [ ] Missing #include <avr/io.h>
- [ ] Undefined PORTB/DDRB/PINB (wrong MCU defined)
- [ ] Syntax errors in bit manipulation (missing parentheses)
- [ ] Check config.h for MCU definition

---

## 8. RESOURCES FOR INSTRUCTORS

### Reference Documents
- **ATmega128 Datasheet (PDF):** Chapter 4 (I/O Ports) - 30 pages
- **AVR Instruction Set Manual (PDF):** SBI, CBI, SBIS, SBIC instructions
- **Application Note AVR151:** Setup and Use of the SPI on AVR (for advanced GPIO)

### Online Resources
- **Microchip Studio:** Free IDE with simulator (formerly Atmel Studio)
- **SimulIDE:** Free circuit simulator with ATmega128 support
- **AVR Freaks Forum:** Community support for AVR programming
- **GitHub:** Example projects and libraries

### Recommended Textbooks
1. **"Making Embedded Systems" by Elecia White**
   - Chapter on GPIO and hardware interfaces
2. **"The AVR Microcontroller and Embedded Systems" by Mazidi**
   - Comprehensive AVR programming reference
3. **"Embedded C Programming" by Michael J. Pont**
   - Focus on portable embedded C techniques

### Lab Equipment
- ATmega128 development board (1 per student pair)
- USB programmer (AVRISP mkII, USBasp, or Arduino as ISP)
- LEDs (red/green/yellow, 10 per board)
- Resistors (220Ω for LEDs, assorted values)
- Push buttons (tactile switches, 2-4 per board)
- Breadboard and jumper wires
- Multimeter (at least 1 per 4 students)
- Oscilloscope (optional, 1 per lab for demonstrations)

---

## 9. GRADING RUBRIC

### Lab Participation (40%)
- **Attendance:** 10% (must attend all 4 labs)
- **In-Lab Quizzes:** 15% (4 quizzes × 3.75% each)
- **Practical Completion:** 15% (all 7 demos working)

### Assessments (30%)
- **Midterm Practical Exam:** 15% (Weeks 1-2 material)
- **Final Practical Exam:** 15% (All material)

### Mini-Project (20%)
- **Functionality:** 10% (Does it work as designed?)
- **Code Quality:** 5% (Comments, organization, efficiency)
- **Documentation:** 5% (Flowchart, written explanation)

### Written Exam (10%)
- **Conceptual Questions:** 5%
- **Code Reading/Writing:** 5%

### Scoring Guide for Mini-Project

**Functionality (10 points):**
- 10: Fully working, handles edge cases
- 8: Works with minor bugs
- 6: Partially working, core features present
- 4: Significant issues, some features work
- 2: Minimal functionality
- 0: Does not compile or run

**Code Quality (5 points):**
- 5: Well-commented, modular, efficient
- 4: Good structure, adequate comments
- 3: Works but messy or inefficient
- 2: Poor organization, minimal comments
- 1: Difficult to understand
- 0: Plagiarized or unreadable

**Documentation (5 points):**
- 5: Complete flowchart + clear written explanation
- 4: Good documentation with minor gaps
- 3: Basic documentation present
- 2: Incomplete or unclear documentation
- 1: Minimal effort
- 0: Missing documentation

---

## 10. COURSE INTEGRATION

### Prerequisite Knowledge
**From Previous Courses:**
- Binary number system and hexadecimal notation
- Boolean algebra (AND, OR, NOT, XOR)
- Basic C programming (variables, loops, conditionals)
- Digital logic gates (TTL/CMOS levels)

### Connects Forward To
**In Future Lessons:**
- **Serial Communications:** UART pins are GPIO with alternate function
- **SPI/I2C:** Uses GPIO pins for serial buses
- **Timers/PWM:** Output compare pins are GPIO-based
- **ADC:** Analog input pins share with PORTA/PORTF GPIO
- **Interrupts:** External interrupt pins (INT0-INT7) on GPIO ports
- **Power Management:** GPIO pin states affect sleep mode current

### Real-World Applications
- **Industrial Control:** PLC-style relay control
- **IoT Devices:** Button interfaces, LED indicators
- **Automotive:** Dashboard indicators, switch inputs
- **Consumer Electronics:** Keypad interfaces, status LEDs
- **Robotics:** Sensor interfaces, motor control signals

---

## 11. CONTINUOUS IMPROVEMENT

### Feedback Collection
- **Student Survey (Week 2):** "Which demo was most helpful?"
- **Exit Ticket (Each Lab):** "One thing I learned, one thing I'm confused about"
- **TA Observations:** Track common mistakes for future emphasis

### Iteration Plan
- **2025 Spring:** Current version with 7 demos
- **2025 Fall:** Add 8th demo (interrupt-driven button)
- **2026 Spring:** Integrate with LCD module project
- **2026 Fall:** Create online simulation lab (SimulIDE exercises)

### Success Metrics
- **Target:** 85% of students complete all 7 demos successfully
- **Target:** Average mini-project score ≥ 75%
- **Target:** < 20% failure rate on practical exams
- **Target:** Student satisfaction ≥ 4.0/5.0 on course evaluation GPIO module

---

## 12. QUICK REFERENCE FOR INSTRUCTORS

### Before Each Lab Session
- [ ] Test all hardware stations (LEDs, buttons work)
- [ ] Verify programming setup (toolchain installed)
- [ ] Print GPIO_REGISTER_QUICK_REFERENCE.md for students
- [ ] Prepare demo board for showing correct behavior
- [ ] Review common mistakes from previous session

### During Lab Session
- [ ] Brief 10-minute introduction/recap
- [ ] Walk through first demo together
- [ ] Circulate to answer questions (target: each student group 2x per session)
- [ ] Note common issues for whole-class discussion
- [ ] Last 15 minutes: Wrap-up and preview next week

### After Lab Session
- [ ] Grade lab quizzes (same day if possible)
- [ ] Update FAQ document based on student questions
- [ ] Email summary to students (what was covered, what's next)
- [ ] Update lecture slides based on observed confusion

---

## 13. CONTACT AND SUPPORT

**Course Instructor:**
- Professor Hong Jeong
- Email: hong.jeong@university.edu
- Office Hours: MW 2-4 PM, Engineering Building Room 305

**Teaching Assistants:**
- (To be assigned each semester)
- Lab Hours: (See course syllabus)

**Technical Support:**
- Lab Manager: (Contact for hardware issues)
- IT Helpdesk: (Contact for software installation)

---

**Document Version:** 1.0  
**Last Updated:** 2025-01-15  
**Next Review:** 2025-08-01  

**For Questions or Suggestions:**  
Contact Professor Hong Jeong or submit feedback via course management system.

---

**END OF DOCUMENTATION**
