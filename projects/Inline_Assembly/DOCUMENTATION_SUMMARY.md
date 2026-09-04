# INLINE ASSEMBLY - INSTRUCTOR DOCUMENTATION SUMMARY
**ATmega128 Embedded Systems | SOC 3050**

---

## DOCUMENT PURPOSE

This guide provides instructors with a comprehensive teaching plan for the **Inline Assembly** module. It includes lecture outlines, lab activities, assessment rubrics, troubleshooting guides, and pedagogical strategies for teaching assembly language integration with C.

---

## MODULE OVERVIEW

### Educational Goals
By the end of this 4-week module, students will be able to:
1. Write inline assembly code using `__asm__ volatile()` syntax
2. Select appropriate register constraints for different instructions
3. Understand register allocation and clobber lists
4. Implement hardware control using direct assembly instructions
5. Optimize critical code sections with assembly
6. Debug inline assembly code using disassembly tools
7. Make informed decisions about when to use assembly vs C
8. Apply inline assembly to real-world embedded applications

### Prerequisites
Students should have completed:
- **Port_Basic**: GPIO fundamentals
- **Interrupt**: Interrupt handling concepts
- **Serial_Communications**: Basic debugging with UART
- C programming fundamentals (pointers, functions, data types)

### Hardware Requirements
- ATmega128 development board (or compatible)
- 8 LEDs connected to PORTB (active low)
- 4 push buttons on PORTD (active low with pull-ups)
- Optional: UART connection for debugging output
- Optional: Oscilloscope for timing verification

### Software Requirements
- AVR-GCC toolchain with inline assembly support
- AVR-OBJDUMP for disassembly analysis
- SimulIDE or hardware programmer
- Serial terminal program (if using UART)

---

## 4-WEEK TEACHING PLAN

### WEEK 1: FUNDAMENTALS OF INLINE ASSEMBLY

#### Day 1: Introduction to Inline Assembly
**Learning Objectives:**
- Understand what inline assembly is and when to use it
- Learn basic `__asm__ volatile()` syntax
- Recognize differences between C and assembly execution

**Lecture Topics (90 minutes):**
1. **Why Inline Assembly?** (20 min)
   - Performance optimization (speed, size)
   - Direct hardware control (atomic operations)
   - Precise timing requirements
   - Trade-offs: complexity vs readability

2. **Basic Syntax** (30 min)
   - `__asm__ volatile()` structure
   - Code section: instructions with `\n\t` separators
   - Operand sections: outputs, inputs, clobbers
   - The role of `volatile` keyword

3. **Simple Examples** (40 min)
   - NOP instruction for timing
   - Direct port output with OUT
   - Reading inputs with IN
   - SBI/CBI for single-bit operations

**Demo: `demo_01_basic_inline_assembly()`**
- Live coding: NOP delays
- Oscilloscope measurement of timing
- Compare C `_delay_ms()` vs assembly NOPs
- Show disassembly to prove inline code insertion

**Lab Exercise 1 (2 hours):**
```
TITLE: "Blink LED with Inline Assembly"

TASK: Create a program that blinks an LED using only inline assembly.
- Use SBI/CBI to toggle PORTB bit 0
- Implement precise 500ms delays using NOP instructions
- Calculate: How many NOPs for 500ms @ 16MHz? (hint: too many!)
- Combine NOPs with nested loop in assembly

LEARNING GOALS:
- Practice basic inline assembly syntax
- Understand instruction timing
- Learn SBI/CBI for bit manipulation

DELIVERABLES:
- Working LED blink program
- Timing calculations document
- Oscilloscope screenshot showing 1Hz square wave
```

---

#### Day 2: GPIO Control with Assembly
**Learning Objectives:**
- Master SBI/CBI/IN/OUT instructions
- Understand I/O register addressing
- Compare C vs assembly for GPIO

**Lecture Topics (90 minutes):**
1. **I/O Register Architecture** (25 min)
   - Low I/O space (0x00-0x1F): Direct addressing
   - High I/O space (0x20-0x3F): Use `_SFR_IO_ADDR()`
   - Extended I/O (0x60+): LDS/STS required
   - Atomic operations with SBI/CBI

2. **GPIO Instructions** (35 min)
   - SBI: Set Bit in I/O (1 cycle for low I/O)
   - CBI: Clear Bit in I/O (1 cycle for low I/O)
   - IN: Input from I/O port
   - OUT: Output to I/O port
   - Why these are faster than C bit manipulation

3. **Constraints for I/O** (30 min)
   - "I" constraint for I/O addresses
   - "I" constraint for bit numbers
   - "r" constraint for data values
   - Compile-time constants vs runtime values

**Demo: `demo_02_gpio_with_assembly()`**
- Show C version: `PORTB |= (1 << 0);`
- Show assembly version: `sbi PORTB, 0`
- Disassembly comparison: 3-5 instructions vs 1 instruction
- Button input reading with IN instruction

**Lab Exercise 2 (2 hours):**
```
TITLE: "Knight Rider LED Pattern"

TASK: Implement a Knight Rider LED effect using inline assembly.
- 8 LEDs on PORTB (active low)
- LED sweeps left-right-left continuously
- Use SBI/CBI for LED control
- Add button input to pause/resume effect

LEARNING GOALS:
- Complex GPIO patterns with assembly
- Bit manipulation loops
- Input reading with IN instruction

BONUS CHALLENGE:
- Measure execution speed: C version vs assembly version
- Optimize for minimum delay between LED changes
- Use oscilloscope to prove assembly is faster

DELIVERABLES:
- Working Knight Rider program (C + inline assembly)
- Performance comparison document (C vs ASM timing)
- Code comments explaining each assembly instruction
```

---

#### Day 3: Arithmetic Operations
**Learning Objectives:**
- Implement ADD/SUB/INC/DEC in assembly
- Understand input/output constraints
- Work with 8-bit and 16-bit arithmetic

**Lecture Topics (90 minutes):**
1. **Arithmetic Instructions** (30 min)
   - ADD/ADC: Addition with/without carry
   - SUB/SBC/SUBI: Subtraction operations
   - INC/DEC: Increment/decrement (faster than ADD/SUB)
   - NEG: Two's complement negation

2. **Operand Constraints** (35 min)
   - Output operands: `=r` (write-only)
   - Input operands: `r` (read-only)
   - Read-write operands: `+r` (both)
   - Matching constraints: `"0"` (reuse operand 0's register)

3. **Multi-byte Arithmetic** (25 min)
   - 16-bit addition with carry (ADD + ADC)
   - %A0 / %B0 notation for low/high bytes
   - 32-bit operations (future topic)
   - Signed vs unsigned operations

**Demo: `demo_03_arithmetic_operations()`**
- Simple 8-bit addition: a + b
- Read-write example: counter increment
- 16-bit addition with carry propagation
- Show register allocation in disassembly

**Lab Exercise 3 (2 hours):**
```
TITLE: "Assembly Calculator"

TASK: Create a simple calculator using inline assembly.
- Input: Two 8-bit numbers (hardcoded or via UART)
- Operations: ADD, SUB, MUL (repeated addition), DIV (repeated subtraction)
- Output: Display result on LEDs or via UART
- Implement ALL arithmetic in inline assembly

LEARNING GOALS:
- Practice arithmetic instructions
- Use input/output constraints correctly
- Implement algorithms in assembly (multiply, divide)

FUNCTIONS TO IMPLEMENT:
1. asm_add(a, b) → a + b
2. asm_sub(a, b) → a - b
3. asm_mul(a, b) → a * b (using loop with ADD)
4. asm_div(a, b) → a / b (using loop with SUB)

DELIVERABLES:
- Working calculator program
- Test cases (at least 10 per operation)
- Performance analysis: Compare with C arithmetic
- Documentation explaining multiplication/division algorithms
```

---

### WEEK 2: ADVANCED OPERATIONS

#### Day 4: Bitwise Logic Operations
**Learning Objectives:**
- Use AND/OR/XOR for bit masking
- Apply shift and rotate operations
- Understand flag register (SREG) updates

**Lecture Topics (90 minutes):**
1. **Logic Instructions** (30 min)
   - AND/ANDI: Masking bits (clear unwanted bits)
   - OR/ORI: Setting bits (set specific bits)
   - EOR: Toggling bits (XOR operation)
   - COM: One's complement (invert all bits)

2. **Shift & Rotate** (35 min)
   - LSL: Logical shift left (multiply by 2)
   - LSR: Logical shift right (divide by 2)
   - ROL/ROR: Rotate through carry
   - SWAP: Exchange nibbles (fast!)

3. **Applications** (25 min)
   - Fast multiply/divide by powers of 2
   - Bit counting algorithms
   - Bit reversal
   - Parity calculation

**Demo: `demo_04_bitwise_logic_operations()`**
- Masking lower 4 bits with ANDI
- Setting upper 4 bits with ORI
- XOR for full inversion
- SWAP for nibble exchange (show 1 cycle vs 8 shifts)

**Lab Exercise 4 (2 hours):**
```
TITLE: "Bit Manipulation Toolkit"

TASK: Implement bit manipulation functions in inline assembly.

FUNCTIONS TO IMPLEMENT:
1. popcount(x) → count number of set bits (1s) in x
2. reverse_bits(x) → reverse bit order (0b10110010 → 0b01001101)
3. ffs(x) → find first set bit (rightmost 1)
4. parity(x) → return 1 if odd number of 1s, else 0
5. next_power_of_2(x) → return next power of 2 greater than x

LEARNING GOALS:
- Complex bit manipulation algorithms
- Efficient shift operations
- Loop structures in inline assembly

BONUS CHALLENGE:
- Optimize popcount using Brian Kernighan's algorithm
- Implement 16-bit versions of all functions

DELIVERABLES:
- Bit manipulation library (asm_bitops.h)
- Unit tests for all functions
- Cycle count analysis per function
- Comparison with C library functions
```

---

#### Day 5: Register Constraints Deep Dive
**Learning Objectives:**
- Choose correct constraints for each instruction
- Understand register allocation by compiler
- Debug constraint-related errors

**Lecture Topics (90 minutes):**
1. **Constraint Types** (40 min)
   - `r`: Any register (r0-r31)
   - `d`: Upper registers (r16-r31) for immediate ops
   - `a`: Register pairs (r16-r23 pairs)
   - `w`: r24:r25 pair
   - `x`, `y`, `z`: Pointer registers
   - `I`, `M`: Immediate constants

2. **Common Mistakes** (30 min)
   - Using `r` constraint with SUBI (needs `d`)
   - Wrong `I` range (0-63 only!)
   - Forgetting clobber list
   - Modifying input-only operand

3. **Numbered Constraints** (20 min)
   - `"0"`, `"1"` to reuse registers
   - Early clobber modifier: `&`
   - Commutative operations

**Demo: `demo_05_register_constraints()`**
- Demonstrate compile error with wrong constraint
- Show `d` constraint requirement for SUBI
- X/Y/Z pointer examples
- Immediate constraint limitations

**Lab Exercise 5 (2 hours):**
```
TITLE: "Constraint Challenge"

TASK: Fix broken inline assembly code with constraint errors.

PROVIDED: 10 buggy inline assembly snippets with constraint problems

BUGS TO FIND:
1. Using 'r' constraint with SUBI (should be 'd')
2. Missing clobber list
3. Wrong immediate constraint range
4. Modifying input-only operand
5. X pointer used incorrectly
6. Early clobber needed but missing
7. Wrong I/O address (forgot _SFR_IO_ADDR)
8. 16-bit value split incorrectly (%A0 / %B0)
9. Matching constraint used wrong
10. volatile keyword missing

LEARNING GOALS:
- Recognize constraint errors
- Understand compiler error messages
- Debug inline assembly code

DELIVERABLES:
- All 10 bugs fixed with explanations
- Document: "Common Constraint Mistakes Cheat Sheet"
- Create 5 NEW buggy examples for peer review
```

---

#### Day 6: Pointer Operations (X, Y, Z)
**Learning Objectives:**
- Use X/Y/Z registers for indirect addressing
- Implement array operations with pointers
- Load from flash memory with LPM

**Lecture Topics (90 minutes):**
1. **Pointer Registers** (30 min)
   - X: r26:r27 (general purpose)
   - Y: r28:r29 (also frame pointer)
   - Z: r30:r31 (used for LPM/ELPM)
   - When to use each

2. **Indirect Addressing Modes** (35 min)
   - `ld Rd, X`: Load from [X], no change
   - `ld Rd, X+`: Load, then X++ (post-increment)
   - `ld Rd, -X`: X--, then load (pre-decrement)
   - `ld Rd, Y+q`: Load from [Y+q], Y unchanged (displacement)

3. **Flash Memory Access** (25 min)
   - LPM: Load Program Memory
   - PROGMEM strings in flash
   - Z register for flash addressing
   - ELPM for >64KB flash

**Lab Exercise 6 (2 hours):**
```
TITLE: "Array Processing with Pointers"

TASK: Implement array algorithms using X/Y/Z pointers.

FUNCTIONS TO IMPLEMENT (inline assembly):
1. array_sum(array, length) → sum all elements
2. array_max(array, length) → find maximum value
3. array_reverse(array, length) → reverse in-place
4. array_copy(src, dest, length) → copy array
5. flash_strcmp(ram_str, flash_str) → compare RAM string with PROGMEM

LEARNING GOALS:
- Pointer register usage
- Post-increment/pre-decrement
- LPM for flash access
- Two-pointer algorithms

BONUS CHALLENGE:
- Implement bubble sort using pointers
- Use Y+q displacement addressing mode

DELIVERABLES:
- Array processing library
- Test suite with various array sizes
- Performance comparison: C pointers vs assembly pointers
- Documentation on pointer addressing modes
```

---

### WEEK 3: PERIPHERALS & INTERRUPTS

#### Day 7: Interrupt Service Routines with Assembly
**Learning Objectives:**
- Write ISRs with inline assembly
- Understand interrupt context saving
- Optimize ISR response time

**Lecture Topics (90 minutes):**
1. **Interrupt Mechanics** (25 min)
   - Hardware saves SREG and PC
   - ISR must save/restore used registers
   - RETI instruction
   - Interrupt latency components

2. **Assembly in ISRs** (40 min)
   - Why use assembly in ISR? (speed!)
   - Atomic operations with CLI/SEI
   - Accessing volatile variables
   - "memory" clobber for volatiles

3. **Optimization Techniques** (25 min)
   - Minimize ISR execution time
   - Use SBI/CBI for fast GPIO
   - Avoid function calls in ISR
   - Set flags vs processing in ISR

**Lab Exercise 7 (2 hours):**
```
TITLE: "Fast Interrupt Handler"

TASK: Implement INT0 interrupt handler with inline assembly.

REQUIREMENTS:
- External interrupt on INT0 (falling edge)
- Toggle LED on PORTB.0 (using assembly SBI/CBI)
- Increment counter (using assembly INC)
- Measure ISR execution time with oscilloscope
- Compare C version vs assembly version latency

LEARNING GOALS:
- ISR structure with inline assembly
- Atomic operations in interrupts
- Performance critical interrupt handling

BONUS CHALLENGE:
- Implement software debouncing in ISR (assembly)
- Use timer interrupt instead of external interrupt
- Create "naked" ISR (no automatic register saving)

DELIVERABLES:
- Fast ISR implementation
- Timing measurements (C vs assembly)
- Oscilloscope screenshots showing latency
- Document: "ISR Optimization Techniques"
```

---

#### Day 8: UART Communication with Assembly
**Learning Objectives:**
- Access UART registers with assembly
- Implement polling-based serial I/O
- Understand UDR, UCSRA, UCSRB

**Lecture Topics (90 minutes):**
1. **UART Register Overview** (30 min)
   - UDR: Data register (transmit/receive)
   - UCSRA: Status flags (UDRE, RXC, TXC)
   - UCSRB: Control register (RXEN, TXEN)
   - Baud rate registers (UBRR)

2. **Assembly UART Operations** (35 min)
   - Wait for UDRE flag before transmit
   - Wait for RXC flag before receive
   - LDS/STS for extended I/O access
   - Polling loops with SBIS/SBIC

3. **String Transmission** (25 min)
   - Character-by-character transmission
   - Null-terminated strings
   - PROGMEM strings with LPM

**Lab Exercise 8 (2 hours):**
```
TITLE: "Assembly UART Library"

TASK: Implement UART functions using inline assembly.

FUNCTIONS TO IMPLEMENT:
1. asm_uart_tx_byte(char c) → send one byte
2. asm_uart_rx_byte() → receive one byte (blocking)
3. asm_uart_tx_string(char *str) → send null-terminated string
4. asm_uart_tx_hex(uint8_t val) → send hex representation
5. asm_uart_rx_available() → check if data available

LEARNING GOALS:
- UART register manipulation
- Polling loops in assembly
- LDS/STS for extended I/O

BONUS CHALLENGE:
- Implement interrupt-driven UART RX
- Add circular buffer in assembly
- Optimize for minimum latency

DELIVERABLES:
- UART assembly library
- Test program (echo server)
- Performance comparison: C UART vs assembly UART
- Timing analysis with oscilloscope
```

---

#### Day 9: Timer Configuration with Assembly
**Learning Objectives:**
- Configure timers using assembly
- Understand TCCRx, TCNTx, OCRx registers
- Implement PWM in assembly

**Lecture Topics (90 minutes):**
1. **Timer Registers** (30 min)
   - TCCRx: Timer control registers
   - TCNTx: Counter value
   - OCRx: Output compare registers
   - TIMSKx: Interrupt masks

2. **Timer Modes** (35 min)
   - Normal mode (overflow)
   - CTC mode (clear on compare)
   - PWM modes (fast, phase-correct)
   - Prescaler configuration

3. **Assembly Timer Setup** (25 min)
   - Writing to control registers
   - Setting compare values
   - Enabling interrupts
   - Reading counter values

**Lab Exercise 9 (2 hours):**
```
TITLE: "PWM LED Dimmer in Assembly"

TASK: Create PWM-controlled LED brightness using inline assembly.

REQUIREMENTS:
- Timer0 in Fast PWM mode
- Duty cycle controlled by buttons
- Increase/decrease brightness smoothly
- All timer configuration in inline assembly
- PWM frequency: ~1kHz

LEARNING GOALS:
- Timer register manipulation
- PWM generation
- Smooth transitions

BONUS CHALLENGE:
- Implement breathing LED effect (sine wave PWM)
- Use Timer1 for 16-bit PWM
- Create RGB LED color mixer (3 PWM channels)

DELIVERABLES:
- PWM dimmer program
- Timer configuration document
- Oscilloscope screenshots (PWM waveforms)
- Button debouncing implementation
```

---

### WEEK 4: OPTIMIZATION & ADVANCED TOPICS

#### Day 10: ADC Reading with Assembly
**Learning Objectives:**
- Configure and trigger ADC with assembly
- Read conversion results
- Implement filtering in assembly

**Lecture Topics (90 minutes):**
1. **ADC Architecture** (25 min)
   - ADMUX: Channel and reference selection
   - ADCSRA: Control and status
   - ADCL/ADCH: 10-bit result registers
   - Prescaler for ADC clock

2. **ADC Operations** (40 min)
   - Start conversion (ADSC bit)
   - Wait for completion (ADIF flag)
   - Read 10-bit result (order matters!)
   - Channel switching

3. **Signal Processing** (25 min)
   - Moving average filter
   - Peak detection
   - Threshold comparisons

**Lab Exercise 10 (2 hours):**
```
TITLE: "Sensor Dashboard with Assembly ADC"

TASK: Read 4 analog sensors, display on LEDs and UART.

REQUIREMENTS:
- ADC configuration in inline assembly
- Read 4 channels (A0-A3)
- Convert to 8-bit range (0-255)
- Display on LEDs (current channel)
- Send values via UART
- Implement 8-point moving average filter in assembly

LEARNING GOALS:
- ADC register manipulation
- Multi-channel reading
- Real-time signal processing

BONUS CHALLENGE:
- Implement median filter instead of average
- Add auto-ranging feature
- Trigger alarm if value exceeds threshold

DELIVERABLES:
- Sensor dashboard program
- Filter algorithm documentation
- Accuracy analysis (filter vs raw)
- UART output format specification
```

---

#### Day 11: Optimization & Performance Comparison
**Learning Objectives:**
- Profile code to find hotspots
- Optimize critical sections with assembly
- Measure and compare performance

**Lecture Topics (90 minutes):**
1. **Profiling Techniques** (30 min)
   - Cycle counting manually
   - Timer-based profiling
   - Oscilloscope measurements
   - SimulIDE debugging tools

2. **Optimization Strategies** (35 min)
   - When to use assembly (criteria)
   - Instruction-level optimization
   - Loop unrolling
   - Using faster instructions (SWAP vs shifts)

3. **Benchmarking** (25 min)
   - Fair comparison setup
   - Compiler optimization levels (-O0, -O2, -O3)
   - Measuring improvement
   - Documentation

**Demo: `demo_11_optimization_comparison()`**
- C version of LED toggle loop
- Assembly version of same loop
- Cycle count comparison
- Disassembly analysis

**Lab Exercise 11 (2 hours):**
```
TITLE: "Optimization Challenge"

TASK: Optimize provided C code using inline assembly.

PROVIDED CODE:
- Slow matrix multiplication function
- Inefficient sorting algorithm
- Non-optimized string processing

REQUIREMENTS:
- Profile original C code (cycle count)
- Identify hotspots (most time-consuming parts)
- Rewrite hotspots in inline assembly
- Measure improvement (must be >20% faster)
- Maintain functionality (same results)

LEARNING GOALS:
- Code profiling skills
- Strategic optimization
- Performance measurement

DELIVERABLES:
- Optimized code with inline assembly
- Profiling report (before/after)
- Speedup percentage calculation
- Document: "When to Use Assembly" guidelines
```

---

#### Day 12: Advanced Techniques & Review
**Learning Objectives:**
- Explore assembly macros
- Understand naked functions
- Review all concepts
- Complete capstone project

**Lecture Topics (90 minutes):**
1. **Advanced Topics** (40 min)
   - Naked functions (`__attribute__((naked))`)
   - Assembly macros for reusable code
   - Extended inline assembly features
   - Mixing C and assembly files (.S files)

2. **Best Practices** (30 min)
   - When to use inline assembly vs C
   - Code maintainability concerns
   - Documentation standards
   - Debugging strategies

3. **Review & Q&A** (20 min)
   - Constraint types summary
   - Common pitfalls
   - Exam preparation
   - Project discussion

**Lab Exercise 12 (4 hours):**
```
TITLE: "CAPSTONE PROJECT: Real-Time Data Logger"

TASK: Design and implement a complete data logging system using inline assembly for critical sections.

SYSTEM REQUIREMENTS:
- Read 2 analog sensors (ADC) every 10ms
- Timestamp each reading (Timer-based RTC)
- Store 100 samples in circular buffer
- Stream data via UART on request
- Button interface for control (start/stop/dump)
- LED status indicators

INLINE ASSEMBLY REQUIREMENTS (minimum):
- ADC reading and conversion
- Circular buffer operations (write/read with wrap)
- Fast interrupt handlers (Timer, Button)
- UART transmission optimized
- LED status updates (SBI/CBI)

LEARNING GOALS:
- System integration
- Real-time constraints
- Strategic use of assembly for optimization
- Professional documentation

DELIVERABLES:
1. Complete working system (code + hardware)
2. System architecture document
3. Inline assembly justification report
4. Performance analysis (C baseline vs optimized)
5. User manual for system operation
6. Video demonstration
```

---

## ASSESSMENT FRAMEWORK

### Assessment Distribution
- **Homework (20%)**: Weekly programming assignments
- **Lab Exercises (30%)**: In-class hands-on activities
- **Quizzes (15%)**: Short concept checks (constraints, instructions)
- **Midterm Exam (15%)**: Week 2 comprehensive test
- **Final Project (20%)**: Capstone project (Week 4)

---

### Grading Rubric for Lab Exercises

#### Functionality (40 points)
- **40-36**: All requirements met, robust error handling, exceeds expectations
- **35-30**: All requirements met, minor bugs acceptable
- **29-24**: Most requirements met, some functionality missing
- **23-18**: Basic functionality, significant gaps
- **17-0**: Incomplete or non-functional

#### Assembly Code Quality (25 points)
- **25-23**: Optimal constraint usage, efficient instructions, excellent register usage
- **22-20**: Correct constraints, appropriate instructions, good register usage
- **19-17**: Mostly correct, some suboptimal choices
- **16-14**: Frequent constraint errors or inefficient code
- **13-0**: Poor assembly practices, many errors

#### Documentation (20 points)
- **20-18**: Comprehensive comments explaining ALL assembly instructions, excellent docs
- **17-15**: Good comments on key sections, adequate documentation
- **14-12**: Minimal comments, basic documentation
- **11-9**: Sparse comments, poor documentation
- **8-0**: No comments or documentation

#### Testing (15 points)
- **15-14**: Thorough test suite, edge cases covered, documented results
- **13-11**: Good test coverage, major cases tested
- **10-8**: Basic testing, some cases missing
- **7-5**: Minimal testing
- **4-0**: No testing or test results

---

### Example Quiz Questions

#### Quiz 1: Basic Syntax (Week 1)
1. What does the `volatile` keyword do in inline assembly? Why is it important?
2. Write inline assembly to set bit 3 of PORTB using SBI instruction.
3. Calculate: How many NOP instructions are needed for a 5µs delay @ 16MHz?
4. What is wrong with this code? `__asm__ ("out PORTB, %0" : : "r" (value));`
5. Explain the difference between SBI and OUT instructions.

#### Quiz 2: Constraints (Week 2)
1. Which constraint is required for SUBI instruction: `r` or `d`? Why?
2. What does the `"0"` constraint mean in input operands?
3. Give an example of when you MUST use the `x` constraint.
4. What is the range of the `I` constraint?
5. List all registers that can be used with the `d` constraint.

#### Quiz 3: Advanced Operations (Week 3)
1. Write inline assembly to add two 16-bit numbers with carry.
2. What is the purpose of the clobber list? Give 3 examples.
3. Explain why inline assembly is beneficial in ISRs.
4. How do you access UART data register (UDR) with inline assembly?
5. What does %A0 and %B0 mean for a 16-bit variable?

#### Quiz 4: Optimization (Week 4)
1. Name 3 situations where inline assembly is faster than C.
2. How many cycles does SWAP take? How does this compare to 4x LSL?
3. What is the risk of using inline assembly without proper clobber list?
4. Explain when you should NOT use inline assembly.
5. How do you verify inline assembly is actually faster than C?

---

## COMMON STUDENT MISTAKES & SOLUTIONS

### Mistake 1: Missing `volatile` Keyword
**Symptom**: Assembly code not executing, optimized away
**Cause**: Compiler removes code thinking it has no effect
**Solution**: ALWAYS use `__asm__ volatile()` for hardware operations

### Mistake 2: Wrong Constraint for Immediate Operations
**Symptom**: Compile error "impossible constraint in 'asm'"
**Cause**: Used `r` constraint with SUBI/ANDI/ORI (need `d`)
**Solution**: Use `d` constraint for r16-r31 when using immediate operands

### Mistake 3: Forgetting Clobber List
**Symptom**: Unpredictable behavior, wrong results
**Cause**: Compiler doesn't know r16 was modified
**Solution**: List ALL modified registers in clobber section

### Mistake 4: Wrong I/O Address
**Symptom**: SBI/CBI doesn't work, or wrong register accessed
**Cause**: Used PORTB instead of `_SFR_IO_ADDR(PORTB)`
**Solution**: Use `_SFR_IO_ADDR()` macro for I/O addresses

### Mistake 5: Incorrect 16-bit Access
**Symptom**: Wrong high/low byte values
**Cause**: Used %0 instead of %A0/%B0 for 16-bit variable
**Solution**: Use %A0 for low byte, %B0 for high byte

### Mistake 6: Input Operand Modified
**Symptom**: Compile warning or wrong results
**Cause**: Modified input operand declared with `"r"` instead of `"+r"`
**Solution**: Use `+` modifier for read-write operands

### Mistake 7: Immediate Constraint Out of Range
**Symptom**: Compile error with `I` constraint
**Cause**: Tried to use value >63 with `I` constraint (limited to 0-63)
**Solution**: Use `M` constraint (0-255) or load into register first

### Mistake 8: Pointer Register Confusion
**Symptom**: Segmentation fault or wrong memory access
**Cause**: Used X pointer but passed regular variable
**Solution**: Ensure pointer variable type matches constraint (uint8_t*)

---

## TROUBLESHOOTING GUIDE FOR TAs

### Issue: Student's Code Compiles But Doesn't Work

**Debugging Steps:**
1. Check `volatile` keyword present
2. Verify I/O addresses use `_SFR_IO_ADDR()`
3. Confirm clobber list includes all modified registers
4. Check constraint types match instruction requirements
5. Look for "memory" clobber if accessing volatile variables
6. Generate disassembly: `avr-objdump -d Main.elf | less`
7. Use SimulIDE step-through debugging

### Issue: Compiler Errors with Constraints

**Common Causes:**
- "impossible constraint": Wrong constraint for instruction (use `d` not `r`)
- "output operand constraint lacks '='": Missing = in output constraint
- "matching constraint references invalid operand": Wrong number in `"0"` etc.

**Solution Process:**
1. Identify which instruction causes error
2. Look up instruction in AVR manual (register requirements)
3. Match constraint to register requirement
4. Add `=` for outputs, `+` for read-write

### Issue: Performance Not Improving with Assembly

**Possible Reasons:**
1. Compiler already optimized C code well (-O3 flag)
2. Wrong part of code optimized (not the hotspot)
3. Assembly not actually better for this operation
4. Overhead of constraint handling negates benefit

**Solution:**
- Profile code first (find actual hotspot)
- Compare disassembly of C vs manual assembly
- Consider compiler optimization level
- Sometimes C is fine!

---

## ADDITIONAL TEACHING RESOURCES

### Recommended Reading
1. **"AVR Instruction Set Manual"** - Atmel/Microchip official documentation
2. **"GCC Inline Assembly Howto"** - Comprehensive constraint guide
3. **"Embedded C Programming" by Mark Siegesmund** - Chapters on optimization

### Online Resources
- AVR Freaks Forum: Community help for AVR assembly
- AVR-GCC Documentation: Inline assembly details
- godbolt.org/z/Compiler_Explorer: See assembly output online

### Video Demonstrations
Create or find videos showing:
- Oscilloscope measurement of assembly timing
- Step-by-step disassembly walkthrough
- Live debugging session with SimulIDE
- Constraint error resolution process

### Guest Lecture Ideas
- Industry engineer: When do you use assembly in production?
- Compiler developer: How does GCC handle inline assembly?
- Embedded systems architect: Optimization strategies

---

## LEARNING OUTCOMES ASSESSMENT

By end of module, students should score 80%+ on:

### Knowledge Check (Written Exam)
- Explain inline assembly syntax components
- Choose correct constraint for given instruction
- Identify errors in assembly code snippets
- Calculate instruction cycle counts
- Justify when to use assembly vs C

### Skills Check (Practical Exam)
- Write inline assembly to control hardware
- Debug constraint-related errors
- Optimize given C code with assembly
- Measure and compare performance
- Document assembly code clearly

### Application Check (Project)
- Integrate assembly into larger system
- Profile and optimize hotspots
- Make informed optimization decisions
- Present technical justification

---

## COURSE IMPROVEMENT FEEDBACK

### Student Survey Questions (End of Module)
1. How confident are you writing inline assembly? (1-5 scale)
2. Which lab exercise was most helpful? Why?
3. What topic needs more explanation time?
4. How useful were the flow diagrams?
5. Would you use inline assembly in future projects?

### Instructor Self-Assessment
- Did students meet learning outcomes?
- Which labs had most questions?
- Were demos clear and effective?
- How can assessment be improved?
- What topics need more/less time?

---

## APPENDIX: SUGGESTED IMPROVEMENTS FOR NEXT ITERATION

### Ideas to Consider:
1. **Add Video Tutorials**: Record screencast demos for each instruction
2. **Online Quiz Platform**: Automate grading for constraint questions
3. **Assembly Debugger Workshop**: Dedicated session on GDB + AVR
4. **Peer Code Review**: Students review each other's assembly code
5. **Industry Case Studies**: Real-world examples where assembly mattered
6. **Performance Competition**: Leaderboard for fastest implementations
7. **Assembly Golf Challenge**: Shortest code to solve problem
8. **Bare-metal ISR Module**: Advanced topic - naked ISRs

### Student Feedback from Previous Semesters:
- "More time on constraints needed" → Added Day 5 deep-dive
- "Want real-world examples" → Added optimization case studies
- "Flow diagrams very helpful" → Created 10 detailed diagrams
- "Hard to debug assembly" → Added disassembly workshop

---

## CONTACT & SUPPORT

**Instructor Office Hours:**
- Days/times for inline assembly help
- Book appointments for project consultation

**TA Support:**
- Lab sessions: In-person help with hardware
- Online forum: Post code questions
- Email: For administrative issues

**Additional Resources:**
- Course website: All slides, demos, references
- GitHub repository: Example code and solutions
- Discussion board: Student collaboration encouraged

---

**END OF DOCUMENTATION SUMMARY**

*This guide should be updated each semester based on student feedback and learning outcome assessment.*

---

## VERSION HISTORY
- **v1.0** (2025-01): Initial inline assembly module creation
- **v1.1** (TBD): Update based on first semester feedback
