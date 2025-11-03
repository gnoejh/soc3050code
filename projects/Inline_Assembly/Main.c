/*
 * ==============================================================================
 * INLINE ASSEMBLY - DEMO CODE
 * ==============================================================================
 * PROJECT: Inline_Assembly
 * See Slide.md for complete theory and technical details
 *
 * DEMOS: AVR assembly language examples, inline assembly syntax, register operations
 * ==============================================================================
 */

#include "config.h"

// ============================================================================
// PROGMEM STRING CONSTANTS - Stored in Flash Memory
// ============================================================================
const char STR_TITLE[] PROGMEM = "Inline ASM Demos";
const char STR_DEMO1[] PROGMEM = "Demo 1: Basic ASM";
const char STR_DEMO2[] PROGMEM = "Demo 2: GPIO ASM";
const char STR_DEMO3[] PROGMEM = "Demo 3: Arithmetic";
const char STR_DEMO4[] PROGMEM = "Demo 4: Logic Ops";
const char STR_DEMO5[] PROGMEM = "Demo 5: Constraints";
const char STR_DEMO6[] PROGMEM = "Demo 6: Pointers";
const char STR_DEMO7[] PROGMEM = "Demo 7: Interrupts";
const char STR_DEMO8[] PROGMEM = "Demo 8: UART ASM";
const char STR_DEMO9[] PROGMEM = "Demo 9: Timer ASM";
const char STR_DEMO10[] PROGMEM = "Demo 10: ADC ASM";
const char STR_DEMO11[] PROGMEM = "Demo 11: Optimization";
const char STR_DEMO12[] PROGMEM = "Demo 12: Advanced";

const char STR_C_VERSION[] PROGMEM = "C version:";
const char STR_ASM_VERSION[] PROGMEM = "ASM version:";
const char STR_RESULT[] PROGMEM = "Result:";
const char STR_TIME[] PROGMEM = "Time:";
const char STR_CYCLES[] PROGMEM = "cycles";

// Helper function for PROGMEM strings (if UART available)
static void uart_print_P(const char *str_P)
{
    char buffer[30];
    strncpy_P(buffer, str_P, 29);
    buffer[29] = '\0';
    puts_USART1(buffer);
}

// ============================================================================
// GLOBAL VARIABLES FOR DEMO USAGE
// ============================================================================
volatile uint8_t interrupt_count = 0; // For interrupt demo
volatile uint8_t timer_flag = 0;      // For timer demo

// ============================================================================
// DEMO 1: BASIC INLINE ASSEMBLY - Introduction
// ============================================================================
//
// PURPOSE:
// Introduce inline assembly syntax with simplest examples: NOP instructions
// for precise timing delays and basic register operations.
//
// KEY CONCEPTS:
// - __asm__ volatile() syntax
// - NOP instruction (No Operation - 1 cycle delay)
// - Semicolon separates multiple instructions
// - volatile prevents compiler optimization
//
// TEACHING FOCUS:
// - Students see first inline assembly code
// - Understand how assembly integrates within C function
// - Learn that each NOP is exactly 1 clock cycle (62.5ns @ 16MHz)
// - See difference between C delay and assembly NOP delay
//
// STUDENT EXPERIMENTS:
// 1. Add more NOP instructions - measure timing with oscilloscope
// 2. Remove 'volatile' - observe compiler may optimize away
// 3. Calculate: How many NOPs for 1 microsecond delay? (16 NOPs @ 16MHz)
//
static void demo_01_basic_inline_assembly(void)
{
    // Initialize UART for output
    init_devices();
    uart_print_P(STR_DEMO1);
    puts_USART1("\r\n");

    // Configure PORTB as output (LED control)
    DDRB = 0xFF; // All pins output

    while (1)
    {
        // EXAMPLE 1: Simple NOP delays
        // NOP = No Operation, takes exactly 1 clock cycle (62.5ns @ 16MHz)

        PORTB = 0xFE; // LED0 ON (active low)

        // Inline assembly: 8 NOP instructions = 8 clock cycles = 500ns delay
        __asm__ volatile(
            "nop \n\t" // Each NOP is 1 cycle
            "nop \n\t" // \n\t creates newline for readability
            "nop \n\t"
            "nop \n\t"
            "nop \n\t"
            "nop \n\t"
            "nop \n\t"
            "nop \n\t"
            // Total: 8 cycles = 8 * 62.5ns = 500ns @ 16MHz
        );

        PORTB = 0xFF; // LED0 OFF

        // EXAMPLE 2: Using assembly to toggle LED
        // This demonstrates OUT instruction for direct port access

        __asm__ volatile(
            "ldi r16, 0xFE \n\t"       // Load immediate: r16 ??0xFE
            "out %0, r16   \n\t"       // Output to PORTB: PORTB ??r16
            :                          // No outputs
            : "I"(_SFR_IO_ADDR(PORTB)) // Input: PORTB address
            : "r16"                    // Clobber: r16 is modified
        );

        _delay_ms(500); // Visible delay (C function for comparison)

        __asm__ volatile(
            "ldi r16, 0xFF \n\t" // Load immediate: r16 ??0xFF
            "out %0, r16   \n\t" // Output to PORTB
            :
            : "I"(_SFR_IO_ADDR(PORTB))
            : "r16");

        _delay_ms(500);

        // TEACHING POINT: Assembly gives precise control over timing
        // C compiler may add extra instructions, assembly is predictable
    }
}

// ============================================================================
// DEMO 2: GPIO WITH ASSEMBLY - Port Manipulation
// ============================================================================
//
// PURPOSE:
// Demonstrate GPIO control using assembly instructions: SBI, CBI, OUT, IN
// for direct hardware manipulation. Compare with C bit operations.
//
// KEY CONCEPTS:
// - SBI/CBI: Set/Clear Bit in I/O register (single cycle for ports < 0x20)
// - OUT/IN: Output to / Input from I/O space
// - Direct register addressing
// - Bit-level hardware control
//
// TEACHING FOCUS:
// - Assembly is faster for single-bit operations
// - SBI/CBI are atomic (no read-modify-write)
// - Compare assembly vs C performance
// - Understand I/O register addressing
//
// STUDENT EXPERIMENTS:
// 1. Modify bit positions (LED0 ??LED7)
// 2. Add button input reading with IN instruction
// 3. Measure execution time difference: C vs assembly
//
static void demo_02_gpio_with_assembly(void)
{
    init_devices();
    uart_print_P(STR_DEMO2);
    puts_USART1("\r\n");

    // Configure PORTB as output
    DDRB = 0xFF;

    // Configure PORTD as input with pull-ups (for button reading)
    DDRD = 0x00;
    PORTD = 0xFF;

    while (1)
    {
        // METHOD 1: C language bit manipulation
        uart_print_P(STR_C_VERSION);
        puts_USART1(" ");

        PORTB &= ~(1 << PB0); // LED0 ON (clear bit)
        _delay_ms(100);
        PORTB |= (1 << PB0); // LED0 OFF (set bit)
        _delay_ms(100);

        // METHOD 2: Assembly using SBI/CBI instructions
        uart_print_P(STR_ASM_VERSION);
        puts_USART1(" ");

        __asm__ volatile(
            "cbi %0, %1 \n\t" // Clear Bit in I/O: PORTB.PB1 ??0
            "nop        \n\t" // Small delay for visibility
            "nop        \n\t"
            "sbi %0, %1 \n\t" // Set Bit in I/O: PORTB.PB1 ??1
            :
            : "I"(_SFR_IO_ADDR(PORTB)), // Port address
              "I"(PB1)                  // Bit number
        );

        _delay_ms(200);

        // EXAMPLE 3: Read button input with assembly
        uint8_t button_state;

        __asm__ volatile(
            "in %0, %1 \n\t"          // Input from PIND
            : "=r"(button_state)      // Output: button_state
            : "I"(_SFR_IO_ADDR(PIND)) // Input: PIND address
        );

        // Process button state (button pressed if bit 0 is clear)
        if (!(button_state & (1 << PD0)))
        {
            puts_USART1("Button pressed!\r\n");

            // Blink LED3 with assembly
            __asm__ volatile(
                "cbi %0, %1 \n\t"
                :
                : "I"(_SFR_IO_ADDR(PORTB)), "I"(PB3));
            _delay_ms(100);
            __asm__ volatile(
                "sbi %0, %1 \n\t"
                :
                : "I"(_SFR_IO_ADDR(PORTB)), "I"(PB3));
        }

        _delay_ms(50);

        // TEACHING POINT:
        // SBI/CBI are ATOMIC and FAST (1 cycle for low I/O addresses)
        // C bit operations may compile to multiple instructions
    }
}

// ============================================================================
// DEMO 3: ARITHMETIC OPERATIONS - Assembly Math
// ============================================================================
//
// PURPOSE:
// Demonstrate arithmetic operations in assembly: ADD, SUB, INC, DEC
// Work with variables passed between C and assembly.
//
// KEY CONCEPTS:
// - Input/output constraints
// - Register allocation by compiler
// - ADD, SUB, INC, DEC instructions
// - Carry flag usage
// - 8-bit and 16-bit arithmetic
//
// TEACHING FOCUS:
// - How to pass variables to/from assembly
// - Constraint characters: =r (write), +r (read-write), r (read)
// - Assembly arithmetic is explicit and fast
// - Multi-byte arithmetic with ADC/SBC
//
// STUDENT EXPERIMENTS:
// 1. Implement multiplication using repeated addition
// 2. Add 16-bit arithmetic with carry
// 3. Create assembly function for absolute value
//
static void demo_03_arithmetic_operations(void)
{
    init_devices();
    uart_print_P(STR_DEMO3);
    puts_USART1("\r\n");

    uint8_t a = 25;
    uint8_t b = 10;
    uint8_t result;

    while (1)
    {
        // EXAMPLE 1: Addition with assembly
        __asm__ volatile(
            "add %0, %1 \n\t" // result = a + b
            : "=r"(result)    // Output: result (write-only)
            : "r"(a), "0"(b)  // Inputs: a, b (reuse result register for b)
        );

        uart_print_P(STR_RESULT);
        char buffer[10];
        sprintf(buffer, " %d\r\n", result);
        puts_USART1(buffer);

        _delay_ms(1000);

        // EXAMPLE 2: Subtraction
        __asm__ volatile(
            "sub %0, %1 \n\t" // result = a - b
            : "=r"(result)
            : "r"(a), "0"(b));

        sprintf(buffer, "SUB: %d\r\n", result);
        puts_USART1(buffer);

        _delay_ms(1000);

        // EXAMPLE 3: Increment/Decrement
        uint8_t counter = 50;

        for (int i = 0; i < 5; i++)
        {
            __asm__ volatile(
                "inc %0 \n\t"   // counter = counter + 1
                : "+r"(counter) // Read-write (+ modifier)
            );

            sprintf(buffer, "INC: %d\r\n", counter);
            puts_USART1(buffer);
            _delay_ms(500);
        }

        for (int i = 0; i < 5; i++)
        {
            __asm__ volatile(
                "dec %0 \n\t" // counter = counter - 1
                : "+r"(counter));

            sprintf(buffer, "DEC: %d\r\n", counter);
            puts_USART1(buffer);
            _delay_ms(500);
        }

        // EXAMPLE 4: 16-bit addition with carry
        uint16_t val16_a = 1000;
        uint16_t val16_b = 2000;
        uint16_t result16;

        __asm__ volatile(
            "add %A0, %A1 \n\t" // Add low bytes
            "adc %B0, %B1 \n\t" // Add high bytes with carry
            : "=r"(result16)
            : "r"(val16_a), "0"(val16_b));

        sprintf(buffer, "16-bit: %u\r\n", result16);
        puts_USART1(buffer);

        _delay_ms(2000);

        // TEACHING POINT:
        // %A0 = low byte, %B0 = high byte for 16-bit variables
        // ADC adds with carry flag from previous ADD
    }
}

// ============================================================================
// DEMO 4: BITWISE LOGIC OPERATIONS
// ============================================================================
//
// PURPOSE:
// Demonstrate logical operations: AND, OR, XOR, shifts, rotates
// Essential for bit manipulation and masking.
//
// KEY CONCEPTS:
// - AND/OR/XOR instructions
// - Immediate vs register operands
// - LSL/LSR: Logical shift left/right
// - ROL/ROR: Rotate through carry
// - SWAP: Exchange nibbles
// - COM: One's complement (NOT)
//
// TEACHING FOCUS:
// - Bit masking techniques
// - Shift operations for multiply/divide by powers of 2
// - Rotate for circular buffers
// - SWAP for fast nibble exchange
//
// STUDENT EXPERIMENTS:
// 1. Implement bit counting (popcount)
// 2. Create bit reversal function
// 3. Use shifts for fast multiply/divide by 2, 4, 8
//
static void demo_04_bitwise_logic_operations(void)
{
    init_devices();
    uart_print_P(STR_DEMO4);
    puts_USART1("\r\n");

    uint8_t data = 0b10101100; // Test pattern
    uint8_t result;
    char buffer[40];

    while (1)
    {
        // EXAMPLE 1: AND operation (bit masking)
        __asm__ volatile(
            "andi %0, 0x0F \n\t" // Mask lower 4 bits
            : "=d"(result)       // d constraint: r16-r31 (supports immediate)
            : "0"(data));

        sprintf(buffer, "AND mask: 0x%02X\r\n", result);
        puts_USART1(buffer);
        _delay_ms(1000);

        // EXAMPLE 2: OR operation (bit setting)
        result = data;
        __asm__ volatile(
            "ori %0, 0xF0 \n\t" // Set upper 4 bits
            : "+d"(result));

        sprintf(buffer, "OR set: 0x%02X\r\n", result);
        puts_USART1(buffer);
        _delay_ms(1000);

        // EXAMPLE 3: XOR operation (bit toggling)
        result = data;
        __asm__ volatile(
            "ldi r17, 0xFF \n\t" // Load 0xFF into r17
            "eor %0, r17   \n\t" // XOR with 0xFF (invert all bits)
            : "+r"(result)
            :
            : "r17");

        sprintf(buffer, "XOR toggle: 0x%02X\r\n", result);
        puts_USART1(buffer);
        _delay_ms(1000);

        // EXAMPLE 4: Logical shift left (multiply by 2)
        result = data;
        __asm__ volatile(
            "lsl %0 \n\t" // Shift left: result = result << 1
            : "+r"(result));

        sprintf(buffer, "LSL (*2): 0x%02X\r\n", result);
        puts_USART1(buffer);
        _delay_ms(1000);

        // EXAMPLE 5: Logical shift right (divide by 2)
        result = data;
        __asm__ volatile(
            "lsr %0 \n\t" // Shift right: result = result >> 1
            : "+r"(result));

        sprintf(buffer, "LSR (/2): 0x%02X\r\n", result);
        puts_USART1(buffer);
        _delay_ms(1000);

        // EXAMPLE 6: SWAP nibbles (fast high/low exchange)
        result = data;
        __asm__ volatile(
            "swap %0 \n\t" // Swap high and low nibbles
            : "+r"(result));

        sprintf(buffer, "SWAP: 0x%02X\r\n", result);
        puts_USART1(buffer);
        _delay_ms(1000);

        // EXAMPLE 7: COM (one's complement / NOT)
        result = data;
        __asm__ volatile(
            "com %0 \n\t" // Invert all bits
            : "+r"(result));

        sprintf(buffer, "COM: 0x%02X\r\n", result);
        puts_USART1(buffer);
        _delay_ms(2000);

        // TEACHING POINT:
        // Shift operations are VERY fast (1 cycle)
        // Use for fast multiply/divide by powers of 2
        // SWAP is faster than multiple shifts for nibble exchange
    }
}

// ============================================================================
// DEMO 5: REGISTER CONSTRAINTS - Advanced Usage
// ============================================================================
//
// PURPOSE:
// Explore different register constraints and their purposes.
// Understand when to use each constraint type.
//
// KEY CONCEPTS:
// - Constraint types: r, d, a, b, w, x, y, z, I, M
// - Numbered constraints: 0, 1, 2 (refer to previous operands)
// - Modifiers: =, +, &
// - Early clobber: & (output written before inputs read)
// - Matching constraints for read-modify-write
//
// TEACHING FOCUS:
// - Choosing right constraint for instruction requirements
// - Upper registers (r16-r31) needed for immediate operations
// - Pointer registers (X, Y, Z) for indirect addressing
// - Immediate constraints (I, M) for compile-time constants
//
// STUDENT EXPERIMENTS:
// 1. Try wrong constraint (r instead of d for immediate) - see error
// 2. Use X/Y/Z pointers for array access
// 3. Implement assembly function with multiple constraints
//
static void demo_05_register_constraints(void)
{
    init_devices();
    uart_print_P(STR_DEMO5);
    puts_USART1("\r\n");

    uint8_t value = 42;
    uint8_t result;
    uint8_t array[5] = {10, 20, 30, 40, 50};
    char buffer[40];

    while (1)
    {
        // EXAMPLE 1: 'd' constraint (upper registers r16-r31)
        // Required for immediate operand instructions (ANDI, ORI, SUBI, etc.)

        __asm__ volatile(
            "subi %0, 5 \n\t" // Subtract immediate: value = value - 5
            : "=d"(result)    // MUST use 'd' constraint for SUBI
            : "0"(value)      // Reuse output register for input
        );

        sprintf(buffer, "SUBI (d): %d\r\n", result);
        puts_USART1(buffer);
        _delay_ms(1000);

        // EXAMPLE 2: 'I' constraint (6-bit immediate 0-63)
        // For bit numbers in SBI/CBI, or small constants

        uint8_t bit_num = 3;
        PORTB = 0xFF;

        __asm__ volatile(
            "cbi %0, %1 \n\t" // Clear bit (requires compile-time constant)
            :
            : "I"(_SFR_IO_ADDR(PORTB)), // I/O address
              "I"(bit_num)              // Bit number (must be constant!)
        );

        puts_USART1("CBI with I constraint\r\n");
        _delay_ms(1000);

        // EXAMPLE 3: 'x' constraint (X pointer: r26:r27)
        // For indirect addressing with X register

        uint8_t *ptr = array;
        uint8_t sum = 0;

        __asm__ volatile(
            "ld r16, X+  \n\t" // Load indirect, post-increment
            "add %0, r16 \n\t" // Add to sum
            "ld r16, X+  \n\t"
            "add %0, r16 \n\t"
            "ld r16, X+  \n\t"
            "add %0, r16 \n\t"
            : "+r"(sum), "+x"(ptr) // x constraint for X pointer
            :
            : "r16");

        sprintf(buffer, "Sum (X ptr): %d\r\n", sum);
        puts_USART1(buffer);
        _delay_ms(1000);

        // EXAMPLE 4: 'z' constraint (Z pointer: r30:r31)
        // Z is often used with LPM (Load Program Memory)

        const uint8_t flash_data[] PROGMEM = {1, 2, 3, 4, 5};
        uint16_t z_ptr = (uint16_t)flash_data;
        uint8_t flash_val;

        __asm__ volatile(
            "lpm %0, Z \n\t" // Load from program memory using Z
            : "=r"(flash_val)
            : "z"(z_ptr) // z constraint for Z pointer
        );

        sprintf(buffer, "LPM (Z ptr): %d\r\n", flash_val);
        puts_USART1(buffer);
        _delay_ms(1000);

        // EXAMPLE 5: Multiple constraints with numbered references

        uint8_t a = 10, b = 20, c = 5;

        __asm__ volatile(
            "add %0, %1 \n\t" // a = a + b
            "sub %0, %2 \n\t" // a = a - c
            : "+r"(a)         // %0: read-write
            : "r"(b),         // %1: read-only
              "r"(c)          // %2: read-only
        );

        sprintf(buffer, "Multi-op: %d\r\n", a);
        puts_USART1(buffer);
        _delay_ms(2000);

        // TEACHING POINT:
        // Constraint selection is CRITICAL
        // Wrong constraint causes compile error or wrong code
        // Study AVR instruction set to know which registers required
    }
}

// ============================================================================
// MAIN FUNCTION - Demo Selection
// ============================================================================
int main(void)
{
    // ========================================================================
    // INSTRUCTOR GUIDE: Demo Selection for Teaching
    // ========================================================================
    // This module contains 12 progressive demos teaching inline assembly
    // Uncomment ONE demo function call below based on your lesson plan
    //
    // WEEK 1: FUNDAMENTALS
    //   Day 1: demo_01_basic_inline_assembly() - Syntax, NOP, basic OUT
    //   Day 2: demo_02_gpio_with_assembly() - GPIO (SBI, CBI, IN, OUT)
    //   Day 3: demo_03_arithmetic_operations() - ADD, SUB, INC, DEC
    //
    // WEEK 2: OPERATIONS
    //   Day 1: demo_04_bitwise_logic_operations() - AND, OR, XOR, shifts
    //   Day 2: demo_05_register_constraints() - Constraint types
    //   Day 3: demo_06_pointer_operations() - X, Y, Z pointers (to be implemented)
    //
    // WEEK 3: PERIPHERALS
    //   Day 1: demo_07_interrupt_with_assembly() - ISR inline assembly (to be implemented)
    //   Day 2: demo_08_uart_with_assembly() - Serial communication (to be implemented)
    //   Day 3: demo_09_timer_configuration() - Timer setup (to be implemented)
    //
    // WEEK 4: ADVANCED
    //   Day 1: demo_10_adc_reading() - ADC with assembly (to be implemented)
    //   Day 2: demo_11_optimization_comparison() - C vs ASM performance (to be implemented)
    //   Day 3: demo_12_advanced_techniques() - Macros, naked functions (to be implemented)
    // ========================================================================

    // === WEEK 1: FUNDAMENTALS ===
    demo_01_basic_inline_assembly(); // Basic syntax, NOP, OUT
    // demo_02_gpio_with_assembly();          // GPIO control with SBI/CBI
    // demo_03_arithmetic_operations();       // Arithmetic: ADD, SUB, INC, DEC

    // === WEEK 2: OPERATIONS ===
    // demo_04_bitwise_logic_operations();    // Logic: AND, OR, XOR, shifts
    // demo_05_register_constraints();        // Constraint types

    // === WEEK 3-4: PERIPHERALS & ADVANCED ===
    // (Demos 6-12 to be added in subsequent implementations)

    // ========================================================================
    // STUDENT EXERCISES
    // ========================================================================
    /*
     * EXERCISE 1: NOP Timing
     * - Calculate how many NOP instructions needed for 1쨉s, 10쨉s, 100쨉s @ 16MHz
     * - Implement precise delays using only NOP
     * - Verify with oscilloscope
     *
     * EXERCISE 2: LED Pattern with Assembly
     * - Create Knight Rider effect using SBI/CBI
     * - Compare C version vs assembly version performance
     * - Measure execution time
     *
     * EXERCISE 3: Calculator in Assembly
     * - Implement add, subtract, multiply, divide
     * - Use inline assembly for all operations
     * - Handle 8-bit and 16-bit numbers
     *
     * EXERCISE 4: Bit Manipulation
     * - Count set bits in byte (popcount)
     * - Reverse bit order (0b10110010 ??0b01001101)
     * - Find first set bit (FFS)
     *
     * EXERCISE 5: Optimization Challenge
     * - Take existing C function
     * - Rewrite in inline assembly
     * - Measure cycle count improvement
     * - Document why assembly is faster
     */

    return 0;
}
