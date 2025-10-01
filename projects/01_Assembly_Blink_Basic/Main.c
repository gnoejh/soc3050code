/*
 * Assembly Blink Basic - Main Application
 *
 * PURPOSE: Demonstrate basic LED blinking using inline assembly
 * LEARNING OBJECTIVES:
 * - Direct register manipulation with inline assembly
 * - Understanding DDR (Data Direction Register) configuration
 * - Basic I/O port control
 * - Assembly instruction syntax in C
 *
 * HARDWARE SETUP:
 * - LEDs connected to PORTB pins
 * - ATmega128 running at 16MHz
 *
 * EDUCATIONAL PROGRESSION:
 * This is the first step in learning embedded programming:
 * Assembly → C Functions → Complex Applications
 */

#include "config.h"
#include <util/delay.h>
#include <stdint.h>

// Global variables
uint16_t seed = 12345; // Pseudo-random number generator seed

// Function prototypes
void main_blink_asm_address(void);
void main_blink_asm_macro(void);
void main_blink_asm_random_simple(void);
void main_blink_asm_random_delay(void);
uint8_t my_random(void);
void custom_delay_ms(uint16_t ms);

/*
 * EDUCATIONAL FUNCTION: Basic Assembly LED Blinking
 *
 * LEARNING FOCUS: Direct register access using inline assembly
 * ASSEMBLY CONCEPTS:
 * - ldi: Load immediate value into register
 * - out: Output register value to I/O port
 * - Register r16: General purpose working register
 * - Port addresses: DDRB=0x17, PORTB=0x18
 */

/* address version */
void main_blink_asm_address(void)
{
    // Set DDRB as output
    asm volatile(
        "ldi r16, 0xFF\n\t" // Load immediate value 0xFF into register r16
        "out 0x17, r16\n\t" // Output r16 to DDRB (Port B Data Direction Register, address 0x04)
    );

    while (1)
    {
        // Set PORTB = 0xAA
        asm volatile(
            "ldi r16, 0xAA\n\t" // Load 0xAA into register r16
            "out 0x18, r16\n\t" // Output r16 to PORTB (Port B Data Register, address 0x05)
        );

        _delay_ms(2000); // 1-second delay

        // Set PORTB = 0x55
        asm volatile(
            "ldi r16, 0x55\n\t" // Load 0x55 into register r16
            "out 0x18, r16\n\t" // Output r16 to PORTB
        );

        _delay_ms(1000); // 1-second delay
    }
}

void main_blink_asm_macro(void)
{
    // STEP 1: Configure PORTB as output using assembly
    // Educational note: DDR (Data Direction Register) controls pin direction
    // 0xFF = all pins as output (binary: 11111111)
    asm volatile(
        "ldi r16, 0xFF\n\t" // Load immediate value 0xFF into register r16
        "out 0x17, r16\n\t" // Output r16 to DDRB (Port B Data Direction Register, address 0x17)
        :                   // No output operands
        :                   // No input operands
        : "r16"             // Clobber list: r16 is modified, Compiler should not use r16 for other purposes
    );

    // STEP 2: Infinite loop with alternating LED patterns
    while (1)
    {
        // Pattern 1: 0xAA = 10101010 (alternating LEDs)
        asm volatile(
            "ldi r16, 0xAA\n\t" // Load 0xAA into register r16
            "out 0x18, r16\n\t" // Output r16 to PORTB (Port B Data Register, address 0x18)
            :                   // No output operands
            :                   // No input operands
            : "r16"             // Clobber list: r16 is modified
        );

        _delay_ms(1000); // 1-second delay - creates visible blinking

        // Pattern 2: 0x55 = 01010101 (opposite alternating LEDs)
        asm volatile(
            "ldi r16, 0x55\n\t" // Load 0x55 into register r16
            "out 0x18, r16\n\t" // Output r16 to PORTB
            :                   // No output operands
            :                   // No input operands
            : "r16"             // Clobber list: r16 is modified
        );

        _delay_ms(1000); // 1-second delay
    }
}

/* random version */
// Function to generate a pseudo-random number
uint8_t my_random(void)
{
    // Linear Congruential Generator (LCG) parameters
    seed = (seed * 1103515245 + 12345); // Update seed
    return (uint8_t)(seed & 0xFF);      // Return an 8-bit pseudo-random number
}

void main_blink_asm_random_simple(void)
{
    // Set DDRB as output using inline assembly
    asm volatile(
        "ldi r16, 0xFF\n\t"              // Load 0xFF into r16 (all bits high)
        "out %[ddrb], r16\n\t"           // Output r16 to DDRB (set PORTB as output)
        :                                /* no output */
        : [ddrb] "I"(_SFR_IO_ADDR(DDRB)) // map DDRB to its I/O address
        : "r16");

    while (1)
    {
        // Set PORTB with a random 8-bit value
        asm volatile(
            "out %[portb], %[rand_val]\n\t"     // Output random value directly to PORTB
            :                                   /* no output */
            : [portb] "I"(_SFR_IO_ADDR(PORTB)), // map PORTB to its I/O address
              [rand_val] "r"(my_random())       // Get a random 8-bit value for PORTB
        );

        // Delay for a random period between 100ms and 1000ms
        _delay_ms(1000); // Generate a random delay between 100ms and 1000ms
    }
}

/* port delay random */
// Custom delay function that delays for approximately the given number of milliseconds
void custom_delay_ms(uint16_t ms)
{
    while (ms--)
    {
        _delay_ms(1); // Delay 1 millisecond in each loop iteration
    }
}

void main_blink_asm_random_delay(void)
{
    // Set DDRB as output using inline assembly
    asm volatile(
        "ldi r16, 0xFF\n\t"              // Load 0xFF into r16 (all bits high)
        "out %[ddrb], r16\n\t"           // Output r16 to DDRB (set PORTB as output)
        :                                /* no output */
        : [ddrb] "I"(_SFR_IO_ADDR(DDRB)) // map DDRB to its I/O address
        : "r16");

    while (1)
    {
        // Set PORTB with a random 8-bit value
        asm volatile(
            "out %[portb], %[rand_val]\n\t"     // Output random value directly to PORTB
            :                                   /* no output */
            : [portb] "I"(_SFR_IO_ADDR(PORTB)), // map PORTB to its I/O address
              [rand_val] "r"(my_random())       // Get a random 8-bit value for PORTB
        );

        // Delay for a random period between 100ms and 1000ms using custom delay function
        custom_delay_ms(100 + (my_random() % 900)); // Generate a random delay between 100ms and 1000ms
    }
}

/*
 * MAIN APPLICATION ENTRY POINT
 */
int main(void)
{
    // Call the educational assembly blink function
    // main_blink_asm_macro();
    // main_blink_asm_address();
    // main_blink_asm_random_simple();
    main_blink_asm_random_delay();

    return 0; // Never reached due to infinite loop
}

/*
 * EDUCATIONAL NOTES:
 *
 * 1. ASSEMBLY SYNTAX:
 *    - "ldi r16, 0xFF" loads immediate value 0xFF into register 16
 *    - "out 0x17, r16" outputs register 16 to I/O address 0x17 (DDRB)
 *    - Addresses 0x17 (DDRB) and 0x18 (PORTB) are ATmega128 specific
 *
 * 2. LED PATTERNS:
 *    - 0xAA = 10101010 binary = LEDs on pins 1,3,5,7 ON, pins 0,2,4,6 OFF
 *    - 0x55 = 01010101 binary = LEDs on pins 0,2,4,6 ON, pins 1,3,5,7 OFF
 *
 * 3. PROGRESSION PATH:
 *    - Next: Learn C function equivalents (Port_init, Port_write)
 *    - Then: Complex patterns and user interaction
 *    - Finally: Sensor integration and communication
 */