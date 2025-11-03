/*
 * =============================================================================
 * PORT I/O PROGRAMMING - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: Port_Basic
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of ATmega128 port I/O operations in C language.
 * Students learn fundamental concepts of digital I/O programming and bit manipulation.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master port direction configuration (DDR registers)
 * 2. Learn output operations (PORT registers)
 * 3. Understand input operations (PIN registers)
 * 4. Practice bit manipulation techniques
 * 5. Compare C vs Assembly port operations
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - 8 LEDs connected to PORTB (PB0-PB7)
 * - Push button on PORTD.7 (active LOW with pull-up)
 * - Serial connection for debugging (9600 baud)
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Basic LED Control
 * - Demo 2: LED Patterns and Animation
 * - Demo 3: Button Input Processing
 * - Demo 4: Combined Input/Output Operations
 * - Demo 5: Advanced Bit Manipulation
 *
 * =============================================================================
 */

#include "config.h"

#define DELAY_SHORT 100
#define DELAY_MEDIUM 250
#define DELAY_LONG 500

// ============================================================================
// Demo 1: Writing to Ports (C equivalent of OUT instruction)
// ============================================================================
/*
 * CONCEPT: Writing entire byte to port
 *
 * In C:        PORTB = 0xFF;
 * In Assembly: OUT PORTB, r16
 *
 * Both do the same thing - write a value to the port register
 */
void demo_01_write_port(void)
{
    // Configure PORTB as output
    DDRB = 0xFF; // All pins are outputs

    while (1)
    {
        // Turn all LEDs ON
        PORTB = 0xFF; // Binary: 11111111
        _delay_ms(DELAY_LONG);

        // Turn all LEDs OFF
        PORTB = 0x00; // Binary: 00000000
        _delay_ms(DELAY_LONG);

        // Alternating pattern
        PORTB = 0xAA; // Binary: 10101010
        _delay_ms(DELAY_LONG);

        // Opposite pattern
        PORTB = 0x55; // Binary: 01010101
        _delay_ms(DELAY_LONG);
    }
}

// ============================================================================
// Demo 2: Reading from Ports (C equivalent of IN instruction)
// ============================================================================
/*
 * CONCEPT: Reading entire byte from port
 *
 * In C:        value = PIND;
 * In Assembly: IN r16, PIND
 *
 * Both read the current state of input pins
 */
void demo_02_read_port(void)
{
    uint8_t input_value;

    // Configure PORTB as output (LEDs)
    DDRB = 0xFF;

    // Configure PORTD.7 as input with pull-up (button)
    DDRD &= ~(1 << 7); // Clear bit 7 = input
    PORTD |= (1 << 7); // Set bit 7 = enable pull-up

    while (1)
    {
        // Read the input port
        input_value = PIND;

        // Check if button is pressed (bit 7 will be 0 when pressed)
        if (input_value & 0x80) // If bit 7 is HIGH (not pressed)
        {
            PORTB = 0x0F; // Low pattern: 00001111
        }
        else // Bit 7 is LOW (pressed)
        {
            PORTB = 0xF0; // High pattern: 11110000
        }

        _delay_ms(10); // Debounce delay
    }
}

// ============================================================================
// Demo 3: Setting Individual Bits (C equivalent of SBI instruction)
// ============================================================================
/*
 * CONCEPT: Setting one bit without affecting others
 *
 * In C:        PORTB |= (1 << bit);
 * In Assembly: SBI PORTB, bit
 *
 * Assembly SBI is ATOMIC (can't be interrupted)
 * C version uses read-modify-write (can be interrupted)
 */
void demo_03_set_bits(void)
{
    // Configure PORTB as output
    DDRB = 0xFF;

    // Start with all LEDs OFF
    PORTB = 0x00;

    while (1)
    {
        // Turn ON LEDs one by one (set bits)
        PORTB |= (1 << 0); // Set bit 0
        _delay_ms(DELAY_MEDIUM);

        PORTB |= (1 << 1); // Set bit 1
        _delay_ms(DELAY_MEDIUM);

        PORTB |= (1 << 2); // Set bit 2
        _delay_ms(DELAY_MEDIUM);

        PORTB |= (1 << 3); // Set bit 3
        _delay_ms(DELAY_MEDIUM);

        PORTB |= (1 << 4); // Set bit 4
        _delay_ms(DELAY_MEDIUM);

        PORTB |= (1 << 5); // Set bit 5
        _delay_ms(DELAY_MEDIUM);

        PORTB |= (1 << 6); // Set bit 6
        _delay_ms(DELAY_MEDIUM);

        PORTB |= (1 << 7); // Set bit 7
        _delay_ms(DELAY_LONG);

        // Reset all to OFF
        PORTB = 0x00;
        _delay_ms(DELAY_LONG);
    }
}

// ============================================================================
// Demo 4: Clearing Individual Bits (C equivalent of CBI instruction)
// ============================================================================
/*
 * CONCEPT: Clearing one bit without affecting others
 *
 * In C:        PORTB &= ~(1 << bit);
 * In Assembly: CBI PORTB, bit
 *
 * Assembly CBI is ATOMIC (can't be interrupted)
 * C version uses read-modify-write (can be interrupted)
 */
void demo_04_clear_bits(void)
{
    // Configure PORTB as output
    DDRB = 0xFF;

    while (1)
    {
        // Start with all LEDs ON
        PORTB = 0xFF;
        _delay_ms(DELAY_LONG);

        // Turn OFF LEDs one by one (clear bits)
        PORTB &= ~(1 << 0); // Clear bit 0
        _delay_ms(DELAY_MEDIUM);

        PORTB &= ~(1 << 1); // Clear bit 1
        _delay_ms(DELAY_MEDIUM);

        PORTB &= ~(1 << 2); // Clear bit 2
        _delay_ms(DELAY_MEDIUM);

        PORTB &= ~(1 << 3); // Clear bit 3
        _delay_ms(DELAY_MEDIUM);

        PORTB &= ~(1 << 4); // Clear bit 4
        _delay_ms(DELAY_MEDIUM);

        PORTB &= ~(1 << 5); // Clear bit 5
        _delay_ms(DELAY_MEDIUM);

        PORTB &= ~(1 << 6); // Clear bit 6
        _delay_ms(DELAY_MEDIUM);

        PORTB &= ~(1 << 7); // Clear bit 7
        _delay_ms(DELAY_LONG);
    }
}

// ============================================================================
// Demo 5: Testing if Bit is Clear (C equivalent of SBIC instruction)
// ============================================================================
/*
 * CONCEPT: Check if a bit is 0 (clear)
 *
 * In C:        if (!(PIND & (1 << bit)))
 * In Assembly: SBIC PIND, bit
 *
 * Assembly SBIC skips next instruction if bit is clear
 * C version uses conditional if-statement
 */
void demo_05_test_bit_clear(void)
{
    // Configure PORTB as output (LEDs)
    DDRB = 0xFF;

    // Configure PORTD.7 as input with pull-up (button)
    DDRD &= ~(1 << 7); // Clear bit 7 = input
    PORTD |= (1 << 7); // Set bit 7 = pull-up

    while (1)
    {
        // Test if bit 7 is CLEAR (button pressed)
        if (!(PIND & (1 << 7))) // If bit 7 is 0
        {
            // Button IS pressed
            PORTB = 0xFF; // All LEDs ON
        }
        else
        {
            // Button NOT pressed
            PORTB = 0x00; // All LEDs OFF
        }

        _delay_ms(10);
    }
}

// ============================================================================
// Demo 6: Testing if Bit is Set (C equivalent of SBIS instruction)
// ============================================================================
/*
 * CONCEPT: Check if a bit is 1 (set)
 *
 * In C:        if (PIND & (1 << bit))
 * In Assembly: SBIS PIND, bit
 *
 * Assembly SBIS skips next instruction if bit is set
 * C version uses conditional if-statement
 */
void demo_06_test_bit_set(void)
{
    // Configure PORTB as output (LEDs)
    DDRB = 0xFF;

    // Configure PORTD.7 as input with pull-up (button)
    DDRD &= ~(1 << 7); // Clear bit 7 = input
    PORTD |= (1 << 7); // Set bit 7 = pull-up

    while (1)
    {
        // Test if bit 7 is SET (button NOT pressed)
        if (PIND & (1 << 7)) // If bit 7 is 1
        {
            // Button NOT pressed
            PORTB = 0xAA; // Alternating pattern: 10101010
        }
        else
        {
            // Button IS pressed
            PORTB = 0x55; // Opposite pattern: 01010101
        }

        _delay_ms(10);
    }
}

// ============================================================================
// Demo 7: Combined Example - All Concepts Together
// ============================================================================
/*
 * This demo combines all the concepts:
 * - Reading input (like IN)
 * - Writing output (like OUT)
 * - Setting bits (like SBI)
 * - Clearing bits (like CBI)
 * - Testing bits (like SBIC/SBIS)
 */
void demo_07_combined(void)
{
    // Setup
    DDRB = 0xFF;       // PORTB all outputs
    DDRD &= ~(1 << 7); // PORTD.7 input
    PORTD |= (1 << 7); // Enable pull-up

    PORTB = 0x00; // Start with LEDs off

    while (1)
    {
        // Test button state
        if (!(PIND & (1 << 7))) // If button pressed
        {
            // Count UP by setting bits one by one
            PORTB |= (1 << 0);
            _delay_ms(DELAY_SHORT);
            PORTB |= (1 << 1);
            _delay_ms(DELAY_SHORT);
            PORTB |= (1 << 2);
            _delay_ms(DELAY_SHORT);
            PORTB |= (1 << 3);
            _delay_ms(DELAY_SHORT);
            PORTB |= (1 << 4);
            _delay_ms(DELAY_SHORT);
            PORTB |= (1 << 5);
            _delay_ms(DELAY_SHORT);
            PORTB |= (1 << 6);
            _delay_ms(DELAY_SHORT);
            PORTB |= (1 << 7);
            _delay_ms(DELAY_MEDIUM);
        }
        else // Button not pressed
        {
            // Count DOWN by clearing bits one by one
            PORTB &= ~(1 << 7);
            _delay_ms(DELAY_SHORT);
            PORTB &= ~(1 << 6);
            _delay_ms(DELAY_SHORT);
            PORTB &= ~(1 << 5);
            _delay_ms(DELAY_SHORT);
            PORTB &= ~(1 << 4);
            _delay_ms(DELAY_SHORT);
            PORTB &= ~(1 << 3);
            _delay_ms(DELAY_SHORT);
            PORTB &= ~(1 << 2);
            _delay_ms(DELAY_SHORT);
            PORTB &= ~(1 << 1);
            _delay_ms(DELAY_SHORT);
            PORTB &= ~(1 << 0);
            _delay_ms(DELAY_MEDIUM);
        }
    }
}

// ============================================================================
// MAIN - Select Your Demo
// ============================================================================
/*
 * LEARNING PROGRESSION (C Language):
 *
 * 1. demo_01_write_port     - Write entire port (like OUT)
 * 2. demo_02_read_port      - Read entire port (like IN)
 * 3. demo_03_set_bits       - Set individual bits (like SBI)
 * 4. demo_04_clear_bits     - Clear individual bits (like CBI)
 * 5. demo_05_test_bit_clear - Test if bit is 0 (like SBIC)
 * 6. demo_06_test_bit_set   - Test if bit is 1 (like SBIS)
 * 7. demo_07_combined       - All concepts together
 *
 * After completing these C demos, move to Port_Assembly to see
 * how the same operations are done in assembly language!
 */
int main(void)
{
     demo_01_write_port(); // ← START HERE: Learn writing to ports
    // demo_02_read_port(); // Learn reading from ports
    // demo_03_set_bits();      // Learn setting bits
    // demo_04_clear_bits();    // Learn clearing bits
    // demo_05_test_bit_clear();// Learn testing if bit is clear
    // demo_06_test_bit_set();  // Learn testing if bit is set
    // demo_07_combined(); // See everything together

    return 0;
}

/*
 * ============================================================================
 * COMPARISON: C vs Assembly
 * ============================================================================
 *
 * OPERATION          C CODE                    ASSEMBLY
 * ---------          ------                    --------
 * Write port         PORTB = 0xFF;             OUT PORTB, r16
 * Read port          val = PIND;               IN r16, PIND
 * Set bit            PORTB |= (1<<3);          SBI PORTB, 3
 * Clear bit          PORTB &= ~(1<<3);         CBI PORTB, 3
 * Test bit clear     if(!(PIND & (1<<7)))      SBIC PIND, 7
 * Test bit set       if(PIND & (1<<7))         SBIS PIND, 7
 *
 * KEY DIFFERENCES:
 * 1. Assembly SBI/CBI are ATOMIC (can't be interrupted)
 *    C bit operations use read-modify-write (can be interrupted)
 *
 * 2. Assembly SBIC/SBIS skip instructions (no branching)
 *    C uses if-statements (requires branching)
 *
 * 3. Assembly is typically faster (fewer cycles)
 *    C is more readable and portable
 *
 * WHEN TO USE ASSEMBLY:
 * - Interrupt service routines (need atomic operations)
 * - Time-critical code (need exact cycle counts)
 * - Bit manipulation where atomicity matters
 *
 * WHEN TO USE C:
 * - Most application code
 * - Complex logic and algorithms
 * - Code that needs to be portable
 * - Rapid development
 *
 * ============================================================================
 */
