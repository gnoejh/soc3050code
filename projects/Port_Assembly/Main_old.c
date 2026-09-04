/*
 * ATmega128 Port Programming with Inline Assembly
 * Educational Framework for Learning Assembly Language Integration
 *
 * PURPOSE: Demonstrate various inline assembly techniques for port control
 *
 * LEARNING OBJECTIVES:
 * - Inline assembly syntax and constraints
 * - Direct register manipulation
 * - Assembly instruction usage
 * - C and assembly integration
 * - Performance optimization techniques
 *
 * HARDWARE SETUP:
 * - PORTB: 8 LEDs (active HIGH for clear visibility)
 * - ATmega128 @ 7.3728MHz
 *
 * ASSEMBLY TECHNIQUES COVERED:
 * 1. Direct address method
 * 2. SFR macro method
 * 3. Input/output constraints
 * 4. Clobber lists
 * 5. Bit manipulation
 * 6. Loop optimization
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// Educational constants for assembly learning
#define DDRB_ADDR 0x17  // DDRB I/O address for ATmega128
#define PORTB_ADDR 0x18 // PORTB I/O address for ATmega128
#define PINB_ADDR 0x16  // PINB I/O address for ATmega128

// Global variables for educational demonstrations
volatile uint16_t asm_counter = 0;
volatile uint8_t pattern_index = 0;

/*
 * Method 1: Direct Address Assembly
 * EDUCATIONAL FOCUS: Raw assembly with direct I/O addresses
 * CONCEPTS: Basic inline assembly, direct register access
 */
void demo_assembly_direct_address(void)
{
    // Configure PORTB as output using direct addresses
    asm volatile(
        "ldi r16, 0xFF       \n\t" // Load 0xFF (all output) into r16
        "out 0x17, r16       \n\t" // Output to DDRB (address 0x17)
        :                          // No output operands
        :                          // No input operands
        : "r16"                    // r16 is clobbered
    );

    while (1)
    {
        // Pattern 1: All LEDs ON (0xFF)
        asm volatile(
            "ldi r16, 0xFF       \n\t" // Load pattern
            "out 0x18, r16       \n\t" // Output to PORTB (address 0x18)
            :
            :
            : "r16");
        _delay_ms(500);

        // Pattern 2: Alternating pattern (0xAA)
        asm volatile(
            "ldi r16, 0xAA       \n\t" // Load alternating pattern
            "out 0x18, r16       \n\t" // Output to PORTB
            :
            :
            : "r16");
        _delay_ms(500);

        // Pattern 3: Opposite alternating (0x55)
        asm volatile(
            "ldi r16, 0x55       \n\t" // Load opposite pattern
            "out 0x18, r16       \n\t" // Output to PORTB
            :
            :
            : "r16");
        _delay_ms(500);

        // Pattern 4: All LEDs OFF (0x00)
        asm volatile(
            "ldi r16, 0x00       \n\t" // Load all off pattern
            "out 0x18, r16       \n\t" // Output to PORTB
            :
            :
            : "r16");
        _delay_ms(500);
    }
}

/*
 * Method 2: SFR Macro Assembly
 * EDUCATIONAL FOCUS: Portable assembly using AVR macros
 * CONCEPTS: SFR macros, input constraints, better portability
 */
void demo_assembly_sfr_macros(void)
{
    // Configure PORTB as output using SFR macros
    asm volatile(
        "ldi r16, 0xFF       \n\t" // Load output configuration
        "out %0, r16         \n\t" // Output using macro
        :                          // No outputs
        : "I"(_SFR_IO_ADDR(DDRB))  // Input: DDRB address via macro
        : "r16"                    // Clobber r16
    );

    uint8_t patterns[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
    uint8_t pattern_count = sizeof(patterns) / sizeof(patterns[0]);

    while (1)
    {
        // Walking LED pattern using assembly
        for (uint8_t i = 0; i < pattern_count; i++)
        {
            asm volatile(
                "out %0, %1          \n\t"  // Output pattern to PORTB
                :                           // No outputs
                : "I"(_SFR_IO_ADDR(PORTB)), // PORTB address
                  "r"(patterns[i])          // Pattern value
            );
            _delay_ms(200);
        }

        // Reverse walking pattern
        for (int8_t i = pattern_count - 1; i >= 0; i--)
        {
            asm volatile(
                "out %0, %1          \n\t" // Output pattern to PORTB
                :
                : "I"(_SFR_IO_ADDR(PORTB)),
                  "r"(patterns[i]));
            _delay_ms(200);
        }
    }
}

/*
 * Method 3: Input/Output Constraints
 * EDUCATIONAL FOCUS: Advanced constraint usage, register allocation
 * CONCEPTS: Input/output operands, register optimization
 */
void demo_assembly_constraints(void)
{
    uint8_t ddr_value = 0xFF;  // DDR configuration
    uint8_t port_value = 0x00; // Initial port value

    // Setup using input constraint
    asm volatile(
        "out %1, %0          \n\t" // Output ddr_value to DDRB
        :                          // No outputs
        : "r"(ddr_value),          // Input: register for DDR value
          "I"(_SFR_IO_ADDR(DDRB))  // Input: DDRB address
    );

    while (1)
    {
        // Increment pattern using assembly with input/output
        asm volatile(
            "inc %0              \n\t" // Increment port_value
            "out %1, %0          \n\t" // Output to PORTB
            : "+r"(port_value)         // Input/output: modify port_value
            : "I"(_SFR_IO_ADDR(PORTB)) // Input: PORTB address
        );
        _delay_ms(100);
    }
}

/*
 * Method 4: Bit Manipulation Assembly
 * EDUCATIONAL FOCUS: Individual bit control with assembly
 * CONCEPTS: Bit operations, masks, precise control
 */
void demo_assembly_bit_manipulation(void)
{
    // Configure PORTB as output
    asm volatile(
        "ldi r16, 0xFF       \n\t"
        "out %0, r16         \n\t"
        :
        : "I"(_SFR_IO_ADDR(DDRB))
        : "r16");

    uint8_t current_state = 0x00;

    while (1)
    {
        // Turn on LEDs one by one using bit manipulation
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            uint8_t bit_mask = (1 << bit);
            asm volatile(
                "or %0, %1           \n\t" // OR with bit mask
                "out %2, %0          \n\t" // Output to PORTB
                : "+r"(current_state)      // Input/output: current state
                : "r"(bit_mask),           // Input: bit mask in register
                  "I"(_SFR_IO_ADDR(PORTB)) // Input: PORTB address
            );
            _delay_ms(300);
        }

        _delay_ms(500);

        // Turn off LEDs one by one using bit manipulation
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            uint8_t inv_mask = ~(1 << bit);
            asm volatile(
                "and %0, %1          \n\t" // AND with inverted mask
                "out %2, %0          \n\t" // Output to PORTB
                : "+r"(current_state)      // Input/output: current state
                : "r"(inv_mask),           // Input: inverted bit mask
                  "I"(_SFR_IO_ADDR(PORTB)) // Input: PORTB address
            );
            _delay_ms(300);
        }

        _delay_ms(500);
    }
}

/*
 * Method 5: Advanced Assembly Techniques
 * EDUCATIONAL FOCUS: Complex assembly operations, optimization
 * CONCEPTS: Multiple instructions, register reuse, performance
 */
void demo_assembly_advanced(void)
{
    // Setup with multiple operations in single asm block
    asm volatile(
        "ldi r16, 0xFF       \n\t" // Load DDR value
        "out %0, r16         \n\t" // Configure DDRB
        "ldi r17, 0x00       \n\t" // Initialize counter
        "ldi r18, 0x01       \n\t" // Initialize bit position
        :                          // No outputs
        : "I"(_SFR_IO_ADDR(DDRB))  // Input: DDRB address
        : "r16", "r17", "r18"      // Multiple clobbered registers
    );

    while (1)
    {
        // Complex pattern generation using assembly
        for (uint8_t cycle = 0; cycle < 8; cycle++)
        {
            asm volatile(
                "mov r16, %1         \n\t"  // Copy cycle to r16
                "lsl r16             \n\t"  // Logical shift left
                "or r16, %1          \n\t"  // OR with original
                "com r16             \n\t"  // Complement (invert all bits)
                "out %0, r16         \n\t"  // Output to PORTB
                :                           // No outputs
                : "I"(_SFR_IO_ADDR(PORTB)), // PORTB address
                  "r"(cycle)                // Cycle value
                : "r16"                     // Clobber r16
            );
            _delay_ms(200);
        }

        // Breathing effect using assembly calculations
        for (uint8_t intensity = 0; intensity < 16; intensity++)
        {
            asm volatile(
                "mov r16, %1         \n\t" // Copy intensity
                "swap r16            \n\t" // Swap nibbles (multiply by 16)
                "or r16, %1          \n\t" // OR with original
                "out %0, r16         \n\t" // Output pattern
                :
                : "I"(_SFR_IO_ADDR(PORTB)),
                  "r"(intensity)
                : "r16");
            _delay_ms(50);
        }
    }
}

/*
 * Method 6: Assembly with Read-Modify-Write
 * EDUCATIONAL FOCUS: Reading ports, modifying, writing back
 * CONCEPTS: Input operations, data flow, feedback loops
 */
void demo_assembly_read_modify_write(void)
{
    // Configure PORTB as output
    asm volatile(
        "ldi r16, 0xFF       \n\t"
        "out %0, r16         \n\t"
        :
        : "I"(_SFR_IO_ADDR(DDRB))
        : "r16");

    // Initialize with a starting pattern
    asm volatile(
        "ldi r16, 0x81       \n\t" // Start with LEDs at ends
        "out %0, r16         \n\t"
        :
        : "I"(_SFR_IO_ADDR(PORTB))
        : "r16");

    while (1)
    {
        // Read current state, modify, and write back
        for (uint8_t step = 0; step < 20; step++)
        {
            asm volatile(
                "in r16, %0          \n\t" // Read current PORTB value
                "rol r16             \n\t" // Rotate left through carry
                "out %0, r16         \n\t" // Write back to PORTB
                :                          // No outputs
                : "I"(_SFR_IO_ADDR(PORTB)) // PORTB address
                : "r16"                    // Clobber r16
            );
            _delay_ms(150);
        }

        _delay_ms(300);

        // Opposite rotation
        for (uint8_t step = 0; step < 20; step++)
        {
            asm volatile(
                "in r16, %0          \n\t" // Read current PORTB value
                "ror r16             \n\t" // Rotate right through carry
                "out %0, r16         \n\t" // Write back to PORTB
                :
                : "I"(_SFR_IO_ADDR(PORTB))
                : "r16");
            _delay_ms(150);
        }

        _delay_ms(300);
    }
}

/*
 * Main function - Educational progression selector
 */
int main(void)
{
    // Choose assembly technique to demonstrate:

    // Level 1: Basic direct address method
     demo_assembly_direct_address();

    // Level 2: Portable SFR macro method
    // demo_assembly_sfr_macros();

    // Level 3: Input/output constraints
    // demo_assembly_constraints();

    // Level 4: Bit manipulation techniques
    //demo_assembly_bit_manipulation();

    // Level 5: Advanced assembly operations
    // demo_assembly_advanced();

    // Level 6: Read-modify-write operations
    // demo_assembly_read_modify_write();

    return 0;
}

/*
 * EDUCATIONAL REFERENCE:
 *
 * INLINE ASSEMBLY SYNTAX:
 * asm volatile (
 *     "instruction operands \n\t"
 *     : outputs
 *     : inputs
 *     : clobbers
 * );
 *
 * CONSTRAINT TYPES:
 * "r"  - General register
 * "I"  - 6-bit positive integer constant (for I/O addresses)
 * "M"  - 8-bit integer constant
 * "+"  - Input/output operand
 * "="  - Output-only operand
 *
 * COMMON AVR INSTRUCTIONS:
 * ldi  - Load immediate
 * out  - Output to I/O space
 * in   - Input from I/O space
 * ori  - OR with immediate
 * andi - AND with immediate
 * rol  - Rotate left through carry
 * ror  - Rotate right through carry
 * inc  - Increment
 * com  - One's complement
 *
 * LEARNING PROGRESSION:
 * 1. Start with direct address method
 * 2. Learn SFR macros for portability
 * 3. Master input/output constraints
 * 4. Practice bit manipulation
 * 5. Explore advanced techniques
 * 6. Understand read-modify-write patterns
 */