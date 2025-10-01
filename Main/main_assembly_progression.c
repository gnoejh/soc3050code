/*
 * main_assembly_progression.c - Educational Assembly → C Progression Example
 * Demonstrates the same functionality implemented at different abstraction levels
 *
 * LEARNING OBJECTIVES:
 * 1. Understand how C functions translate to assembly instructions
 * 2. See the progression from direct register access to function calls
 * 3. Compare assembly efficiency vs C readability
 * 4. Bridge low-level hardware control to high-level programming
 */

#include "config.h"

#ifdef ASSEMBLY_PROGRESSION_EXAMPLE

/*
 * ============================================================================
 * ASSEMBLY LEVEL IMPLEMENTATION
 * Direct register manipulation - students see exactly what the hardware does
 * ============================================================================
 */

/*
 * Assembly Level: LED Blink
 * Students learn: Direct register access, bit manipulation, timing loops
 */
void assembly_led_blink(void)
{
    // Configure PORTB as output (DDR = Data Direction Register)
    // Assembly: LDI R16, 0xFF; OUT DDRB, R16
    asm volatile(
        "ldi r16, 0xFF      \n\t" // Load immediate 0xFF into register 16
        "out %0, r16        \n\t" // Output to DDRB (make all pins output)
        :                         // No output operands
        : "I"(_SFR_IO_ADDR(DDRB)) // Input: DDRB register address
        : "r16"                   // Clobbered register
    );

    while (1)
    {
        // Turn LEDs ON (PORTB = 0x00 because LEDs are active LOW)
        // Assembly: LDI R16, 0x00; OUT PORTB, R16
        asm volatile(
            "ldi r16, 0x00      \n\t"  // Load 0x00 (LEDs ON)
            "out %0, r16        \n\t"  // Output to PORTB
            :                          // No output operands
            : "I"(_SFR_IO_ADDR(PORTB)) // Input: PORTB register address
            : "r16"                    // Clobbered register
        );

        // Delay loop (approximately 1 second at 16MHz)
        // Assembly: Simple loop with counter
        asm volatile(
            "ldi r24, 0x10      \n\t" // Outer loop counter (16 iterations)
            "outer_loop1:       \n\t"
            "ldi r25, 0xFF      \n\t" // Middle loop counter (255 iterations)
            "middle_loop1:      \n\t"
            "ldi r26, 0xFF      \n\t" // Inner loop counter (255 iterations)
            "inner_loop1:       \n\t"
            "dec r26            \n\t" // Decrement inner counter
            "brne inner_loop1   \n\t" // Branch if not zero
            "dec r25            \n\t" // Decrement middle counter
            "brne middle_loop1  \n\t" // Branch if not zero
            "dec r24            \n\t" // Decrement outer counter
            "brne outer_loop1   \n\t" // Branch if not zero
            :                         // No operands
            :                         // No operands
            : "r24", "r25", "r26"     // Clobbered registers
        );

        // Turn LEDs OFF (PORTB = 0xFF because LEDs are active LOW)
        // Assembly: LDI R16, 0xFF; OUT PORTB, R16
        asm volatile(
            "ldi r16, 0xFF      \n\t"  // Load 0xFF (LEDs OFF)
            "out %0, r16        \n\t"  // Output to PORTB
            :                          // No output operands
            : "I"(_SFR_IO_ADDR(PORTB)) // Input: PORTB register address
            : "r16"                    // Clobbered register
        );

        // Delay loop (second delay)
        asm volatile(
            "ldi r24, 0x10      \n\t" // Outer loop counter
            "outer_loop2:       \n\t"
            "ldi r25, 0xFF      \n\t" // Middle loop counter
            "middle_loop2:      \n\t"
            "ldi r26, 0xFF      \n\t" // Inner loop counter
            "inner_loop2:       \n\t"
            "dec r26            \n\t" // Decrement inner counter
            "brne inner_loop2   \n\t" // Branch if not zero
            "dec r25            \n\t" // Decrement middle counter
            "brne middle_loop2  \n\t" // Branch if not zero
            "dec r24            \n\t" // Decrement outer counter
            "brne outer_loop2   \n\t" // Branch if not zero
            :                         // No operands
            :                         // No operands
            : "r24", "r25", "r26"     // Clobbered registers
        );
    }
}

/*
 * ============================================================================
 * C REGISTER LEVEL IMPLEMENTATION
 * C code with direct register access - shows C syntax but same hardware control
 * ============================================================================
 */

/*
 * C Register Level: LED Blink
 * Students learn: C syntax for hardware control, still direct register access
 */
void c_register_led_blink(void)
{
    // Configure PORTB as output using C register access
    DDRB = 0xFF; // Equivalent to assembly LDI/OUT instructions

    while (1)
    {
        // Turn LEDs ON (active LOW configuration)
        PORTB = 0x00; // Equivalent to assembly LDI/OUT

        // Delay using C loops (compiler generates assembly)
        volatile unsigned int delay;
        for (delay = 0; delay < 65000; delay++)
        {
            // Compiler generates loop assembly similar to hand-written version
            // volatile prevents compiler optimization
        }

        // Turn LEDs OFF
        PORTB = 0xFF; // Equivalent to assembly LDI/OUT

        // Second delay
        for (delay = 0; delay < 65000; delay++)
        {
            // Second delay loop
        }
    }
}

/*
 * ============================================================================
 * C FUNCTION LEVEL IMPLEMENTATION
 * C code using library functions - abstraction begins
 * ============================================================================
 */

/*
 * C Function Level: LED Blink
 * Students learn: Function calls abstract hardware details
 */
void c_function_led_blink(void)
{
    // Initialize hardware using library function
    // Function call abstracts register configuration details
    Port_init(); // Initializes DDRB, DDRA, etc.

    while (1)
    {
        // Use library functions instead of direct register access
        LED_All_On();    // Function handles PORTB register
        _delay_ms(1000); // Library function for precise timing

        LED_All_Off();   // Function handles PORTB register
        _delay_ms(1000); // More accurate than manual loops
    }
}

/*
 * ============================================================================
 * C HIGH-LEVEL IMPLEMENTATION
 * Advanced C with patterns and logic - full abstraction
 * ============================================================================
 */

/*
 * C High-Level: LED Pattern Display
 * Students learn: Algorithms, data structures, advanced programming concepts
 */
void c_highlevel_led_patterns(void)
{
    // Pattern array - demonstrates data structures
    unsigned char patterns[] = {
        0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, // Chase left
        0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01, // Chase right
        0xFF, 0x00, 0xFF, 0x00,                         // Blink all
        0xAA, 0x55, 0xAA, 0x55                          // Alternate pattern
    };

    unsigned char pattern_count = sizeof(patterns) / sizeof(patterns[0]);
    unsigned char i;

    // Initialize using high-level function
    init_basic_io(); // Educational initialization function

    while (1)
    {
        // Loop through patterns using algorithm
        for (i = 0; i < pattern_count; i++)
        {
            LED_Set_Pattern(patterns[i]); // High-level pattern setting
            _delay_ms(200);               // Precise timing
        }
    }
}

/*
 * ============================================================================
 * MAIN FUNCTION - EDUCATIONAL PROGRESSION DEMONSTRATION
 * Students can uncomment different implementations to see progression
 * ============================================================================
 */

void main_assembly_progression(void)
{
    // PHASE 1: Pure Assembly (uncomment to try)
    // assembly_led_blink();            // Direct assembly instructions

    // PHASE 2: C with Register Access (uncomment to try)
    // c_register_led_blink();          // C syntax, direct registers

    // PHASE 3: C with Function Calls (uncomment to try)
    // c_function_led_blink();          // C functions, basic abstraction

    // PHASE 4: C High-Level Programming (default)
    c_highlevel_led_patterns(); // Algorithms, data structures
}

/*
 * ============================================================================
 * EDUCATIONAL ANALYSIS SECTION
 * Code size and performance comparison for student learning
 * ============================================================================
 */

/*
 * EDUCATIONAL NOTES FOR STUDENTS:
 *
 * 1. ASSEMBLY LEVEL:
 *    - Pros: Maximum control, minimal code size, precise timing
 *    - Cons: Hard to read, error-prone, not portable
 *    - Use: Performance-critical code, hardware drivers
 *
 * 2. C REGISTER LEVEL:
 *    - Pros: More readable than assembly, still efficient
 *    - Cons: Still hardware-specific, requires register knowledge
 *    - Use: Device drivers, embedded system programming
 *
 * 3. C FUNCTION LEVEL:
 *    - Pros: Readable, maintainable, less error-prone
 *    - Cons: Slight overhead, abstraction hides details
 *    - Use: Application programming, rapid development
 *
 * 4. C HIGH-LEVEL:
 *    - Pros: Algorithm focus, data structure usage, complex logic
 *    - Cons: Higher resource usage, further from hardware
 *    - Use: Complex applications, user interfaces, algorithms
 *
 * PROGRESSION LESSONS:
 * - Each level builds on the previous
 * - Higher levels trade efficiency for maintainability
 * - Understanding lower levels helps debug higher levels
 * - Real projects often use a mix of all levels
 */

/*
 * ASSEMBLY VS C COMPARISON EXERCISE:
 *
 * Students should:
 * 1. Compile each version and compare code size
 * 2. Measure execution timing differences
 * 3. Count lines of code for maintainability comparison
 * 4. Identify which approach suits different project requirements
 *
 * COMPILER EXPLORATION:
 * - Use 'objdump -d' to see assembly generated from C
 * - Compare hand-written vs compiler-generated assembly
 * - Understand optimization levels and their effects
 */

#endif // ASSEMBLY_PROGRESSION_EXAMPLE