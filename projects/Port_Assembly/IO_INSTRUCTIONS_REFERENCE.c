/*
 * ============================================================================
 * ESSENTIAL AVR I/O ASSEMBLY INSTRUCTIONS REFERENCE
 * ATmega128 Port Programming - Complete Instruction Set
 * ============================================================================
 *
 * This document provides examples of ALL essential I/O instructions for
 * AVR port programming. Add these demos to your Port_Assembly project.
 *
 * CRITICAL I/O INSTRUCTIONS:
 * 1. IN/OUT   - Read from / Write to I/O space
 * 2. SBI/CBI  - Set Bit / Clear Bit in I/O (atomic, fast)
 * 3. SBIS/SBIC - Skip if Bit is Set / Skip if Bit is Clear
 * 4. SBRS/SBRC - Skip if Bit in Register is Set/Clear
 * ============================================================================
 */

// Add this demo to Port_Assembly/Main.c after demo_05

// ============================================================================
// DEMO: SBI/CBI - Atomic Bit Manipulation (ESSENTIAL!)
// ============================================================================
/*
 * CONCEPTS TAUGHT:
 * - SBI (Set Bit in I/O): Atomic single-bit set operation
 * - CBI (Clear Bit in I/O): Atomic single-bit clear operation
 * - Advantages: Atomic (no interruption), fast (1 cycle), no read-modify-write
 *
 * SYNTAX:
 * SBI ioaddr, bit  : Set bit 'bit' in I/O register at address 'ioaddr'
 * CBI ioaddr, bit  : Clear bit 'bit' in I/O register at address 'ioaddr'
 *
 * LIMITATIONS:
 * - Only works with I/O addresses 0x00-0x1F (first 32 I/O registers)
 * - Bit number must be a compile-time constant (0-7)
 * - PORTB (0x18), PORTC (0x15), PORTD (0x12) are within range
 *
 * WHY USE SBI/CBI?
 * - Faster than read-modify-write (1 cycle vs 3+ cycles)
 * - Atomic (no race conditions with interrupts)
 * - Clearer intent in code
 */
void demo_sbi_cbi_atomic_operations(void)
{
    // Configure PORTB as output
    asm volatile(
        "ldi r16, 0xFF       \n\t"
        "out %0, r16         \n\t"
        :
        : "I"(_SFR_IO_ADDR(DDRB))
        : "r16");

    // Start with all LEDs off
    asm volatile(
        "ldi r16, 0x00       \n\t"
        "out %0, r16         \n\t"
        :
        : "I"(_SFR_IO_ADDR(PORTB))
        : "r16");

    while (1)
    {
        // Turn ON LEDs one by one using SBI (Set Bit in I/O)
        // Note: Each SBI is compile-time constant bit number
        asm volatile(
            "sbi %0, 0           \n\t" // Set bit 0 - LED 0 ON
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_SHORT);

        asm volatile(
            "sbi %0, 1           \n\t" // Set bit 1 - LED 1 ON
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_SHORT);

        asm volatile(
            "sbi %0, 2           \n\t" // Set bit 2 - LED 2 ON
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_SHORT);

        asm volatile(
            "sbi %0, 3           \n\t" // Set bit 3 - LED 3 ON
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_SHORT);

        asm volatile(
            "sbi %0, 4           \n\t" // Set bit 4 - LED 4 ON
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_SHORT);

        asm volatile(
            "sbi %0, 5           \n\t" // Set bit 5 - LED 5 ON
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_SHORT);

        asm volatile(
            "sbi %0, 6           \n\t" // Set bit 6 - LED 6 ON
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_SHORT);

        asm volatile(
            "sbi %0, 7           \n\t" // Set bit 7 - LED 7 ON
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_MEDIUM);

        // Turn OFF LEDs one by one using CBI (Clear Bit in I/O)
        asm volatile(
            "cbi %0, 0           \n\t" // Clear bit 0 - LED 0 OFF
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_SHORT);

        asm volatile(
            "cbi %0, 1           \n\t" // Clear bit 1 - LED 1 OFF
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_SHORT);

        asm volatile(
            "cbi %0, 2           \n\t" // Clear bit 2 - LED 2 OFF
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_SHORT);

        asm volatile(
            "cbi %0, 3           \n\t" // Clear bit 3 - LED 3 OFF
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_SHORT);

        asm volatile(
            "cbi %0, 4           \n\t" // Clear bit 4 - LED 4 OFF
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_SHORT);

        asm volatile(
            "cbi %0, 5           \n\t" // Clear bit 5 - LED 5 OFF
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_SHORT);

        asm volatile(
            "cbi %0, 6           \n\t" // Clear bit 6 - LED 6 OFF
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_SHORT);

        asm volatile(
            "cbi %0, 7           \n\t" // Clear bit 7 - LED 7 OFF
            : : "I"(_SFR_IO_ADDR(PORTB)) :);
        _delay_ms(DELAY_MEDIUM);
    }
}

// ============================================================================
// DEMO: SBIS/SBIC - Skip if Bit is Set/Clear (ESSENTIAL!)
// ============================================================================
/*
 * CONCEPTS TAUGHT:
 * - SBIS (Skip if Bit in I/O is Set): Skip next instruction if bit is 1
 * - SBIC (Skip if Bit in I/O is Clear): Skip next instruction if bit is 0
 * - Conditional execution without branches
 * - Efficient bit testing for inputs
 *
 * SYNTAX:
 * SBIS ioaddr, bit : Skip next instruction if bit is SET (1)
 * SBIC ioaddr, bit : Skip next instruction if bit is CLEAR (0)
 *
 * COMMON USAGE:
 * - Button/switch reading without polling loops
 * - Fast conditional operations
 * - Interrupt flag checking
 *
 * EXAMPLE PATTERN:
 * SBIC PIND, 7     ; Skip next instruction if PD7 is CLEAR (button pressed)
 * RJMP no_press    ; Jump to no_press if button NOT pressed
 * ; ... button press code here ...
 * no_press:
 */
void demo_sbis_sbic_conditional_skip(void)
{
    // Configure PORTB as output
    asm volatile(
        "ldi r16, 0xFF       \n\t"
        "out %0, r16         \n\t"
        :
        : "I"(_SFR_IO_ADDR(DDRB))
        : "r16");

    // Configure PORTD.7 as input with pull-up
    asm volatile(
        "cbi %0, 7           \n\t" // Clear DDRD.7 (input)
        "sbi %1, 7           \n\t" // Set PORTD.7 (enable pull-up)
        :
        : "I"(_SFR_IO_ADDR(DDRD)),
          "I"(_SFR_IO_ADDR(PORTD))
        :);

    while (1)
    {
        // Method 1: Using SBIC to detect button press (active LOW)
        // If bit is CLEAR (0), skip the next instruction
        asm volatile(
            "sbic %0, 7          \n\t" // Skip next if PD7 is CLEAR (button pressed)
            "rjmp not_pressed    \n\t" // Button not pressed, jump away

            // Button IS pressed (this code executes)
            "ldi r16, 0xFF       \n\t" // All LEDs ON
            "out %1, r16         \n\t"
            "rjmp end_check      \n\t"

            "not_pressed:        \n\t"
            // Button not pressed
            "ldi r16, 0x00       \n\t" // All LEDs OFF
            "out %1, r16         \n\t"

            "end_check:          \n\t"
            :
            : "I"(_SFR_IO_ADDR(PIND)),
              "I"(_SFR_IO_ADDR(PORTB))
            : "r16");

        _delay_ms(10); // Small delay for stability
    }
}

// ============================================================================
// DEMO: SBRS/SBRC - Skip if Bit in Register (ESSENTIAL!)
// ============================================================================
/*
 * CONCEPTS TAUGHT:
 * - SBRS (Skip if Bit in Register is Set): Skip if bit in register is 1
 * - SBRC (Skip if Bit in Register is Clear): Skip if bit in register is 0
 * - Testing bits in general-purpose registers
 * - Useful after IN instruction
 *
 * SYNTAX:
 * SBRS Rr, bit : Skip next instruction if bit in register Rr is SET
 * SBRC Rr, bit : Skip next instruction if bit in register Rr is CLEAR
 *
 * DIFFERENCE FROM SBIS/SBIC:
 * - SBIS/SBIC: Test bits in I/O registers directly
 * - SBRS/SBRC: Test bits in general registers (r0-r31)
 * - Use SBRS/SBRC after reading I/O into a register with IN
 */
void demo_sbrs_sbrc_register_skip(void)
{
    // Configure PORTB as output
    asm volatile(
        "ldi r16, 0xFF       \n\t"
        "out %0, r16         \n\t"
        :
        : "I"(_SFR_IO_ADDR(DDRB))
        : "r16");

    // Configure PORTD.7 as input with pull-up
    asm volatile(
        "cbi %0, 7           \n\t"
        "sbi %1, 7           \n\t"
        :
        : "I"(_SFR_IO_ADDR(DDRD)),
          "I"(_SFR_IO_ADDR(PORTD))
        :);

    while (1)
    {
        // Read PIND into register, then test bit
        asm volatile(
            "in r17, %0          \n\t" // Read PIND into r17

            // Test bit 7 of r17 using SBRC (Skip if Bit in Register is Clear)
            "sbrc r17, 7         \n\t" // Skip next if bit 7 is CLEAR (button pressed)
            "rjmp button_not_pressed \n\t"

            // Button IS pressed
            "ldi r16, 0xAA       \n\t" // Alternating pattern
            "out %1, r16         \n\t"
            "rjmp done           \n\t"

            "button_not_pressed: \n\t"
            "ldi r16, 0x55       \n\t" // Opposite pattern
            "out %1, r16         \n\t"

            "done:               \n\t"
            :
            : "I"(_SFR_IO_ADDR(PIND)),
              "I"(_SFR_IO_ADDR(PORTB))
            : "r16", "r17");

        _delay_ms(50);
    }
}

// ============================================================================
// DEMO: COMPREHENSIVE I/O INSTRUCTION COMPARISON
// ============================================================================
/*
 * PERFORMANCE AND USAGE COMPARISON
 *
 * INSTRUCTION    CYCLES   USE CASE
 * -----------    ------   --------
 * IN             1        Read entire I/O register
 * OUT            1        Write entire I/O register
 * SBI            2        Set single bit atomically (I/O 0x00-0x1F)
 * CBI            2        Clear single bit atomically (I/O 0x00-0x1F)
 * SBIS           1/2/3    Skip if bit in I/O is set (conditional)
 * SBIC           1/2/3    Skip if bit in I/O is clear (conditional)
 * SBRS           1/2/3    Skip if bit in register is set
 * SBRC           1/2/3    Skip if bit in register is clear
 *
 * BEST PRACTICES:
 * 1. Use SBI/CBI for single-bit changes (faster, atomic)
 * 2. Use IN/OUT for multi-bit changes
 * 3. Use SBIS/SBIC for direct I/O bit testing
 * 4. Use SBRS/SBRC for register bit testing after IN
 * 5. Remember: SBI/CBI only work with I/O addresses 0x00-0x1F
 */
void demo_io_instruction_comparison(void)
{
    // Configure PORTB
    asm volatile(
        "ldi r16, 0xFF       \n\t"
        "out %0, r16         \n\t"
        :
        : "I"(_SFR_IO_ADDR(DDRB))
        : "r16");

    uint8_t counter = 0;

    while (1)
    {
        counter++;

        // Demo different methods to achieve same result

        // Method 1: Using OUT (write entire port)
        if (counter & 0x01)
        {
            asm volatile(
                "ldi r16, 0xFF       \n\t"
                "out %0, r16         \n\t"
                :
                : "I"(_SFR_IO_ADDR(PORTB))
                : "r16");
        }

        // Method 2: Using SBI/CBI (atomic bit operations)
        if (counter & 0x02)
        {
            asm volatile(
                "sbi %0, 0           \n\t"
                "sbi %0, 1           \n\t"
                "sbi %0, 2           \n\t"
                :
                : "I"(_SFR_IO_ADDR(PORTB))
                :);
        }
        else
        {
            asm volatile(
                "cbi %0, 0           \n\t"
                "cbi %0, 1           \n\t"
                "cbi %0, 2           \n\t"
                :
                : "I"(_SFR_IO_ADDR(PORTB))
                :);
        }

        // Method 3: Read-Modify-Write using IN/OUT
        asm volatile(
            "in r16, %0          \n\t" // Read current value
            "ori r16, 0x08       \n\t" // Set bit 3
            "out %0, r16         \n\t" // Write back
            :
            : "I"(_SFR_IO_ADDR(PORTB))
            : "r16");

        _delay_ms(DELAY_MEDIUM);
    }
}

// ============================================================================
// IMPORTANT NOTES FOR TEACHING
// ============================================================================
/*
 * 1. IN/OUT INSTRUCTIONS:
 *    - Work with I/O space (0x00-0x3F)
 *    - Very fast (1 cycle each)
 *    - Cannot be interrupted mid-instruction
 *
 * 2. SBI/CBI INSTRUCTIONS:
 *    - ATOMIC operations (cannot be interrupted)
 *    - Only for I/O addresses 0x00-0x1F
 *    - PORTB (0x18), PORTC (0x15), PORTD (0x12) are supported
 *    - Bit number must be compile-time constant
 *    - Perfect for interrupt-safe single-bit changes
 *
 * 3. SBIS/SBIC INSTRUCTIONS:
 *    - Skip 1 instruction if bit condition is true
 *    - Skip 2 instructions if skipped instruction is 2 words (like JMP)
 *    - Perfect for button polling without branches
 *    - Same address limitations as SBI/CBI (0x00-0x1F)
 *
 * 4. SBRS/SBRC INSTRUCTIONS:
 *    - Work with ANY general-purpose register (r0-r31)
 *    - Use after IN to test bits from I/O registers
 *    - No address limitations (works with all ports)
 *
 * 5. WHY THIS MATTERS:
 *    - ATmega128 has ports beyond 0x1F (like PORTE, PORTF, PORTG)
 *    - For those ports, must use IN/SBRS or IN/OUT, cannot use SBI/SBIS
 *    - Understanding limitations prevents bugs
 *
 * TEACHING ORDER:
 * 1. Start with IN/OUT (basic read/write)
 * 2. Show SBI/CBI for efficiency
 * 3. Demonstrate SBIS/SBIC for conditional logic
 * 4. Explain SBRS/SBRC for extended port support
 * 5. Compare performance and use cases
 */
