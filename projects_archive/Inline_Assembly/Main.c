/*
 * Inline Assembly - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Understand inline assembly syntax and integration
 * - Learn register manipulation and constraints
 * - Practice optimization with assembly code
 * - Master mixing C and assembly programming
 *
 * HARDWARE SETUP:
 * - LEDs on PORTB for assembly operation visualization
 * - UART for assembly execution reports
 * - Optional: Logic analyzer for timing measurements
 */

#include "config.h"

// Global variables for inline assembly demonstrations
volatile uint8_t asm_counter = 0;
volatile uint16_t timing_result = 0;

// Function demonstrating basic inline assembly
void inline_assembly_basic()
{
    puts_USART1("=== Basic Inline Assembly Demo ===\r\n");

    uint8_t input_val = 0x55;
    uint8_t output_val;

    // Example 1: Simple register operations
    puts_USART1("Example 1: Register manipulation\r\n");

    asm volatile(
        "mov %0, %1\n\t"   // Move input to output
        "com %0\n\t"       // Complement (invert) all bits
        : "=r"(output_val) // Output operand
        : "r"(input_val)   // Input operand
        :                  // No clobbered registers
    );

    char buffer[60];
    sprintf(buffer, "Input: 0x%02X, Complemented: 0x%02X\r\n", input_val, output_val);
    puts_USART1(buffer);

    // Example 2: Bit manipulation with inline assembly
    puts_USART1("Example 2: Bit manipulation\r\n");

    uint8_t bit_test = 0x01;

    asm volatile(
        "lsl %0\n\t"     // Logical shift left
        "lsl %0\n\t"     // Shift again
        "lsl %0\n\t"     // And again (now bit 3 is set)
        : "=r"(bit_test) // Output/input operand
        : "0"(bit_test)  // Use same register
        :);

    sprintf(buffer, "After 3 left shifts: 0x%02X\r\n", bit_test);
    puts_USART1(buffer);
}

// Function demonstrating memory operations with inline assembly
void inline_assembly_memory()
{
    puts_USART1("=== Memory Operations with Inline Assembly ===\r\n");

    // Example 3: Direct port manipulation
    puts_USART1("Example 3: Direct port access\r\n");

    asm volatile(
        "ldi r16, 0xFF\n\t"       // Load immediate 0xFF into r16
        "out %0, r16\n\t"         // Output to DDRB (set as outputs)
        :                         // No outputs
        : "I"(_SFR_IO_ADDR(DDRB)) // Input: DDRB address
        : "r16"                   // Clobbered register
    );

    puts_USART1("PORTB configured as output using inline assembly\r\n");

    // Example 4: LED pattern with assembly timing
    puts_USART1("Example 4: LED pattern with precise timing\r\n");

    for (uint8_t pattern = 0; pattern < 8; pattern++)
    {
        asm volatile(
            "ldi r16, %1\n\t" // Load pattern into r16
            "out %0, r16\n\t" // Output to PORTB

            // Precise delay loop in assembly
            "ldi r17, 100\n\t" // Outer loop counter
            "delay_outer_%=:\n\t"
            "ldi r18, 255\n\t" // Inner loop counter
            "delay_inner_%=:\n\t"
            "dec r18\n\t"             // Decrement inner counter
            "brne delay_inner_%=\n\t" // Branch if not zero
            "dec r17\n\t"             // Decrement outer counter
            "brne delay_outer_%=\n\t" // Branch if not zero

            : // No outputs
            : "I"(_SFR_IO_ADDR(PORTB)), "M"(~(1 << pattern))
            : "r16", "r17", "r18" // Clobbered registers
        );

        sprintf(buffer, "Pattern %u displayed\r\n", pattern);
        puts_USART1(buffer);
    }
}

// Function demonstrating arithmetic operations in assembly
void inline_assembly_arithmetic()
{
    puts_USART1("=== Arithmetic Operations in Inline Assembly ===\r\n");

    uint8_t a = 15, b = 7;
    uint8_t sum, difference, product;
    uint16_t wide_product;

    // Example 5: Addition and subtraction
    puts_USART1("Example 5: Basic arithmetic\r\n");

    asm volatile(
        "add %0, %2\n\t" // Add a + b
        "mov %1, %3\n\t" // Copy a to difference
        "sub %1, %2\n\t" // Subtract b from a
        : "=r"(sum), "=r"(difference)
        : "r"(b), "r"(a)
        :);

    char buffer[80];
    sprintf(buffer, "%u + %u = %u, %u - %u = %u\r\n", a, b, sum, a, b, difference);
    puts_USART1(buffer);

    // Example 6: Multiplication (8-bit result)
    puts_USART1("Example 6: Multiplication\r\n");

    asm volatile(
        "mul %1, %2\n\t" // Multiply a * b (result in r1:r0)
        "mov %0, r0\n\t" // Get low byte of result
        "clr r1\n\t"     // Clear r1 (mul uses it)
        : "=r"(product)
        : "r"(a), "r"(b)
        : "r0", "r1");

    sprintf(buffer, "%u * %u = %u (8-bit result)\r\n", a, b, product);
    puts_USART1(buffer);

    // Example 7: 16-bit multiplication result
    puts_USART1("Example 7: 16-bit multiplication\r\n");

    asm volatile(
        "mul %A1, %A2\n\t" // Multiply low bytes
        "movw %0, r0\n\t"  // Move r1:r0 to result
        "clr r1\n\t"       // Clear r1
        : "=r"(wide_product)
        : "r"(a), "r"(b)
        : "r0", "r1");

    sprintf(buffer, "%u * %u = %u (16-bit result)\r\n", a, b, wide_product);
    puts_USART1(buffer);
}

// Function demonstrating interrupt-safe operations
void inline_assembly_interrupts()
{
    puts_USART1("=== Interrupt-Safe Operations ===\r\n");

    uint16_t critical_data = 0x1234;
    uint8_t sreg_backup;

    // Example 8: Atomic operations
    puts_USART1("Example 8: Atomic 16-bit operation\r\n");

    asm volatile(
        "in %0, __SREG__\n\t" // Save status register
        "cli\n\t"             // Disable interrupts

        "ldi r16, 0x78\n\t" // Load new value
        "ldi r17, 0x56\n\t"
        "movw %1, r16\n\t" // Store atomically

        "out __SREG__, %0\n\t" // Restore status register
        : "=&r"(sreg_backup), "=r"(critical_data)
        :
        : "r16", "r17");

    char buffer[50];
    sprintf(buffer, "Atomic update completed: 0x%04X\r\n", critical_data);
    puts_USART1(buffer);
}

// Function demonstrating performance optimization
void inline_assembly_optimization()
{
    puts_USART1("=== Performance Optimization Examples ===\r\n");

    // Example 9: Fast bit counting (population count)
    puts_USART1("Example 9: Fast bit counting\r\n");

    uint8_t test_value = 0b10110101; // Has 5 bits set
    uint8_t bit_count = 0;

    asm volatile(
        "mov r16, %1\n\t" // Copy input value
        "clr %0\n\t"      // Clear bit counter

        "count_loop_%=:\n\t"
        "tst r16\n\t"            // Test if zero
        "breq count_done_%=\n\t" // Branch if zero

        "inc %0\n\t"             // Increment counter
        "mov r17, r16\n\t"       // Copy current value
        "dec r17\n\t"            // Subtract 1
        "and r16, r17\n\t"       // Clear lowest set bit
        "rjmp count_loop_%=\n\t" // Continue loop

        "count_done_%=:\n\t"
        : "=r"(bit_count)
        : "r"(test_value)
        : "r16", "r17");

    char buffer[60];
    sprintf(buffer, "Value 0x%02X has %u bits set\r\n", test_value, bit_count);
    puts_USART1(buffer);

    // Example 10: Fast division by power of 2
    puts_USART1("Example 10: Fast division optimization\r\n");

    uint8_t dividend = 100;
    uint8_t quotient;

    asm volatile(
        "lsr %0\n\t" // Divide by 2 (shift right)
        "lsr %0\n\t" // Divide by 4
        "lsr %0\n\t" // Divide by 8
        : "=r"(quotient)
        : "0"(dividend)
        :);

    sprintf(buffer, "%u / 8 = %u (using bit shifts)\r\n", dividend, quotient);
    puts_USART1(buffer);
}

int main(void)
{
    // Initialize system components
    init_devices(); // Initialize all peripherals

    // Initialize UART for assembly reports
    Uart1_init();
    puts_USART1("Inline Assembly Demo Started\r\n");
    puts_USART1("Educational C/Assembly Integration Laboratory\r\n");
    puts_USART1("Demonstrating inline assembly programming techniques\r\n");

    // Configure LED output for assembly demonstrations
    DDRB = 0xFF;  // Set PORTB as output for LEDs
    PORTB = 0xFF; // Turn off all LEDs initially

    uint8_t demo_cycle = 0;

    while (1)
    {
        demo_cycle++;

        char buffer[50];
        sprintf(buffer, "\r\n>>> Assembly Demo Cycle #%u <<<\r\n", demo_cycle);
        puts_USART1(buffer);

        // Run different inline assembly demonstrations
        switch (demo_cycle % 5)
        {
        case 1:
            inline_assembly_basic();
            break;

        case 2:
            inline_assembly_memory();
            break;

        case 3:
            inline_assembly_arithmetic();
            break;

        case 4:
            inline_assembly_interrupts();
            break;

        case 0:
            inline_assembly_optimization();
            break;
        }

        // Visual completion indicator
        puts_USART1("Demo completed - LED flash sequence\r\n");
        for (uint8_t i = 0; i < 3; i++)
        {
            PORTB = 0x00; // All LEDs on
            _delay_ms(200);
            PORTB = 0xFF; // All LEDs off
            _delay_ms(200);
        }

        // Wait before next demo cycle
        puts_USART1("Waiting 5 seconds before next demo...\r\n");
        _delay_ms(5000);
    }

    return 0;
}