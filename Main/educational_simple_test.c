/*
 * educational_simple_test.c - Simple Educational Framework Test
 * Tests basic functionality without complex dependencies
 */

#include "config.h"

#ifdef EDUCATIONAL_SIMPLE_TEST

/*
 * Simple LED and UART test using existing libraries
 */
void main_educational_simple_test(void)
{
    // Initialize systems using existing libraries
    init_devices();

    // Send startup message
    puts_USART1("\r\n=== ATmega128 Educational Framework Test ===\r\n");
    puts_USART1("Assembly → C → Python Learning Progression\r\n");
    puts_USART1("Type characters to test UART echo...\r\n");

    // Simple LED blink pattern to show system is working
    unsigned char led_pattern = 0x01;
    unsigned char pattern_count = 0;

    while (1)
    {
        // Check for UART input
        if (UCSR1A & (1 << RXC1))
        {
            char received = UDR1;

            // Echo the character
            while (!(UCSR1A & (1 << UDRE1)))
                ;
            UDR1 = received;

            // Update LED pattern on each character
            PORTB = ~led_pattern;                                  // LEDs are active LOW
            led_pattern = (led_pattern << 1) | (led_pattern >> 7); // Rotate pattern
        }

        // Slow LED animation when no input
        pattern_count++;
        if (pattern_count >= 200)
        {
            pattern_count = 0;
            PORTB = ~led_pattern;
            led_pattern = (led_pattern << 1) | (led_pattern >> 7);
        }

        _delay_ms(10);
    }
}

/*
 * Educational demonstration showing register access vs abstraction
 */
void demo_register_vs_abstraction(void)
{
    puts_USART1("\r\n--- Register Access vs Abstraction Demo ---\r\n");

    // Method 1: Direct register access (Assembly style)
    puts_USART1("Method 1: Direct Register Access\r\n");
    DDRB = 0xFF;  // Set PORTB as output
    PORTB = 0x00; // Turn all LEDs ON (active LOW)
    _delay_ms(1000);
    PORTB = 0xFF; // Turn all LEDs OFF

    // Method 2: Using existing library functions (C abstraction)
    puts_USART1("Method 2: C Function Abstraction\r\n");
    Port_init(); // Initialize using library function

    // Pattern sequence using bit manipulation
    unsigned char patterns[] = {0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF};

    for (unsigned char i = 0; i < 8; i++)
    {
        PORTB = ~patterns[i]; // Show pattern (LEDs active LOW)
        _delay_ms(200);
    }

    puts_USART1("Demo complete!\r\n");
}

#endif // EDUCATIONAL_SIMPLE_TEST