/*
 * =============================================================================
 * TIMER/COUNTER PROGRAMMING - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: Timer_Basic
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of ATmega128 timer/counter operations and timing control.
 * Students learn precise timing generation and timer-based event management.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master timer/counter configuration and modes
 * 2. Learn prescaler settings for timing control
 * 3. Understand overflow and compare match operations
 * 4. Practice interrupt-driven timer programming
 * 5. Generate precise timing delays and frequencies
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - LEDs on PORTB for visual timing indication
 * - Optional: Oscilloscope on Timer2 output pin
 * - Serial connection for timing measurements (9600 baud)
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Basic Timer Configuration
 * - Demo 2: Precise Delay Generation
 * - Demo 3: Timer Overflow Interrupts
 * - Demo 4: Compare Match Operations
 * - Demo 5: Frequency Generation
 *
 * =============================================================================
 */

#include "config.h"

int main(void)
{
    // Initialize system components
    init_devices(); // Initialize all peripherals

    // Initialize Timer2 for precise timing
    Timer2_init(); // Configure Timer2

    // Initialize UART for timing reports
    Uart1_init();
    puts_USART1("Timer Basic Demo Started\r\n");
    puts_USART1("Demonstrating precise timing with Timer2\r\n");

    uint8_t led_state = 0;
    uint16_t seconds_counter = 0;
    char buffer[50];

    while (1)
    {
        // Wait for timer overflow (approximately 1 second)
        // Timer2 with prescaler creates regular intervals

        // Method 1: Polling timer overflow flag
        if (TIFR & (1 << TOV2)) // Timer2 overflow flag
        {
            TIFR |= (1 << TOV2); // Clear overflow flag

            // Toggle LED state every second
            led_state = !led_state;

            if (led_state)
            {
                PORTB = 0x00; // Turn on all LEDs (active LOW)
            }
            else
            {
                PORTB = 0xFF; // Turn off all LEDs
            }

            // Send timing report
            seconds_counter++;
            sprintf(buffer, "Timer tick %u - LEDs %s\r\n",
                    seconds_counter, led_state ? "ON" : "OFF");
            puts_USART1(buffer);
        }

        // Method 2: Demonstrate different timing patterns
        // Fast blink pattern every 10 seconds
        if (seconds_counter % 10 == 0 && seconds_counter > 0)
        {
            puts_USART1("Fast blink sequence...\r\n");

            for (uint8_t i = 0; i < 5; i++)
            {
                PORTB = 0x00; // LEDs ON
                _delay_ms(100);
                PORTB = 0xFF; // LEDs OFF
                _delay_ms(100);
            }

            puts_USART1("Returning to normal timing\r\n");
            seconds_counter++; // Prevent immediate repeat
        }

        // Additional timer demonstrations
        // Check timer value for sub-second timing
        uint8_t timer_value = TCNT2;

        // Create a "breathing" effect on one LED based on timer value
        if (timer_value < 128)
        {
            // Timer in first half - LED brightness increasing
            PORTB = ~(1 << 0); // Turn on LED 0
        }
        else
        {
            // Timer in second half - LED off
            PORTB |= (1 << 0); // Turn off LED 0
        }
    }

    return 0;
}