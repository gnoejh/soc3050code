/*
 * Interrupt Basic - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Understand interrupt mechanism and ISR programming
 * - Learn external interrupt configuration
 * - Practice timer interrupt handling
 * - Master interrupt priorities and nesting
 *
 * HARDWARE SETUP:
 * - Push button on INT0 (PD0) - external interrupt source
 * - LEDs on PORTB for interrupt status indication
 * - Optional: additional buttons on INT1-7 for advanced testing
 */

#include "config.h"

// Global variables for interrupt handling
volatile uint8_t external_interrupt_count = 0;
volatile uint8_t timer_interrupt_count = 0;
volatile uint8_t button_pressed = 0;

// External Interrupt 0 ISR (INT0 - PD0)
ISR(INT0_vect)
{
    // External interrupt triggered
    external_interrupt_count++;
    button_pressed = 1;

    // Toggle LED to show interrupt occurred
    PORTB ^= (1 << 0); // Toggle LED 0

    // Debounce delay (simple method)
    _delay_ms(50);
}

// Timer2 Overflow ISR
ISR(TIMER2_OVF_vect)
{
    // Timer interrupt triggered (approximately every 1 second)
    timer_interrupt_count++;

    // Toggle LED to show timer interrupt
    PORTB ^= (1 << 1); // Toggle LED 1
}

int main(void)
{
    // Initialize system components
    init_devices(); // Initialize all peripherals

    // Initialize UART for interrupt reports
    Uart1_init();
    puts_USART1("Interrupt Basic Demo Started\r\n");
    puts_USART1("Press button on INT0 (PD0) to trigger external interrupt\r\n");
    puts_USART1("Timer interrupt will trigger automatically\r\n");

    // Configure External Interrupt 0 (INT0)
    DDRD &= ~(1 << PD0); // Set PD0 as input (INT0)
    PORTD |= (1 << PD0); // Enable pull-up resistor

    // Configure interrupt trigger mode
    EICRA |= (1 << ISC01); // Falling edge trigger for INT0
    EICRA &= ~(1 << ISC00);

    // Enable External Interrupt 0
    EIMSK |= (1 << INT0);

    // Initialize Timer2 for interrupt generation
    Timer2_init();

    // Enable Timer2 overflow interrupt
    TIMSK |= (1 << TOIE2);

    // Enable global interrupts
    sei();

    char buffer[80];
    uint8_t last_ext_count = 0;
    uint8_t last_timer_count = 0;

    puts_USART1("Interrupt system initialized and running\r\n");

    while (1)
    {
        // Check for external interrupt activity
        if (button_pressed)
        {
            button_pressed = 0; // Clear flag

            sprintf(buffer, "External Interrupt #%u - Button pressed!\r\n",
                    external_interrupt_count);
            puts_USART1(buffer);

            // Demonstrate interrupt response time
            puts_USART1("Interrupt handled quickly - LED toggled\r\n");
        }

        // Report timer interrupt activity
        if (timer_interrupt_count != last_timer_count)
        {
            last_timer_count = timer_interrupt_count;

            sprintf(buffer, "Timer Interrupt #%u - 1 second elapsed\r\n",
                    timer_interrupt_count);
            puts_USART1(buffer);
        }

        // Periodic status report
        if (timer_interrupt_count % 10 == 0 && timer_interrupt_count > 0)
        {
            sprintf(buffer, "\r\n--- Status Report ---\r\n");
            puts_USART1(buffer);
            sprintf(buffer, "External interrupts: %u\r\n", external_interrupt_count);
            puts_USART1(buffer);
            sprintf(buffer, "Timer interrupts: %u\r\n", timer_interrupt_count);
            puts_USART1(buffer);
            sprintf(buffer, "Uptime: %u seconds\r\n\r\n", timer_interrupt_count);
            puts_USART1(buffer);
        }

        // Main loop can do other work
        // Interrupts will preempt this loop when needed

        // Demonstrate non-blocking operation
        for (uint8_t i = 0; i < 8; i++)
        {
            PORTB = ~(1 << i); // Light up one LED at a time
            _delay_ms(100);    // Brief delay
        }
        PORTB = 0xFF; // Turn off all LEDs
        _delay_ms(200);
    }

    return 0;
}