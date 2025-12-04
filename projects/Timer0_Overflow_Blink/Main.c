/*
 * =============================================================================
 * TIMER0 OVERFLOW INTERRUPT - LED BLINK DEMOS
 * =============================================================================
 * PROJECT: Timer0_Overflow_Blink
 *
 * LEARNING OBJECTIVES:
 * - Understand Timer0 8-bit counter operation (0-255)
 * - Learn overflow interrupt mechanism
 * - Master non-blocking LED blinking without delay()
 * - Compare POLLING vs INTERRUPT methods
 *
 * HARDWARE:
 * - LED on PB0 (active LOW)
 * - ATmega128 @ 16 MHz
 *
 * THEORY:
 * Timer0 overflow frequency = F_CPU / (Prescaler × 256)
 * Example: 16000000 / (1024 × 256) = 61.035 Hz (every 16.4ms)
 *
 * DEMOS:
 * 1. Polling Method - Check overflow flag in loop
 * 2. Interrupt Method - ISR handles overflow automatically
 * 3. Variable Speed - Change blink rate dynamically
 * 4. Multitasking - LED blinks while main loop counts
 * =============================================================================
 */

#include "config.h"

// Global variables for interrupt demos
volatile uint16_t overflow_count = 0;
volatile uint8_t blink_rate = 31; // 31 overflows ≈ 0.5 sec (2 Hz)

// Function prototypes
void demo1_polling(void);
void demo2_interrupt(void);
void demo3_variable_speed(void);
void demo4_multitasking(void);

// ===== INTERRUPT SERVICE ROUTINE =====
// Called automatically when Timer0 overflows (255 → 0)
ISR(TIMER0_OVF_vect)
{
    overflow_count++;

    if (overflow_count >= blink_rate)
    {
        PORTB ^= (1 << PB0); // Toggle LED
        overflow_count = 0;
    }
}

// ===== MAIN - SELECT YOUR DEMO =====
int main(void)
{
    // Initialize hardware
    DDRB = 0xFF;  // All Port B pins as OUTPUT
    PORTB = 0xFF; // All LEDs OFF (active LOW)

    // Uncomment ONE demo to run:
    // demo1_polling(); // Polling overflow flag
    // demo2_interrupt(); // Interrupt-driven blinking
    // demo3_variable_speed();    // Dynamic speed control
    demo4_multitasking(); // Main loop free for other work

    while (1)
    {
    }
    return 0;
}

// =============================================================================
// DEMO 1: POLLING METHOD
// =============================================================================
// Check overflow flag manually in main loop
// CPU is BLOCKED waiting for flag
void demo1_polling(void)
{
    uint16_t count = 0;

    // Setup Timer0
    TCCR0 = 0x00;         // Stop timer
    TCNT0 = 0;            // Clear counter
    TIFR = (1 << TOV0);   // Clear overflow flag
    PORTB &= ~(1 << PB0); // LED ON initially

    // Start Timer0: Prescaler = 1024
    // Overflow rate = 16MHz / (1024 × 256) = 61 Hz
    TCCR0 = (1 << CS02) | (1 << CS01) | (1 << CS00);

    while (1)
    {
        // Poll overflow flag
        if (TIFR & (1 << TOV0))
        {
            count++;
            TIFR = (1 << TOV0); // Clear flag by writing 1

            if (count >= 31) // 31 overflows ≈ 0.5 sec
            {
                PORTB ^= (1 << PB0); // Toggle LED
                count = 0;
            }
        }
        // NOTE: Main loop is STUCK polling!
        // Can't do other work easily
    }
}

// =============================================================================
// DEMO 2: INTERRUPT METHOD
// =============================================================================
// ISR handles overflow automatically
// CPU is FREE in main loop!
void demo2_interrupt(void)
{
    overflow_count = 0;
    blink_rate = 31; // 31 overflows ≈ 0.5 sec (2 Hz)

    // Setup Timer0
    TCCR0 = 0x00;
    TCNT0 = 0;
    TIFR = (1 << TOV0);
    PORTB &= ~(1 << PB0); // LED ON

    // Enable Timer0 overflow interrupt
    TIMSK |= (1 << TOIE0);

    // Start timer with prescaler 1024
    TCCR0 = (1 << CS02) | (1 << CS01) | (1 << CS00);

    // Enable global interrupts
    sei();

    while (1)
    {
        // Main loop is FREE!
        // ISR handles LED toggling in background
        // You could do other work here
        for (volatile uint16_t i = 0; i < 1000; i++)
            asm volatile("nop");
    }
}

// =============================================================================
// DEMO 3: VARIABLE SPEED
// =============================================================================
// Change blink rate by modifying overflow count threshold
void demo3_variable_speed(void)
{
    overflow_count = 0;

    // Blink rates (in overflows):
    // 61 = 1 Hz, 31 = 2 Hz, 15 = 4 Hz, 8 = 7.6 Hz
    const uint8_t speeds[] = {61, 31, 15, 8};
    uint8_t speed_index = 0;

    // Setup Timer0 with interrupt
    TCCR0 = 0x00;
    TCNT0 = 0;
    TIMSK |= (1 << TOIE0);
    TCCR0 = (1 << CS02) | (1 << CS01) | (1 << CS00);
    sei();

    blink_rate = speeds[0]; // Start with slowest
    PORTB &= ~(1 << PB0);

    while (1)
    {
        // Change speed every 3 seconds
        _delay_ms(3000);

        speed_index++;
        if (speed_index >= 4)
            speed_index = 0;

        blink_rate = speeds[speed_index];

        // Visual indication of speed change
        for (uint8_t i = 0; i < 3; i++)
        {
            PORTB &= ~(1 << PB0); // LED ON
            _delay_ms(100);
            PORTB |= (1 << PB0); // LED OFF
            _delay_ms(100);
        }
    }
}

// =============================================================================
// DEMO 4: MULTITASKING
// =============================================================================
// LED blinks in ISR while main loop does other work
void demo4_multitasking(void)
{
    overflow_count = 0;
    blink_rate = 15; // Fast blink (4 Hz)
    uint32_t main_counter = 0;

    // Setup Timer0 with interrupt
    TCCR0 = 0x00;
    TCNT0 = 0;
    TIMSK |= (1 << TOIE0);
    TCCR0 = (1 << CS02) | (1 << CS01) | (1 << CS00);
    sei();

    PORTB &= ~(1 << PB0);

    while (1)
    {
        // Main "task": counting
        main_counter++;

        // Show we're doing work (toggle PB1 every 100000 iterations)
        if (main_counter % 100000 == 0)
        {
            PORTB ^= (1 << PB1);
        }

        // Note: PB0 (LED) blinks automatically via ISR
        // Main loop is completely free to do this counting work!
        // This is the KEY ADVANTAGE of timer interrupts
    }
}

// =============================================================================
// LEARNING NOTES
// =============================================================================
/*
 * POLLING vs INTERRUPT:
 *
 * POLLING (Demo 1):
 * - Main loop checks flag repeatedly
 * - CPU is BLOCKED waiting
 * - Hard to do multiple tasks
 * - Simple to understand
 *
 * INTERRUPT (Demo 2-4):
 * - ISR runs automatically on overflow
 * - Main loop is FREE
 * - Easy multitasking
 * - Slightly more complex
 *
 * WHEN TO USE EACH:
 * - Use POLLING for simple, single-task programs
 * - Use INTERRUPT when you need to do multiple things at once
 *
 * OVERFLOW RATE CALCULATION:
 * Timer0 is 8-bit: counts 0 → 255 (256 counts total)
 * With prescaler 1024: Timer increments every 1024 CPU cycles
 * Overflow every: 256 × 1024 = 262,144 CPU cycles
 * At 16 MHz: 262,144 / 16,000,000 = 16.384 ms
 * Frequency: 1 / 0.016384 = 61.035 Hz
 *
 * To get 1 Hz LED blink (toggle every 0.5 sec):
 * Need: 61 overflows × 16.384ms = 999.4ms ≈ 1 second
 */
