/*
 * =============================================================================
 * TIMER STOPWATCH - PRACTICAL TIMING APPLICATION
 * =============================================================================
 * PROJECT: Timer_Stopwatch
 *
 * LEARNING OBJECTIVES:
 * - Apply Timer1 CTC mode to real-world application
 * - Implement precise time counting (centiseconds)
 * - Build multi-mode timing system
 * - Format time display (MM:SS.CC)
 *
 * HARDWARE:
 * - LED on PB0 (running indicator)
 * - LEDs on PB1-PB7 (time display)
 * - ATmega128 @ 16 MHz
 *
 * THEORY:
 * Uses Timer1 CTC mode for 100 Hz (10ms) interrupts
 * Count interrupts to track centiseconds
 * OCR1A = (16MHz / 64 / 100) - 1 = 2499
 *
 * DEMOS:
 * 1. Basic Stopwatch - Count up from 00:00.00
 * 2. Countdown Timer - Count down from set time
 * 3. Lap Counter - Record multiple lap times
 * 4. Split Timer - Pause/Resume timing
 * =============================================================================
 */

#include "config.h"

// Time keeping variables
volatile uint32_t centiseconds = 0; // Total time in 1/100 second
volatile uint8_t running = 0;       // Stopwatch state: 0=stopped, 1=running

// Lap times storage
#define MAX_LAPS 8
uint32_t lap_times[MAX_LAPS];
uint8_t lap_count = 0;

// Function prototypes
void demo1_basic_stopwatch(void);
void demo2_countdown_timer(void);
void demo3_lap_counter(void);
void demo4_split_timer(void);
void display_time(uint32_t cs);

// ===== INTERRUPT SERVICE ROUTINE =====
// Fires every 10ms (100 Hz)
ISR(TIMER1_COMPA_vect)
{
    if (running)
    {
        centiseconds++;
    }
}

// ===== MAIN - SELECT YOUR DEMO =====
int main(void)
{
    // Initialize hardware
    DDRB = 0xFF;  // All Port B as OUTPUT
    PORTB = 0xFF; // All LEDs OFF

    // Uncomment ONE demo to run:
    demo1_basic_stopwatch(); // Basic count-up stopwatch
    // demo2_countdown_timer();    // Count down from time
    // demo3_lap_counter();        // Record lap times
    // demo4_split_timer();        // Pause and resume

    while (1)
    {
    }
    return 0;
}

// =============================================================================
// TIMER SETUP
// =============================================================================
void setup_timer_100hz(void)
{
    // Timer1 CTC Mode for 100 Hz (10ms period)
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1 = 0;

    // CTC Mode (WGM12=1), Prescaler 64 (CS11=1, CS10=1)
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    // 100 Hz: OCR1A = (16000000 / 64 / 100) - 1 = 2499
    OCR1A = 2499;

    // Enable compare match interrupt
    TIMSK |= (1 << OCIE1A);
    sei();
}

// =============================================================================
// DEMO 1: BASIC STOPWATCH
// =============================================================================
// Count up from 00:00.00, auto-stop at 99:59.99
void demo1_basic_stopwatch(void)
{
    centiseconds = 0;
    running = 0;

    setup_timer_100hz();

    // Auto-start after 1 second
    _delay_ms(1000);
    running = 1;

    while (1)
    {
        // Display current time on LEDs
        display_time(centiseconds);

        // Running indicator (blink PB0 every second)
        if ((centiseconds / 100) % 2 == 0)
            PORTB &= ~(1 << PB0);
        else
            PORTB |= (1 << PB0);

        // Auto-stop at 99:59.99 (359999 centiseconds)
        if (centiseconds >= 359999)
        {
            running = 0;
            // Rapid blink to indicate stop
            for (uint8_t i = 0; i < 6; i++)
            {
                PORTB ^= 0xFF;
                _delay_ms(100);
            }
            centiseconds = 0; // Reset
            _delay_ms(1000);
            running = 1; // Restart
        }

        _delay_ms(10);
    }
}

// =============================================================================
// DEMO 2: COUNTDOWN TIMER
// =============================================================================
// Count down from preset time, alarm when reaches zero
void demo2_countdown_timer(void)
{
    // Set countdown time: 10 seconds (1000 centiseconds)
    uint32_t countdown_time = 1000;

    centiseconds = countdown_time;
    running = 0;

    setup_timer_100hz();

    // Wait 2 seconds, then start
    _delay_ms(2000);
    running = 1;

    while (1)
    {
        // Check if time expired
        if (centiseconds == 0 && running)
        {
            running = 0;

            // ALARM! Blink all LEDs rapidly
            for (uint8_t i = 0; i < 20; i++)
            {
                PORTB = 0x00; // All ON
                _delay_ms(100);
                PORTB = 0xFF; // All OFF
                _delay_ms(100);
            }

            // Reset for next countdown
            centiseconds = countdown_time;
            _delay_ms(1000);
            running = 1;
        }

        // Display remaining time
        display_time(centiseconds);

        // Count DOWN instead of up
        if (running && centiseconds > 0)
        {
            // Timer ISR increments, so we decrement by 2 to go down
            if (centiseconds >= 2)
                centiseconds -= 2;
            else
                centiseconds = 0;
        }

        _delay_ms(10);
    }
}

// =============================================================================
// DEMO 3: LAP COUNTER
// =============================================================================
// Record multiple lap times, display on LEDs
void demo3_lap_counter(void)
{
    centiseconds = 0;
    running = 0;
    lap_count = 0;

    setup_timer_100hz();

    // Start timing
    _delay_ms(1000);
    running = 1;

    while (1)
    {
        // Display current time
        display_time(centiseconds);

        // Record lap every 5 seconds (for demo purposes)
        // In real app, this would be a button press
        if (centiseconds > 0 && (centiseconds % 500 == 0))
        {
            if (lap_count < MAX_LAPS)
            {
                lap_times[lap_count] = centiseconds;
                lap_count++;

                // Flash to indicate lap recorded
                PORTB = 0x00;
                _delay_ms(50);
                PORTB = 0xFF;
                _delay_ms(50);
            }
        }

        // When all laps recorded, stop and display
        if (lap_count >= MAX_LAPS)
        {
            running = 0;

            // Display each lap time
            for (uint8_t i = 0; i < lap_count; i++)
            {
                for (uint8_t j = 0; j < 10; j++) // Show for 1 second
                {
                    display_time(lap_times[i]);
                    PORTB &= ~(i & 0x07); // Show lap number
                    _delay_ms(100);
                }
                _delay_ms(500);
            }

            // Reset
            centiseconds = 0;
            lap_count = 0;
            _delay_ms(1000);
            running = 1;
        }

        _delay_ms(10);
    }
}

// =============================================================================
// DEMO 4: SPLIT TIMER
// =============================================================================
// Pause and resume timing
void demo4_split_timer(void)
{
    centiseconds = 0;
    running = 0;
    uint8_t split_count = 0;

    setup_timer_100hz();

    // Start timing
    _delay_ms(1000);
    running = 1;

    while (1)
    {
        display_time(centiseconds);

        // Toggle run/pause every 3 seconds (for demo)
        // In real app, this would be a button
        if (centiseconds > 0 && (centiseconds % 300 == 0))
        {
            running = !running;
            split_count++;

            // Visual feedback
            if (running)
            {
                PORTB &= ~(1 << PB0); // LED ON when running
            }
            else
            {
                PORTB |= (1 << PB0); // LED OFF when paused
                // Rapid blink to show pause
                for (uint8_t i = 0; i < 4; i++)
                {
                    PORTB ^= (1 << PB1);
                    _delay_ms(100);
                }
            }

            _delay_ms(100); // Debounce delay
        }

        // Reset after 5 splits
        if (split_count >= 5)
        {
            centiseconds = 0;
            split_count = 0;
            running = 0;
            PORTB = 0xFF;
            _delay_ms(1000);
            running = 1;
        }

        _delay_ms(10);
    }
}

// =============================================================================
// HELPER FUNCTION: DISPLAY TIME ON LEDs
// =============================================================================
// Display time as MM:SS.CC using LED patterns
// Shows seconds in binary on PB1-PB7 (0-99)
void display_time(uint32_t cs)
{
    uint8_t seconds = (cs / 100) % 60;
    uint8_t minutes = (cs / 6000) % 100;

    // Display seconds on LEDs (lower 7 bits)
    // PB0 reserved for running indicator
    uint8_t display = ~(seconds & 0x7F) << 1;

    // Keep PB0 for running indicator
    display |= (PORTB & 0x01);

    PORTB = display;
}

// =============================================================================
// LEARNING NOTES
// =============================================================================
/*
 * TIME UNIT CONVERSIONS:
 * 1 second = 100 centiseconds
 * 1 minute = 6000 centiseconds
 * 1 hour = 360000 centiseconds
 *
 * To extract time components from centiseconds:
 * Centiseconds = cs % 100
 * Seconds = (cs / 100) % 60
 * Minutes = (cs / 6000) % 60
 * Hours = cs / 360000
 *
 * TIMER CONFIGURATION FOR 100 Hz:
 * Need interrupt every 10ms (0.01 second)
 * F_CPU = 16 MHz
 * Prescaler = 64
 * Timer clock = 16000000 / 64 = 250000 Hz
 * For 100 Hz: 250000 / 100 = 2500 ticks
 * OCR1A = 2500 - 1 = 2499
 *
 * STOPWATCH FEATURES:
 * ✓ Start/Stop control
 * ✓ Lap time recording
 * ✓ Split time (pause/resume)
 * ✓ Countdown mode
 * ✓ Precise timing (±10ms accuracy)
 *
 * REAL-WORLD APPLICATIONS:
 * - Sports timing systems
 * - Lab experiment timing
 * - Process control timing
 * - Cooking timers
 * - Exercise interval timers
 *
 * ACCURACY:
 * Resolution: 10ms (centisecond)
 * Accuracy depends on crystal oscillator
 * Typical: ±20 ppm (parts per million)
 * Error: ~1.7 seconds per day
 */
