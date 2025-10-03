/*
 * ===========================================================================
 * TIMER OVERFLOW - Basic Timer Interrupts
 * ATmega128 @ 16MHz
 * ===========================================================================
 * FOCUS: Understanding timer overflow and interrupt-based timing
 *
 * TIMER CONCEPT:
 * A timer is a hardware counter that increments automatically with each
 * clock cycle (or prescaled clock). When it reaches its maximum value,
 * it "overflows" back to zero and can trigger an interrupt.
 *
 * TIMER0 (8-bit):
 * - Counts from 0 to 255 (2^8 - 1)
 * - Overflows at 256 counts
 * - Can use prescalers: 1, 8, 64, 256, 1024
 *
 * TIMER1 (16-bit):
 * - Counts from 0 to 65535 (2^16 - 1)
 * - More precise, longer intervals
 * - Can use prescalers: 1, 8, 64, 256, 1024
 *
 * LEARNING OBJECTIVES:
 * 1. Configure timer registers (TCCR, TIMSK, TCNT)
 * 2. Calculate overflow frequency
 * 3. Write interrupt service routines (ISR)
 * 4. Use volatile variables
 * 5. Create accurate time delays
 *
 * HARDWARE:
 * - ATmega128 @ 16MHz
 * - LED on PORTB.0 (for timing visualization)
 */

#include "config.h"

// ============================================================================
// SHARED VARIABLES AND ISR
// ============================================================================
// All demos share these ISRs but use different modes
// Only ONE demo should run at a time!

// Global volatile variables (modified in ISR, used in main)
volatile uint16_t overflow_count = 0;
volatile uint8_t count_100ms = 0;
volatile uint8_t count_500ms = 0;
volatile uint8_t count_1s = 0;
volatile uint8_t task_ready = 0;
volatile uint16_t seconds = 0;
volatile uint8_t button_count = 0;
volatile uint8_t display_count = 0;
volatile uint8_t second_elapsed = 0;

// Demo mode selector
volatile uint8_t demo_mode = 1; // Set by each demo function

// Timer0 ISR - handles demo 1 and demo 4
ISR(TIMER0_OVF_vect)
{
    if (demo_mode == 1)
    {
        // Demo 1: Simple overflow counter
        overflow_count++;
        if (overflow_count >= 61)
        {
            PORTB ^= (1 << 0);
            overflow_count = 0;
        }
    }
    else if (demo_mode == 4)
    {
        // Demo 4: Multiple intervals
        count_100ms++;
        count_500ms++;
        count_1s++;

        if (count_100ms >= 6)
        {
            PORTB ^= (1 << 3);
            count_100ms = 0;
        }

        if (count_500ms >= 30)
        {
            PORTB ^= (1 << 4);
            count_500ms = 0;
        }

        if (count_1s >= 61)
        {
            PORTB ^= (1 << 5);
            count_1s = 0;
        }
    }
}

// Timer1 ISR - handles demos 2, 3, 5, 6
ISR(TIMER1_OVF_vect)
{
    if (demo_mode == 2)
    {
        // Demo 2: Simple 16-bit overflow
        PORTB ^= (1 << 1);
    }
    else if (demo_mode == 3)
    {
        // Demo 3: Accurate 1 second with preload
        TCNT1 = 3036; // Preload for 1 second
        PORTB ^= (1 << 2);
    }
    else if (demo_mode == 5)
    {
        // Demo 5: Flag synchronization
        TCNT1 = 3036;
        task_ready = 1;
        seconds++;
    }
    else if (demo_mode == 6)
    {
        // Demo 6: Event counter
        TCNT1 = 3036;
        second_elapsed = 1;
        display_count = button_count;
        button_count = 0;
    }
}

// ============================================================================
// Demo 1: Timer0 Basic Overflow (8-bit timer)
// ============================================================================
/*
 * CALCULATION:
 * Timer0: 8-bit (0-255)
 * Clock: 16MHz / 1024 (prescaler) = 15,625 Hz
 * Overflow time: 256 / 15,625 = 16.384 ms
 * For 1 second: 1000ms / 16.384ms = 61.035 overflows ≈ 61
 */

void demo_01_timer0_overflow(void)
{
    demo_mode = 1; // Set mode for ISR
    overflow_count = 0;

    // Configure LED pin
    DDRB |= (1 << 0);   // PB0 as output
    PORTB &= ~(1 << 0); // LED off initially

    /*
     * TIMER0 CONTROL REGISTER (TCCR0)
     *
     * Bits 2:0 (CS02:CS00) - Clock Select:
     * 000 = No clock (timer stopped)
     * 001 = clk/1 (no prescaling)
     * 010 = clk/8
     * 011 = clk/64
     * 100 = clk/256
     * 101 = clk/1024
     */
    TCCR0 = (1 << CS02) | (1 << CS00); // Prescaler = 1024

    /*
     * TIMER INTERRUPT MASK REGISTER (TIMSK)
     * Bit 0 (TOIE0) - Timer0 Overflow Interrupt Enable
     */
    TIMSK |= (1 << TOIE0); // Enable Timer0 overflow interrupt

    /*
     * TIMER COUNTER REGISTER (TCNT0)
     * Initialize to 0 (will count from 0 to 255)
     */
    TCNT0 = 0;

    // Enable global interrupts
    sei();

    while (1)
    {
        // Main loop does nothing - all work in ISR
        // LED toggles automatically via interrupt
    }
}

// ============================================================================
// Demo 2: Timer1 Basic Overflow (16-bit timer)
// ============================================================================
/*
 * CALCULATION:
 * Timer1: 16-bit (0-65535)
 * Clock: 16MHz / 1024 = 15,625 Hz
 * Overflow time: 65536 / 15,625 = 4.194 seconds
 *
 * This is much longer than Timer0!
 */

void demo_02_timer1_overflow(void)
{
    demo_mode = 2;

    // Configure LED pin
    DDRB |= (1 << 1);   // PB1 as output
    PORTB &= ~(1 << 1); // LED off initially

    /*
     * TIMER1 CONTROL REGISTER B (TCCR1B)
     * Bits 2:0 (CS12:CS10) - Clock Select (same as Timer0)
     */
    TCCR1B = (1 << CS12) | (1 << CS10); // Prescaler = 1024

    // Enable Timer1 overflow interrupt
    TIMSK |= (1 << TOIE1);

    // Initialize counter
    TCNT1 = 0;

    // Enable global interrupts
    sei();

    while (1)
    {
        // LED toggles every ~4.2 seconds via interrupt
    }
}

// ============================================================================
// Demo 3: Accurate 1 Second Timing with Preload
// ============================================================================
/*
 * TECHNIQUE: Preload the timer to get exact timing
 *
 * For exactly 1 second with Timer1 @ 16MHz / 256 prescaler:
 * Timer freq: 16MHz / 256 = 62,500 Hz
 * For 1 second: need 62,500 counts
 * Timer1 max: 65,536
 * Preload value: 65536 - 62500 = 3036
 *
 * Timer will count from 3036 to 65535 (62,500 counts = 1 second)
 */

void demo_03_accurate_1_second(void)
{
    demo_mode = 3;

    // Configure LED pin
    DDRB |= (1 << 2);   // PB2 as output
    PORTB &= ~(1 << 2); // LED off initially

    // Timer1: Prescaler = 256
    TCCR1B = (1 << CS12); // CS12=1, CS11=0, CS10=0 → clk/256

    // Preload timer for 1 second interval
    TCNT1 = 3036;

    // Enable Timer1 overflow interrupt
    TIMSK |= (1 << TOIE1);

    sei();

    while (1)
    {
        // Accurate 1 second LED toggle
    }
}

// ============================================================================
// Demo 4: Multiple Time Intervals (Software Divider)
// ============================================================================
/*
 * TECHNIQUE: Use one timer overflow to create multiple intervals
 *
 * Timer0 overflows every ~16.384ms (prescaler 1024)
 * Count overflows to create different time intervals
 */

void demo_04_multiple_intervals(void)
{
    demo_mode = 4;
    count_100ms = 0;
    count_500ms = 0;
    count_1s = 0;

    // Configure 3 LED pins
    DDRB |= (1 << 3) | (1 << 4) | (1 << 5);
    PORTB &= ~((1 << 3) | (1 << 4) | (1 << 5));

    // Timer0: Prescaler = 1024
    TCCR0 = (1 << CS02) | (1 << CS00);

    // Enable Timer0 overflow interrupt
    TIMSK |= (1 << TOIE0);

    TCNT0 = 0;
    sei();

    while (1)
    {
        // Three LEDs blinking at different rates!
        // PB3: ~100ms (fast)
        // PB4: ~500ms (medium)
        // PB5: ~1000ms (slow)
    }
}

// ============================================================================
// Demo 5: Timer Flag for Main Loop Synchronization
// ============================================================================
/*
 * TECHNIQUE: Use ISR to set a flag, process in main loop
 *
 * This is useful when you want to:
 * - Keep ISR short and fast
 * - Do complex processing in main loop
 * - Maintain responsive system
 */

void demo_05_flag_synchronization(void)
{
    demo_mode = 5;
    task_ready = 0;
    seconds = 0;

    uint8_t led_state = 0;

    // Configure LED
    DDRB |= (1 << 6);
    PORTB &= ~(1 << 6);

    // Timer1: 1 second interval
    TCCR1B = (1 << CS12);
    TCNT1 = 3036;
    TIMSK |= (1 << TOIE1);

    sei();

    while (1)
    {
        // Wait for timer flag
        if (task_ready)
        {
            task_ready = 0; // Clear flag

            // Do your task here (runs every 1 second)
            led_state = !led_state;

            if (led_state)
                PORTB |= (1 << 6);
            else
                PORTB &= ~(1 << 6);

            // Could do complex processing here
            // without blocking the interrupt
        }

        // Main loop can do other things while waiting
    }
}

// ============================================================================
// Demo 6: Counting Events with Timer
// ============================================================================
/*
 * APPLICATION: Measure how many times button is pressed per second
 *
 * Uses Timer1 for 1-second intervals
 * Counts button presses between intervals
 * Displays count on LEDs
 */

void demo_06_event_counter(void)
{
    demo_mode = 6;
    button_count = 0;
    display_count = 0;
    second_elapsed = 0;

    uint8_t button_last = 1;
    uint8_t button_current;

    // Configure PORTB as output (display count on LEDs)
    DDRB = 0xFF;
    PORTB = 0x00;

    // Configure PORTD.7 as input (button)
    DDRD &= ~(1 << 7);
    PORTD |= (1 << 7); // Pull-up

    // Timer1: 1 second interval
    TCCR1B = (1 << CS12);
    TCNT1 = 3036;
    TIMSK |= (1 << TOIE1);

    sei();

    while (1)
    {
        // Read button
        button_current = (PIND & (1 << 7)) ? 1 : 0;

        // Detect button press (falling edge)
        if (button_last == 1 && button_current == 0)
        {
            button_count++;
        }

        button_last = button_current;

        // Display count on LEDs when second elapses
        if (second_elapsed)
        {
            PORTB = display_count; // Show count on 8 LEDs
            second_elapsed = 0;
        }

        _delay_ms(10); // Debounce delay
    }
}

// ============================================================================
// MAIN - Select Your Demo
// ============================================================================
/*
 * LEARNING PROGRESSION:
 *
 * 1. demo_01_timer0_overflow     - Basic 8-bit timer overflow
 * 2. demo_02_timer1_overflow     - 16-bit timer (longer intervals)
 * 3. demo_03_accurate_1_second   - Preload technique for accuracy
 * 4. demo_04_multiple_intervals  - Software divider for multiple timings
 * 5. demo_05_flag_synchronization - ISR flag + main loop processing
 * 6. demo_06_event_counter       - Count events per time interval
 *
 * KEY CONCEPTS:
 * - Timer overflow = counter reaches maximum and wraps to 0
 * - Prescaler = divide clock to slow timer
 * - ISR = Interrupt Service Routine (runs automatically on overflow)
 * - Volatile = variable can change outside current code flow
 * - Preload = set initial timer value for exact timing
 */
int main(void)
{
    demo_01_timer0_overflow(); // ← START HERE
    // demo_02_timer1_overflow();
    // demo_03_accurate_1_second();
    // demo_04_multiple_intervals();
    // demo_05_flag_synchronization();
    // demo_06_event_counter();

    return 0;
}

/*
 * ============================================================================
 * TIMER OVERFLOW FORMULAS
 * ============================================================================
 *
 * Overflow Frequency = F_CPU / (Prescaler × Timer_Max)
 *
 * Timer0 (8-bit):  Max = 256
 * Timer1 (16-bit): Max = 65536
 *
 * EXAMPLES @ 16MHz:
 *
 * Timer0, Prescaler 1024:
 * Overflow freq = 16,000,000 / (1024 × 256) = 61.035 Hz
 * Overflow time = 1 / 61.035 = 16.384 ms
 *
 * Timer1, Prescaler 1024:
 * Overflow freq = 16,000,000 / (1024 × 65536) = 0.238 Hz
 * Overflow time = 1 / 0.238 = 4.194 seconds
 *
 * Timer1, Prescaler 256 (for 1 second):
 * Need 62,500 counts for 1 second
 * Preload = 65536 - 62500 = 3036
 *
 * ============================================================================
 */
