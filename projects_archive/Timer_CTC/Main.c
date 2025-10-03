/*
 * ===========================================================================
 * TIMER CTC MODE - Clear Timer on Compare Match
 * ATmega128 @ 16MHz
 * ===========================================================================
 * FOCUS: Using CTC mode for precise timing intervals
 *
 * CTC MODE CONCEPT:
 * Instead of counting to the timer's maximum value (overflow), CTC mode
 * lets you set a custom compare value. When the timer reaches this value:
 * 1. Timer automatically resets to 0
 * 2. Compare match interrupt fires (if enabled)
 * 3. You get EXACT timing without manual reload
 *
 * ADVANTAGES OVER OVERFLOW MODE:
 * - Any interval length (not limited to max timer value)
 * - More accurate (no preload needed)
 * - Easier calculations
 * - Hardware handles reset automatically
 *
 * TIMER1 CTC MODE:
 * - Use OCR1A register for compare value
 * - Set WGM12 bit in TCCR1B for CTC mode
 * - Interrupt: TIMER1_COMPA_vect
 *
 * LEARNING OBJECTIVES:
 * 1. Configure CTC mode registers
 * 2. Calculate OCR values for desired intervals
 * 3. Generate precise time intervals
 * 4. Create square waves / PWM manually
 * 5. Build real-time applications
 *
 * HARDWARE:
 * - ATmega128 @ 16MHz
 * - LEDs on PORTB (for visualization)
 */

#include "config.h"

// Global variables
volatile uint16_t milliseconds = 0;
volatile uint8_t seconds = 0;
volatile uint8_t minutes = 0;
volatile uint8_t task_flag = 0;

// ============================================================================
// SHARED ISR - Demo mode selector
// ============================================================================
volatile uint8_t demo_mode = 1;

ISR(TIMER1_COMPA_vect)
{
    if (demo_mode == 1)
    {
        // Demo 1: Simple 1 second toggle
        PORTB ^= (1 << 0);
    }
    else if (demo_mode == 2)
    {
        // Demo 2: 100ms precision timing
        PORTB ^= (1 << 1);
    }
    else if (demo_mode == 3)
    {
        // Demo 3: Real-time clock
        milliseconds += 10; // Increment by 10ms
        if (milliseconds >= 1000)
        {
            milliseconds = 0;
            seconds++;
            PORTB ^= (1 << 2); // Blink every second

            if (seconds >= 60)
            {
                seconds = 0;
                minutes++;
            }
        }
    }
    else if (demo_mode == 4)
    {
        // Demo 4: Task scheduler
        task_flag = 1;
    }
    else if (demo_mode == 5)
    {
        // Demo 5: Square wave generation
        PORTB ^= (1 << 4); // Toggle for square wave
    }
}

// ============================================================================
// Demo 1: 1 Second Interval with CTC
// ============================================================================
/*
 * CALCULATION FOR 1 SECOND:
 * F_CPU = 16,000,000 Hz
 * Prescaler = 256
 * Timer freq = 16,000,000 / 256 = 62,500 Hz
 *
 * For 1 second:
 * OCR1A = 62,500 - 1 = 62,499
 *
 * (Subtract 1 because timer counts from 0)
 */

void demo_01_1_second_ctc(void)
{
    demo_mode = 1;

    // Configure LED
    DDRB |= (1 << 0);
    PORTB &= ~(1 << 0);

    /*
     * TIMER1 CONTROL REGISTERS:
     *
     * TCCR1A (Control Register A):
     * - Normal port operation (OC1A/OC1B disconnected)
     *
     * TCCR1B (Control Register B):
     * - WGM12 = 1: CTC mode (Clear Timer on Compare Match)
     * - CS12 = 1: Prescaler = 256
     */
    TCCR1A = 0;                          // Normal port operation
    TCCR1B = (1 << WGM12) | (1 << CS12); // CTC mode, prescaler 256

    /*
     * Output Compare Register 1A (OCR1A):
     * Timer compares TCNT1 with this value
     * When TCNT1 == OCR1A:
     * - Timer resets to 0
     * - Interrupt fires (if enabled)
     */
    OCR1A = 62499; // Compare value for 1 second

    /*
     * Timer Interrupt Mask Register (TIMSK):
     * OCIE1A = 1: Enable Timer1 Compare Match A interrupt
     */
    TIMSK |= (1 << OCIE1A);

    // Initialize counter
    TCNT1 = 0;

    // Enable global interrupts
    sei();

    while (1)
    {
        // LED toggles every 1 second via interrupt
    }
}

// ============================================================================
// Demo 2: 100ms Precision Interval
// ============================================================================
/*
 * CALCULATION FOR 100ms (0.1 second):
 * Timer freq = 16,000,000 / 256 = 62,500 Hz
 * For 0.1 second: 62,500 × 0.1 = 6,250
 * OCR1A = 6,250 - 1 = 6,249
 */

void demo_02_100ms_interval(void)
{
    demo_mode = 2;

    // Configure LED
    DDRB |= (1 << 1);
    PORTB &= ~(1 << 1);

    // CTC mode, prescaler 256
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12);

    // Compare value for 100ms
    OCR1A = 6249;

    // Enable compare match interrupt
    TIMSK |= (1 << OCIE1A);

    TCNT1 = 0;
    sei();

    while (1)
    {
        // Fast LED blink (100ms on, 100ms off = 5Hz)
    }
}

// ============================================================================
// Demo 3: Real-Time Clock (10ms tick)
// ============================================================================
/*
 * CALCULATION FOR 10ms:
 * Timer freq = 16,000,000 / 256 = 62,500 Hz
 * For 0.01 second: 62,500 × 0.01 = 625
 * OCR1A = 625 - 1 = 624
 *
 * Use ISR to count milliseconds, seconds, minutes
 */

void demo_03_real_time_clock(void)
{
    demo_mode = 3;
    milliseconds = 0;
    seconds = 0;
    minutes = 0;

    // Configure LED
    DDRB |= (1 << 2);
    PORTB &= ~(1 << 2);

    // CTC mode, prescaler 256
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12);

    // Compare value for 10ms tick
    OCR1A = 624;

    // Enable compare match interrupt
    TIMSK |= (1 << OCIE1A);

    TCNT1 = 0;
    sei();

    while (1)
    {
        // Real-time clock running in background
        // LED blinks every second
        // Could display time on LCD or send via UART

        // Example: Check if 5 seconds passed
        if (seconds >= 5 && seconds < 6)
        {
            // Do something at 5 seconds
        }
    }
}

// ============================================================================
// Demo 4: Task Scheduler (Multiple Tasks at Different Rates)
// ============================================================================
/*
 * TECHNIQUE: Use 10ms time base to schedule multiple tasks
 *
 * Task 1: Every 100ms
 * Task 2: Every 500ms
 * Task 3: Every 1000ms
 */

volatile uint8_t task1_counter = 0;
volatile uint8_t task2_counter = 0;
volatile uint8_t task3_counter = 0;

void task1_100ms(void)
{
    // Fast LED toggle
    PORTB ^= (1 << 3);
}

void task2_500ms(void)
{
    // Medium LED toggle
    PORTB ^= (1 << 4);
}

void task3_1000ms(void)
{
    // Slow LED toggle
    PORTB ^= (1 << 5);
}

void demo_04_task_scheduler(void)
{
    demo_mode = 4;
    task1_counter = 0;
    task2_counter = 0;
    task3_counter = 0;

    // Configure 3 LED pins
    DDRB |= (1 << 3) | (1 << 4) | (1 << 5);
    PORTB &= ~((1 << 3) | (1 << 4) | (1 << 5));

    // CTC mode, 10ms tick
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12);
    OCR1A = 624; // 10ms

    TIMSK |= (1 << OCIE1A);
    TCNT1 = 0;
    sei();

    while (1)
    {
        if (task_flag)
        {
            task_flag = 0;

            // Increment counters (each tick = 10ms)
            task1_counter++;
            task2_counter++;
            task3_counter++;

            // Task 1: Every 100ms (10 ticks)
            if (task1_counter >= 10)
            {
                task1_100ms();
                task1_counter = 0;
            }

            // Task 2: Every 500ms (50 ticks)
            if (task2_counter >= 50)
            {
                task2_500ms();
                task2_counter = 0;
            }

            // Task 3: Every 1000ms (100 ticks)
            if (task3_counter >= 100)
            {
                task3_1000ms();
                task3_counter = 0;
            }
        }
    }
}

// ============================================================================
// Demo 5: Square Wave Generation (1 kHz)
// ============================================================================
/*
 * CALCULATION FOR 1 kHz SQUARE WAVE:
 * Period = 1 / 1000 Hz = 1 ms
 * Half period = 0.5 ms (toggle every 0.5ms for 1kHz)
 *
 * With prescaler 1 (no prescaling):
 * Timer freq = 16,000,000 Hz
 * For 0.5ms: 16,000,000 × 0.0005 = 8,000
 * OCR1A = 8,000 - 1 = 7,999
 */

void demo_05_square_wave_1khz(void)
{
    demo_mode = 5;

    // Configure pin for square wave output
    DDRB |= (1 << 4);
    PORTB &= ~(1 << 4);

    // CTC mode, NO PRESCALER for higher frequency
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS10); // Prescaler = 1

    // Compare value for 0.5ms (toggle creates 1kHz)
    OCR1A = 7999;

    // Enable compare match interrupt
    TIMSK |= (1 << OCIE1A);

    TCNT1 = 0;
    sei();

    while (1)
    {
        // 1 kHz square wave on PB4
        // Can be measured with oscilloscope
    }
}

// ============================================================================
// Demo 6: Variable Frequency Generator
// ============================================================================
/*
 * APPLICATION: Generate different frequencies
 * Use button to change frequency
 */

volatile uint16_t frequency_table[] = {
    7999,  // 1000 Hz
    15999, // 500 Hz
    31999, // 250 Hz
    63999  // 125 Hz
};

volatile uint8_t freq_index = 0;

void demo_06_variable_frequency(void)
{
    demo_mode = 5; // Use same ISR as demo 5
    freq_index = 0;

    uint8_t button_last = 1;
    uint8_t button_current;

    // Configure output pin
    DDRB |= (1 << 4);
    PORTB &= ~(1 << 4);

    // Configure button
    DDRD &= ~(1 << 7);
    PORTD |= (1 << 7); // Pull-up

    // CTC mode, prescaler = 1
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS10);

    // Start with first frequency
    OCR1A = frequency_table[freq_index];

    TIMSK |= (1 << OCIE1A);
    TCNT1 = 0;
    sei();

    while (1)
    {
        // Read button
        button_current = (PIND & (1 << 7)) ? 1 : 0;

        // Button pressed - change frequency
        if (button_last == 1 && button_current == 0)
        {
            freq_index++;
            if (freq_index >= 4)
                freq_index = 0;

            // Update compare value
            OCR1A = frequency_table[freq_index];

            // Indicate frequency change on another LED
            PORTB ^= (1 << 6);
        }

        button_last = button_current;
        _delay_ms(50); // Debounce
    }
}

// ============================================================================
// MAIN - Select Your Demo
// ============================================================================
/*
 * LEARNING PROGRESSION:
 *
 * 1. demo_01_1_second_ctc        - Basic CTC operation (1 second)
 * 2. demo_02_100ms_interval      - Faster interval (100ms)
 * 3. demo_03_real_time_clock     - Build real-time clock
 * 4. demo_04_task_scheduler      - Multiple tasks at different rates
 * 5. demo_05_square_wave_1khz    - Generate 1kHz square wave
 * 6. demo_06_variable_frequency  - Change frequency with button
 *
 * KEY CONCEPTS:
 * - CTC mode: Timer resets automatically at compare match
 * - OCR1A: Set compare value for desired interval
 * - WGM12: Mode bit for CTC
 * - More flexible than overflow mode
 * - Easier calculations
 */
int main(void)
{
    demo_01_1_second_ctc(); // ← START HERE
    // demo_02_100ms_interval();
    // demo_03_real_time_clock();
    // demo_04_task_scheduler();
    // demo_05_square_wave_1khz();
    // demo_06_variable_frequency();

    return 0;
}

/*
 * ============================================================================
 * CTC MODE FORMULAS
 * ============================================================================
 *
 * OCR VALUE CALCULATION:
 * OCR1A = (F_CPU / (Prescaler × Desired_Frequency)) - 1
 *
 * INTERRUPT FREQUENCY:
 * Frequency = F_CPU / (Prescaler × (OCR1A + 1))
 *
 * TIME INTERVAL:
 * Interval = (Prescaler × (OCR1A + 1)) / F_CPU
 *
 * PRESCALER OPTIONS FOR TIMER1:
 * 1, 8, 64, 256, 1024
 *
 * EXAMPLES @ 16MHz:
 *
 * For 1 second (prescaler 256):
 * OCR1A = (16,000,000 / (256 × 1)) - 1 = 62,499
 *
 * For 100ms (prescaler 256):
 * OCR1A = (16,000,000 / (256 × 10)) - 1 = 6,249
 *
 * For 1kHz square wave (prescaler 1):
 * Need toggle every 0.5ms
 * OCR1A = (16,000,000 / (1 × 2000)) - 1 = 7,999
 *
 * ============================================================================
 * CTC vs OVERFLOW MODE
 * ============================================================================
 *
 * OVERFLOW MODE:
 * - Fixed maximum count (255 or 65535)
 * - Requires preload for custom intervals
 * - Must reload in ISR for accuracy
 * - Limited flexibility
 *
 * CTC MODE:
 * - Any count value (0 to 65535 for Timer1)
 * - Hardware auto-resets timer
 * - No preload needed
 * - Easy frequency calculations
 * - More accurate
 * - Recommended for most applications!
 *
 * ============================================================================
 */
