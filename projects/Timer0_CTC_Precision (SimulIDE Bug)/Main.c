/*
 * =============================================================================
 * TIMER0 CTC MODE - PRECISION TIMING DEMOS
 * =============================================================================
 * PROJECT: Timer0_CTC_Precision
 *
 * LEARNING OBJECTIVES:
 * - Master CTC (Clear Timer on Compare Match) mode for 8-bit Timer0
 * - Generate EXACT frequencies with 8-bit resolution
 * - Understand OCR0 compare register (8-bit vs Timer1's 16-bit)
 * - Compare polling vs interrupt methods
 * - Work with Timer0's limited prescaler options
 *
 * HARDWARE:
 * - LED on PB0 (active LOW)
 * - ATmega128 @ 16 MHz
 *
 * THEORY - TIMER0 CTC MODE:
 * Timer0 is 8-bit: counts 0 → OCR0 (max 255), then RESETS to 0
 * This creates EXACT frequencies within 8-bit range!
 *
 * Formula: F_output = F_CPU / (Prescaler × (OCR0 + 1))
 * Rearranged: OCR0 = (F_CPU / (Prescaler × F_desired)) - 1
 *
 * Example for 1 kHz with prescaler 64:
 * OCR0 = (16000000 / (64 × 1000)) - 1 = 249
 *
 * TIMER0 vs TIMER1 DIFFERENCES:
 * - Timer0: 8-bit (0-255), Timer1: 16-bit (0-65535)
 * - Timer0: OCR0, Timer1: OCR1A
 * - Timer0: Different prescaler options (no /1, has /32 and /128)
 * - Timer0: Better for high-frequency (kHz range)
 * - Timer1: Better for low-frequency (Hz range)
 *
 * DEMOS:
 * 1. Polling 1 kHz - Check compare flag manually
 * 2. Interrupt 1 kHz - ISR handles compare match
 * 3. Multiple Frequencies - 1 kHz, 2 kHz, 4 kHz, 8 kHz
 * 4. Precision Test - Measure actual timing accuracy
 *
 * =============================================================================
 * TIMER0 REGISTER REFERENCE - QUICK LOOKUP TABLES
 * =============================================================================
 *
 * TCCR0 - Timer/Counter Control Register
 * ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
 * │  7   │  6   │  5   │  4   │  3   │  2   │  1   │  0   │
 * │ FOC0 │WGM00 │COM01 │COM00 │WGM01 │ CS02 │ CS01 │ CS00 │
 * └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 *
 * For CTC Mode: WGM01=1, WGM00=0
 *
 * WGM01:00 - Waveform Generation Mode
 * ┌───────┬───────┬──────────────────────┬─────────┐
 * │ WGM01 │ WGM00 │ Mode                 │ TOP     │
 * ├───────┼───────┼──────────────────────┼─────────┤
 * │   0   │   0   │ Normal               │ 0xFF    │
 * │   0   │   1   │ PWM, Phase Correct   │ 0xFF    │
 * │   1   │   0   │ CTC                  │ OCR0    │ ← USE THIS
 * │   1   │   1   │ Fast PWM             │ 0xFF    │
 * └───────┴───────┴──────────────────────┴─────────┘
 *
 * CS02:00 - Clock Select (Prescaler)
 * ┌──────┬──────┬──────┬────────────┬─────────────────────┐
 * │ CS02 │ CS01 │ CS00 │ Prescaler  │ Timer Freq @ 16MHz  │
 * ├──────┼──────┼──────┼────────────┼─────────────────────┤
 * │  0   │  0   │  0   │ Stop       │ -                   │
 * │  0   │  0   │  1   │ /1         │ NOT AVAILABLE       │
 * │  0   │  1   │  0   │ /8         │ 2 MHz               │
 * │  0   │  1   │  1   │ /64        │ 250 kHz  ⭐ BEST    │
 * │  1   │  0   │  0   │ /256       │ 62.5 kHz            │
 * │  1   │  0   │  1   │ /1024      │ 15.625 kHz          │
 * │  1   │  1   │  0   │ Ext T0 ↓   │ External            │
 * │  1   │  1   │  1   │ Ext T0 ↑   │ External            │
 * └──────┴──────┴──────┴────────────┴─────────────────────┘
 *
 * TIMSK - Timer/Counter Interrupt Mask Register
 * ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
 * │  7   │  6   │  5   │  4   │  3   │  2   │  1   │  0   │
 * │OCIE2 │TOIE2 │TICIE1│OCIE1A│OCIE1B│TOIE1 │OCIE0 │TOIE0 │
 * └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 *                                              ↑      ↑
 *                                           CTC IRQ  OVF IRQ
 * OCIE0: Timer0 Output Compare Match Interrupt Enable
 * TOIE0: Timer0 Overflow Interrupt Enable
 *
 * TIFR - Timer/Counter Interrupt Flag Register
 * ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
 * │  7   │  6   │  5   │  4   │  3   │  2   │  1   │  0   │
 * │ OCF2 │ TOV2 │ ICF1 │OCF1A │OCF1B │ TOV1 │ OCF0 │ TOV0 │
 * └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 *                                              ↑      ↑
 *                                           CTC Flag OVF Flag
 * OCF0: Timer0 Output Compare Match Flag (clear by writing 1)
 * TOV0: Timer0 Overflow Flag
 *
 * TCNT0 - Timer/Counter Register (8-bit)
 * - Current counter value (0-255)
 * - Cleared automatically in CTC mode when TCNT0 = OCR0
 *
 * OCR0 - Output Compare Register (8-bit)
 * - Compare value (0-255)
 * - Timer resets to 0 when TCNT0 matches OCR0
 * - Determines output frequency
 *
 * =============================================================================
 * FREQUENCY CALCULATION REFERENCE
 * =============================================================================
 *
 * FORMULA:
 * OCR0 = (F_CPU / (Prescaler × F_desired)) - 1
 * ⚠️ OCR0 MUST be ≤ 255 (8-bit limit)
 *
 * COMMON VALUES @ 16 MHz, Prescaler 64:
 * ┌───────────┬────────────┬──────────┬──────────────────┐
 * │ Frequency │ Period     │ OCR0     │ Use Case         │
 * ├───────────┼────────────┼──────────┼──────────────────┤
 * │  1 kHz    │ 1 ms       │   249    │ millis()         │
 * │  2 kHz    │ 500 µs     │   124    │ Fast timing      │
 * │  4 kHz    │ 250 µs     │    62    │ Audio/PWM        │
 * │  8 kHz    │ 125 µs     │    31    │ High-speed       │
 * │ 10 kHz    │ 100 µs     │    24    │ ADC sampling     │
 * │ 16 kHz    │ 62.5 µs    │    15    │ Ultrasonic       │
 * │ 20 kHz    │ 50 µs      │    11    │ PWM (flicker-free)│
 * └───────────┴────────────┴──────────┴──────────────────┘
 *
 * FREQUENCY RANGE BY PRESCALER @ 16 MHz:
 * ┌───────────┬────────────┬────────────┬──────────────────┐
 * │ Prescaler │ Min Freq   │ Max Freq   │ Best For         │
 * ├───────────┼────────────┼────────────┼──────────────────┤
 * │    /8     │  7.8 kHz   │  2 MHz     │ Very high freq   │
 * │   /64     │  976 Hz    │  250 kHz   │ kHz range ⭐     │
 * │  /256     │  244 Hz    │  62.5 kHz  │ Medium freq      │
 * │ /1024     │   61 Hz    │  15.6 kHz  │ Low freq         │
 * └───────────┴────────────┴────────────┴──────────────────┘
 *
 * Min Freq = F_CPU / (Prescaler × 256)
 * Max Freq = F_CPU / (Prescaler × 2)
 *
 * INTERRUPT VECTOR:
 * ISR(TIMER0_COMP_vect) { }  // Called when TCNT0 == OCR0
 *
 * =============================================================================
 */

#include "config.h"

// Global variables
volatile uint16_t compare_count = 0;
volatile uint16_t milliseconds = 0;

// Function prototypes
void demo1_polling_1khz(void);
void demo2_interrupt_1khz(void);
void demo3_multi_frequency(void);
void demo4_precision_test(void);

// ===== INTERRUPT SERVICE ROUTINE =====
ISR(TIMER0_COMP_vect)
{
    compare_count++;

    // For 1 kHz: increment milliseconds every compare match
    milliseconds++;
}

// ===== MAIN - SELECT YOUR DEMO =====
int main(void)
{
    // Initialize hardware
    DDRB = 0xFF;  // All Port B as OUTPUT
    PORTB = 0xFF; // All LEDs OFF (active LOW)

    // Uncomment ONE demo to run:
    demo1_polling_1khz(); // Polling compare flag
    // demo2_interrupt_1khz(); // Interrupt-driven
    // demo3_multi_frequency();  // Try different frequencies
    // demo4_precision_test();   // Measure accuracy

    while (1)
    {
    }
    return 0;
}

// =============================================================================
// DEMO 1: POLLING METHOD - 1 kHz (1ms period)
// =============================================================================
// Check compare match flag manually
void demo1_polling_1khz(void)
{
    uint16_t tick_count = 0;

    // Setup Timer0 CTC Mode
    TCCR0 = 0x00;       // Stop timer
    TCNT0 = 0;          // Clear counter
    TIFR = (1 << OCF0); // Clear compare flag

    // CTC Mode: WGM01 = 1 (Mode 2)
    // Prescaler 64: CS02=0, CS01=1, CS00=1
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);

    // Set compare value for 1 kHz (1ms)
    // 16MHz / 64 / 1000Hz - 1 = 249
    OCR0 = 249;

    PORTB &= ~(1 << PB0); // LED ON

    while (1)
    {
        // Poll compare match flag
        if (TIFR & (1 << OCF0))
        {
            TIFR = (1 << OCF0); // Clear flag
            tick_count++;

            // Toggle LED every 500ms (500 ticks)
            if (tick_count >= 500)
            {
                PORTB ^= (1 << PB0);
                tick_count = 0;
            }
        }
    }
}

// =============================================================================
// DEMO 2: INTERRUPT METHOD - 1 kHz (1ms period)
// =============================================================================
// ISR handles compare match automatically
void demo2_interrupt_1khz(void)
{
    compare_count = 0;
    milliseconds = 0;

    // Setup Timer0 CTC Mode
    TCCR0 = 0x00;
    TCNT0 = 0;
    TIFR = (1 << OCF0);

    // CTC Mode with prescaler 64
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);

    // 1 kHz: OCR0 = 249
    OCR0 = 249;

    // Enable compare match interrupt
    TIMSK |= (1 << OCIE0);
    sei();

    PORTB &= ~(1 << PB0);

    while (1)
    {
        // Main loop FREE!
        // ISR increments milliseconds

        // Toggle LED every 500ms
        if (milliseconds >= 500)
        {
            PORTB ^= (1 << PB0);
            milliseconds = 0;
        }

        // Optional: Display milliseconds on other LEDs
        PORTB = ~((milliseconds >> 6) & 0x0F) | (PORTB & (1 << PB0));

        for (volatile uint16_t i = 0; i < 100; i++)
            asm volatile("nop");
    }
}

// =============================================================================
// DEMO 3: MULTIPLE FREQUENCIES
// =============================================================================
// Cycles through different frequencies: 1kHz, 2kHz, 4kHz, 8kHz
void demo3_multi_frequency(void)
{
    // Frequency settings: OCR0 values (all use prescaler 64)
    const uint8_t ocr_values[] = {
        249, // 1 kHz:  16MHz / 64 / 1000  - 1
        124, // 2 kHz:  16MHz / 64 / 2000  - 1
        62,  // 4 kHz:  16MHz / 64 / 4000  - 1
        31   // 8 kHz:  16MHz / 64 / 8000  - 1
    };

    const uint16_t toggle_delays[] = {
        500,  // 1 kHz: toggle every 500 matches (500ms)
        1000, // 2 kHz: toggle every 1000 matches (500ms)
        2000, // 4 kHz: toggle every 2000 matches (500ms)
        4000  // 8 kHz: toggle every 4000 matches (500ms)
    };

    uint8_t freq_index = 0;
    uint16_t toggle_count = 0;

    TCCR0 = 0x00;
    TCNT0 = 0;
    PORTB &= ~(1 << PB0);

    while (1)
    {
        // Set current frequency (all use prescaler 64)
        TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);
        OCR0 = ocr_values[freq_index];
        TIFR = (1 << OCF0);

        toggle_count = 0;

        // Run for ~2 seconds at this frequency (4 toggles = 2 seconds)
        for (uint8_t toggles = 0; toggles < 4; toggles++)
        {
            while (toggle_count < toggle_delays[freq_index])
            {
                if (TIFR & (1 << OCF0))
                {
                    TIFR = (1 << OCF0);
                    toggle_count++;
                }
            }
            PORTB ^= (1 << PB0);
            toggle_count = 0;
        }

        // Pause between frequencies
        _delay_ms(500);

        // Next frequency
        freq_index++;
        if (freq_index >= 4)
            freq_index = 0;
    }
}

// =============================================================================
// DEMO 4: PRECISION TEST
// =============================================================================
// Measure actual timing by counting compare matches
void demo4_precision_test(void)
{
    uint32_t match_count = 0;
    uint8_t test_seconds = 10;

    // Setup Timer0 for 1 kHz (easier to count)
    TCCR0 = 0x00;
    TCNT0 = 0;

    // 1 kHz: 16MHz / 64 / 1000 - 1 = 249
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);
    OCR0 = 249;
    TIFR = (1 << OCF0);

    PORTB &= ~(1 << PB0); // LED ON during test

    // Count matches for test_seconds
    // Expected: 1000 matches per second × 10 seconds = 10000 matches
    while (match_count < (1000UL * test_seconds))
    {
        if (TIFR & (1 << OCF0))
        {
            TIFR = (1 << OCF0);
            match_count++;

            // Toggle LED every 1000 matches (1 second)
            if (match_count % 1000 == 0)
                PORTB ^= (1 << PB0);
        }
    }

    // Test complete - blink rapidly
    while (1)
    {
        PORTB ^= (1 << PB0);
        _delay_ms(100);
    }

    // In real application, send match_count via UART
    // Expected: 10000, Actual: match_count
    // Error = (actual - expected) / expected × 100%
}

// =============================================================================
// LEARNING NOTES
// =============================================================================
/*
 * TIMER0 CTC MODE SPECIFICS:
 *
 * REGISTER DIFFERENCES (Timer0 vs Timer1):
 * - TCCR0 (single register) vs TCCR1A/TCCR1B (two registers)
 * - OCR0 (8-bit) vs OCR1A (16-bit)
 * - WGM01 bit for CTC vs WGM12 bit
 * - OCIE0 interrupt enable vs OCIE1A
 * - TIMER0_COMP_vect ISR vs TIMER1_COMPA_vect
 *
 * PRESCALER OPTIONS (Timer0):
 * - None (stopped)
 * - /8    (CS01=1, CS00=0)
 * - /64   (CS01=1, CS00=1) ← Most common for kHz range
 * - /256  (CS02=1)
 * - /1024 (CS02=1, CS00=1)
 * Note: Timer0 does NOT have /1 prescaler!
 *
 * 8-BIT LIMITATIONS:
 * - Max OCR0 value: 255
 * - Lower frequencies require larger prescalers
 * - For Hz range (1-100 Hz), use Timer1 instead
 * - Timer0 best for kHz range frequencies
 *
 * EXAMPLE CALCULATIONS:
 *
 * For 1 kHz (1ms period) with prescaler 64:
 * OCR0 = (16000000 / (64 × 1000)) - 1 = 249 ✓
 *
 * For 2 kHz (0.5ms period) with prescaler 64:
 * OCR0 = (16000000 / (64 × 2000)) - 1 = 124 ✓
 *
 * For 4 kHz (0.25ms period) with prescaler 64:
 * OCR0 = (16000000 / (64 × 4000)) - 1 = 62 ✓
 *
 * For 8 kHz (0.125ms period) with prescaler 64:
 * OCR0 = (16000000 / (64 × 8000)) - 1 = 31 ✓
 *
 * For 100 Hz (10ms period) with prescaler 256:
 * OCR0 = (16000000 / (256 × 100)) - 1 = 624 ✗ OVERFLOW! (>255)
 * → Use Timer1 for frequencies below ~1 kHz
 *
 * FREQUENCY RANGE GUIDE:
 *
 * With Prescaler 8:
 *   Max: ~2 MHz (OCR0=0)
 *   Min: ~7.8 kHz (OCR0=255)
 *
 * With Prescaler 64:
 *   Max: ~250 kHz (OCR0=0)
 *   Min: ~976 Hz (OCR0=255) ← Sweet spot for kHz range
 *
 * With Prescaler 256:
 *   Max: ~62.5 kHz (OCR0=0)
 *   Min: ~244 Hz (OCR0=255)
 *
 * With Prescaler 1024:
 *   Max: ~15.6 kHz (OCR0=0)
 *   Min: ~61 Hz (OCR0=255)
 *
 * WHEN TO USE TIMER0 CTC:
 * ✓ High-frequency signals (kHz range)
 * ✓ Millisecond-precision timing
 * ✓ Fast PWM generation (when combined with OC0)
 * ✓ When Timer1/Timer3 are busy with other tasks
 * ✓ Simple 8-bit comparisons
 *
 * WHEN TO USE TIMER1 INSTEAD:
 * ✓ Low frequencies (Hz range: 1-100 Hz)
 * ✓ Long periods (seconds)
 * ✓ Need 16-bit resolution
 * ✓ More prescaler options (/1 available)
 *
 * ADVANTAGES OF TIMER0:
 * ✓ Faster ISR (8-bit comparison)
 * ✓ Less memory overhead
 * ✓ Perfect for millisecond timing
 * ✓ Available when Timer1 is in use
 *
 * PRACTICAL APPLICATIONS:
 * - Millisecond timekeeping (millis() function)
 * - High-speed sampling (ADC at kHz rates)
 * - Audio tone generation (440 Hz = A note)
 * - Fast LED PWM control
 * - Communication bit timing (UART, SPI helpers)
 */
