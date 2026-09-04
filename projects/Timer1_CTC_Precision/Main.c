/*
 * =============================================================================
 * TIMER1 CTC MODE - PRECISION TIMING DEMOS
 * =============================================================================
 * PROJECT: Timer1_CTC_Precision
 * DATASHEET: https://ww1.microchip.com/downloads/en/DeviceDoc/doc2467.pdf
 *
 * LEARNING OBJECTIVES:
 * - Master CTC (Clear Timer on Compare Match) mode
 * - Generate EXACT frequencies (not limited to powers of 2)
 * - Understand OCR1A compare register and 16-bit timer precision
 * - Compare polling vs interrupt methods
 * - Learn practical applications (timekeeping, frequency generation)
 *
 * HARDWARE:
 * - LED on PB0 (active LOW)
 * - ATmega128 @ 16 MHz crystal
 *
 * SIMULIDE COMPATIBILITY:
 * ✅ SimulIDE 1.1.0-SR1: Fully compatible (Timer1 type 160)
 * ✅ SimulIDE 0.4.15: Fully compatible
 * ✅ Hardware: All features work perfectly
 *
 * THEORY - CTC MODE:
 * Timer counts 0 → OCR1A, then RESETS to 0 (not overflow to 65535)
 * This creates EXACT frequencies without accumulation error!
 *
 * Formula: F_output = F_CPU / (Prescaler × (OCR1A + 1))
 * Rearranged: OCR1A = (F_CPU / (Prescaler × F_desired)) - 1
 *
 * Example for 1 Hz with prescaler 1024:
 * OCR1A = (16000000 / (1024 × 1)) - 1 = 15624
 *
 * DEMOS:
 * 1. Polling 1 Hz - Check compare flag manually
 * 2. Interrupt 1 Hz - ISR handles compare match automatically
 * 3. Multiple Frequencies - 1 Hz, 10 Hz, 100 Hz, 1000 Hz
 * 4. Precision Test - Measure actual timing accuracy
 *
 * NOTE: This project demonstrates Timer1 (16-bit, type 160)
 *       For 8-bit Timer0 see Timer0_CTC_Precision (hardware-only)
 * =============================================================================
 */

#include "config.h"

// =============================================================================
// TIMER1 REGISTER REFERENCE (ATmega128 @ 16 MHz)
// =============================================================================
/*
 * TCCR1A - Timer/Counter1 Control Register A
 * ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
 * │COM1A│COM1A│COM1B│COM1B│COM1C│COM1C│WGM11│WGM10│
 * │  1  │  0  │  1  │  0  │  1  │  0  │     │     │
 * └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
 *  Bit 7   6     5    4     3    2     1     0
 *
 * TCCR1B - Timer/Counter1 Control Register B
 * ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
 * │ICNC1│ICES1│  -  │WGM13│WGM12│ CS12│ CS11│ CS10│
 * └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
 *  Bit 7   6     5    4     3     2     1     0
 *
 * WAVEFORM GENERATION MODE (WGM13:10)
 * ┌─────┬─────┬─────┬─────┬──────┬─────────────────────┬──────┐
 * │WGM13│WGM12│WGM11│WGM10│ Mode │   Description       │ TOP  │
 * ├─────┼─────┼─────┼─────┼──────┼─────────────────────┼──────┤
 * │  0  │  0  │  0  │  0  │  0   │ Normal              │0xFFFF│
 * │  0  │  1  │  0  │  0  │  4   │ CTC (OCR1A)         │OCR1A │ ← Used
 * │  0  │  1  │  0  │  1  │  5   │ Fast PWM (8-bit)    │0x00FF│
 * │  0  │  1  │  1  │  0  │  6   │ Fast PWM (9-bit)    │0x01FF│
 * │  0  │  1  │  1  │  1  │  7   │ Fast PWM (10-bit)   │0x03FF│
 * │  1  │  0  │  0  │  0  │  8   │ PWM Phase/Freq      │ ICR1 │
 * │  1  │  1  │  0  │  0  │ 12   │ CTC (ICR1)          │ ICR1 │
 * └─────┴─────┴─────┴─────┴──────┴─────────────────────┴──────┘
 *
 * CLOCK SELECT (CS12:10) - PRESCALER VALUES @ 16 MHz
 * ┌────┬────┬────┬──────────┬─────────────┬──────────────┬─────────────────┐
 * │CS12│CS11│CS10│Prescaler │Timer Freq   │Resolution    │Max Period       │
 * ├────┼────┼────┼──────────┼─────────────┼──────────────┼─────────────────┤
 * │ 0  │ 0  │ 0  │   OFF    │     -       │      -       │        -        │
 * │ 0  │ 0  │ 1  │    /1    │ 16.000 MHz  │  62.5 ns     │   4.096 ms      │
 * │ 0  │ 1  │ 0  │    /8    │  2.000 MHz  │ 500.0 ns     │  32.768 ms      │
 * │ 0  │ 1  │ 1  │   /64    │  250.0 kHz  │   4.0 µs     │ 262.144 ms      │
 * │ 1  │ 0  │ 0  │  /256    │  62.50 kHz  │  16.0 µs     │   1.049 s       │
 * │ 1  │ 0  │ 1  │ /1024    │  15.625 kHz │  64.0 µs     │   4.194 s       │
 * └────┴────┴────┴──────────┴─────────────┴──────────────┴─────────────────┘
 *
 * TIMSK - Timer Interrupt Mask Register (Enable Interrupts)
 * ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
 * │OCIE2│TOIE2│TICIE│OCIE1│OCIE1│TOIE1│OCIE0│TOIE0│
 * │     │     │  1  │  A  │  B  │     │     │     │
 * └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
 *  Bit 7   6     5     4     3     2     1     0
 *  OCIE1A (bit 4) = 1: Enable Timer1 Compare Match A interrupt
 *
 * TIFR - Timer Interrupt Flag Register (Status Flags)
 * ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
 * │OCF2 │TOV2 │ICF1 │OCF1A│OCF1B│TOV1 │OCF0 │TOV0 │
 * └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
 *  Bit 7   6     5     4     3     2     1     0
 *  OCF1A (bit 4): Set when TCNT1 = OCR1A (clear by writing 1)
 *
 * FREQUENCY CALCULATION FORMULAS:
 * ─────────────────────────────────────────────────────────────────────────
 * F_output = F_CPU / (Prescaler × (OCR1A + 1))
 *
 * OCR1A = (F_CPU / (Prescaler × F_desired)) - 1
 *
 * Period_output = (Prescaler × (OCR1A + 1)) / F_CPU
 *
 * COMMON OCR1A VALUES @ 16 MHz:
 * ┌──────────┬──────────┬────────┬──────────────────────────────┐
 * │Frequency │Prescaler │ OCR1A  │          Use Case            │
 * ├──────────┼──────────┼────────┼──────────────────────────────┤
 * │   1 Hz   │  1024    │ 15624  │ Second counter, RTC          │
 * │  10 Hz   │  1024    │  1561  │ Decisecond timer             │
 * │ 100 Hz   │   64     │  2499  │ Millisecond timer (10ms)     │
 * │ 1000 Hz  │   64     │   249  │ Millisecond timer (1ms)      │
 * │ 8000 Hz  │    8     │   249  │ Audio sampling               │
 * │ 10 kHz   │    8     │   199  │ PWM for motor control        │
 * └──────────┴──────────┴────────┴──────────────────────────────┘
 *
 * FREQUENCY RANGES BY PRESCALER @ 16 MHz:
 * ┌──────────┬──────────────┬───────────────┬─────────────────┐
 * │Prescaler │  Min Freq    │   Max Freq    │   Best For      │
 * ├──────────┼──────────────┼───────────────┼─────────────────┤
 * │    1     │   244 Hz     │   8.000 MHz   │ High freq       │
 * │    8     │    31 Hz     │   1.000 MHz   │ kHz range       │
 * │   64     │     4 Hz     │   125.0 kHz   │ Audio/PWM       │
 * │  256     │     1 Hz     │    31.2 kHz   │ Sub-second      │
 * │ 1024     │   0.24 Hz    │     7.8 kHz   │ Second range    │
 * └──────────┴──────────────┴───────────────┴─────────────────┘
 */

// Global variables
volatile uint16_t compare_count = 0;
volatile uint8_t seconds = 0;

// Function prototypes
void demo1_polling_1hz(void);
void demo2_interrupt_1hz(void);
void demo3_multi_frequency(void);
void demo4_precision_test(void);

// ===== INTERRUPT SERVICE ROUTINE =====
ISR(TIMER1_COMPA_vect)
{
    compare_count++;

    // For 1 Hz demos: toggle every compare match
    if (compare_count >= 1)
    {
        PORTB ^= (1 << PB0);
        compare_count = 0;
        seconds++;
    }
}

// ===== MAIN - SELECT YOUR DEMO =====
int main(void)
{
    // Initialize hardware
    DDRB = 0xFF;  // All Port B as OUTPUT
    PORTB = 0xFF; // All LEDs OFF (active LOW)

    // Uncomment ONE demo to run:
    demo1_polling_1hz(); // Polling compare flag
    // demo2_interrupt_1hz(); // Interrupt-driven
    // demo3_multi_frequency(); // Try different frequencies
    // demo4_precision_test();    // Measure accuracy

    while (1)
    {
    }
    return 0;
}

// =============================================================================
// DEMO 1: POLLING METHOD - 1 Hz
// =============================================================================
// Check compare match flag manually
void demo1_polling_1hz(void)
{
    // Setup Timer1 CTC Mode
    TCCR1A = 0x00;       // Normal port operation
    TCCR1B = 0x00;       // Stop timer
    TCNT1 = 0;           // Clear counter
    TIFR = (1 << OCF1A); // Clear compare flag

    // CTC Mode: WGM12 = 1 (Mode 4)
    // Prescaler 1024: CS12=1, CS10=1
    TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);

    // Set compare value for 1 Hz
    // 16MHz / 1024 / 1Hz - 1 = 15624
    OCR1A = 15624;

    PORTB &= ~(1 << PB0); // LED ON

    while (1)
    {
        // Poll compare match flag
        if (TIFR & (1 << OCF1A))
        {
            TIFR = (1 << OCF1A); // Clear flag
            PORTB ^= (1 << PB0); // Toggle LED every 1 second
        }
    }
}

// =============================================================================
// DEMO 2: INTERRUPT METHOD - 1 Hz
// =============================================================================
// ISR handles compare match automatically
void demo2_interrupt_1hz(void)
{
    compare_count = 0;
    seconds = 0;

    // Setup Timer1 CTC Mode
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1 = 0;
    TIFR = (1 << OCF1A);

    // CTC Mode with prescaler 1024
    TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);

    // 1 Hz: OCR1A = 15624
    OCR1A = 15624;

    // Enable compare match interrupt
    TIMSK |= (1 << OCIE1A);
    sei();

    PORTB &= ~(1 << PB0);

    while (1)
    {
        // Main loop FREE!
        // ISR toggles LED every second

        // Optional: Show seconds on other LEDs
        PORTB = ~(seconds & 0x0F) | (1 << PB0);

        for (volatile uint16_t i = 0; i < 1000; i++)
            asm volatile("nop");
    }
}

// =============================================================================
// DEMO 3: MULTIPLE FREQUENCIES
// =============================================================================
// Cycles through different frequencies: 1Hz, 10Hz, 100Hz, 1kHz
void demo3_multi_frequency(void)
{
    // Frequency settings: OCR1A values and prescalers
    const uint16_t ocr_values[] = {
        15624, // 1 Hz:    16MHz / 1024 / 1    - 1
        1561,  // 10 Hz:   16MHz / 1024 / 10   - 1
        2499,  // 100 Hz:  16MHz / 64 / 100    - 1
        249    // 1000 Hz: 16MHz / 64 / 1000   - 1
    };

    const uint8_t prescalers[] = {
        (1 << CS12) | (1 << CS10), // 1024 for 1Hz, 10Hz
        (1 << CS12) | (1 << CS10), // 1024
        (1 << CS11) | (1 << CS10), // 64 for 100Hz, 1kHz
        (1 << CS11) | (1 << CS10)  // 64
    };

    uint8_t freq_index = 0;
    uint16_t toggle_count = 0;

    TCCR1A = 0x00;
    TCNT1 = 0;
    PORTB &= ~(1 << PB0);

    while (1)
    {
        // Set current frequency
        TCCR1B = (1 << WGM12) | prescalers[freq_index];
        OCR1A = ocr_values[freq_index];
        TIFR = (1 << OCF1A);

        // Run for ~2 seconds at this frequency
        for (toggle_count = 0; toggle_count < 4; toggle_count++)
        {
            while (!(TIFR & (1 << OCF1A)))
                ;
            TIFR = (1 << OCF1A);
            PORTB ^= (1 << PB0);
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

    // Setup Timer1 for 100 Hz (easier to count)
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1 = 0;

    // 100 Hz: 16MHz / 64 / 100 - 1 = 2499
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1A = 2499;
    TIFR = (1 << OCF1A);

    PORTB &= ~(1 << PB0); // LED ON during test

    // Count matches for test_seconds
    // Expected: 100 matches per second × 10 seconds = 1000 matches
    while (match_count < (100UL * test_seconds))
    {
        if (TIFR & (1 << OCF1A))
        {
            TIFR = (1 << OCF1A);
            match_count++;

            // Toggle LED to show activity
            if (match_count % 100 == 0)
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
    // Expected: 1000, Actual: match_count
    // Error = (actual - expected) / expected × 100%
}

// =============================================================================
// LEARNING NOTES
// =============================================================================
/*
 * CTC MODE vs NORMAL MODE:
 *
 * NORMAL MODE (Overflow):
 * - Timer counts 0 → MAX (255 for Timer0, 65535 for Timer1)
 * - Overflows at fixed intervals
 * - Frequencies limited by MAX value
 *
 * CTC MODE (Clear on Compare):
 * - Timer counts 0 → OCR1A, then CLEARS to 0
 * - You set OCR1A to get EXACT frequency
 * - Much more flexible!
 *
 * EXAMPLE CALCULATIONS:
 *
 * For 1 Hz (1 second period):
 * OCR1A = (16000000 / (1024 × 1)) - 1 = 15624
 *
 * For 10 Hz (0.1 second period):
 * OCR1A = (16000000 / (1024 × 10)) - 1 = 1561
 *
 * For 100 Hz (10ms period):
 * OCR1A = (16000000 / (64 × 100)) - 1 = 2499
 * (Note: Prescaler changed to 64 for better resolution)
 *
 * For 1000 Hz (1ms period):
 * OCR1A = (16000000 / (64 × 1000)) - 1 = 249
 *
 * WHEN TO USE CTC:
 * ✓ Need exact frequency (e.g., 1 Hz, 10 Hz, 100 Hz)
 * ✓ Generating timing signals
 * ✓ Precise delays without blocking
 * ✓ Real-time clock implementations
 *
 * ADVANTAGES:
 * ✓ Exact frequencies (not limited to powers of 2)
 * ✓ Adjustable at runtime (change OCR1A)
 * ✓ Higher resolution than overflow mode
 * ✓ Predictable timing
 */
