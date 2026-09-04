/*
 * =============================================================================
 * TIMER1 INPUT CAPTURE - FREQUENCY MEASUREMENT DEMOS
 * =============================================================================
 * PROJECT: Timer1_Input_Capture
 *
 * LEARNING OBJECTIVES:
 * - Master Input Capture Unit (ICU)
 * - Measure external signal frequencies
 * - Calculate pulse width and duty cycle
 * - Handle timer overflow between captures
 *
 * HARDWARE:
 * - ICP1 pin (PD4 / SW4) - Input Capture Pin (Push Button 4)
 * - LED on PB0 (active LOW)
 * - ATmega128 @ 16 MHz
 *
 * NOTE: ICP1 is on PD4 where push button SW4 is connected.
 * Press SW4 to generate edges for testing input capture!
 *
 * THEORY - INPUT CAPTURE:
 * ICP1 pin detects edges (rising/falling)
 * Timer value is captured automatically in ICR1 register
 * No CPU intervention needed for capture!
 *
 * Frequency calculation:
 * F = F_CPU / (Prescaler × (Capture_new - Capture_old))
 * F = 16000000 / (64 × Ticks) = 250000 / Ticks Hz
 *
 * DEMOS:
 * 1. Polling - Check capture flag manually
 * 2. Interrupt - ISR handles captures
 * 3. Frequency Measurement - Calculate input frequency
 * 4. Pulse Width - Measure HIGH time
 *
 * SIMULIDE COMPATIBILITY:
 * ✅ SimulIDE 1.1.0-SR1: Fully functional (Timer1 Type 160 works perfectly)
 * ✅ SimulIDE 0.4.15: Fully functional
 * ✅ Hardware: Fully functional
 * Note: Timer1 Input Capture works in ALL versions (Type 160 stable)
 *
 * =============================================================================
 * TIMER1 INPUT CAPTURE REGISTER REFERENCE
 * =============================================================================
 *
 * TCCR1B - Timer/Counter1 Control Register B (Focus: Input Capture)
 * ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
 * │ ICNC1│ ICES1│  -   │ WGM13│ WGM12│ CS12 │ CS11 │ CS10 │
 * └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 * Bit:  7      6      5      4      3      2      1      0
 *
 * Input Capture Control Bits:
 * - ICNC1 (Bit 7): Input Capture Noise Canceler
 *   * 1 = Enable 4-clock filter (reduce glitches)
 *   * 0 = Disable (faster response)
 * - ICES1 (Bit 6): Input Capture Edge Select
 *   * 1 = Capture on rising edge (LOW→HIGH)
 *   * 0 = Capture on falling edge (HIGH→LOW)
 *
 * Clock Select (CS12:10) - Same as Timer1 CTC:
 * ┌─────┬─────┬─────┬──────────────┬────────────┬──────────────┐
 * │CS12 │CS11 │CS10 │   Source     │ @ 16 MHz   │  Resolution  │
 * ├─────┼─────┼─────┼──────────────┼────────────┼──────────────┤
 * │  0  │  0  │  0  │ Stopped      │     -      │      -       │
 * │  0  │  0  │  1  │ /1           │ 16.000 MHz │  62.5 ns     │
 * │  0  │  1  │  0  │ /8           │  2.000 MHz │  500 ns      │
 * │  0  │  1  │  1  │ /64          │  250 kHz   │  4.0 µs      │
 * │  1  │  0  │  0  │ /256         │ 62.5 kHz   │  16 µs       │
 * │  1  │  0  │  1  │ /1024        │ 15.625 kHz │  64 µs       │
 * └─────┴─────┴─────┴──────────────┴────────────┴──────────────┘
 *
 * TIMSK - Timer Interrupt Mask Register
 * ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
 * │OCIE2 │TOIE2 │TICIE1│OCIE1A│OCIE1B│TOIE1 │  -   │  -   │
 * └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 * Bit:  7      6      5      4      3      2      1      0
 *
 * Input Capture Interrupt:
 * - TICIE1 (Bit 5): Timer1 Input Capture Interrupt Enable
 *   * 1 = Enable TIMER1_CAPT_vect ISR
 *   * 0 = Polling mode only
 *
 * TIFR - Timer Interrupt Flag Register
 * ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
 * │ OCF2 │ TOV2 │ ICF1 │OCF1A │OCF1B │ TOV1 │  -   │  -   │
 * └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 * Bit:  7      6      5      4      3      2      1      0
 *
 * Input Capture Flag:
 * - ICF1 (Bit 5): Input Capture Flag
 *   * Set when edge detected on ICP1 pin
 *   * Clear by writing 1: TIFR |= (1 << ICF1);
 *   * Or automatically cleared when ISR executes
 *
 * FREQUENCY MEASUREMENT @ 16 MHz
 * ┌──────────┬─────────────┬──────────┬───────────┬─────────────┐
 * │Prescaler │ Timer Freq  │ Min Freq │  Max Freq │  Best Range │
 * ├──────────┼─────────────┼──────────┼───────────┼─────────────┤
 * │    /1    │  16.000 MHz │  244 Hz  │  8.0 MHz  │ > 10 kHz    │
 * │    /8    │   2.000 MHz │   31 Hz  │  1.0 MHz  │  1k - 100k  │
 * │   /64    │   250 kHz   │    4 Hz  │  125 kHz  │ 10 - 10k Hz │
 * │   /256   │  62.5 kHz   │    1 Hz  │ 31.25 kHz │  1 - 1k Hz  │
 * │  /1024   │ 15.625 kHz  │  0.24 Hz │  7.8 kHz  │ < 1 Hz      │
 * └──────────┴─────────────┴──────────┴───────────┴─────────────┘
 *
 * Calculation: F_signal = F_timer / (Capture2 - Capture1)
 * Example (Prescaler /64):
 *   - Timer freq = 250 kHz
 *   - 2500 ticks → 100 Hz
 *   - 250 ticks → 1 kHz
 *   - 25 ticks → 10 kHz
 *
 * PULSE WIDTH MEASUREMENT
 * Method: Toggle edge detection (rising → falling → rising)
 * 1. Capture on rising edge (start of pulse)
 * 2. Switch to falling edge
 * 3. Capture on falling edge (end of pulse)
 * 4. Calculate: Pulse_Width = Capture_Fall - Capture_Rise
 * 5. Convert to time: Time_us = Pulse_Width × (Prescaler / 16)
 *
 * Example (Prescaler /8):
 *   - Each tick = 0.5 µs
 *   - 2000 ticks = 1000 µs = 1 ms pulse
 *
 * COMMON USE CASES
 * ┌──────────────────────┬────────────┬─────────────────────┐
 * │    Application       │ Prescaler  │    Typical Range    │
 * ├──────────────────────┼────────────┼─────────────────────┤
 * │ Servo PWM (50 Hz)    │    /64     │  1-2 ms pulses      │
 * │ Ultrasonic Echo      │    /8      │  150 µs - 25 ms     │
 * │ RPM Sensor           │   /256     │  1-100 Hz           │
 * │ Frequency Counter    │    /64     │  10 Hz - 10 kHz     │
 * │ IR Remote (38 kHz)   │    /8      │  560 µs - 2.25 ms   │
 * └──────────────────────┴────────────┴─────────────────────┘
 *
 * PIN ASSIGNMENT
 * - ICP1: Port D, Pin 4 (PD4) - Input Capture Pin 1 (Push Button SW4)
 * - Configure as input: DDRD &= ~(1 << PD4);
 * - Enable pull-up: PORTD |= (1 << PD4);  // Required for button
 *
 * TESTING: Press SW4 (button on PD4) to trigger input capture events!
 *
 * =============================================================================
 */

#include "config.h"

// Global variables
volatile uint16_t capture_value = 0;
volatile uint16_t previous_capture = 0;
volatile uint16_t pulse_width = 0;
volatile uint8_t capture_ready = 0;

// Function prototypes
void demo1_polling_capture(void);
void demo2_interrupt_capture(void);
void demo3_frequency_meter(void);
void demo4_pulse_width(void);

// ===== INTERRUPT SERVICE ROUTINE =====
ISR(TIMER1_CAPT_vect)
{
    capture_value = ICR1; // Read captured timer value

    // Calculate difference (handles single overflow)
    if (capture_value >= previous_capture)
        pulse_width = capture_value - previous_capture;
    else
        pulse_width = (0xFFFF - previous_capture) + capture_value + 1;

    previous_capture = capture_value;
    capture_ready = 1;
}

// ===== MAIN - SELECT YOUR DEMO =====
int main(void)
{
    // Initialize hardware
    DDRB = 0xFF;         // Port B as OUTPUT
    PORTB = 0xFF;        // LEDs OFF
    DDRD &= ~(1 << PD4); // PD4 (ICP1) as INPUT
    PORTD |= (1 << PD4); // Enable pull-up (required for button SW4)

    // Uncomment ONE demo to run:
    // demo1_polling_capture(); // Poll capture flag
    // demo2_interrupt_capture();  // Interrupt-driven
    // demo3_frequency_meter();    // Measure frequency
    // demo4_pulse_width();        // Measure pulse width

    while (1)
    {
    }
    return 0;
}

// =============================================================================
// DEMO 1: POLLING METHOD
// =============================================================================
// Check capture flag manually, toggle LED on each capture
void demo1_polling_capture(void)
{
    // Setup Timer1
    TCCR1A = 0x00;      // Normal port operation
    TCCR1B = 0x00;      // Stop timer
    TCNT1 = 0;          // Clear counter
    TIFR = (1 << ICF1); // Clear capture flag

    // Input Capture on rising edge, prescaler 64
    // ICES1=1 (rising edge), CS11=1 CS10=1 (prescaler 64)
    TCCR1B = (1 << ICES1) | (1 << CS11) | (1 << CS10);

    PORTB &= ~(1 << PB0); // LED ON

    while (1)
    {
        // Poll Input Capture Flag
        if (TIFR & (1 << ICF1))
        {
            capture_value = ICR1; // Read captured value
            TIFR = (1 << ICF1);   // Clear flag
            PORTB ^= (1 << PB0);  // Toggle LED on each capture
        }
    }
}

// =============================================================================
// DEMO 2: INTERRUPT METHOD
// =============================================================================
// ISR handles captures automatically
void demo2_interrupt_capture(void)
{
    capture_ready = 0;
    previous_capture = 0;

    // Setup Timer1
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1 = 0;
    TIFR = (1 << ICF1);

    // Rising edge, prescaler 64
    TCCR1B = (1 << ICES1) | (1 << CS11) | (1 << CS10);

    // Enable Input Capture interrupt
    TIMSK |= (1 << TICIE1);
    sei();

    PORTB &= ~(1 << PB0);

    while (1)
    {
        if (capture_ready)
        {
            capture_ready = 0;
            PORTB ^= (1 << PB0); // Toggle LED

            // Display pulse_width on LEDs (lower 8 bits)
            PORTB = ~(pulse_width & 0xFF);
        }

        for (volatile uint16_t i = 0; i < 100; i++)
            asm volatile("nop");
    }
}

// =============================================================================
// DEMO 3: FREQUENCY MEASUREMENT
// =============================================================================
// Measure frequency of signal on ICP1
// Displays frequency using LED patterns
void demo3_frequency_meter(void)
{
    uint16_t ticks = 0;
    uint32_t frequency = 0;

    // Setup Timer1
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1 = 0;
    TIFR = (1 << ICF1);

    // Rising edge, prescaler 64
    // Frequency range: 4 Hz to 250 kHz
    TCCR1B = (1 << ICES1) | (1 << CS11) | (1 << CS10);

    previous_capture = 0;

    while (1)
    {
        // Wait for first capture
        while (!(TIFR & (1 << ICF1)))
            ;
        previous_capture = ICR1;
        TIFR = (1 << ICF1);

        // Wait for second capture
        while (!(TIFR & (1 << ICF1)))
            ;
        capture_value = ICR1;
        TIFR = (1 << ICF1);

        // Calculate ticks between captures
        if (capture_value >= previous_capture)
            ticks = capture_value - previous_capture;
        else
            ticks = (0xFFFF - previous_capture) + capture_value + 1;

        // Calculate frequency: F = 250000 / ticks
        // (16MHz / 64 = 250000 Hz)
        if (ticks > 0)
            frequency = 250000UL / ticks;

        // Display frequency range on LEDs:
        // < 10 Hz: 1 LED
        // 10-100 Hz: 2 LEDs
        // 100-1kHz: 3 LEDs
        // > 1kHz: 4 LEDs
        if (frequency < 10)
            PORTB = ~0x01;
        else if (frequency < 100)
            PORTB = ~0x03;
        else if (frequency < 1000)
            PORTB = ~0x07;
        else
            PORTB = ~0x0F;

        _delay_ms(100); // Update rate
    }
}

// =============================================================================
// DEMO 4: PULSE WIDTH MEASUREMENT
// =============================================================================
// Measure HIGH pulse width using edge toggling
void demo4_pulse_width(void)
{
    uint16_t high_time = 0;
    uint8_t edge = 1; // 1=rising, 0=falling

    // Setup Timer1
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1 = 0;

    // Start with rising edge, prescaler 64
    TCCR1B = (1 << ICES1) | (1 << CS11) | (1 << CS10);

    while (1)
    {
        // Wait for capture
        while (!(TIFR & (1 << ICF1)))
            ;

        if (edge) // Rising edge - start of pulse
        {
            previous_capture = ICR1;
            edge = 0;
            // Switch to falling edge detection
            TCCR1B &= ~(1 << ICES1);
            PORTB &= ~(1 << PB0); // LED ON
        }
        else // Falling edge - end of pulse
        {
            capture_value = ICR1;
            edge = 1;
            // Switch to rising edge detection
            TCCR1B |= (1 << ICES1);

            // Calculate pulse width
            if (capture_value >= previous_capture)
                high_time = capture_value - previous_capture;
            else
                high_time = (0xFFFF - previous_capture) + capture_value + 1;

            // Display pulse width on LEDs (scaled)
            PORTB = ~((high_time >> 8) & 0xFF);
            _delay_ms(10);
        }

        TIFR = (1 << ICF1); // Clear flag
    }
}

// =============================================================================
// LEARNING NOTES
// =============================================================================
/*
 * INPUT CAPTURE ADVANTAGES:
 * ✓ Hardware captures timer value automatically
 * ✓ No CPU cycles wasted polling
 * ✓ Precise timing measurement
 * ✓ Can detect both edges
 *
 * TYPICAL APPLICATIONS:
 * - Frequency counters
 * - Tachometers (RPM measurement)
 * - Pulse width modulation (PWM) decoders
 * - Ultrasonic distance sensors
 * - IR remote control receivers
 *
 * FREQUENCY CALCULATION:
 * With prescaler 64:
 * Timer clock = 16MHz / 64 = 250 kHz
 * If signal has period of N timer ticks:
 * Frequency = 250000 / N Hz
 *
 * Example:
 * - 2500 ticks → 100 Hz
 * - 250 ticks → 1 kHz
 * - 25 ticks → 10 kHz
 *
 * EDGE DETECTION:
 * ICES1 bit in TCCR1B selects edge:
 * ICES1 = 1: Rising edge (LOW → HIGH)
 * ICES1 = 0: Falling edge (HIGH → LOW)
 *
 * OVERFLOW HANDLING:
 * If time between captures > 65535 ticks:
 * - Timer overflows
 * - Need to count overflows
 * - Final ticks = (overflows × 65536) + difference
 *
 * PRESCALER SELECTION:
 * - Fast signals (>1kHz): Use small prescaler (1, 8)
 * - Medium signals (10Hz-1kHz): Use prescaler 64
 * - Slow signals (<10Hz): Use prescaler 256 or 1024
 */
