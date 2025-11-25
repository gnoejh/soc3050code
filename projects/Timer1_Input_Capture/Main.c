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
 * - ICP1 pin (PE7) - Input Capture Pin
 * - LED on PB0 (active LOW)
 * - ATmega128 @ 16 MHz
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
    DDRE &= ~(1 << PE7); // PE7 (ICP1) as INPUT
    PORTE |= (1 << PE7); // Enable pull-up

    // Uncomment ONE demo to run:
    demo1_polling_capture(); // Poll capture flag
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
