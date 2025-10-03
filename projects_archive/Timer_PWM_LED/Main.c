/*
 * ===========================================================================
 * TIMER PWM - LED Brightness Control
 * ATmega128 @ 16MHz
 * ===========================================================================
 * FOCUS: Using hardware PWM for LED dimming
 *
 * PWM (Pulse Width Modulation) CONCEPT:
 * PWM creates an analog-like effect using digital signals by rapidly
 * switching between HIGH and LOW. The ratio of ON time to total time
 * is called the "duty cycle."
 *
 * Duty Cycle = (ON time / Period) × 100%
 *
 * For LED brightness:
 * - 0% duty cycle = LED OFF (always LOW)
 * - 50% duty cycle = LED at half brightness
 * - 100% duty cycle = LED fully ON (always HIGH)
 *
 * FAST PWM MODE:
 * - Timer counts from 0 to TOP (255 for 8-bit, OCR for custom)
 * - Compares TCNT with OCR value
 * - Output pin toggles at compare match
 * - Creates PWM signal automatically in hardware
 *
 * LEARNING OBJECTIVES:
 * 1. Configure Fast PWM mode
 * 2. Set PWM frequency
 * 3. Control duty cycle with OCR register
 * 4. Create smooth fading effects
 * 5. Understand hardware vs software PWM
 *
 * HARDWARE:
 * - ATmega128 @ 16MHz
 * - LED on PB5 (OC1A - Timer1 output compare pin)
 * - LED on PB4 (OC0 - Timer0 output compare pin)
 */

#include "config.h"

// ============================================================================
// Demo 1: Basic Fast PWM - 50% Duty Cycle
// ============================================================================
/*
 * TIMER0 FAST PWM (8-bit):
 * - Counts from 0 to 255
 * - OCR0 sets the duty cycle
 * - OC0 pin (PB4) outputs PWM signal
 *
 * Formula:
 * Duty Cycle = OCR0 / 256 × 100%
 *
 * For 50%: OCR0 = 128
 */

void demo_01_basic_pwm_50_percent(void)
{
    // Configure PB4 (OC0) as output
    DDRB |= (1 << 4);

    /*
     * TIMER0 CONTROL REGISTER (TCCR0):
     *
     * COM01:COM00 = 10: Clear OC0 on compare match, set at BOTTOM
     *                   (non-inverting mode)
     * WGM01:WGM00 = 11: Fast PWM mode
     * CS02:CS00 = 011: Prescaler = 64
     */
    TCCR0 = (1 << WGM01) | (1 << WGM00)  // Fast PWM mode
            | (1 << COM01)               // Non-inverting mode
            | (1 << CS01) | (1 << CS00); // Prescaler = 64

    /*
     * OUTPUT COMPARE REGISTER 0 (OCR0):
     * Sets the duty cycle
     * 0 = 0% (always LOW)
     * 128 = 50% (half brightness)
     * 255 = 100% (always HIGH)
     */
    OCR0 = 128; // 50% duty cycle

    while (1)
    {
        // LED at 50% brightness automatically!
        // No code needed - hardware PWM runs continuously
    }
}

// ============================================================================
// Demo 2: Variable Brightness with Button
// ============================================================================
/*
 * Use button to cycle through brightness levels:
 * 0% → 25% → 50% → 75% → 100% → 0% ...
 */

void demo_02_variable_brightness(void)
{
    uint8_t brightness_levels[] = {0, 64, 128, 192, 255};
    uint8_t level_index = 2; // Start at 50%

    uint8_t button_last = 1;
    uint8_t button_current;

    // Configure PWM output
    DDRB |= (1 << 4);

    // Configure button
    DDRD &= ~(1 << 7);
    PORTD |= (1 << 7); // Pull-up

    // Fast PWM mode, prescaler 64
    TCCR0 = (1 << WGM01) | (1 << WGM00) | (1 << COM01) | (1 << CS01) | (1 << CS00);

    // Initial brightness
    OCR0 = brightness_levels[level_index];

    while (1)
    {
        // Read button
        button_current = (PIND & (1 << 7)) ? 1 : 0;

        // Button pressed - change brightness
        if (button_last == 1 && button_current == 0)
        {
            level_index++;
            if (level_index >= 5)
                level_index = 0;

            OCR0 = brightness_levels[level_index];
        }

        button_last = button_current;
        _delay_ms(50); // Debounce
    }
}

// ============================================================================
// Demo 3: Smooth Fading (Breathing Effect)
// ============================================================================
/*
 * Create smooth fade in/out effect
 * Gradually increase then decrease brightness
 */

void demo_03_smooth_fading(void)
{
    uint8_t brightness = 0;
    int8_t direction = 1; // 1 = increasing, -1 = decreasing

    // Configure PWM output
    DDRB |= (1 << 4);

    // Fast PWM mode
    TCCR0 = (1 << WGM01) | (1 << WGM00) | (1 << COM01) | (1 << CS01) | (1 << CS00);

    while (1)
    {
        // Update brightness
        OCR0 = brightness;

        // Change brightness
        brightness += direction;

        // Reverse direction at limits
        if (brightness == 255)
            direction = -1;
        else if (brightness == 0)
            direction = 1;

        _delay_ms(5); // Smooth fading speed
    }
}

// ============================================================================
// Demo 4: Dual LED Control (Timer0 + Timer1)
// ============================================================================
/*
 * Control two LEDs independently using two timers
 * LED1: Fast breathing
 * LED2: Slow breathing
 */

void demo_04_dual_led_pwm(void)
{
    uint8_t brightness1 = 0;
    uint8_t brightness2 = 0;
    int8_t direction1 = 1;
    int8_t direction2 = 1;
    uint8_t counter = 0;

    // Configure both PWM outputs
    DDRB |= (1 << 4); // OC0 (Timer0)
    DDRB |= (1 << 5); // OC1A (Timer1)

    // Timer0: 8-bit Fast PWM
    TCCR0 = (1 << WGM01) | (1 << WGM00) | (1 << COM01) | (1 << CS01) | (1 << CS00);

    /*
     * Timer1: 8-bit Fast PWM (WGM12:10 = 101)
     * Using ICR1 as TOP for 8-bit operation
     */
    TCCR1A = (1 << COM1A1)                             // Non-inverting mode on OC1A
             | (1 << WGM10);                           // Fast PWM 8-bit
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // Prescaler = 64

    while (1)
    {
        // LED1: Fast breathing (Timer0)
        OCR0 = brightness1;
        brightness1 += direction1;
        if (brightness1 == 255)
            direction1 = -1;
        else if (brightness1 == 0)
            direction1 = 1;

        // LED2: Slow breathing (Timer1)
        // Update only every 4 cycles
        counter++;
        if (counter >= 4)
        {
            counter = 0;
            OCR1A = brightness2;
            brightness2 += direction2;
            if (brightness2 == 255)
                direction2 = -1;
            else if (brightness2 == 0)
                direction2 = 1;
        }

        _delay_ms(5);
    }
}

// ============================================================================
// Demo 5: RGB LED Color Mixing (3 PWM channels)
// ============================================================================
/*
 * Control RGB LED using 3 PWM outputs
 * Create different colors by mixing Red, Green, Blue
 *
 * Requires:
 * - Common cathode RGB LED or 3 separate LEDs
 * - Timer0: Red (PB4/OC0)
 * - Timer1: Green (PB5/OC1A)
 * - Timer3: Blue (PB6/OC1B) or software PWM
 */

void demo_05_rgb_color_mixing(void)
{
    // Color table: {Red, Green, Blue}
    uint8_t colors[][3] = {
        {255, 0, 0},     // Red
        {0, 255, 0},     // Green
        {0, 0, 255},     // Blue
        {255, 255, 0},   // Yellow
        {255, 0, 255},   // Magenta
        {0, 255, 255},   // Cyan
        {255, 255, 255}, // White
        {128, 0, 128}    // Purple
    };

    uint8_t color_index = 0;

    // Configure 3 outputs for RGB
    DDRB |= (1 << 4) | (1 << 5) | (1 << 6);

    // Timer0 for Red
    TCCR0 = (1 << WGM01) | (1 << WGM00) | (1 << COM01) | (1 << CS01) | (1 << CS00);

    // Timer1 for Green and Blue
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) // Both OC1A and OC1B
             | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    while (1)
    {
        // Set color
        OCR0 = colors[color_index][0];  // Red
        OCR1A = colors[color_index][1]; // Green
        OCR1B = colors[color_index][2]; // Blue

        _delay_ms(1000); // Display each color for 1 second

        // Next color
        color_index++;
        if (color_index >= 8)
            color_index = 0;
    }
}

// ============================================================================
// Demo 6: Brightness Control with Potentiometer (ADC + PWM)
// ============================================================================
/*
 * Read potentiometer with ADC
 * Use ADC value to control LED brightness
 * Demonstrates ADC + PWM integration
 */

void demo_06_adc_controlled_brightness(void)
{
    uint16_t adc_value;
    uint8_t brightness;

    // Configure PWM output
    DDRB |= (1 << 4);

    // Configure ADC on PA0
    DDRA &= ~(1 << 0); // Input

    // ADC setup
    ADMUX = (1 << REFS0);                                  // AVCC reference, ADC0
    ADCSRA = (1 << ADEN)                                   // Enable ADC
             | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128

    // Fast PWM mode
    TCCR0 = (1 << WGM01) | (1 << WGM00) | (1 << COM01) | (1 << CS01) | (1 << CS00);

    while (1)
    {
        // Start ADC conversion
        ADCSRA |= (1 << ADSC);

        // Wait for conversion
        while (ADCSRA & (1 << ADSC))
            ;

        // Read 10-bit ADC value
        adc_value = ADC;

        // Convert to 8-bit for PWM (0-1023 → 0-255)
        brightness = adc_value >> 2;

        // Update PWM
        OCR0 = brightness;

        _delay_ms(10); // Small delay
    }
}

// ============================================================================
// MAIN - Select Your Demo
// ============================================================================
/*
 * LEARNING PROGRESSION:
 *
 * 1. demo_01_basic_pwm_50_percent       - Basic PWM at 50%
 * 2. demo_02_variable_brightness        - Button-controlled levels
 * 3. demo_03_smooth_fading              - Breathing effect
 * 4. demo_04_dual_led_pwm               - Two independent LEDs
 * 5. demo_05_rgb_color_mixing           - RGB color control
 * 6. demo_06_adc_controlled_brightness  - Potentiometer control
 *
 * KEY CONCEPTS:
 * - PWM = rapid ON/OFF switching
 * - Duty cycle = ON time percentage
 * - OCR value controls duty cycle
 * - Fast PWM mode = hardware-generated PWM
 * - Multiple timers = multiple independent PWM outputs
 */
int main(void)
{
    demo_01_basic_pwm_50_percent(); // ← START HERE
    // demo_02_variable_brightness();
    // demo_03_smooth_fading();
    // demo_04_dual_led_pwm();
    // demo_05_rgb_color_mixing();
    // demo_06_adc_controlled_brightness();

    return 0;
}

/*
 * ============================================================================
 * PWM FORMULAS
 * ============================================================================
 *
 * DUTY CYCLE:
 * Duty Cycle (%) = (OCR / TOP) × 100
 *
 * For 8-bit Fast PWM: TOP = 255
 * Duty Cycle (%) = (OCR0 / 256) × 100
 *
 * PWM FREQUENCY:
 * f_PWM = F_CPU / (Prescaler × 256)
 *
 * @ 16MHz, Prescaler 64:
 * f_PWM = 16,000,000 / (64 × 256) = 976.5 Hz
 *
 * BRIGHTNESS vs OCR:
 * OCR = 0   → 0% duty   → LED OFF
 * OCR = 64  → 25% duty  → Dim
 * OCR = 128 → 50% duty  → Medium
 * OCR = 192 → 75% duty  → Bright
 * OCR = 255 → 100% duty → Full brightness
 *
 * ============================================================================
 */
