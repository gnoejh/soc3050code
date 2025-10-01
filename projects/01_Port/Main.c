/*
 * ATmega128 Port Programming Comprehensive Examples
 * Educational Framework for Learning Port Manipulation
 *
 * This project demonstrates various port programming techniques:
 * - Basic port output control
 * - Individual bit manipulation
 * - Input/output interaction
 * - Multi-port coordination
 * - Advanced pattern generation
 *
 * Hardware Setup:
 * - PORTB: 8 LEDs (active LOW)
 * - PORTD7: Push button (with pull-up)
 * - PORTC: Additional output indicators
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// Educational Constants
#define LED_PORT PORTB
#define LED_DDR DDRB
#define LED_ALL_ON 0x00    // Active LOW LEDs
#define LED_ALL_OFF 0xFF   // Active LOW LEDs
#define LED_PATTERN_1 0xAA // Alternating pattern 1
#define LED_PATTERN_2 0x55 // Alternating pattern 2

#define BUTTON_PORT PORTD
#define BUTTON_DDR DDRD
#define BUTTON_PIN PIND
#define BUTTON_BIT PD7

// Delay constants for better readability
#define DELAY_SHORT 100
#define DELAY_MEDIUM 200
#define DELAY_LONG 500

/*
 * Example 1: Basic Port Output Control
 * Demonstrates fundamental port manipulation
 */
void demo_basic_port_output(void)
{
    // Initialize PORTB as output for LEDs
    LED_DDR = 0xFF; // All pins as output

    while (1)
    {
        // Pattern sequence demonstration
        LED_PORT = LED_PATTERN_1; // Pattern: 10101010
        _delay_ms(DELAY_LONG);

        LED_PORT = LED_PATTERN_2; // Pattern: 01010101
        _delay_ms(DELAY_LONG);

        LED_PORT = LED_ALL_ON; // All LEDs on
        _delay_ms(DELAY_LONG);

        LED_PORT = LED_ALL_OFF; // All LEDs off
        _delay_ms(DELAY_LONG);
    }
}

/*
 * Example 2: Individual Bit Manipulation
 * Demonstrates precise bit-level control
 */
void demo_bit_manipulation(void)
{
    LED_DDR = 0xFF;         // Configure as output
    LED_PORT = LED_ALL_OFF; // Start with all LEDs off

    while (1)
    {
        // Sequential LED turn-on (right to left)
        for (uint8_t i = 0; i < 8; i++)
        {
            LED_PORT &= ~(1 << i); // Clear bit i (turn on LED i)
            _delay_ms(DELAY_MEDIUM);
        }

        _delay_ms(DELAY_LONG);

        // Sequential LED turn-off (right to left)
        for (uint8_t i = 0; i < 8; i++)
        {
            LED_PORT |= (1 << i); // Set bit i (turn off LED i)
            _delay_ms(DELAY_MEDIUM);
        }

        _delay_ms(DELAY_LONG);

        // Bit toggle demonstration
        for (uint8_t cycle = 0; cycle < 5; cycle++)
        {
            LED_PORT ^= LED_PATTERN_1; // Toggle pattern
            _delay_ms(DELAY_MEDIUM);
            LED_PORT ^= LED_PATTERN_1; // Toggle back
            _delay_ms(DELAY_MEDIUM);
        }
    }
}

/*
 * Example 3: Input/Output Interaction
 * Demonstrates reading inputs and controlling outputs
 */
void demo_input_output_interaction(void)
{
    // Configure ports
    LED_DDR = 0xFF;                   // PORTB as output (LEDs)
    BUTTON_DDR &= ~(1 << BUTTON_BIT); // PORTD7 as input (button)
    BUTTON_PORT |= (1 << BUTTON_BIT); // Enable pull-up resistor

    uint8_t led_position = 0;      // Current LED position (0-7)
    uint8_t last_button_state = 1; // Previous button state

    // Initialize display
    LED_PORT = ~(1 << led_position); // Turn on first LED

    while (1)
    {
        // Read current button state (active low)
        uint8_t current_button_state = (BUTTON_PIN & (1 << BUTTON_BIT)) ? 1 : 0;

        // Detect button press (falling edge)
        if (current_button_state == 0 && last_button_state == 1)
        {
            // Move to next LED position
            led_position = (led_position + 1) % 8;

            // Update LED display
            LED_PORT = ~(1 << led_position);

            // Simple debouncing
            _delay_ms(50);
        }

        last_button_state = current_button_state;
        _delay_ms(10); // Small delay for stable reading
    }
}

/*
 * Example 4: Multi-Port Coordination
 * Demonstrates coordinated control of multiple ports
 */
void demo_multi_port_control(void)
{
    // Configure ports
    DDRB = 0xFF;  // PORTB: LED display
    DDRC = 0xFF;  // PORTC: Status indicators
    DDRD = 0x00;  // PORTD: Input sensors
    PORTD = 0xFF; // Enable pull-ups on all PORTD pins

    uint8_t counter = 0;
    uint8_t direction = 1; // 1 = up, 0 = down

    while (1)
    {
        // Display counter on PORTB (LEDs)
        PORTB = ~counter;

        // Display counter status on PORTC
        PORTC = counter;

        // Check input on PORTD
        uint8_t input_state = PIND;

        if (input_state != 0xFF) // Any button pressed
        {
            // Change direction on button press
            direction = !direction;
            _delay_ms(DELAY_MEDIUM); // Debounce
        }

        // Update counter based on direction
        if (direction)
        {
            counter++;
        }
        else
        {
            counter--;
        }

        _delay_ms(DELAY_SHORT);
    }
}

/*
 * Example 5: Advanced Pattern Generation
 * Demonstrates complex LED patterns and effects
 */
void demo_advanced_patterns(void)
{
    LED_DDR = 0xFF; // Configure PORTB as output

    while (1)
    {
        // Pattern 1: Knight Rider (bouncing LED)
        uint8_t led = 0x01;

        // Move right
        for (uint8_t i = 0; i < 7; i++)
        {
            LED_PORT = ~led;
            led <<= 1;
            _delay_ms(DELAY_SHORT);
        }

        // Move left
        for (uint8_t i = 0; i < 7; i++)
        {
            LED_PORT = ~led;
            led >>= 1;
            _delay_ms(DELAY_SHORT);
        }

        // Pattern 2: Binary counter display
        for (uint16_t count = 0; count < 256; count++)
        {
            LED_PORT = ~((uint8_t)count);
            _delay_ms(30); // Fast counting
        }

        // Pattern 3: Breathing effect
        for (uint8_t breath = 0; breath < 3; breath++)
        {
            // Fade in effect (simulated)
            for (uint8_t intensity = 0; intensity < 8; intensity++)
            {
                LED_PORT = ~((1 << (intensity + 1)) - 1); // Progressive fill
                _delay_ms(DELAY_SHORT);
            }

            // Fade out effect (simulated)
            for (int8_t intensity = 7; intensity >= 0; intensity--)
            {
                LED_PORT = ~((1 << (intensity + 1)) - 1); // Progressive empty
                _delay_ms(DELAY_SHORT);
            }
        }

        // Pattern 4: Random-like pattern (pseudo-random)
        uint8_t seed = 0x5A; // Pseudo-random seed
        for (uint8_t i = 0; i < 32; i++)
        {
            seed = (seed << 1) ^ (seed >> 7) ^ 0x1D; // Simple LFSR
            LED_PORT = ~seed;
            _delay_ms(DELAY_SHORT);
        }
    }
}

/*
 * Main function - Select which demonstration to run
 */
int main(void)
{
    // Choose which Port Programming demonstration to run:

    // Level 1: Basic port output patterns
    // demo_basic_port_output();

    // Level 2: Individual bit manipulation techniques
    demo_bit_manipulation();

    // Level 3: Interactive input/output control
    // demo_input_output_interaction();

    // Level 4: Multi-port coordination
    // demo_multi_port_control();

    // Level 5: Advanced pattern generation
    // demo_advanced_patterns();

    return 0; // Never reached due to infinite loops
}