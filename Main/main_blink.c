/*
 * =============================================================================
 * EDUCATIONAL ATmega128 LED BLINKING DEMONSTRATIONS
 * =============================================================================
 *
 * COURSE: SOC 3050 - Embedded Systems and IoT
 * AUTHOR: Professor Kim (Modernized from hjeong original)
 *
 * PURPOSE:
 * Demonstrate LED control using modernized port library functions.
 * Shows progression from direct register access to structured programming.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Learn GPIO (General Purpose Input/Output) concepts
 * 2. Understand port initialization and control
 * 3. Practice timing and delay mechanisms
 * 4. Explore pattern generation and state machines
 * 5. Integrate button input with LED output
 *
 * LEARNING PROGRESSION:
 * Assembly → C → Python → IoT
 * Direct registers → Library functions → Object patterns → Remote control
 *
 * =============================================================================
 */

#include "config.h"

// Conditional compilation - only compile if a relevant example is selected
#if defined(C_LED_BASIC) || defined(C_LED_PATTERNS) || defined(C_LED_BUTTON_INTERACTIVE) || defined(PORT_BLINKING) || defined(PORT_ROTATION) || defined(BLINK_PORT)

/*
 * =============================================================================
 * EDUCATIONAL DEMO: Basic Port-Level LED Blinking
 * =============================================================================
 *
 * PURPOSE: Demonstrate simple port-based LED control with alternating patterns
 * CONFIGURATION: Requires BLINK_PORT defined in config.h
 *
 * HARDWARE SETUP:
 * - LEDs connected to PORTB (PB0-PB7)
 * - Active low configuration (0 = LED on, 1 = LED off)
 *
 * EDUCATIONAL VALUE:
 * - Port initialization using modernized library
 * - Pattern generation with binary operations
 * - Timing control with delay functions
 */
#ifdef BLINK_PORT

void main_blink_port(void)
{
    /*
     * EDUCATIONAL STEP 1: Initialize port using modernized library
     *
     * Modern approach: Use library function instead of direct register access
     * Old way: DDRB = 0xFF;
     * New way: Port_init_output(PORT_B, 0xFF);
     */
    Port_init_output(PORT_B, 0xFF); // Configure all PORTB pins as output

    /*
     * EDUCATIONAL STEP 2: LED pattern generation loop
     *
     * This demonstrates alternating pattern control:
     * Pattern A: 0xAA = 10101010 (alternating LEDs)
     * Pattern B: 0x55 = 01010101 (opposite pattern)
     */
    while (1)
    {
        /* Modern library approach for port output */
        Port_write(PORT_B, 0xAA); // Set alternating pattern A

        /*
         * EDUCATIONAL NOTE: Timing considerations
         * - 1000ms delay provides visible pattern change
         * - Long enough for human perception
         * - Educational timing (not optimized for real applications)
         */
        _delay_ms(1000); // Wait 1 second

        Port_write(PORT_B, 0x55); // Set alternating pattern B
        _delay_ms(1000);          // Wait 1 second

        /*
         * EDUCATIONAL EXERCISE:
         * Try these pattern modifications:
         * 1. Port_write(PORT_B, 0x00); // All LEDs on
         * 2. Port_write(PORT_B, 0xFF); // All LEDs off
         * 3. Port_write(PORT_B, 0xF0); // Half on, half off
         * 4. Implement a counting pattern (0x01, 0x02, 0x04, 0x08...)
         */
    }
}
#endif

/*
 * =============================================================================
 * EDUCATIONAL DEMO: Interactive LED Control with Button Input
 * =============================================================================
 *
 * PURPOSE: Demonstrate GPIO input/output integration with state machines
 * CONFIGURATION: Requires BLINK_PIN defined in config.h
 *
 * HARDWARE SETUP:
 * - LEDs connected to PORTB (PB0-PB7) - active low
 * - Button connected to PD7 with pull-up resistor
 *
 * EDUCATIONAL VALUE:
 * - Input/output port configuration
 * - Button debouncing and edge detection
 * - State machine implementation
 * - Binary pattern manipulation
 */
#ifdef ASSEMBLY_BLINK_INDIVIDUAL

void main_blink_pin(void)
{
    /*
     * EDUCATIONAL STEP 1: Initialize ports using modernized library
     */
    Port_init();   // Initialize port system
    button_init(); // Initialize button system

    /*
     * EDUCATIONAL STEP 2: Initialize state machine variables
     */
    unsigned char direction = 0;         // 0 = clockwise, 1 = counterclockwise
    unsigned char led_state = 0x01;      // Start with first LED (binary: 00000001)
    unsigned char last_button_state = 1; // Track button state for edge detection

    /*
     * EDUCATIONAL STEP 3: Main control loop with state machine
     */
    while (1)
    {
        /*
         * EDUCATIONAL STEP 3.1: Button input processing
         *
         * Modern approach: Use library function for cleaner code
         * Old way: uint8_t current_button_state = PIND & (1 << PD7);
         * New way: unsigned char current_button_state = Port_read_pin(PORT_D, PD7);
         */
        unsigned char current_button_state = read_buttons();

        /*
         * EDUCATIONAL STEP 3.2: Edge detection (falling edge)
         *
         * Button press detection using state comparison:
         * - current_button_state == 0: Button is pressed (active low)
         * - last_button_state != 0: Button was not pressed before
         * - This combination = falling edge = button press event
         */
        if (current_button_state == 0 && last_button_state != 0)
        {
            direction = !direction; // Toggle direction on button press

            /*
             * EDUCATIONAL NOTE: Debouncing considerations
             * In real applications, add delay for debouncing:
             * _delay_ms(50);  // Simple debounce delay
             */
        }

        /* Update button state for next iteration */
        last_button_state = current_button_state;

        /*
         * EDUCATIONAL STEP 3.3: LED pattern generation
         *
         * State machine for LED rotation:
         * - Direction 0: Clockwise rotation (left shift)
         * - Direction 1: Counterclockwise rotation (right shift)
         */
        if (direction == 0) // Clockwise
        {
            /* Use modernized port library for output */
            led_pattern(led_state);

            led_state <<= 1;       // Shift left (next LED)
            if (led_state == 0x00) // Wrap around check
            {
                led_state = 0x01; // Reset to first LED
            }
        }
        else // Counterclockwise
        {
            led_pattern(led_state); // Output pattern

            led_state >>= 1;       // Shift right (previous LED)
            if (led_state == 0x00) // Wrap around check
            {
                led_state = 0x80; // Reset to last LED
            }
        }

        /*
         * EDUCATIONAL STEP 3.4: Timing control
         *
         * 500ms delay provides smooth visual rotation
         * Fast enough for responsiveness, slow enough to see
         */
        _delay_ms(500);

        /*
         * EDUCATIONAL EXERCISES:
         * 1. Add multiple LEDs on at once (led_state = 0x03 for two LEDs)
         * 2. Implement bouncing pattern (direction changes at ends)
         * 3. Add speed control with different buttons
         * 4. Create custom patterns (heart beat, police lights, etc.)
         */
    }
}
#endif

/*
 * =============================================================================
 * EDUCATIONAL SUMMARY AND LEARNING OBJECTIVES
 * =============================================================================
 *
 * This module demonstrates:
 *
 * 1. MODERN LIBRARY USAGE:
 *    - Port_init_output() for clean port initialization
 *    - Port_write() for structured output operations
 *    - Port_read_pin() for reliable input reading
 *
 * 2. GPIO PROGRAMMING CONCEPTS:
 *    - Port direction configuration (input/output)
 *    - Pull-up resistor activation
 *    - Active-low LED control
 *
 * 3. PROGRAMMING TECHNIQUES:
 *    - State machine implementation
 *    - Edge detection for button inputs
 *    - Binary pattern manipulation
 *    - Loop-based timing control
 *
 * 4. EDUCATIONAL PROGRESSION:
 *    - Assembly: Direct register manipulation (DDRB, PORTB)
 *    - C: Structured library functions (Port_init, Port_write)
 *    - Python: Object-oriented GPIO classes
 *    - IoT: Remote LED control via web interface
 *
 * =============================================================================
 */

#endif // Conditional compilation guard
