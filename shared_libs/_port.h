
/*
 * _port.h - ATmega128 Educational Port Control Library Header
 * Part of Assembly → C → Python Learning Progression
 *
 * EDUCATIONAL PURPOSE:
 * Provides systematic port control functions with clear learning objectives
 * and comprehensive I/O management for ATmega128 educational projects
 */

#ifndef _PORT_H_
#define _PORT_H_

/*
 * SYSTEM INITIALIZATION FUNCTIONS
 */
void Port_init(void); // Initialize all ports for educational board

/*
 * LED CONTROL FUNCTIONS
 * Educational focus: Visual feedback and pattern generation
 */
void led_on(unsigned char led_number);     // Turn on specific LED (0-7)
void led_off(unsigned char led_number);    // Turn off specific LED (0-7)
void led_toggle(unsigned char led_number); // Toggle specific LED (0-7)
void led_pattern(unsigned char pattern);   // Set LED pattern (8-bit value)
void led_all_off(void);                    // Turn all LEDs off
void led_all_on(void);                     // Turn all LEDs on

/*
 * BUTTON INPUT FUNCTIONS
 * Educational focus: User input and event detection
 */
unsigned char button_pressed(unsigned char button_number); // Check if button pressed (0-7)
unsigned char read_buttons(void);                          // Read all button states
void wait_for_button_press(void);                          // Wait for any button press

/*
 * EDUCATIONAL DEMONSTRATION FUNCTIONS
 * Visual patterns for learning binary, counting, and algorithms
 */
void led_binary_count(unsigned char max_count); // Binary counting demonstration
void led_running_light(unsigned char cycles);   // Running light pattern
void led_knight_rider(unsigned char cycles);    // Knight Rider bouncing pattern

/*
 * DIAGNOSTIC AND TESTING FUNCTIONS
 * Verify hardware operation and learn system validation
 */
void test_all_leds(void);    // Test each LED individually
void test_all_buttons(void); // Test buttons with LED feedback

/*
 * =============================================================================
 * ENHANCED PORT FEATURES - Professional Quality Additions
 * =============================================================================
 */

/**
 * @brief Read button state with software debouncing
 *
 * @param button_number Button to read (0-7)
 * @return 1 if button pressed (debounced), 0 if not pressed
 *
 * EDUCATIONAL NOTE: Implements software debounce algorithm to filter
 * mechanical switch bounce. Waits for stable reading before returning.
 */
unsigned char button_pressed_debounced(unsigned char button_number);

/**
 * @brief Wait for button release (debounced)
 *
 * @param button_number Button to wait for (0-7)
 *
 * EDUCATIONAL NOTE: Blocks until button is released and debounced.
 * Prevents detecting single press multiple times.
 */
void button_wait_release(unsigned char button_number);

/**
 * @brief Enable internal pull-up resistor for input pin
 *
 * @param pin_number Pin to enable pull-up (0-7, typically PORTD)
 *
 * EDUCATIONAL NOTE: Internal pull-ups eliminate need for external resistors.
 * Essential for button inputs to prevent floating inputs.
 */
void port_enable_pullup(unsigned char pin_number);

/**
 * @brief Disable internal pull-up resistor for input pin
 *
 * @param pin_number Pin to disable pull-up (0-7)
 */
void port_disable_pullup(unsigned char pin_number);

/**
 * @brief Read port with bit masking
 *
 * @param port_register Port to read (e.g., PIND)
 * @param mask Bit mask to apply
 * @return Masked port value
 *
 * EDUCATIONAL NOTE: Reads only specific bits of interest.
 * Example: port_read_masked(PIND, 0x0F) reads lower 4 bits only.
 */
unsigned char port_read_masked(volatile unsigned char port_register, unsigned char mask);

/**
 * @brief Write to port with bit masking
 *
 * @param port_register Port to write (e.g., PORTB)
 * @param mask Bit mask (which bits to modify)
 * @param value Value to write to masked bits
 *
 * EDUCATIONAL NOTE: Modifies only specific bits without affecting others.
 * Example: port_write_masked(PORTB, 0xF0, 0xA0) sets upper nibble to 0xA.
 */
void port_write_masked(volatile unsigned char *port_register, unsigned char mask, unsigned char value);

/**
 * @brief Set debounce delay in milliseconds
 *
 * @param delay_ms Debounce delay (default: 20ms)
 *
 * EDUCATIONAL NOTE: Adjustable debounce time for different switch types.
 * Typical values: 10-50ms depending on switch quality.
 */
void port_set_debounce_delay(unsigned char delay_ms);

/**
 * @brief Get debounce delay
 *
 * @return Current debounce delay in milliseconds
 */
unsigned char port_get_debounce_delay(void);

/**
 * @brief Read multiple buttons as a byte
 *
 * @return Button states as 8-bit value (1=pressed, 0=not pressed)
 *
 * EDUCATIONAL NOTE: All buttons debounced simultaneously.
 */
unsigned char read_buttons_debounced(void);

/**
 * @brief Wait for any button press (debounced)
 *
 * @return Button number that was pressed (0-7)
 */
unsigned char wait_for_any_button_debounced(void);

#endif /* _PORT_H_ */