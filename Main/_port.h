
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

#endif /* _PORT_H_ */