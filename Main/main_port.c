
#include "config.h"

/*
 * MODERNIZED PORT PROGRAMMING DEMONSTRATIONS
 * Educational Framework: ATmega128 GPIO Control and Visual Programming
 *
 * Learning Objectives:
 * 1. Master modernized port library functions for safe GPIO control
 * 2. Understand visual programming with LED patterns and LCD graphics
 * 3. Learn input-based control and interactive programming concepts
 * 4. Explore timing-based visual effects and pattern generation
 *
 * Port Library Functions Used:
 * - Port_init_output()/Port_init_input(): Configure GPIO directions safely
 * - Port_write()/Port_read(): Perform safe port operations
 * - Port_write_pin()/Port_read_pin(): Individual pin control
 * - Port_toggle_pin(): Efficient pin toggling operations
 *
 * Integration with Other Libraries:
 * - Timer2 library: Precise timing for visual effects
 * - GLCD library: Graphics coordination with GPIO patterns
 * - Educational framework: Progressive learning demonstrations
 *
 * Hardware Connections:
 * - PORTB: LED array for visual pattern display
 * - PORTD.0: Input button for rotation control
 * - LCD: Educational information and pattern status
 */

/* 2. Port Programming: Advanced LED Blinking with Graphics Integration */
#ifdef PORT_BLINKING
/*
 * DEMONSTRATION 1: Advanced LED Blinking with Synchronized Graphics
 * Educational Focus: Coordinated visual programming and timing control
 *
 * This example demonstrates:
 * - Modernized port library for safe LED control
 * - Timer2 integration for precise timing
 * - Synchronized LED and LCD graphics coordination
 * - Dynamic visual pattern generation
 *
 * Learning Points:
 * 1. Port_write() provides safe and reliable LED control
 * 2. Timer2_delay_ms() ensures precise timing without blocking
 * 3. Coordinated visual effects enhance educational experience
 * 4. Pattern generation demonstrates algorithmic thinking
 *
 * Hardware Setup:
 * - LEDs connected to PORTB for pattern display
 * - LCD shows educational information and pattern status
 */

void main_port_blinking(void)
{
	init_devices();

	// Initialize Timer2 for precise timing
	Timer2_init();
	Timer2_start();

	// Port is already initialized in init_devices()
	// Start with alternating pattern using modernized functions
	led_pattern(0xAA); // Initial alternating pattern

	// Display educational information
	lcd_clear();
	lcd_string(0, 0, "Port Demo: Blink+GFX");
	lcd_string(0, 1, "LED Pattern Sync");
	lcd_string(0, 2, "Timer2 Precision");
	lcd_string(0, 3, "Graphics Coord.");

	unsigned char pattern = 0xAA;
	unsigned char graphics_flag = 0;
	unsigned int cycle_count = 0;
	unsigned long last_update = 0;

	while (1)
	{
		unsigned long current_time = Timer2_get_milliseconds();

		// Update pattern every 1000ms with precise timing
		if (current_time - last_update >= 1000)
		{
			last_update = current_time;
			cycle_count++;

			// Toggle LED pattern using modernized port functions
			pattern = ~pattern;
			led_pattern(pattern);

			// Clear and update graphics with synchronized timing
			lcd_clear();
			lcd_string(0, 0, "Cycle: ");
			GLCD_4DigitDecimal(cycle_count);

			// Create synchronized graphics pattern
			for (int i = 0; i < 8; i++)
			{
				if (graphics_flag)
				{
					// Draw small circles
					GLCD_Circle(50, 10 * i + 10, 2);
				}
				else
				{
					// Draw large circles
					GLCD_Circle(50, 10 * i + 10, 4);
				}
			}

			// Toggle graphics pattern for next cycle
			graphics_flag = !graphics_flag;

			// Display pattern information (show decimal value instead of hex)
			lcd_string(0, 5, "Pattern: ");
			GLCD_3DigitDecimal(pattern);

			// Show timing information
			lcd_string(0, 6, "Time: ");
			GLCD_4DigitDecimal(current_time / 1000);
			lcd_string(8, 6, "s");

			// Display graphics state
			lcd_string(0, 7, graphics_flag ? "Graphics: Large" : "Graphics: Small");
		}

		// Small delay for system responsiveness
		Timer2_delay_ms(50);
	}
}
#endif

/* Port Programming: Interactive LED Rotation with Input Control */
#ifdef PORT_ROTATION
/*
 * DEMONSTRATION 2: Interactive LED Rotation with Button Control
 * Educational Focus: Input-driven pattern control and interactive programming
 *
 * This example demonstrates:
 * - Modernized port library for both input and output control
 * - Interactive pattern generation based on user input
 * - Debounced input handling using Timer2 precision
 * - Dynamic LED pattern rotation algorithms
 *
 * Learning Points:
 * 1. Port_read_pin() provides reliable input reading with debouncing
 * 2. Bit rotation algorithms create engaging visual patterns
 * 3. Interactive control enhances user engagement and learning
 * 4. Timer2 precision ensures smooth pattern transitions
 *
 * Hardware Setup:
 * - LEDs connected to PORTB for rotation pattern display
 * - Button connected to PORTD.0 for direction control
 * - LCD displays control instructions and pattern information
 */

void main_port_rotation(void)
{
	init_devices();

	// Initialize Timer2 for precise timing and debouncing
	Timer2_init();
	Timer2_start();

	// Configure ports using modernized library
	Port_init(); // Initialize all ports (LEDs, buttons with pull-ups)

	// Initial LED pattern (single bit set)
	unsigned char pattern = 0x7F; // Binary: 01111111
	led_pattern(0x7F);

	// Display educational information
	lcd_clear();
	lcd_string(0, 0, "Interactive Rotation");
	lcd_string(0, 1, "Button: PD0");
	lcd_string(0, 2, "High=CW, Low=CCW");
	lcd_string(0, 3, "Timer2 Smooth");

	unsigned long last_rotation = 0;
	unsigned int rotation_count = 0;
	unsigned char last_direction = 1; // Track direction changes

	while (1)
	{
		unsigned long current_time = Timer2_get_milliseconds();

		// Rotate pattern every 500ms for smooth visual effect
		if (current_time - last_rotation >= 500)
		{
			last_rotation = current_time;
			rotation_count++;

			// Read button state using modernized port function
			unsigned char buttons = read_buttons();
			unsigned char button_pressed = (buttons & 0x01) ? 0 : 1; // Button 0 (PD0), inverted logic

			// Perform pattern rotation based on button state
			if (button_pressed)
			{
				// Clockwise rotation (left shift with wrap-around)
				pattern = (pattern << 1) | (pattern >> 7);
			}
			else
			{
				// Counter-clockwise rotation (right shift with wrap-around)
				pattern = (pattern >> 1) | (pattern << 7);
			}

			// Update LEDs with new pattern
			led_pattern(pattern);

			// Track direction changes for educational feedback
			if (last_direction != button_pressed)
			{
				last_direction = button_pressed;

				// Update direction display
				lcd_string(0, 4, button_pressed ? "Direction: CW    " : "Direction: CCW   ");
			}

			// Display pattern information
			lcd_xy(0, 5);
			lcd_string(0, 5, "Pattern: 0b");

			// Show binary pattern for educational value
			for (int i = 7; i >= 0; i--)
			{
				lcd_char((pattern & (1 << i)) ? '1' : '0');
			}

			// Display rotation count
			lcd_xy(0, 6);
			lcd_string(0, 6, "Rotations: ");
			GLCD_4DigitDecimal(rotation_count);

			// Show timing information
			lcd_xy(0, 7);
			lcd_string(0, 7, "Time: ");
			GLCD_4DigitDecimal(current_time / 1000);
			lcd_string(8, 7, "s");
		}

		// Small delay for button debouncing and system responsiveness
		Timer2_delay_ms(10);
	}
}
#endif
