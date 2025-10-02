#include "config.h"		// Include config.h to access macros
#include <util/delay.h> // For _delay_ms() function

/*
 * EDUCATIONAL MAIN - Basic Port Programming Example
 * Demonstrates: Basic LED blinking using library functions
 * Learning: C programming with ATmega128 hardware abstraction
 *
 * HARDWARE SETUP:
 * - Connect 8 LEDs to PORTB (PB0-PB7) with current-limiting resistors
 * - LEDs should be connected between port pins and ground (active high)
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Learn proper system initialization sequence
 * 2. Understand basic port manipulation in C
 * 3. Master timing using delay functions
 * 4. Practice while loop control structures
 */
int main(void)
{
	// Initialize system - demonstrates proper initialization sequence
	init_devices();

	// Simple LED blinking demonstration
	while (1)
	{
		// Turn on all LEDs on PORTB
		PORTB = 0xFF;
		_delay_ms(500);

		// Turn off all LEDs on PORTB
		PORTB = 0x00;
		_delay_ms(500);
	}

	return 0; // Never reached in embedded applications
}