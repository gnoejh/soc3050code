/*
 * _init.c - ATmega128 Educational Initialization Library
 * Standalone shared library for microcontroller initialization
 *
 * EDUCATIONAL OBJECTIVES:
 * - Demonstrate proper microcontroller initialization
 * - Show progressive complexity from basic to advanced setups
 * - Provide reusable initialization functions for educational projects
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "_init.h"
#include "_port.h"

/*
 * EDUCATIONAL FUNCTION: Basic System Initialization
 *
 * PURPOSE: Initialize core peripherals for basic projects
 * LEARNING: Demonstrates fundamental initialization sequence
 * USE CASE: For simple LED, button, and basic I/O projects
 */
void init_basic(void)
{
	cli();
	Port_init();
	sei();
}

/*
 * EDUCATIONAL FUNCTION: System Status Check
 *
 * PURPOSE: Verify that initialization completed successfully
 * LEARNING: Shows how to validate system state
 * RETURNS: 1 if all systems initialized, 0 if problems detected
 */
unsigned char check_init_status(void)
{
	// Check global interrupt flag
	if (SREG & (1 << 7))
	{
		return 1; // System initialized
	}
	else
	{
		return 0; // Initialization incomplete
	}
}
/*
 * EDUCATIONAL FUNCTION: Complete Device Initialization
 *
 * PURPOSE: Initialize all peripherals for advanced projects
 * LEARNING: Shows comprehensive system setup sequence
 * USE CASE: For projects using multiple peripherals (GLCD, sensors, etc.)
 */
/*
 * EDUCATIONAL FUNCTION: Complete Device Initialization
 * Simplified version - only initializes available peripherals
 *
 * PURPOSE: Initialize available peripherals for graphics projects
 * LEARNING: Shows basic system setup sequence
 * USE CASE: For projects using GLCD and basic I/O
 */
void init_devices(void)
{
	cli();		 // Disable interrupts during initialization
	Port_init(); // Initialize I/O ports (this function exists)

	// Declare lcd_init function
	extern void lcd_init(void);
	lcd_init(); // Initialize Graphics LCD

	sei(); // Enable global interrupts
}
