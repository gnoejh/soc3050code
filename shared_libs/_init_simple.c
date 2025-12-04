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