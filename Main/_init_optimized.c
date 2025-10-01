/*
 * _init.c - ATmega128 System Initialization Library
 * Educational Version for Assembly→C→Python Learning Progression
 *
 * LEARNING OBJECTIVES:
 * 1. Understand system initialization sequence and order importance
 * 2. Learn interrupt enable/disable concepts (cli/sei)
 * 3. See how C functions abstract complex initialization procedures
 * 4. Bridge to Python's automatic initialization handling
 *
 * ASSEMBLY EQUIVALENT CONCEPTS:
 * - cli()  ≡  CLI instruction (Clear Interrupt flag)
 * - sei()  ≡  SEI instruction (Set Interrupt flag)
 * - Function calls ≡ CALL/RET instruction sequences
 */

#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "_main.h"
#include "_init.h"

// Include all hardware module headers
#include "_port.h"
#include "_uart.h"
#include "_timer2.h"
#include "_interrupt.h"
#include "_adc.h"
#include "_buzzer.h"
#include "_eeprom.h"
#include "_glcd.h"

/*
 * init_devices() - Master initialization function
 *
 * EDUCATIONAL NOTES:
 * - Initialization order is critical in embedded systems
 * - Interrupts must be disabled during hardware configuration
 * - Each subsystem requires specific register setup
 * - Interrupts are re-enabled after all systems are ready
 *
 * ASSEMBLY EQUIVALENT SEQUENCE:
 * CLI                    ; Disable interrupts
 * CALL Port_init         ; Initialize ports
 * CALL Uart1_init        ; Initialize UART
 * ... (other initializations)
 * SEI                    ; Re-enable interrupts
 */
void init_devices(void)
{
    // Step 1: Disable all interrupts during initialization
    // Assembly equivalent: CLI
    cli(); // Critical: prevent interrupts during setup

    // Step 2: Initialize hardware in dependency order
    // Note: Order matters! Some modules depend on others

    Port_init();      // Must be first: sets up I/O directions
    Interrupt_init(); // Configure external interrupts
    Timer2_init();    // Initialize timer/counter systems
    Uart1_init();     // Initialize serial communication
    Adc_init();       // Initialize analog-to-digital converter
    lcd_init();       // Initialize graphics LCD display

    // Step 3: Re-enable interrupts after all systems ready
    // Assembly equivalent: SEI
    sei(); // Allow interrupts to function normally
}

/*
 * Educational Initialization Functions
 * These allow students to understand individual subsystem setup
 */

/*
 * init_basic_io() - Initialize only basic I/O (no interrupts)
 * For early assembly/C lessons where students learn register manipulation
 */
void init_basic_io(void)
{
    cli();       // Safety: disable interrupts
    Port_init(); // Only basic port setup
    // Note: No sei() call - interrupts remain disabled for safety
}

/*
 * init_communication() - Initialize communication systems
 * For lessons focusing on UART and serial communication
 */
void init_communication(void)
{
    cli();        // Disable interrupts
    Port_init();  // Required for UART pins
    Uart1_init(); // Serial communication
    sei();        // Enable interrupts for UART
}

/*
 * init_timing() - Initialize timing systems
 * For lessons focusing on timers and precise timing
 */
void init_timing(void)
{
    cli();         // Disable interrupts
    Port_init();   // Required for timer output pins
    Timer2_init(); // Timer/counter systems
    sei();         // Enable interrupts for timers
}

/*
 * init_sensors() - Initialize sensor reading systems
 * For lessons focusing on analog input and data acquisition
 */
void init_sensors(void)
{
    cli();       // Disable interrupts
    Port_init(); // Required for ADC pins
    Adc_init();  // Analog-to-digital converter
    sei();       // Enable interrupts if ADC uses them
}

/*
 * init_display() - Initialize display systems
 * For lessons focusing on graphics and visual output
 */
void init_display(void)
{
    cli();       // Disable interrupts
    Port_init(); // Required for LCD control/data pins
    lcd_init();  // Graphics LCD initialization
    sei();       // Enable interrupts (usually not needed for LCD)
}

/*
 * System Status and Diagnostic Functions
 * These help students understand system state
 */

/*
 * get_interrupt_status() - Check if interrupts are enabled
 * Educational function to understand interrupt flag
 */
unsigned char get_interrupt_status(void)
{
    return (SREG & (1 << 7)) ? 1 : 0; // Check I-bit in Status Register
}

/*
 * system_reset() - Software reset function
 * Educational function showing how to restart the system
 */
void system_reset(void)
{
    cli(); // Disable interrupts
    // In a real application, we would use watchdog timer reset
    // For education, we show the concept
    while (1)
    {
        // Infinite loop - forces watchdog reset if enabled
        // Or use: asm volatile ("jmp 0");  // Jump to reset vector
    }
}

/*
 * wait_for_systems_ready() - Diagnostic delay after initialization
 * Some hardware needs time to stabilize after configuration
 */
void wait_for_systems_ready(void)
{
    _delay_ms(100); // Allow 100ms for system stabilization
}

/*
 * EDUCATIONAL PROGRESSION NOTES:
 *
 * 1. ASSEMBLY LEVEL (Direct Control):
 *    Students learn: CLI; LDI R16, 0xFF; OUT DDRB, R16; SEI
 *    Understanding direct register manipulation and interrupt control
 *
 * 2. C ABSTRACTION LEVEL (This Library):
 *    Students learn: init_devices(); or init_basic_io();
 *    Understanding function calls and system initialization patterns
 *
 * 3. PYTHON LEVEL (High-Level Interface):
 *    Students learn: atmega = ATmega128Interface() # Auto-initialization
 *    Understanding object constructors and automatic resource management
 *
 * PROGRESSION TOPICS:
 * - Critical sections and interrupt management
 * - Hardware dependency ordering
 * - System initialization vs configuration
 * - Error handling and system recovery
 * - Resource management and cleanup
 */