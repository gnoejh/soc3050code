/*
 * =============================================================================
 * SAFE INITIALIZATION SYSTEM - Conflict-Free Library Design
 * =============================================================================
 * 
 * This module provides safe initialization that prevents conflicts between
 * different libraries when they're used together in projects.
 * 
 * EDUCATIONAL OBJECTIVES:
 * 1. Learn proper system resource management
 * 2. Understand initialization conflicts and how to avoid them
 * 3. Practice modular system design
 * 4. Master hardware abstraction techniques
 * 
 * =============================================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>

// Resource usage tracking
static uint8_t port_usage[8] = {0};  // Track which ports are in use
static uint8_t timer_usage = 0;      // Track timer usage
static uint8_t adc_usage = 0;        // Track ADC usage

// Port usage definitions
#define PORT_GLCD_CONTROL  0x01
#define PORT_GLCD_DATA     0x02
#define PORT_UART_TX       0x04
#define PORT_UART_RX       0x08
#define PORT_LED_ARRAY     0x10
#define PORT_BUTTONS       0x20
#define PORT_ADC_INPUTS    0x40
#define PORT_BUZZER        0x80

/*
 * SAFE FUNCTION: Check for port conflicts
 * 
 * PURPOSE: Prevent multiple libraries from conflicting on same port pins
 * RETURNS: 1 if conflict detected, 0 if safe to proceed
 */
uint8_t check_port_conflict(uint8_t port, uint8_t pins)
{
    if ((port_usage[port] & pins) != 0) {
        return 1;  // Conflict detected
    }
    return 0;  // No conflict
}

/*
 * SAFE FUNCTION: Reserve port pins for specific use
 * 
 * PURPOSE: Mark port pins as used by specific library
 * RETURNS: 1 if successful, 0 if pins already in use
 */
uint8_t reserve_port_pins(uint8_t port, uint8_t pins)
{
    if (check_port_conflict(port, pins)) {
        return 0;  // Cannot reserve - conflict
    }
    port_usage[port] |= pins;
    return 1;  // Successfully reserved
}

/*
 * SAFE FUNCTION: Release port pins
 * 
 * PURPOSE: Free up port pins when library is no longer needed
 */
void release_port_pins(uint8_t port, uint8_t pins)
{
    port_usage[port] &= ~pins;
}

/*
 * SAFE FUNCTION: GLCD-safe initialization
 * 
 * PURPOSE: Initialize system for GLCD use without conflicts
 * ONLY CONFIGURES: Ports needed for GLCD operation
 */
void init_safe_for_glcd(void)
{
    cli();
    
    // Reserve GLCD control pins (example: PORTC bits 0-4)
    if (reserve_port_pins(2, PORT_GLCD_CONTROL)) {  // PORTC = port 2
        DDRC |= 0x1F;  // Set bits 0-4 as outputs for GLCD control
    }
    
    // Reserve GLCD data pins (example: PORTA all bits)
    if (reserve_port_pins(0, PORT_GLCD_DATA)) {  // PORTA = port 0
        DDRA = 0xFF;   // Set all bits as outputs for GLCD data
    }
    
    // Leave other ports untouched for other libraries
    sei();
}

/*
 * SAFE FUNCTION: UART-safe initialization
 * 
 * PURPOSE: Initialize system for UART use without conflicts
 * ONLY CONFIGURES: Ports needed for UART operation
 */
void init_safe_for_uart(void)
{
    cli();
    
    // Reserve UART pins (PORTD bits 0-1 for TX/RX)
    if (reserve_port_pins(3, PORT_UART_TX | PORT_UART_RX)) {  // PORTD = port 3
        DDRD |= 0x02;  // Set PD1 as output (TX), PD0 as input (RX)
        PORTD |= 0x01; // Enable pull-up on RX pin
    }
    
    // Configure UART registers
    UBRR1H = 0;
    UBRR1L = 103;  // 9600 baud at 16MHz
    UCSR1B = (1 << TXEN1) | (1 << RXEN1);
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
    
    sei();
}

/*
 * SAFE FUNCTION: LED-safe initialization
 * 
 * PURPOSE: Initialize system for LED use without conflicts
 * ONLY CONFIGURES: Ports needed for LED operation
 */
void init_safe_for_leds(void)
{
    cli();
    
    // Reserve LED pins (PORTB all bits)
    if (reserve_port_pins(1, PORT_LED_ARRAY)) {  // PORTB = port 1
        DDRB = 0xFF;   // Set all bits as outputs for LEDs
        PORTB = 0x00;  // Turn off all LEDs initially
    }
    
    sei();
}

/*
 * SAFE FUNCTION: ADC-safe initialization
 * 
 * PURPOSE: Initialize system for ADC use without conflicts
 * ONLY CONFIGURES: Ports and registers needed for ADC operation
 */
void init_safe_for_adc(void)
{
    cli();
    
    // Reserve ADC pins (PORTF - ADC inputs)
    if (reserve_port_pins(5, PORT_ADC_INPUTS)) {  // PORTF = port 5
        DDRF = 0x00;   // Set all bits as inputs for ADC
    }
    
    // Configure ADC
    ADMUX = 0x40;      // AVCC reference, channel 0
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
    sei();
}

/*
 * SAFE FUNCTION: Combined initialization
 * 
 * PURPOSE: Initialize multiple systems safely without conflicts
 * PARAMETERS: Bit flags for which systems to initialize
 */
void init_safe_combined(uint8_t systems)
{
    cli();
    
    // Initialize requested systems in safe order
    if (systems & 0x01) init_safe_for_glcd();
    if (systems & 0x02) init_safe_for_uart();
    if (systems & 0x04) init_safe_for_leds();
    if (systems & 0x08) init_safe_for_adc();
    
    sei();
}

/*
 * SAFE FUNCTION: Get current resource usage
 * 
 * PURPOSE: Debug function to see which resources are in use
 * RETURNS: Bit pattern showing resource usage
 */
uint8_t get_resource_usage(void)
{
    uint8_t usage = 0;
    for (int i = 0; i < 8; i++) {
        if (port_usage[i] != 0) {
            usage |= (1 << i);
        }
    }
    return usage;
}

/*
 * SAFE FUNCTION: Check if system is safe for specific use
 * 
 * PURPOSE: Verify that system can safely use specific functionality
 * RETURNS: 1 if safe, 0 if conflicts would occur
 */
uint8_t is_system_safe_for(uint8_t system_flags)
{
    // Check for conflicts before allowing initialization
    if ((system_flags & 0x01) && check_port_conflict(2, PORT_GLCD_CONTROL)) return 0;
    if ((system_flags & 0x02) && check_port_conflict(3, PORT_UART_TX | PORT_UART_RX)) return 0;
    if ((system_flags & 0x04) && check_port_conflict(1, PORT_LED_ARRAY)) return 0;
    if ((system_flags & 0x08) && check_port_conflict(5, PORT_ADC_INPUTS)) return 0;
    
    return 1;  // Safe to proceed
}
