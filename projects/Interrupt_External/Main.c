/*
 * External Interrupts (INT0-INT7)
 * Interrupt-driven button handling
 */

#include "config.h"

volatile uint8_t int_count = 0;

// INT0 interrupt
ISR(INT0_vect)
{
    int_count++;
    PORTB = int_count; // Display count on LEDs
}

// Demo 1: Basic external interrupt
void demo_01_basic_external_int(void)
{
    DDRB = 0xFF; // LEDs output
    PORTB = 0x00;

    // INT0 on PD0 (falling edge)
    DDRD &= ~(1 << 0);
    PORTD |= (1 << 0); // Pull-up

    // Enable INT0, falling edge
    EICRA = (1 << ISC01); // Falling edge
    EIMSK = (1 << INT0);  // Enable INT0

    sei();

    while (1)
    {
    } // Wait for interrupts
}

// Demo 2: Multiple external interrupts
ISR(INT1_vect) { PORTB |= (1 << 0); }  // Set LED0
ISR(INT2_vect) { PORTB &= ~(1 << 0); } // Clear LED0

void demo_02_multiple_interrupts(void)
{
    DDRB = 0xFF;

    // INT0, INT1, INT2 setup
    DDRD &= ~((1 << 0) | (1 << 1) | (1 << 2));
    PORTD |= (1 << 0) | (1 << 1) | (1 << 2);

    EICRA = (1 << ISC01) | (1 << ISC11) | (1 << ISC21);
    EIMSK = (1 << INT0) | (1 << INT1) | (1 << INT2);

    sei();
    while (1)
    {
    }
}

// Demo 3: Edge detection modes
void demo_03_edge_modes(void)
{
    DDRB = 0xFF;
    DDRD &= ~(1 << 0);
    PORTD |= (1 << 0);

    // ANY edge detection
    EICRA = (1 << ISC00); // Any logical change
    EIMSK = (1 << INT0);

    sei();
    while (1)
    {
    }
}

int main(void)
{
    demo_01_basic_external_int();
    return 0;
}
