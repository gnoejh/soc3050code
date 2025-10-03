/* Pin Change Interrupts (PCINT) - ATmega128 doesn't have PCINT, using external interrupts instead */
#include "config.h"

volatile uint8_t pin_change_count = 0;

ISR(INT0_vect)
{
    pin_change_count++;
    PORTB = pin_change_count;
}

int main(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    // INT0 on any edge (simulating pin change)
    DDRD &= ~(1 << 0);
    PORTD |= (1 << 0);

    EICRA = (1 << ISC00); // Any logical change
    EIMSK = (1 << INT0);

    sei();
    while (1)
    {
    }
}
