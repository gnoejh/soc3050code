/* Nested Interrupts and Priority */
#include "config.h"

volatile uint8_t high_priority_count = 0;
volatile uint8_t low_priority_count = 0;

ISR(INT0_vect)
{          // Higher priority (lower vector number)
    sei(); // Allow nesting
    high_priority_count++;
    PORTB = high_priority_count;
    _delay_ms(100);
}

ISR(TIMER1_COMPA_vect)
{ // Lower priority
    low_priority_count++;
    PORTC = low_priority_count;
}

int main(void)
{
    DDRB = 0xFF;
    DDRC = 0xFF;

    // INT0 setup
    EICRA = (1 << ISC01);
    EIMSK = (1 << INT0);

    // Timer1 10ms tick
    TCCR1B = (1 << WGM12) | (1 << CS12);
    OCR1A = 624;
    TIMSK |= (1 << OCIE1A);

    sei();
    while (1)
    {
    }
}
