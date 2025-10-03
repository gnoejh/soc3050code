/* Timer Interrupts - overflow and compare */
#include "config.h"

volatile uint8_t seconds = 0;

ISR(TIMER1_COMPA_vect)
{
    seconds++;
    PORTB = seconds;
}

int main(void)
{
    DDRB = 0xFF;

    // 1 second CTC
    TCCR1B = (1 << WGM12) | (1 << CS12);
    OCR1A = 62499;
    TIMSK |= (1 << OCIE1A);

    sei();
    while (1)
    {
    }
}
