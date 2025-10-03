/* Advanced: Critical sections, volatile, atomic operations */
#include "config.h"

volatile uint16_t shared_variable = 0;

ISR(TIMER1_COMPA_vect)
{
    shared_variable++; // Modified in ISR
}

int main(void)
{
    uint16_t local_copy;

    // Timer setup
    TCCR1B = (1 << WGM12) | (1 << CS12);
    OCR1A = 624;
    TIMSK |= (1 << OCIE1A);

    sei();

    while (1)
    {
        // WRONG: Not atomic (16-bit read)
        // if (shared_variable > 1000) { ... }

        // RIGHT: Atomic read in critical section
        cli(); // Disable interrupts
        local_copy = shared_variable;
        sei(); // Re-enable interrupts

        if (local_copy > 1000)
        {
            // Do something
        }

        _delay_ms(100);
    }
}
