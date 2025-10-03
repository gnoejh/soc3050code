/*
 * Timer Stopwatch
 * Precision timing with start/stop/reset
 */

#include "config.h"

volatile uint16_t milliseconds = 0;
volatile uint8_t seconds = 0;
volatile uint8_t minutes = 0;
volatile uint8_t running = 0;

ISR(TIMER1_COMPA_vect)
{
    if (running)
    {
        milliseconds += 10;
        if (milliseconds >= 1000)
        {
            milliseconds = 0;
            seconds++;
            PORTB ^= (1 << 0); // Blink every second
            if (seconds >= 60)
            {
                seconds = 0;
                minutes++;
            }
        }
    }
}

int main(void)
{
    uint8_t btn_start_last = 1, btn_reset_last = 1;
    uint8_t btn_start_curr, btn_reset_curr;

    DDRB = 0xFF;
    PORTB = 0x00;

    // Configure buttons: PD6=START, PD7=RESET
    DDRD &= ~((1 << 6) | (1 << 7));
    PORTD |= (1 << 6) | (1 << 7);

    // 10ms tick
    TCCR1B = (1 << WGM12) | (1 << CS12);
    OCR1A = 624;
    TIMSK |= (1 << OCIE1A);

    sei();

    while (1)
    {
        // Read buttons
        btn_start_curr = (PIND & (1 << 6)) ? 1 : 0;
        btn_reset_curr = (PIND & (1 << 7)) ? 1 : 0;

        // START/STOP button
        if (btn_start_last && !btn_start_curr)
        {
            running = !running;
        }

        // RESET button
        if (btn_reset_last && !btn_reset_curr)
        {
            running = 0;
            milliseconds = 0;
            seconds = 0;
            minutes = 0;
            PORTB = 0x00;
        }

        // Display time on LEDs (show seconds on 6 bits)
        if (!running)
        {
            PORTB = (PORTB & 0xC0) | (seconds & 0x3F);
        }

        btn_start_last = btn_start_curr;
        btn_reset_last = btn_reset_curr;
        _delay_ms(50);
    }
}
