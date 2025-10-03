/*
 * Timer Multi-Task Scheduler
 * Cooperative multitasking using timer interrupts
 */

#include "config.h"

volatile uint16_t ticks = 0;
volatile uint8_t task1_ready = 0;
volatile uint8_t task2_ready = 0;
volatile uint8_t task3_ready = 0;

ISR(TIMER1_COMPA_vect)
{
    ticks++;
    if (ticks % 10 == 0)
        task1_ready = 1; // Every 100ms
    if (ticks % 50 == 0)
        task2_ready = 1; // Every 500ms
    if (ticks % 100 == 0)
        task3_ready = 1; // Every 1 second
}

void task1(void) { PORTB ^= (1 << 0); } // Fast blink
void task2(void) { PORTB ^= (1 << 1); } // Medium blink
void task3(void) { PORTB ^= (1 << 2); } // Slow blink

int main(void)
{
    DDRB = 0xFF;

    // 10ms tick (CTC mode)
    TCCR1B = (1 << WGM12) | (1 << CS12);
    OCR1A = 624; // 10ms @ 16MHz/256
    TIMSK |= (1 << OCIE1A);

    sei();

    while (1)
    {
        if (task1_ready)
        {
            task1_ready = 0;
            task1();
        }
        if (task2_ready)
        {
            task2_ready = 0;
            task2();
        }
        if (task3_ready)
        {
            task3_ready = 0;
            task3();
        }
    }
}
