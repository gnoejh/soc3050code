/* INT External Pins - INT0-INT7 Interrupts */
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

volatile uint32_t int0_count = 0, int1_count = 0;

ISR(INT0_vect)
{
    int0_count++;
    PORTB ^= (1 << PB7);
}
ISR(INT1_vect)
{
    int1_count++;
    PORTB ^= (1 << PB6);
}

void uart_init_i(void)
{
    uint16_t ubrr = (F_CPU / (16UL * BAUD)) - 1;
    UBRR1H = ubrr >> 8;
    UBRR1L = ubrr;
    UCSR1B = (1 << RXEN1) | (1 << TXEN1);
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}
void uart_putc(char c)
{
    while (!(UCSR1A & (1 << UDRE1)))
        ;
    UDR1 = c;
}
void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

void demo1_falling_edge(void)
{
    uart_puts("\r\n=== Falling Edge INT0 ===\r\n");
    EICRA = (1 << ISC01); // Falling edge INT0
    EIMSK = (1 << INT0);
    sei();
    uart_puts("Trigger INT0 (PD0)...\r\n");
    for (uint8_t i = 0; i < 100; i++)
    {
        char msg[40];
        sprintf(msg, "\rCount: %lu  ", int0_count);
        uart_puts(msg);
        _delay_ms(100);
    }
    EIMSK = 0;
}

void demo2_rising_edge(void)
{
    uart_puts("\r\n=== Rising Edge INT1 ===\r\n");
    EICRA = (1 << ISC11) | (1 << ISC10); // Rising edge INT1
    EIMSK = (1 << INT1);
    sei();
    uart_puts("Trigger INT1 (PD1)...\r\n");
    for (uint8_t i = 0; i < 100; i++)
    {
        char msg[40];
        sprintf(msg, "\rCount: %lu  ", int1_count);
        uart_puts(msg);
        _delay_ms(100);
    }
    EIMSK = 0;
}

int main(void)
{
    init_devices();
    uart_init_i();
    DDRB |= (1 << PB7) | (1 << PB6);
    DDRD &= ~((1 << PD0) | (1 << PD1));
    PORTD |= (1 << PD0) | (1 << PD1);

    uart_puts("\r\n=== External Interrupts ===\r\n");
    while (1)
    {
        uart_puts("\r\n1. Falling Edge\r\n2. Rising Edge\r\nChoice: ");
        while (!(UCSR1A & (1 << RXC1)))
            ;
        char c = UDR1;
        uart_putc(c);
        uart_puts("\r\n");

        if (c == '1')
            demo1_falling_edge();
        else if (c == '2')
            demo2_rising_edge();
    }
    return 0;
}
