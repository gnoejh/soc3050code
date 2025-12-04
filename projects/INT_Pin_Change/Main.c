/* INT Pin Change - Multiple INT Monitoring (ATmega128) */
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

volatile uint8_t int_flags = 0;

ISR(INT0_vect) { int_flags |= 0x01; }
ISR(INT1_vect) { int_flags |= 0x02; }
ISR(INT2_vect) { int_flags |= 0x04; }
ISR(INT3_vect) { int_flags |= 0x08; }

void uart_init_p(void)
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

int main(void)
{
    init_devices();
    uart_init_p();

    DDRB |= 0x0F;  // PB0-3 outputs
    DDRD &= ~0x0F; // PD0-3 inputs
    PORTD |= 0x0F; // Pull-ups

    uart_puts("\r\n=== Multi-Pin Interrupt Monitor ===\r\n");
    uart_puts("Monitoring INT0-INT3 (PD0-PD3)\r\n");

    EICRA = 0xAA; // Falling edge for all
    EIMSK = 0x0F; // Enable INT0-3
    sei();

    while (1)
    {
        if (int_flags)
        {
            char msg[60];
            sprintf(msg, "Triggered: ");
            uart_puts(msg);
            if (int_flags & 0x01)
            {
                uart_puts("INT0 ");
                PORTB ^= 0x01;
            }
            if (int_flags & 0x02)
            {
                uart_puts("INT1 ");
                PORTB ^= 0x02;
            }
            if (int_flags & 0x04)
            {
                uart_puts("INT2 ");
                PORTB ^= 0x04;
            }
            if (int_flags & 0x08)
            {
                uart_puts("INT3 ");
                PORTB ^= 0x08;
            }
            uart_puts("\r\n");
            int_flags = 0;
        }
        _delay_ms(10);
    }
    return 0;
}
