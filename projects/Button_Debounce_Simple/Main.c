/* Button Debounce Simple - Software Debouncing */
#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define DEBOUNCE_MS 50

void uart_init_b(void)
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

uint8_t button_read(uint8_t pin)
{
    if (!(PIND & (1 << pin)))
    {
        _delay_ms(DEBOUNCE_MS);
        if (!(PIND & (1 << pin)))
        {
            while (!(PIND & (1 << pin)))
                ; // Wait release
            _delay_ms(DEBOUNCE_MS);
            return 1;
        }
    }
    return 0;
}

int main(void)
{
    init_devices();
    uart_init_b();

    DDRD &= ~((1 << PD0) | (1 << PD1) | (1 << PD2));
    PORTD |= (1 << PD0) | (1 << PD1) | (1 << PD2);
    DDRB |= (1 << PB7) | (1 << PB6) | (1 << PB5);

    uart_puts("\r\n=== Button Debounce ===\r\n");
    uart_puts("PD0/PD1/PD2 -> LEDs\r\n");

    uint32_t count0 = 0, count1 = 0, count2 = 0;

    while (1)
    {
        if (button_read(0))
        {
            count0++;
            PORTB ^= (1 << PB7);
            char msg[40];
            sprintf(msg, "BTN0: %lu\r\n", count0);
            uart_puts(msg);
        }
        if (button_read(1))
        {
            count1++;
            PORTB ^= (1 << PB6);
            char msg[40];
            sprintf(msg, "BTN1: %lu\r\n", count1);
            uart_puts(msg);
        }
        if (button_read(2))
        {
            count2++;
            PORTB ^= (1 << PB5);
            char msg[40];
            sprintf(msg, "BTN2: %lu\r\n", count2);
            uart_puts(msg);
        }
    }
    return 0;
}
