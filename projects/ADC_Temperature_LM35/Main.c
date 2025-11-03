/* ADC Temperature LM35 - Celsius Measurement */
#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

void uart_init_t(void)
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
    uart_init_t();
    Adc_init();
    DDRB |= (1 << PB7);

    uart_puts("\r\n=== LM35 Temperature Sensor ===\r\n");
    uart_puts("Connect LM35 to ADC0\r\n");
    uart_puts("LM35: 10mV/°C\r\n\r\n");

    while (1)
    {
        uint16_t adc = Read_Adc_Averaged(0, 32);
        uint32_t mv = (adc * 5000UL) / 1023;
        int16_t celsius = mv / 10; // LM35: 10mV per degree
        int16_t fahrenheit = (celsius * 9 / 5) + 32;

        char msg[80];
        sprintf(msg, "\rTemp: %d°C (%d°F) | ADC=%u (%lumV)  ", celsius, fahrenheit, adc, mv);
        uart_puts(msg);

        if (celsius > 30)
            PORTB |= (1 << PB7);
        else
            PORTB &= ~(1 << PB7);

        _delay_ms(500);
    }
    return 0;
}
