/* ADC Voltage Meter - 0-5V Measurement */
#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

void uart_init_v(void)
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
    uart_init_v();
    Adc_init();

    uart_puts("\r\n=== Voltage Meter ===\r\n");
    uart_puts("Reading ADC0-ADC3\r\n");

    while (1)
    {
        uart_puts("\r\n");
        for (uint8_t ch = 0; ch < 4; ch++)
        {
            uint16_t adc = Read_Adc_Averaged(ch, 16);
            uint32_t mv = (adc * 5000UL) / 1023;
            char msg[50];
            sprintf(msg, "CH%u: %u.%03uV (%u)\r\n", ch, (uint16_t)(mv / 1000), (uint16_t)(mv % 1000), adc);
            uart_puts(msg);
        }
        _delay_ms(1000);
    }
    return 0;
}
