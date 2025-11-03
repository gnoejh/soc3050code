/* ADC Multi Channel Scan - Sequential ADC Reading */
#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

void uart_init_m(void)
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
    uart_init_m();
    Adc_init();

    uart_puts("\r\n=== Multi-Channel ADC Scanner ===\r\n");
    uart_puts("Scanning 8 channels\r\n\r\n");

    while (1)
    {
        uart_puts("\r\n--- Scan Results ---\r\n");
        uint16_t values[8];
        uint16_t min = 1023, max = 0;
        uint32_t sum = 0;

        for (uint8_t ch = 0; ch < 8; ch++)
        {
            values[ch] = Read_Adc(ch);
            sum += values[ch];
            if (values[ch] < min)
                min = values[ch];
            if (values[ch] > max)
                max = values[ch];

            char msg[40];
            sprintf(msg, "ADC%u: %4u ", ch, values[ch]);
            uart_puts(msg);

            // Bar graph
            uint8_t bars = values[ch] / 64;
            uart_puts("[");
            for (uint8_t i = 0; i < 16; i++)
            {
                uart_putc(i < bars ? '#' : ' ');
            }
            uart_puts("]\r\n");
        }

        uint16_t avg = sum / 8;
        char stats[80];
        sprintf(stats, "\r\nStats: Min=%u Max=%u Avg=%u Range=%u\r\n", min, max, avg, max - min);
        uart_puts(stats);

        _delay_ms(1000);
    }
    return 0;
}
