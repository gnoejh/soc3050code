/* USART Command Parser - AT-Style Command Processing */
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

char cmd_buffer[64];
uint8_t cmd_idx = 0;
volatile uint8_t cmd_ready = 0;

ISR(USART1_RX_vect)
{
    char c = UDR1;
    if (c == '\r' || c == '\n')
    {
        cmd_buffer[cmd_idx] = '\0';
        if (cmd_idx > 0)
            cmd_ready = 1;
    }
    else if (cmd_idx < 63)
    {
        cmd_buffer[cmd_idx++] = c;
    }
}

void uart_init_cmd(void)
{
    uint16_t ubrr = (F_CPU / (16UL * BAUD)) - 1;
    UBRR1H = ubrr >> 8;
    UBRR1L = ubrr;
    UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
    sei();
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

void cmd_led(char *args)
{
    if (strcmp(args, "ON") == 0)
    {
        PORTB |= (1 << PB7);
        uart_puts("OK: LED ON\r\n");
    }
    else if (strcmp(args, "OFF") == 0)
    {
        PORTB &= ~(1 << PB7);
        uart_puts("OK: LED OFF\r\n");
    }
    else
    {
        uart_puts("ERROR: Use LED ON or LED OFF\r\n");
    }
}

void cmd_adc(char *args)
{
    uint8_t ch = atoi(args);
    if (ch > 7)
    {
        uart_puts("ERROR: Channel 0-7\r\n");
        return;
    }
    ADMUX = (1 << REFS0) | ch;
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
        ;
    char msg[40];
    sprintf(msg, "OK: ADC%u = %u\r\n", ch, ADC);
    uart_puts(msg);
}

void cmd_help(void)
{
    uart_puts("Commands:\r\n");
    uart_puts("  LED ON/OFF - Control LED\r\n");
    uart_puts("  ADC <ch> - Read ADC channel\r\n");
    uart_puts("  HELP - This message\r\n");
    uart_puts("  VER - Version info\r\n");
}

void parse_command(void)
{
    char *space = strchr(cmd_buffer, ' ');
    char *args = "";
    if (space)
    {
        *space = '\0';
        args = space + 1;
    }

    if (strcmp(cmd_buffer, "LED") == 0)
        cmd_led(args);
    else if (strcmp(cmd_buffer, "ADC") == 0)
        cmd_adc(args);
    else if (strcmp(cmd_buffer, "HELP") == 0)
        cmd_help();
    else if (strcmp(cmd_buffer, "VER") == 0)
        uart_puts("OK: Version 1.0\r\n");
    else
        uart_puts("ERROR: Unknown command (try HELP)\r\n");
}

int main(void)
{
    init_devices();
    uart_init_cmd();
    DDRB |= (1 << PB7);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    uart_puts("\r\n=== Command Parser ===\r\n");
    uart_puts("Type HELP for commands\r\n> ");

    while (1)
    {
        if (cmd_ready)
        {
            uart_puts(cmd_buffer);
            uart_puts("\r\n");
            parse_command();
            cmd_idx = 0;
            cmd_ready = 0;
            uart_puts("> ");
        }
        _delay_ms(10);
    }
    return 0;
}
