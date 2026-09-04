/* INT Rotary Encoder - Polling Quadrature Decoding (ATmega128) */
#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

volatile int16_t encoder_pos = 0;
uint8_t last_state = 0;

void uart_init_r(void)
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

void read_encoder(void)
{
    uint8_t a = (PINB & (1 << PB0)) ? 1 : 0;
    uint8_t b = (PINB & (1 << PB1)) ? 1 : 0;
    uint8_t curr = (a << 1) | b;
    uint8_t combined = (last_state << 2) | curr;

    if (combined == 0b0001 || combined == 0b0111 || combined == 0b1110 || combined == 0b1000)
        encoder_pos++;
    else if (combined == 0b0010 || combined == 0b0100 || combined == 0b1101 || combined == 0b1011)
        encoder_pos--;

    last_state = curr;
    PORTB = (PORTB & 0x0F) | ((encoder_pos & 0x0F) << 4);
}

int main(void)
{
    init_devices();
    uart_init_r();

    DDRB = 0xF0;  // PB0,PB1 input, PB4-7 output
    PORTB = 0x03; // Pull-ups on PB0,PB1

    uart_puts("\r\n=== Rotary Encoder (Polling) ===\r\n");
    uart_puts("Connect: A->PB0, B->PB1\r\n");
    uart_puts("Polling mode (no PCINT on ATmega128)\r\n");

    last_state = PINB & 0x03;

    while (1)
    {
        read_encoder();

        char msg[40];
        sprintf(msg, "\rPosition: %d  ", encoder_pos);
        uart_puts(msg);
        _delay_ms(20);
    }
    return 0;
}
