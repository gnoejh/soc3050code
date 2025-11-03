/* USART Ring Buffer - Advanced Circular Buffer Implementation */
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#define BUFFER_SIZE 128
volatile uint8_t rx_buffer[BUFFER_SIZE];
volatile uint8_t rx_head = 0, rx_tail = 0;

ISR(USART1_RX_vect)
{
    uint8_t data = UDR1;
    uint8_t next = (rx_head + 1) % BUFFER_SIZE;
    if (next != rx_tail)
    {
        rx_buffer[rx_head] = data;
        rx_head = next;
    }
}

void usart_init_rb(void)
{
    uint16_t ubrr = (F_CPU / (16UL * BAUD)) - 1;
    UBRR1H = ubrr >> 8;
    UBRR1L = ubrr;
    UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
    sei();
}

uint8_t rb_available(void) { return (rx_head - rx_tail + BUFFER_SIZE) % BUFFER_SIZE; }
uint8_t rb_read(void)
{
    uint8_t data = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % BUFFER_SIZE;
    return data;
}
void rb_write(char c)
{
    while (!(UCSR1A & (1 << UDRE1)))
        ;
    UDR1 = c;
}
void rb_puts(const char *s)
{
    while (*s)
        rb_write(*s++);
}

void demo1_basic_buffer(void)
{
    rb_puts("\r\n=== Ring Buffer Demo ===\r\nType and see buffered data\r\n");
    while (1)
    {
        if (rb_available() > 0)
        {
            char c = rb_read();
            if (c == 'q')
                break;
            char msg[40];
            sprintf(msg, "Got: '%c' | Buffer: %u/%u\r\n", c, rb_available(), BUFFER_SIZE);
            rb_puts(msg);
        }
        _delay_ms(10);
    }
}

void demo2_burst_handling(void)
{
    rb_puts("\r\n=== Burst Test ===\r\nPaste text fast!\r\n");
    uint32_t count = 0;
    while (count < 100)
    {
        if (rb_available() > 0)
        {
            char c = rb_read();
            rb_write(c);
            count++;
        }
        if (count % 10 == 0 && count > 0)
        {
            char stat[30];
            sprintf(stat, "\r\nProcessed: %lu\r\n", count);
            rb_puts(stat);
        }
    }
}

int main(void)
{
    init_devices();
    usart_init_rb();
    DDRB |= (1 << PB7);

    rb_puts("\r\n=== USART Ring Buffer ===\r\n");
    while (1)
    {
        rb_puts("\r\n1. Basic Buffer\r\n2. Burst Test\r\nChoice: ");
        while (!rb_available())
            ;
        char c = rb_read();
        rb_write(c);
        rb_puts("\r\n");

        if (c == '1')
            demo1_basic_buffer();
        else if (c == '2')
            demo2_burst_handling();
    }
    return 0;
}
