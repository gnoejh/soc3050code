/* USART Binary Protocol - Frame-Based Communication */
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#define START_BYTE 0xAA
#define END_BYTE 0x55

typedef struct
{
    uint8_t start;
    uint8_t cmd;
    uint8_t len;
    uint8_t data[16];
    uint8_t checksum;
    uint8_t end;
} Frame;

volatile uint8_t rx_state = 0;
volatile Frame rx_frame;
volatile uint8_t frame_ready = 0;

uint8_t calc_checksum(Frame *f)
{
    uint8_t sum = f->cmd + f->len;
    for (uint8_t i = 0; i < f->len; i++)
        sum += f->data[i];
    return sum;
}

ISR(USART1_RX_vect)
{
    static uint8_t data_idx = 0;
    uint8_t byte = UDR1;

    switch (rx_state)
    {
    case 0:
        if (byte == START_BYTE)
        {
            rx_frame.start = byte;
            rx_state = 1;
        }
        break;
    case 1:
        rx_frame.cmd = byte;
        rx_state = 2;
        break;
    case 2:
        rx_frame.len = byte;
        data_idx = 0;
        rx_state = 3;
        break;
    case 3:
        rx_frame.data[data_idx++] = byte;
        if (data_idx >= rx_frame.len)
            rx_state = 4;
        break;
    case 4:
        rx_frame.checksum = byte;
        rx_state = 5;
        break;
    case 5:
        if (byte == END_BYTE)
        {
            rx_frame.end = byte;
            frame_ready = 1;
        }
        rx_state = 0;
        break;
    }
}

void uart_init_bin(void)
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

void send_frame(uint8_t cmd, uint8_t *data, uint8_t len)
{
    Frame f;
    f.start = START_BYTE;
    f.cmd = cmd;
    f.len = len;
    for (uint8_t i = 0; i < len; i++)
        f.data[i] = data[i];
    f.checksum = calc_checksum(&f);
    f.end = END_BYTE;

    uart_putc(f.start);
    uart_putc(f.cmd);
    uart_putc(f.len);
    for (uint8_t i = 0; i < len; i++)
        uart_putc(f.data[i]);
    uart_putc(f.checksum);
    uart_putc(f.end);
}

void process_frame(void)
{
    uint8_t calc = calc_checksum((Frame *)&rx_frame);

    if (calc != rx_frame.checksum)
    {
        uart_puts("\r\nERROR: Checksum mismatch\r\n");
        return;
    }

    char msg[80];
    sprintf(msg, "\r\nFrame OK: Cmd=0x%02X Len=%u Data=", rx_frame.cmd, rx_frame.len);
    uart_puts(msg);
    for (uint8_t i = 0; i < rx_frame.len; i++)
    {
        sprintf(msg, "%02X ", rx_frame.data[i]);
        uart_puts(msg);
    }
    uart_puts("\r\n");

    // Echo back
    send_frame(rx_frame.cmd | 0x80, (uint8_t *)rx_frame.data, rx_frame.len);
}

int main(void)
{
    init_devices();
    uart_init_bin();
    DDRB |= (1 << PB7);

    uart_puts("\r\n=== Binary Protocol ===\r\n");
    uart_puts("Frame: [0xAA][CMD][LEN][DATA..][CHK][0x55]\r\n");
    uart_puts("Waiting for frames...\r\n");

    uint8_t test_data[] = {0x01, 0x02, 0x03};
    send_frame(0x10, test_data, 3);

    while (1)
    {
        if (frame_ready)
        {
            process_frame();
            frame_ready = 0;
            PORTB ^= (1 << PB7);
        }
        _delay_ms(10);
    }
    return 0;
}
