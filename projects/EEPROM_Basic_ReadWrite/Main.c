/* EEPROM Basic Read/Write - Internal EEPROM Access */
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdio.h>

void uart_init_e(void)
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
char uart_getc(void)
{
    while (!(UCSR1A & (1 << RXC1)))
        ;
    return UDR1;
}

void demo1_byte_access(void)
{
    uart_puts("\r\n=== Byte Access ===\r\n");

    // Write test
    uart_puts("Writing 0xAA to addr 0x10...\r\n");
    eeprom_write_byte((uint8_t *)0x10, 0xAA);
    _delay_ms(10);

    uart_puts("Writing 0x55 to addr 0x11...\r\n");
    eeprom_write_byte((uint8_t *)0x11, 0x55);
    _delay_ms(10);

    // Read test
    uint8_t val1 = eeprom_read_byte((uint8_t *)0x10);
    uint8_t val2 = eeprom_read_byte((uint8_t *)0x11);

    char msg[50];
    sprintf(msg, "Read: 0x10=0x%02X, 0x11=0x%02X\r\n", val1, val2);
    uart_puts(msg);
}

void demo2_block_access(void)
{
    uart_puts("\r\n=== Block Access ===\r\n");

    char write_data[] = "Hello EEPROM!";
    char read_data[32] = {0};

    uart_puts("Writing block...\r\n");
    eeprom_write_block(write_data, (void *)0x20, sizeof(write_data));
    _delay_ms(50);

    uart_puts("Reading block...\r\n");
    eeprom_read_block(read_data, (void *)0x20, sizeof(write_data));

    char msg[50];
    sprintf(msg, "Read: '%s'\r\n", read_data);
    uart_puts(msg);
}

void demo3_counter(void)
{
    uart_puts("\r\n=== Power-On Counter ===\r\n");

    uint32_t counter = eeprom_read_dword((uint32_t *)0x40);

    char msg[50];
    sprintf(msg, "Boot count: %lu\r\n", counter);
    uart_puts(msg);

    counter++;
    eeprom_write_dword((uint32_t *)0x40, counter);
    _delay_ms(20);

    uart_puts("Counter incremented!\r\n");
}

void demo4_erase_test(void)
{
    uart_puts("\r\n=== Erase Test ===\r\n");
    uart_puts("Erasing 16 bytes at 0x50...\r\n");

    for (uint8_t i = 0; i < 16; i++)
    {
        eeprom_write_byte((uint8_t *)(0x50 + i), 0xFF);
    }
    _delay_ms(50);

    uart_puts("Verifying erase...\r\n");
    uint8_t ok = 1;
    for (uint8_t i = 0; i < 16; i++)
    {
        if (eeprom_read_byte((uint8_t *)(0x50 + i)) != 0xFF)
            ok = 0;
    }

    if (ok)
        uart_puts("OK: All bytes = 0xFF\r\n");
    else
        uart_puts("ERROR: Erase failed\r\n");
}

int main(void)
{
    init_devices();
    uart_init_e();
    DDRB |= (1 << PB7);

    uart_puts("\r\n=== EEPROM Basic ReadWrite ===\r\n");
    uart_puts("ATmega128: 4KB EEPROM\r\n");

    while (1)
    {
        uart_puts("\r\n1. Byte Access\r\n2. Block Access\r\n");
        uart_puts("3. Counter Demo\r\n4. Erase Test\r\nChoice: ");

        char c = uart_getc();
        uart_putc(c);
        uart_puts("\r\n");

        if (c == '1')
            demo1_byte_access();
        else if (c == '2')
            demo2_block_access();
        else if (c == '3')
            demo3_counter();
        else if (c == '4')
            demo4_erase_test();

        PORTB ^= (1 << PB7);
    }
    return 0;
}
