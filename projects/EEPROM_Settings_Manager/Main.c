/* EEPROM Settings Manager - Configuration Persistence */
#include "config.h"
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    uint16_t magic;
    uint8_t led_mode;
    uint16_t threshold;
    char device_name[16];
    uint8_t checksum;
} Settings;

Settings EEMEM ee_settings;
Settings ram_settings;

#define MAGIC_NUMBER 0xABCD

uint8_t calc_checksum(Settings *s)
{
    uint8_t sum = 0;
    uint8_t *ptr = (uint8_t *)s;
    for (uint8_t i = 0; i < sizeof(Settings) - 1; i++)
        sum += ptr[i];
    return sum;
}

void uart_init_s(void)
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

void save_settings(void); // Forward declaration

void load_settings(void)
{
    eeprom_read_block(&ram_settings, &ee_settings, sizeof(Settings));

    if (ram_settings.magic != MAGIC_NUMBER ||
        calc_checksum(&ram_settings) != ram_settings.checksum)
    {
        uart_puts("Invalid settings, loading defaults...\r\n");
        ram_settings.magic = MAGIC_NUMBER;
        ram_settings.led_mode = 1;
        ram_settings.threshold = 512;
        strcpy(ram_settings.device_name, "ATmega128");
        ram_settings.checksum = calc_checksum(&ram_settings);
        save_settings();
    }
    else
    {
        uart_puts("Settings loaded from EEPROM\r\n");
    }
}

void save_settings(void)
{
    ram_settings.checksum = calc_checksum(&ram_settings);
    eeprom_write_block(&ram_settings, &ee_settings, sizeof(Settings));
    _delay_ms(50);
    uart_puts("Settings saved!\r\n");
}

void show_settings(void)
{
    char msg[80];
    uart_puts("\r\n--- Current Settings ---\r\n");
    sprintf(msg, "LED Mode: %u\r\nThreshold: %u\r\nDevice: %s\r\n",
            ram_settings.led_mode, ram_settings.threshold, ram_settings.device_name);
    uart_puts(msg);
}

int main(void)
{
    init_devices();
    uart_init_s();
    DDRB |= (1 << PB7);

    uart_puts("\r\n=== Settings Manager ===\r\n");
    load_settings();
    show_settings();

    while (1)
    {
        uart_puts("\r\n1. Show Settings\r\n2. Toggle LED Mode\r\n");
        uart_puts("3. Set Threshold\r\n4. Save\r\n5. Reset to Defaults\r\nChoice: ");

        char c = uart_getc();
        uart_putc(c);
        uart_puts("\r\n");

        if (c == '1')
            show_settings();
        else if (c == '2')
        {
            ram_settings.led_mode = !ram_settings.led_mode;
            uart_puts(ram_settings.led_mode ? "LED ON\r\n" : "LED OFF\r\n");
            if (ram_settings.led_mode)
                PORTB |= (1 << PB7);
            else
                PORTB &= ~(1 << PB7);
        }
        else if (c == '3')
        {
            uart_puts("Enter threshold (0-1023): ");
            ram_settings.threshold = (ram_settings.threshold + 100) % 1024;
            char msg[30];
            sprintf(msg, "%u\r\n", ram_settings.threshold);
            uart_puts(msg);
        }
        else if (c == '4')
            save_settings();
        else if (c == '5')
        {
            ram_settings.magic = 0;
            load_settings();
        }
    }
    return 0;
}
