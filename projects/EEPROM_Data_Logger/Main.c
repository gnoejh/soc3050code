/* EEPROM Data Logger - Circular Buffer Logging */
#include "config.h"
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdio.h>

#define LOG_START 0x100
#define LOG_SIZE 256
#define LOG_END (LOG_START + LOG_SIZE)

typedef struct
{
    uint16_t value;
    uint8_t timestamp;
} LogEntry;

uint16_t EEMEM ee_log_head;
uint16_t log_head;

void uart_init_l(void)
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

void log_init(void)
{
    log_head = eeprom_read_word(&ee_log_head);
    if (log_head >= LOG_SIZE)
        log_head = 0;
}

void log_write(uint16_t value, uint8_t timestamp)
{
    LogEntry entry = {value, timestamp};
    uint16_t addr = LOG_START + (log_head * sizeof(LogEntry)) % LOG_SIZE;

    eeprom_write_block(&entry, (void *)addr, sizeof(LogEntry));
    _delay_ms(10);

    log_head = (log_head + 1) % (LOG_SIZE / sizeof(LogEntry));
    eeprom_write_word(&ee_log_head, log_head);
    _delay_ms(10);
}

void log_read_all(void)
{
    uart_puts("\r\n--- Log Entries ---\r\n");
    uint16_t max_entries = LOG_SIZE / sizeof(LogEntry);

    for (uint16_t i = 0; i < max_entries; i++)
    {
        LogEntry entry;
        uint16_t addr = LOG_START + (i * sizeof(LogEntry));
        eeprom_read_block(&entry, (void *)addr, sizeof(LogEntry));

        if (entry.value != 0xFFFF)
        {
            char msg[50];
            sprintf(msg, "[%u] Val=%u Time=%u\r\n", i, entry.value, entry.timestamp);
            uart_puts(msg);
        }
    }
}

void log_clear(void)
{
    uart_puts("Clearing log...\r\n");
    for (uint16_t addr = LOG_START; addr < LOG_END; addr++)
    {
        eeprom_write_byte((uint8_t *)addr, 0xFF);
    }
    log_head = 0;
    eeprom_write_word(&ee_log_head, 0);
    _delay_ms(100);
    uart_puts("Log cleared!\r\n");
}

int main(void)
{
    init_devices();
    uart_init_l();
    Adc_init();
    DDRB |= (1 << PB7);

    uart_puts("\r\n=== EEPROM Data Logger ===\r\n");
    log_init();

    uint8_t time_counter = 0;

    while (1)
    {
        uart_puts("\r\n1. Log ADC0\r\n2. View Log\r\n3. Clear Log\r\n4. Auto-log (10x)\r\nChoice: ");

        char c = uart_getc();
        uart_putc(c);
        uart_puts("\r\n");

        if (c == '1')
        {
            uint16_t adc = Read_Adc(0);
            log_write(adc, time_counter++);
            char msg[40];
            sprintf(msg, "Logged: ADC=%u @ T=%u\r\n", adc, time_counter - 1);
            uart_puts(msg);
        }
        else if (c == '2')
            log_read_all();
        else if (c == '3')
        {
            log_clear();
            time_counter = 0;
        }
        else if (c == '4')
        {
            uart_puts("Auto-logging 10 samples...\r\n");
            for (uint8_t i = 0; i < 10; i++)
            {
                uint16_t adc = Read_Adc(0);
                log_write(adc, time_counter++);
                PORTB ^= (1 << PB7);
                _delay_ms(500);
            }
            uart_puts("Done!\r\n");
        }
    }
    return 0;
}
