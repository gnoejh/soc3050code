/*
 * ==============================================================================
 * SIMPLE DUAL UART TEST - Safe for SimulIDE without ESP-01
 * ==============================================================================
 * This version just demonstrates UART0/UART1 bridge without state machine
 * Use this to verify UARTs work before testing full AT command logic
 * ==============================================================================
 */

#include "config.h"

#define ESP_BAUD 115200UL
#define DEBUG_BAUD 9600UL

/* UART0 Init (PE0/PE1) */
void uart0_init(void)
{
    uint16_t ubrr = (F_CPU / (16UL * ESP_BAUD)) - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

/* UART1 Init (PD2/PD3) - align with Serial_Communications (U2X=1 for accuracy) */
void uart1_init(void)
{
    // Enable double speed for tighter baud accuracy at 16 MHz
    UCSR1A = (1 << U2X1);
    // With U2X=1, UBRR = F_CPU/(8*BAUD) - 1
    uint16_t ubrr = (F_CPU / (8UL * DEBUG_BAUD)) - 1;
    UBRR1H = (uint8_t)(ubrr >> 8);
    UBRR1L = (uint8_t)ubrr;
    // Enable RX/TX
    UCSR1B = (1 << RXEN1) | (1 << TXEN1);
    // 8N1
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}
}

/* Blocking send */
void uart0_putc(char c)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = c;
}

void uart1_putc(char c)
{
    while (!(UCSR1A & (1 << UDRE1)))
        ;
    UDR1 = c;
}

void uart1_puts(const char *str)
{
    while (*str)
        uart1_putc(*str++);
}

/* Check data available */
uint8_t uart0_available(void)
{
    return (UCSR0A & (1 << RXC0));
}

uint8_t uart1_available(void)
{
    return (UCSR1A & (1 << RXC1));
}

/* Blocking read */
char uart0_getc(void)
{
    while (!uart0_available())
        ;
    return UDR0;
}

char uart1_getc(void)
{
    while (!uart1_available())
        ;
    return UDR1;
}

int main(void)
{
    // Init ports
    DDRA = 0x00;
    PORTA = 0xFF;
    DDRB = 0xFF;
    PORTB = 0x00;

    // Init UARTs
    uart0_init();
    uart1_init();

    _delay_ms(100);

    uart1_puts("\r\n\r\n");
    uart1_puts("================================\r\n");
    uart1_puts("  Simple UART Bridge Test\r\n");
    uart1_puts("================================\r\n");
    uart1_puts("UART0 (PE0/PE1): 115200 baud\r\n");
    uart1_puts("UART1 (PD2/PD3): 9600 baud\r\n");
    uart1_puts("================================\r\n");
    uart1_puts("\r\n");
    uart1_puts("Type commands - they will be\r\n");
    uart1_puts("forwarded to UART0 and echoed\r\n");
    uart1_puts("\r\n");
    uart1_puts("Ready!\r\n\r\n");

    uint16_t counter = 0;

    // Simple bridge loop
    while (1)
    {
        // UART1 → UART0 (with local echo)
        if (uart1_available())
        {
            char c = uart1_getc();
            uart1_putc(c); // Local echo
            uart0_putc(c); // Forward to UART0

            // Show activity
            if (++counter % 10 == 0)
            {
                uart1_puts(" [TX]");
            }
        }

        // UART0 → UART1
        if (uart0_available())
        {
            char c = uart0_getc();
            uart1_puts("\r\n<< ");
            uart1_putc(c);
            uart1_puts("\r\n");
        }
    }

    return 0;
}
