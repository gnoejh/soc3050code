/* Button Events Advanced - Long Press, Double Click */
#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define LONG_PRESS_MS 1000
#define DOUBLE_CLICK_MS 300

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

uint8_t detect_button_event(uint8_t pin)
{
    static uint16_t press_time = 0;
    static uint16_t last_release = 0;
    static uint8_t click_count = 0;

    if (!(PIND & (1 << pin)))
    {                  // Pressed
        _delay_ms(50); // Debounce
        if (!(PIND & (1 << pin)))
        {
            press_time = 0;
            while (!(PIND & (1 << pin)))
            {
                _delay_ms(10);
                press_time += 10;
                if (press_time >= LONG_PRESS_MS)
                {
                    while (!(PIND & (1 << pin)))
                        ;
                    return 3; // Long press
                }
            }
            _delay_ms(50); // Release debounce

            uint16_t since_last = 0; // Simplified timing
            if (click_count == 0)
            {
                click_count = 1;
                _delay_ms(DOUBLE_CLICK_MS);
                if (click_count == 1)
                {
                    click_count = 0;
                    return 1; // Single click
                }
            }
            else
            {
                click_count = 0;
                return 2; // Double click
            }
        }
    }
    return 0;
}

int main(void)
{
    init_devices();
    uart_init_e();

    DDRD &= ~(1 << PD0);
    PORTD |= (1 << PD0);
    DDRB |= (1 << PB7) | (1 << PB6) | (1 << PB5);

    uart_puts("\r\n=== Advanced Button Events ===\r\n");
    uart_puts("Try: Click, Double-click, Long-press\r\n");

    while (1)
    {
        uint8_t event = detect_button_event(0);

        if (event == 1)
        {
            uart_puts("Event: CLICK\r\n");
            PORTB ^= (1 << PB7);
        }
        else if (event == 2)
        {
            uart_puts("Event: DOUBLE-CLICK\r\n");
            PORTB ^= (1 << PB6);
        }
        else if (event == 3)
        {
            uart_puts("Event: LONG-PRESS\r\n");
            PORTB ^= (1 << PB5);
        }

        _delay_ms(10);
    }
    return 0;
}
