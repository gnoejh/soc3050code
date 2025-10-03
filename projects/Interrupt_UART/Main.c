/* UART RX Interrupt */
#include "config.h"

volatile char received_char = 0;

ISR(USART0_RX_vect)
{
    received_char = UDR0;
    PORTB = received_char; // Display on LEDs
}

int main(void)
{
    DDRB = 0xFF;

    // UART init @ 9600 baud
    UBRR0H = 0;
    UBRR0L = 103;                           // 16MHz/9600
    UCSR0B = (1 << RXEN0) | (1 << RXCIE0);  // RX + RX interrupt
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1

    sei();
    while (1)
    {
    }
}
