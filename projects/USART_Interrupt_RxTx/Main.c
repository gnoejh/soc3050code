/* ============================================================================
 * USART Interrupt-Driven Communication
 * ============================================================================
 *
 * PROJECT: USART_Interrupt_RxTx
 * MCU: ATmega128
 * F_CPU: 7.3728 MHz
 *
 * DESCRIPTION:
 * Demonstrates interrupt-driven USART communication for non-blocking I/O.
 * RX and TX interrupts allow the CPU to perform other tasks while waiting
 * for serial data.
 *
 * LEARNING OBJECTIVES:
 * 1. Configure USART RX/TX interrupts
 * 2. Implement ISR-based character handling
 * 3. Non-blocking serial I/O
 * 4. Buffer management in ISRs
 *
 * DEMOS:
 * 1. Interrupt-based echo
 * 2. Background data processing
 * 3. TX interrupt with queuing
 * 4. Full-duplex communication
 *
 * ============================================================================
 */

#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

/* ============================================================================
 * BUFFER CONFIGURATION
 * ============================================================================ */

#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64

volatile char rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;
volatile uint8_t rx_count = 0;

volatile char tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_head = 0;
volatile uint8_t tx_tail = 0;
volatile uint8_t tx_count = 0;

volatile uint32_t rx_char_count = 0;
volatile uint32_t tx_char_count = 0;

/* ============================================================================
 * INTERRUPT SERVICE ROUTINES
 * ============================================================================ */

// USART RX Complete Interrupt
ISR(USART1_RX_vect)
{
    char received = UDR1; // Read received character

    // Store in buffer if not full
    if (rx_count < RX_BUFFER_SIZE)
    {
        rx_buffer[rx_head] = received;
        rx_head = (rx_head + 1) % RX_BUFFER_SIZE;
        rx_count++;
        rx_char_count++;
    }
    // If buffer full, character is lost (could set error flag)
}

// USART TX Complete Interrupt
ISR(USART1_TX_vect)
{
    // This fires when transmission is complete (not when UDR is empty)
    // Usually not needed for most applications
}

// USART Data Register Empty Interrupt
ISR(USART1_UDRE_vect)
{
    if (tx_count > 0)
    {
        // Send next character from buffer
        UDR1 = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
        tx_count--;
        tx_char_count++;
    }
    else
    {
        // Buffer empty, disable UDRE interrupt
        UCSR1B &= ~(1 << UDRIE1);
    }
}

/* ============================================================================
 * USART FUNCTIONS
 * ============================================================================ */

// Initialize USART with interrupts
void usart_init_interrupt(void)
{
    // Set baud rate: UBRR = (F_CPU / (16 * BAUD)) - 1
    uint16_t ubrr_val = (F_CPU / (16UL * BAUD)) - 1;
    UBRR1H = (uint8_t)(ubrr_val >> 8);
    UBRR1L = (uint8_t)ubrr_val;

    // Enable RX, TX, and RX interrupt
    UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);

    // 8 data bits, 1 stop bit, no parity
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);

    sei(); // Enable global interrupts
}

// Check if data available in RX buffer
uint8_t usart_rx_available(void)
{
    return rx_count;
}

// Read character from RX buffer (non-blocking)
char usart_rx_get(void)
{
    if (rx_count == 0)
        return 0;

    char c = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;

    cli(); // Atomic operation
    rx_count--;
    sei();

    return c;
}

// Queue character for transmission
uint8_t usart_tx_put(char c)
{
    // Check if buffer full
    if (tx_count >= TX_BUFFER_SIZE)
    {
        return 0; // Failed
    }

    // Add to buffer
    tx_buffer[tx_head] = c;
    tx_head = (tx_head + 1) % TX_BUFFER_SIZE;

    cli();
    tx_count++;
    sei();

    // Enable UDRE interrupt to start transmission
    UCSR1B |= (1 << UDRIE1);

    return 1; // Success
}

// Queue string for transmission
void usart_tx_puts(const char *str)
{
    while (*str)
    {
        // Wait if buffer full
        while (tx_count >= TX_BUFFER_SIZE)
            ;
        usart_tx_put(*str++);
    }
}

/* ============================================================================
 * DEMO 1: Interrupt-Based Echo
 * ============================================================================ */

void demo1_interrupt_echo(void)
{
    usart_tx_puts("\r\n=== DEMO 1: Interrupt Echo ===\r\n");
    usart_tx_puts("Type characters (they echo back)\r\n");
    usart_tx_puts("Press 'q' to quit\r\n\r\n");

    while (1)
    {
        if (usart_rx_available())
        {
            char c = usart_rx_get();

            if (c == 'q')
                break;

            // Echo character back
            usart_tx_put(c);

            // Show hex value for special characters
            if (c < 32 || c > 126)
            {
                char hex[10];
                sprintf(hex, " [0x%02X]", (uint8_t)c);
                usart_tx_puts(hex);
            }
        }

        // CPU can do other work here!
        // Simulating background task
        PORTB ^= (1 << PB7); // Blink LED to show CPU is free
        _delay_ms(100);
    }
}

/* ============================================================================
 * DEMO 2: Background Processing
 * ============================================================================ */

void demo2_background_processing(void)
{
    usart_tx_puts("\r\n=== DEMO 2: Background Processing ===\r\n");
    usart_tx_puts("CPU performs tasks while receiving data\r\n");
    usart_tx_puts("Type anything - press 'q' to quit\r\n\r\n");

    uint32_t counter = 0;
    uint32_t last_report = 0;

    while (1)
    {
        // Main task: increment counter (simulating work)
        counter++;

        // Check for received data
        if (usart_rx_available())
        {
            char c = usart_rx_get();
            if (c == 'q')
                break;

            // Process received character
            char msg[50];
            sprintf(msg, "RX: '%c' (0x%02X) | Counter: %lu\r\n", c, (uint8_t)c, counter);
            usart_tx_puts(msg);
        }

        // Periodic status report every ~100000 iterations
        if (counter - last_report >= 100000)
        {
            char status[60];
            sprintf(status, "Background counter: %lu | RX: %lu | TX: %lu\r\n",
                    counter, rx_char_count, tx_char_count);
            usart_tx_puts(status);
            last_report = counter;
        }

        // Small delay
        _delay_us(10);
    }

    char final[50];
    sprintf(final, "\r\nFinal counter value: %lu\r\n", counter);
    usart_tx_puts(final);
}

/* ============================================================================
 * DEMO 3: TX Interrupt with Queuing
 * ============================================================================ */

void demo3_tx_queuing(void)
{
    usart_tx_puts("\r\n=== DEMO 3: TX Interrupt Queuing ===\r\n");
    usart_tx_puts("Demonstrating non-blocking transmission\r\n");
    usart_tx_puts("Press 's' to send burst | 'q' to quit\r\n\r\n");

    while (1)
    {
        if (usart_rx_available())
        {
            char c = usart_rx_get();

            if (c == 'q')
                break;

            if (c == 's')
            {
                // Queue a large message instantly (non-blocking)
                usart_tx_puts("\r\n[BURST START]\r\n");

                for (uint8_t i = 0; i < 10; i++)
                {
                    char line[60];
                    sprintf(line, "Line %02u: The quick brown fox jumps over the lazy dog\r\n", i + 1);
                    usart_tx_puts(line);
                }

                usart_tx_puts("[BURST END]\r\n\r\n");

                // Show buffer status
                char status[50];
                sprintf(status, "TX buffer count: %u / %u\r\n", tx_count, TX_BUFFER_SIZE);
                usart_tx_puts(status);
            }
        }

        // CPU is free while TX interrupt sends buffered data
        PORTB ^= (1 << PB7);
        _delay_ms(50);
    }
}

/* ============================================================================
 * DEMO 4: Full-Duplex Communication
 * ============================================================================ */

void demo4_full_duplex(void)
{
    usart_tx_puts("\r\n=== DEMO 4: Full-Duplex Communication ===\r\n");
    usart_tx_puts("Send and receive simultaneously\r\n");
    usart_tx_puts("Type to interact | 'q' to quit\r\n\r\n");

    uint16_t message_count = 0;

    while (1)
    {
        // Receive and echo
        if (usart_rx_available())
        {
            char c = usart_rx_get();

            if (c == 'q')
                break;

            // Echo with annotation
            usart_tx_puts("Echo: ");
            usart_tx_put(c);
            usart_tx_puts("\r\n");
        }

        // Periodically send status (TX while RX is also active)
        static uint32_t last_status = 0;
        static uint32_t tick_counter = 0;
        tick_counter++;

        if (tick_counter - last_status >= 50000)
        {
            char status[80];
            sprintf(status, "[Status #%u] RX buf: %u/%u | TX buf: %u/%u | Total RX: %lu | Total TX: %lu\r\n",
                    ++message_count, rx_count, RX_BUFFER_SIZE, tx_count, TX_BUFFER_SIZE,
                    rx_char_count, tx_char_count);
            usart_tx_puts(status);
            last_status = tick_counter;
        }

        _delay_us(10);
    }
}

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */

int main(void)
{
    // Initialize system
    init_devices();
    usart_init_interrupt();

    // Configure LED
    DDRB |= (1 << PB7);
    PORTB &= ~(1 << PB7);

    usart_tx_puts("\r\n");
    usart_tx_puts("========================================\r\n");
    usart_tx_puts("  USART Interrupt-Driven Communication \r\n");
    usart_tx_puts("========================================\r\n");

    while (1)
    {
        usart_tx_puts("\r\nSelect Demo:\r\n");
        usart_tx_puts("1. Interrupt Echo\r\n");
        usart_tx_puts("2. Background Processing\r\n");
        usart_tx_puts("3. TX Queuing\r\n");
        usart_tx_puts("4. Full-Duplex Communication\r\n");
        usart_tx_puts("\r\nChoice: ");

        // Wait for input
        while (!usart_rx_available())
            ;
        char choice = usart_rx_get();
        usart_tx_put(choice);
        usart_tx_puts("\r\n");

        switch (choice)
        {
        case '1':
            demo1_interrupt_echo();
            break;
        case '2':
            demo2_background_processing();
            break;
        case '3':
            demo3_tx_queuing();
            break;
        case '4':
            demo4_full_duplex();
            break;
        default:
            usart_tx_puts("Invalid choice!\r\n");
        }
    }

    return 0;
}
