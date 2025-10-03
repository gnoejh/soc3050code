/*
 * ===========================================================================
 * SERIAL BUFFERED - Full-Duplex Buffered UART Communication
 * ATmega128 @ 16MHz, UART0 @ 9600 baud
 * ===========================================================================
 *
 * FOCUS: Complete interrupt-driven UART with RX and TX ring buffers
 *
 * RING BUFFER ADVANTAGES:
 * - Full-duplex: Simultaneous send and receive
 * - Non-blocking: Never waits for serial operations
 * - Efficient: Automatic overflow handling
 * - Scalable: Easy to adjust buffer sizes
 *
 * RING BUFFER IMPLEMENTATION:
 * - Circular array with head (write) and tail (read) pointers
 * - Full when: (head + 1) % SIZE == tail
 * - Empty when: head == tail
 * - Size must be power of 2 for efficiency (use & instead of %)
 *
 * LEARNING OBJECTIVES:
 * 1. Implement circular buffer data structure
 * 2. Handle buffer full/empty conditions
 * 3. Coordinate RX and TX interrupts
 * 4. Build complete non-blocking UART library
 * 5. Understand thread-safe buffer access
 *
 * HARDWARE:
 * - ATmega128 @ 16MHz
 * - USB-Serial adapter connected to UART0 (PD0=RX, PD1=TX)
 * - LEDs on PORTB for status display
 */

#include "config.h"

// ============================================================================
// BUFFER CONFIGURATION
// ============================================================================

#define RX_BUFFER_SIZE 64  // Must be power of 2
#define TX_BUFFER_SIZE 128 // Larger TX buffer for burst transmissions

volatile char rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;

volatile char tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_head = 0;
volatile uint8_t tx_tail = 0;

// ============================================================================
// UART INTERRUPTS
// ============================================================================

// RX Complete Interrupt - store received byte
ISR(USART0_RX_vect)
{
    char c = UDR0;
    uint8_t next_head = (rx_head + 1) & (RX_BUFFER_SIZE - 1);

    if (next_head != rx_tail) // Buffer not full
    {
        rx_buffer[rx_head] = c;
        rx_head = next_head;
    }
    // else: buffer full, byte lost (could set error flag here)
}

// TX Data Register Empty Interrupt - send next byte
ISR(USART0_UDRE_vect)
{
    if (tx_head != tx_tail) // Buffer has data
    {
        UDR0 = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) & (TX_BUFFER_SIZE - 1);
    }
    else // Buffer empty
    {
        UCSR0B &= ~(1 << UDRIE0); // Disable UDRE interrupt
    }
}

// ============================================================================
// UART LIBRARY FUNCTIONS
// ============================================================================

void uart_init(void)
{
    // 9600 baud @ 16MHz
    UBRR0H = 0;
    UBRR0L = 103;

    // Enable RX, TX, and RX Complete Interrupt
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

    // 8-bit data, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Check if data available in RX buffer
uint8_t uart_available(void)
{
    return (rx_head != rx_tail);
}

// Read one byte from RX buffer (blocking if empty)
char uart_read(void)
{
    // Wait if buffer empty
    while (rx_head == rx_tail)
        ;

    char c = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) & (RX_BUFFER_SIZE - 1);
    return c;
}

// Read one byte (non-blocking, returns 0 if empty)
char uart_read_nowait(void)
{
    if (rx_head == rx_tail)
        return 0;

    char c = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) & (RX_BUFFER_SIZE - 1);
    return c;
}

// Check how many bytes in RX buffer
uint8_t uart_rx_count(void)
{
    return (rx_head - rx_tail) & (RX_BUFFER_SIZE - 1);
}

// Write one byte to TX buffer
void uart_write(char c)
{
    uint8_t next_head = (tx_head + 1) & (TX_BUFFER_SIZE - 1);

    // Wait if buffer full
    while (next_head == tx_tail)
        ;

    tx_buffer[tx_head] = c;
    tx_head = next_head;

    // Enable UDRE interrupt
    UCSR0B |= (1 << UDRIE0);
}

// Write string to TX buffer
void uart_print(const char *str)
{
    while (*str)
        uart_write(*str++);
}

// Check if TX buffer empty
uint8_t uart_tx_empty(void)
{
    return (tx_head == tx_tail);
}

// Check how many bytes in TX buffer
uint8_t uart_tx_count(void)
{
    return (tx_head - tx_tail) & (TX_BUFFER_SIZE - 1);
}

// Flush TX buffer (wait for all data sent)
void uart_flush(void)
{
    while (!uart_tx_empty())
        ;
}

// ============================================================================
// Demo 1: Echo Test - Full Duplex
// ============================================================================
/*
 * Echo received characters back
 * Demonstrates simultaneous RX and TX
 */

void demo_01_echo_test(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei();

    uart_print("=== Buffered UART Echo ===\r\n");
    uart_print("Type to test full-duplex operation\r\n\r\n");

    while (1)
    {
        if (uart_available())
        {
            char c = uart_read();
            uart_write(c); // Echo back

            PORTB = c; // Display on LEDs
        }

        // Main loop free for other tasks
    }
}

// ============================================================================
// Demo 2: Line Buffering
// ============================================================================
/*
 * Read complete lines (until \r or \n)
 * Process the line when complete
 */

#define LINE_BUFFER_SIZE 32
char line_buffer[LINE_BUFFER_SIZE];
uint8_t line_index = 0;

void process_line(void)
{
    line_buffer[line_index] = '\0'; // Null terminate

    uart_print("\r\nReceived: ");
    uart_print(line_buffer);
    uart_print("\r\n");

    // Process commands
    if (line_buffer[0] == 'O' && line_buffer[1] == 'N')
    {
        PORTB = 0xFF;
        uart_print("LEDs ON\r\n");
    }
    else if (line_buffer[0] == 'O' && line_buffer[1] == 'F' && line_buffer[2] == 'F')
    {
        PORTB = 0x00;
        uart_print("LEDs OFF\r\n");
    }

    uart_print("> ");
    line_index = 0;
}

void demo_02_line_buffering(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei();

    uart_print("=== Line Buffering Demo ===\r\n");
    uart_print("Enter commands (ON, OFF)\r\n");
    uart_print("> ");

    while (1)
    {
        if (uart_available())
        {
            char c = uart_read();
            uart_write(c); // Echo

            if (c == '\r' || c == '\n')
            {
                if (line_index > 0)
                    process_line();
            }
            else if (line_index < LINE_BUFFER_SIZE - 1)
            {
                line_buffer[line_index++] = c;
            }
        }
    }
}

// ============================================================================
// Demo 3: Buffer Statistics
// ============================================================================
/*
 * Monitor buffer usage
 * Demonstrates buffer management
 */

void print_number(uint8_t num)
{
    if (num >= 100)
        uart_write('0' + (num / 100));
    if (num >= 10)
        uart_write('0' + ((num / 10) % 10));
    uart_write('0' + (num % 10));
}

void demo_03_buffer_stats(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei();

    uart_print("=== Buffer Statistics ===\r\n");
    uart_print("Type to see buffer usage\r\n\r\n");

    while (1)
    {
        // Echo any received data
        while (uart_available())
        {
            char c = uart_read();
            uart_write(c);
        }

        // Display buffer stats every second
        uart_print("RX: ");
        print_number(uart_rx_count());
        uart_print("/");
        print_number(RX_BUFFER_SIZE);
        uart_print("  TX: ");
        print_number(uart_tx_count());
        uart_print("/");
        print_number(TX_BUFFER_SIZE);
        uart_print("\r\n");

        PORTB = uart_rx_count(); // Visualize RX buffer on LEDs

        _delay_ms(1000);
    }
}

// ============================================================================
// Demo 4: High-Speed Data Transfer
// ============================================================================
/*
 * Send large amounts of data
 * Tests buffer performance
 */

void demo_04_high_speed_transfer(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei();

    uart_print("=== High-Speed Transfer Test ===\r\n");
    uart_print("Sending 1000 bytes...\r\n\r\n");

    uint16_t bytes_sent = 0;

    while (1)
    {
        // Send burst of data
        for (uint16_t i = 0; i < 1000; i++)
        {
            uart_write('A' + (i % 26)); // Cycle through alphabet

            if (i % 50 == 49) // Newline every 50 chars
                uart_print("\r\n");
        }

        uart_print("\r\n--- 1000 bytes queued ---\r\n");
        PORTB = 0xFF;

        // Wait for transmission complete
        while (!uart_tx_empty())
        {
            PORTB ^= 0x01; // Blink LED while sending
            _delay_ms(10);
        }

        PORTB = 0x00;
        uart_print("--- Transmission complete ---\r\n\r\n");

        _delay_ms(2000);
    }
}

// ============================================================================
// MAIN - Select Demo
// ============================================================================

int main(void)
{
    // CHOOSE ONE DEMO TO RUN:

    // demo_01_echo_test();          // Full-duplex echo
    // demo_02_line_buffering();     // Line-based input
    // demo_03_buffer_stats();       // Monitor buffers
    demo_04_high_speed_transfer(); // Performance test

    return 0;
}
