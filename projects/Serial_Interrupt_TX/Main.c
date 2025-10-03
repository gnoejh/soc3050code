/*
 * ===========================================================================
 * SERIAL INTERRUPT TX - Interrupt-Based Transmit
 * ATmega128 @ 16MHz, UART0 @ 9600 baud
 * ===========================================================================
 *
 * FOCUS: Using interrupts for UART transmission with TX buffering
 *
 * WHY INTERRUPT-BASED TX?
 * - Non-blocking: Main loop doesn't wait for each byte to send
 * - Efficient: Hardware sends data in background
 * - Buffered: Can queue multiple bytes to send
 * - CPU free: Processor can do other work while transmitting
 *
 * TX INTERRUPT MECHANISM:
 * - UDRIE0 bit enables "Data Register Empty" interrupt
 * - ISR(USART0_UDRE_vect) called when TX buffer empty
 * - ISR writes next byte from buffer to UDR0
 * - When buffer empty, ISR disables itself
 *
 * LEARNING OBJECTIVES:
 * 1. Configure UART for TX interrupt mode
 * 2. Implement circular transmit buffer
 * 3. Write UDRE interrupt service routine
 * 4. Handle buffer full/empty conditions
 * 5. Understand non-blocking transmission
 *
 * HARDWARE:
 * - ATmega128 @ 16MHz
 * - USB-Serial adapter connected to UART0 (PD0=RX, PD1=TX)
 * - LEDs on PORTB for status display
 */

#include "config.h"

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void uart_init(void);
void uart_send_byte(uint8_t data);
void uart_send_string(const char *str);
uint8_t uart_tx_busy(void);

// ============================================================================
// TX BUFFER AND ISR
// ============================================================================

#define TX_BUFFER_SIZE 64 // Must be power of 2 for efficiency

volatile char tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_head = 0; // Write index
volatile uint8_t tx_tail = 0; // Read index

// UART Data Register Empty Interrupt
// Called when hardware is ready for next byte
ISR(USART0_UDRE_vect)
{
    if (tx_head != tx_tail)
    {
        // Buffer has data - send next byte
        UDR0 = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) & (TX_BUFFER_SIZE - 1); // Wrap around
    }
    else
    {
        // Buffer empty - disable interrupt
        UCSR0B &= ~(1 << UDRIE0);
    }
}

// ============================================================================
// UART FUNCTIONS
// ============================================================================

void uart_init(void)
{
    // 9600 baud @ 16MHz: UBRR = (16000000/16/9600) - 1 = 103
    UBRR0H = 0;
    UBRR0L = 103;

    // Enable TX (no RX for these demos)
    UCSR0B = (1 << TXEN0);

    // 8-bit data, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Send a single byte (non-blocking)
void uart_send_byte(uint8_t data)
{
    uint8_t next_head = (tx_head + 1) & (TX_BUFFER_SIZE - 1);

    // Wait if buffer full
    while (next_head == tx_tail)
        ;

    // Add byte to buffer
    tx_buffer[tx_head] = data;
    tx_head = next_head;

    // Enable UDRE interrupt
    UCSR0B |= (1 << UDRIE0);
}

// Send a string (non-blocking)
void uart_send_string(const char *str)
{
    while (*str)
    {
        uart_send_byte(*str++);
    }
}

// Check if transmission complete
uint8_t uart_tx_busy(void)
{
    return (tx_head != tx_tail);
}

// ============================================================================
// Demo 1: Basic TX Interrupt
// ============================================================================
/*
 * Send periodic messages using interrupt-based transmission
 * Notice: Main loop continues running while data transmits
 */

void demo_01_basic_tx_interrupt(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei();

    uart_send_string("TX Interrupt Demo Starting\r\n\r\n");

    uint8_t counter = 0;

    while (1)
    {
        // Send message
        uart_send_string("Message #");
        uart_send_byte('0' + (counter % 10));
        uart_send_string("\r\n");

        // Update LED
        PORTB = counter;
        counter++;

        // Main loop does other work while TX happens in background!
        // Notice: No blocking delay waiting for transmission
        for (uint8_t i = 0; i < 100; i++)
        {
            PORTB ^= 0x01; // Blink LED rapidly
            _delay_ms(5);
        }
    }
}

// ============================================================================
// Demo 2: Buffered Transmission
// ============================================================================
/*
 * Fill TX buffer with data, then let hardware send it all
 * Demonstrates buffering advantage
 */

void demo_02_buffered_transmission(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei();

    while (1)
    {
        // Prepare large message
        uart_send_string("=== Buffered TX Demo ===\r\n");
        uart_send_string("Line 1: This is a test\r\n");
        uart_send_string("Line 2: Multiple lines\r\n");
        uart_send_string("Line 3: All queued fast\r\n");
        uart_send_string("Line 4: Hardware sends\r\n");
        uart_send_string("Line 5: In background!\r\n");
        uart_send_string("\r\n");

        PORTB = 0xFF; // Turn on LEDs while queueing

        // Wait for transmission to complete
        while (uart_tx_busy())
        {
            // Main loop free to do other work
            PORTB ^= 0x01; // Blink one LED
            _delay_ms(10);
        }

        PORTB = 0x00; // All LEDs off when done
        _delay_ms(1000);
    }
}

// ============================================================================
// Demo 3: Print Formatted Numbers
// ============================================================================
/*
 * Send numeric values as ASCII text
 * Demonstrates building on basic TX functions
 */

void uart_print_dec(uint16_t value)
{
    char buffer[6]; // Max 65535 = 5 digits + null
    uint8_t i = 0;

    if (value == 0)
    {
        uart_send_byte('0');
        return;
    }

    // Convert to ASCII digits (reverse order)
    while (value > 0)
    {
        buffer[i++] = '0' + (value % 10);
        value /= 10;
    }

    // Send in correct order
    while (i > 0)
    {
        uart_send_byte(buffer[--i]);
    }
}

void uart_print_hex(uint8_t value)
{
    const char hex[] = "0123456789ABCDEF";
    uart_send_string("0x");
    uart_send_byte(hex[value >> 4]);
    uart_send_byte(hex[value & 0x0F]);
}

void demo_03_print_numbers(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei();

    uart_send_string("Number Formatting Demo\r\n\r\n");

    uint16_t counter = 0;

    while (1)
    {
        // Print decimal
        uart_send_string("Decimal: ");
        uart_print_dec(counter);
        uart_send_string("  ");

        // Print hex
        uart_send_string("Hex: ");
        uart_print_hex((uint8_t)counter);
        uart_send_string("\r\n");

        // Update display
        PORTB = (uint8_t)counter;
        counter++;

        _delay_ms(500);
    }
}

// ============================================================================
// Demo 4: Sensor Data Streaming
// ============================================================================
/*
 * Simulate reading sensor and streaming data via UART
 * Shows practical application of interrupt-based TX
 */

uint8_t read_adc_sensor(void)
{
    // Simulate ADC reading
    static uint8_t fake_value = 128;
    fake_value = (fake_value + 7) % 256; // Simple pattern
    return fake_value;
}

void demo_04_sensor_streaming(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei();

    uart_send_string("Sensor Data Streaming\r\n");
    uart_send_string("Format: [TIME] SENSOR=VALUE\r\n\r\n");

    uint16_t timestamp = 0;

    while (1)
    {
        // Read sensor (non-blocking!)
        uint8_t sensor_value = read_adc_sensor();
        PORTB = sensor_value;

        // Queue data for transmission
        uart_send_string("[");
        uart_print_dec(timestamp);
        uart_send_string("] SENSOR=");
        uart_print_dec(sensor_value);
        uart_send_string(" (");
        uart_print_hex(sensor_value);
        uart_send_string(")\r\n");

        timestamp++;

        // Main loop continues immediately!
        // No waiting for transmission to complete
        _delay_ms(100);
    }
}

// ============================================================================
// MAIN - Select Demo
// ============================================================================

int main(void)
{
    // CHOOSE ONE DEMO TO RUN:

    // demo_01_basic_tx_interrupt();    // Basic interrupt TX
    // demo_02_buffered_transmission(); // Buffer multiple lines
    // demo_03_print_numbers();         // Format numbers
    demo_04_sensor_streaming(); // Practical application

    return 0;
}
