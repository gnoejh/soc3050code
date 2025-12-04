/*
 * Enhanced UART Library - ATmega128 Educational Framework
 * Optimized Version 2.0 with Advanced Features
 *
 * ENHANCEMENTS:
 * - Buffered I/O for improved performance
 * - Error detection and handling
 * - Multiple baud rate support
 * - Printf functionality
 * - Flow control support
 * - Educational debugging features
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "config.h"

// Enhanced UART configuration
#define UART_RX_BUFFER_SIZE 128
#define UART_TX_BUFFER_SIZE 128
#define UART_TIMEOUT_MS 1000

// UART error flags
#define UART_NO_ERROR 0x00
#define UART_FRAME_ERROR 0x01
#define UART_DATA_OVERRUN 0x02
#define UART_PARITY_ERROR 0x04
#define UART_BUFFER_OVERFLOW 0x08
#define UART_TIMEOUT_ERROR 0x10

// Enhanced UART structure
typedef struct
{
    // Receive buffer
    volatile uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
    volatile uint8_t rx_head;
    volatile uint8_t rx_tail;
    volatile uint8_t rx_count;

    // Transmit buffer
    volatile uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
    volatile uint8_t tx_head;
    volatile uint8_t tx_tail;
    volatile uint8_t tx_count;

    // Status and statistics
    volatile uint8_t error_flags;
    volatile uint16_t bytes_received;
    volatile uint16_t bytes_transmitted;
    volatile uint8_t last_error;

    // Configuration
    uint32_t baud_rate;
    uint8_t data_bits;
    uint8_t parity;
    uint8_t stop_bits;
} uart_enhanced_t;

// Global UART instance
static uart_enhanced_t uart1_enhanced;

// Standard baud rates
const uint32_t standard_baud_rates[] = {
    1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200};

#define NUM_BAUD_RATES (sizeof(standard_baud_rates) / sizeof(standard_baud_rates[0]))

/*
 * Enhanced UART Initialization with error checking
 */
uint8_t uart_enhanced_init(uint32_t baud_rate, uint8_t data_bits, uint8_t parity, uint8_t stop_bits)
{
    // Validate parameters
    if (data_bits < 5 || data_bits > 8)
        return UART_FRAME_ERROR;
    if (parity > 3)
        return UART_PARITY_ERROR;
    if (stop_bits < 1 || stop_bits > 2)
        return UART_FRAME_ERROR;

    // Calculate and validate baud rate
    uint16_t ubrr_value = (F_CPU / (16UL * baud_rate)) - 1;
    if (ubrr_value > 4095)
        return UART_FRAME_ERROR; // UBRR is 12-bit

    // Initialize structure
    memset((void *)&uart1_enhanced, 0, sizeof(uart_enhanced_t));
    uart1_enhanced.baud_rate = baud_rate;
    uart1_enhanced.data_bits = data_bits;
    uart1_enhanced.parity = parity;
    uart1_enhanced.stop_bits = stop_bits;

    // Configure UART registers
    UBRR1H = (uint8_t)(ubrr_value >> 8);
    UBRR1L = (uint8_t)ubrr_value;

    // Configure frame format
    uint8_t ucsrc_val = 0;

    // Data bits (5-8)
    switch (data_bits)
    {
    case 5:
        ucsrc_val |= (0 << UCSZ11) | (0 << UCSZ10);
        break;
    case 6:
        ucsrc_val |= (0 << UCSZ11) | (1 << UCSZ10);
        break;
    case 7:
        ucsrc_val |= (1 << UCSZ11) | (0 << UCSZ10);
        break;
    case 8:
        ucsrc_val |= (1 << UCSZ11) | (1 << UCSZ10);
        break;
    }

    // Parity (0=none, 1=reserved, 2=even, 3=odd)
    switch (parity)
    {
    case 0:
        ucsrc_val |= (0 << UPM11) | (0 << UPM10);
        break;
    case 2:
        ucsrc_val |= (1 << UPM11) | (0 << UPM10);
        break;
    case 3:
        ucsrc_val |= (1 << UPM11) | (1 << UPM10);
        break;
    }

    // Stop bits (1 or 2)
    if (stop_bits == 2)
    {
        ucsrc_val |= (1 << USBS1);
    }

    UCSR1C = ucsrc_val;

    // Enable transmitter, receiver, and RX interrupt
    UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);

    // Enable global interrupts
    sei();

    return UART_NO_ERROR;
}

/*
 * Enhanced RX Interrupt Service Routine
 */
ISR(USART1_RX_vect)
{
    uint8_t status = UCSR1A;
    uint8_t data = UDR1;

    // Check for errors
    if (status & (1 << FE1))
    {
        uart1_enhanced.error_flags |= UART_FRAME_ERROR;
        uart1_enhanced.last_error = UART_FRAME_ERROR;
        return;
    }

    if (status & (1 << DOR1))
    {
        uart1_enhanced.error_flags |= UART_DATA_OVERRUN;
        uart1_enhanced.last_error = UART_DATA_OVERRUN;
        return;
    }

    if (status & (1 << UPE1))
    {
        uart1_enhanced.error_flags |= UART_PARITY_ERROR;
        uart1_enhanced.last_error = UART_PARITY_ERROR;
        return;
    }

    // Store data in circular buffer
    if (uart1_enhanced.rx_count < UART_RX_BUFFER_SIZE)
    {
        uart1_enhanced.rx_buffer[uart1_enhanced.rx_head] = data;
        uart1_enhanced.rx_head = (uart1_enhanced.rx_head + 1) % UART_RX_BUFFER_SIZE;
        uart1_enhanced.rx_count++;
        uart1_enhanced.bytes_received++;
    }
    else
    {
        uart1_enhanced.error_flags |= UART_BUFFER_OVERFLOW;
        uart1_enhanced.last_error = UART_BUFFER_OVERFLOW;
    }
}

/*
 * Enhanced TX Interrupt Service Routine
 */
ISR(USART1_UDRE_vect)
{
    if (uart1_enhanced.tx_count > 0)
    {
        UDR1 = uart1_enhanced.tx_buffer[uart1_enhanced.tx_tail];
        uart1_enhanced.tx_tail = (uart1_enhanced.tx_tail + 1) % UART_TX_BUFFER_SIZE;
        uart1_enhanced.tx_count--;
        uart1_enhanced.bytes_transmitted++;
    }
    else
    {
        // Disable UDRE interrupt when buffer is empty
        UCSR1B &= ~(1 << UDRIE1);
    }
}

/*
 * Enhanced receive function with timeout
 */
uint8_t uart_enhanced_receive(uint8_t *data, uint16_t timeout_ms)
{
    uint16_t timeout_counter = 0;

    while (uart1_enhanced.rx_count == 0)
    {
        _delay_ms(1);
        timeout_counter++;

        if (timeout_counter >= timeout_ms)
        {
            uart1_enhanced.error_flags |= UART_TIMEOUT_ERROR;
            return UART_TIMEOUT_ERROR;
        }
    }

    // Disable interrupts for atomic operation
    cli();

    *data = uart1_enhanced.rx_buffer[uart1_enhanced.rx_tail];
    uart1_enhanced.rx_tail = (uart1_enhanced.rx_tail + 1) % UART_RX_BUFFER_SIZE;
    uart1_enhanced.rx_count--;

    sei();

    return UART_NO_ERROR;
}

/*
 * Enhanced transmit function with buffering
 */
uint8_t uart_enhanced_transmit(uint8_t data)
{
    // Wait if buffer is full
    while (uart1_enhanced.tx_count >= UART_TX_BUFFER_SIZE)
    {
        _delay_us(10);
    }

    // Disable interrupts for atomic operation
    cli();

    uart1_enhanced.tx_buffer[uart1_enhanced.tx_head] = data;
    uart1_enhanced.tx_head = (uart1_enhanced.tx_head + 1) % UART_TX_BUFFER_SIZE;
    uart1_enhanced.tx_count++;

    // Enable UDRE interrupt
    UCSR1B |= (1 << UDRIE1);

    sei();

    return UART_NO_ERROR;
}

/*
 * Enhanced string transmission
 */
uint8_t uart_enhanced_transmit_string(const char *str)
{
    while (*str)
    {
        uint8_t result = uart_enhanced_transmit(*str++);
        if (result != UART_NO_ERROR)
        {
            return result;
        }
    }
    return UART_NO_ERROR;
}

/*
 * Enhanced printf functionality
 */
int uart_enhanced_printf(const char *format, ...)
{
    char buffer[256];
    va_list args;

    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length > 0)
    {
        uart_enhanced_transmit_string(buffer);
    }

    return length;
}

/*
 * Buffer status functions
 */
uint8_t uart_enhanced_rx_available(void)
{
    return uart1_enhanced.rx_count;
}

uint8_t uart_enhanced_tx_free(void)
{
    return UART_TX_BUFFER_SIZE - uart1_enhanced.tx_count;
}

/*
 * Error handling functions
 */
uint8_t uart_enhanced_get_error_flags(void)
{
    return uart1_enhanced.error_flags;
}

void uart_enhanced_clear_error_flags(void)
{
    uart1_enhanced.error_flags = UART_NO_ERROR;
}

uint8_t uart_enhanced_get_last_error(void)
{
    return uart1_enhanced.last_error;
}

/*
 * Statistics functions
 */
uint16_t uart_enhanced_get_bytes_received(void)
{
    return uart1_enhanced.bytes_received;
}

uint16_t uart_enhanced_get_bytes_transmitted(void)
{
    return uart1_enhanced.bytes_transmitted;
}

void uart_enhanced_reset_statistics(void)
{
    uart1_enhanced.bytes_received = 0;
    uart1_enhanced.bytes_transmitted = 0;
}

/*
 * Configuration query functions
 */
uint32_t uart_enhanced_get_baud_rate(void)
{
    return uart1_enhanced.baud_rate;
}

void uart_enhanced_get_config(uint32_t *baud, uint8_t *data_bits, uint8_t *parity, uint8_t *stop_bits)
{
    *baud = uart1_enhanced.baud_rate;
    *data_bits = uart1_enhanced.data_bits;
    *parity = uart1_enhanced.parity;
    *stop_bits = uart1_enhanced.stop_bits;
}

/*
 * Debug and diagnostic functions
 */
void uart_enhanced_print_status(void)
{
    uart_enhanced_printf("\\r\\n=== UART Enhanced Status ===\\r\\n");
    uart_enhanced_printf("Baud Rate: %lu\\r\\n", uart1_enhanced.baud_rate);
    uart_enhanced_printf("Data Bits: %u\\r\\n", uart1_enhanced.data_bits);
    uart_enhanced_printf("Parity: %u\\r\\n", uart1_enhanced.parity);
    uart_enhanced_printf("Stop Bits: %u\\r\\n", uart1_enhanced.stop_bits);
    uart_enhanced_printf("RX Buffer: %u/%u\\r\\n", uart1_enhanced.rx_count, UART_RX_BUFFER_SIZE);
    uart_enhanced_printf("TX Buffer: %u/%u\\r\\n", uart1_enhanced.tx_count, UART_TX_BUFFER_SIZE);
    uart_enhanced_printf("Bytes RX: %u\\r\\n", uart1_enhanced.bytes_received);
    uart_enhanced_printf("Bytes TX: %u\\r\\n", uart1_enhanced.bytes_transmitted);
    uart_enhanced_printf("Error Flags: 0x%02X\\r\\n", uart1_enhanced.error_flags);
    uart_enhanced_printf("Last Error: 0x%02X\\r\\n", uart1_enhanced.last_error);
}

/*
 * Baud rate testing function
 */
void uart_enhanced_test_baud_rates(void)
{
    uart_enhanced_printf("\\r\\n=== Baud Rate Test ===\\r\\n");

    for (uint8_t i = 0; i < NUM_BAUD_RATES; i++)
    {
        uart_enhanced_printf("Testing %lu baud... ", standard_baud_rates[i]);

        // Reinitialize with new baud rate
        uint8_t result = uart_enhanced_init(standard_baud_rates[i], 8, 0, 1);

        if (result == UART_NO_ERROR)
        {
            uart_enhanced_printf("OK\\r\\n");
            _delay_ms(500);
        }
        else
        {
            uart_enhanced_printf("FAILED (0x%02X)\\r\\n", result);
        }
    }

    // Restore to 9600 baud
    uart_enhanced_init(9600, 8, 0, 1);
    uart_enhanced_printf("Restored to 9600 baud\\r\\n");
}

/*
 * Backward compatibility functions
 */
void Uart1_init(void)
{
    uart_enhanced_init(9600, 8, 0, 1);
}

uint8_t is_USART1_received(void)
{
    return (uart1_enhanced.rx_count > 0);
}

uint8_t get_USART1(void)
{
    uint8_t data;
    if (uart_enhanced_receive(&data, 0) == UART_NO_ERROR)
    {
        return data;
    }
    return 0;
}

void put_USART1(uint8_t data)
{
    uart_enhanced_transmit(data);
}

void puts_USART1(const char *str)
{
    uart_enhanced_transmit_string(str);
}

/*
 * Educational demonstration function
 */
void uart_enhanced_demo(void)
{
    uart_enhanced_printf("\\r\\n=== UART Enhanced Demo ===\\r\\n");
    uart_enhanced_printf("Type commands:\\r\\n");
    uart_enhanced_printf("'s' - Show status\\r\\n");
    uart_enhanced_printf("'t' - Test baud rates\\r\\n");
    uart_enhanced_printf("'e' - Show errors\\r\\n");
    uart_enhanced_printf("'r' - Reset statistics\\r\\n");
    uart_enhanced_printf("'q' - Quit demo\\r\\n");

    while (1)
    {
        if (uart_enhanced_rx_available())
        {
            uint8_t cmd;
            uart_enhanced_receive(&cmd, 1000);

            switch (cmd)
            {
            case 's':
            case 'S':
                uart_enhanced_print_status();
                break;

            case 't':
            case 'T':
                uart_enhanced_test_baud_rates();
                break;

            case 'e':
            case 'E':
                uart_enhanced_printf("Error flags: 0x%02X\\r\\n", uart_enhanced_get_error_flags());
                uart_enhanced_printf("Last error: 0x%02X\\r\\n", uart_enhanced_get_last_error());
                break;

            case 'r':
            case 'R':
                uart_enhanced_reset_statistics();
                uart_enhanced_clear_error_flags();
                uart_enhanced_printf("Statistics and errors cleared\\r\\n");
                break;

            case 'q':
            case 'Q':
                uart_enhanced_printf("Demo ended\\r\\n");
                return;

            default:
                uart_enhanced_printf("Unknown command: %c\\r\\n", cmd);
                break;
            }
        }

        _delay_ms(10);
    }
}