/*
 * ===========================================================================
 * SERIAL PROTOCOL - Command-Response Protocol with Error Detection
 * ATmega128 @ 16MHz, UART0 @ 9600 baud
 * ===========================================================================
 *
 * FOCUS: Implementing reliable serial communication protocols
 *
 * WHY PROTOCOLS?
 * - Error detection: Detect corrupted data
 * - Framing: Know where messages start/end
 * - Reliability: Verify data integrity
 * - Structure: Organize complex communications
 *
 * PROTOCOL TECHNIQUES:
 * - Start/End markers: Frame boundaries
 * - Length fields: Know message size
 * - Checksums: Detect corruption
 * - Sequence numbers: Track messages
 * - ACK/NAK: Confirm reception
 *
 * LEARNING OBJECTIVES:
 * 1. Design simple binary protocols
 * 2. Implement checksum algorithms
 * 3. Handle protocol errors
 * 4. Parse structured messages
 * 5. Build command-response systems
 *
 * HARDWARE:
 * - ATmega128 @ 16MHz
 * - USB-Serial adapter connected to UART0 (PD0=RX, PD1=TX)
 * - LEDs on PORTB for status display
 */

#include "config.h"

// ============================================================================
// PROTOCOL DEFINITIONS
// ============================================================================

// Simple packet structure:
// [START] [CMD] [LEN] [DATA...] [CHECKSUM] [END]

#define PROTO_START 0x02 // STX - Start of Text
#define PROTO_END 0x03   // ETX - End of Text
#define PROTO_ACK 0x06   // ACK - Acknowledge
#define PROTO_NAK 0x15   // NAK - Negative Acknowledge

// Commands
#define CMD_LED_ON 0x10
#define CMD_LED_OFF 0x11
#define CMD_LED_SET 0x12
#define CMD_LED_GET 0x13
#define CMD_PING 0x20
#define CMD_VERSION 0x21

#define MAX_PACKET_SIZE 32

// Packet structure
typedef struct
{
    uint8_t cmd;
    uint8_t len;
    uint8_t data[MAX_PACKET_SIZE];
    uint8_t checksum;
} packet_t;

// ============================================================================
// UART WITH BUFFERS (from Serial_Buffered)
// ============================================================================

#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 64

volatile char rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;

volatile char tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_head = 0;
volatile uint8_t tx_tail = 0;

ISR(USART0_RX_vect)
{
    char c = UDR0;
    uint8_t next_head = (rx_head + 1) & (RX_BUFFER_SIZE - 1);
    if (next_head != rx_tail)
    {
        rx_buffer[rx_head] = c;
        rx_head = next_head;
    }
}

ISR(USART0_UDRE_vect)
{
    if (tx_head != tx_tail)
    {
        UDR0 = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) & (TX_BUFFER_SIZE - 1);
    }
    else
    {
        UCSR0B &= ~(1 << UDRIE0);
    }
}

void uart_init(void)
{
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

uint8_t uart_available(void)
{
    return (rx_head != rx_tail);
}

char uart_read(void)
{
    while (rx_head == rx_tail)
        ;
    char c = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) & (RX_BUFFER_SIZE - 1);
    return c;
}

void uart_write(char c)
{
    uint8_t next_head = (tx_head + 1) & (TX_BUFFER_SIZE - 1);
    while (next_head == tx_tail)
        ;
    tx_buffer[tx_head] = c;
    tx_head = next_head;
    UCSR0B |= (1 << UDRIE0);
}

// ============================================================================
// PROTOCOL FUNCTIONS
// ============================================================================

// Calculate simple checksum (XOR of all bytes)
uint8_t calc_checksum(uint8_t cmd, uint8_t len, uint8_t *data)
{
    uint8_t sum = cmd ^ len;
    for (uint8_t i = 0; i < len; i++)
        sum ^= data[i];
    return sum;
}

// Send a packet
void send_packet(uint8_t cmd, uint8_t len, uint8_t *data)
{
    uint8_t checksum = calc_checksum(cmd, len, data);

    uart_write(PROTO_START);
    uart_write(cmd);
    uart_write(len);
    for (uint8_t i = 0; i < len; i++)
        uart_write(data[i]);
    uart_write(checksum);
    uart_write(PROTO_END);
}

// Send simple ACK/NAK
void send_ack(void)
{
    uart_write(PROTO_ACK);
}

void send_nak(void)
{
    uart_write(PROTO_NAK);
}

// Receive a packet (blocking)
uint8_t receive_packet(packet_t *pkt)
{
    // Wait for START byte
    while (uart_read() != PROTO_START)
        ;

    // Read packet
    pkt->cmd = uart_read();
    pkt->len = uart_read();

    if (pkt->len > MAX_PACKET_SIZE)
        return 0; // Invalid length

    for (uint8_t i = 0; i < pkt->len; i++)
        pkt->data[i] = uart_read();

    pkt->checksum = uart_read();

    uint8_t end = uart_read();
    if (end != PROTO_END)
        return 0; // Missing end marker

    // Verify checksum
    uint8_t calc_sum = calc_checksum(pkt->cmd, pkt->len, pkt->data);
    if (calc_sum != pkt->checksum)
        return 0; // Checksum error

    return 1; // Success
}

// ============================================================================
// COMMAND HANDLERS
// ============================================================================

void handle_led_on(void)
{
    PORTB = 0xFF;
    send_ack();
}

void handle_led_off(void)
{
    PORTB = 0x00;
    send_ack();
}

void handle_led_set(uint8_t value)
{
    PORTB = value;
    send_ack();
}

void handle_led_get(void)
{
    uint8_t value = PORTB;
    send_packet(CMD_LED_GET, 1, &value);
}

void handle_ping(void)
{
    uint8_t response[] = {'P', 'O', 'N', 'G'};
    send_packet(CMD_PING, 4, response);
}

void handle_version(void)
{
    uint8_t version[] = {1, 0}; // Version 1.0
    send_packet(CMD_VERSION, 2, version);
}

// ============================================================================
// Demo 1: Simple Command Server
// ============================================================================
/*
 * Receive commands and execute them
 * Send ACK/NAK responses
 */

void demo_01_command_server(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei();

    // Send ready message
    uint8_t ready[] = {'R', 'E', 'A', 'D', 'Y'};
    send_packet(CMD_PING, 5, ready);

    packet_t pkt;

    while (1)
    {
        if (receive_packet(&pkt))
        {
            // Valid packet received
            switch (pkt.cmd)
            {
            case CMD_LED_ON:
                handle_led_on();
                break;

            case CMD_LED_OFF:
                handle_led_off();
                break;

            case CMD_LED_SET:
                if (pkt.len == 1)
                    handle_led_set(pkt.data[0]);
                else
                    send_nak();
                break;

            case CMD_LED_GET:
                handle_led_get();
                break;

            case CMD_PING:
                handle_ping();
                break;

            case CMD_VERSION:
                handle_version();
                break;

            default:
                send_nak(); // Unknown command
                break;
            }
        }
        else
        {
            // Invalid packet
            send_nak();
        }
    }
}

// ============================================================================
// Demo 2: ASCII Protocol (Human-Readable)
// ============================================================================
/*
 * Simple text-based protocol for debugging
 * Format: "CMD:PARAM\r\n"
 * Example: "LED:FF\r\n"
 */

#define ASCII_BUFFER_SIZE 16
char ascii_buffer[ASCII_BUFFER_SIZE];
uint8_t ascii_index = 0;

void process_ascii_command(void)
{
    ascii_buffer[ascii_index] = '\0';

    // Parse "LED:XX"
    if (ascii_buffer[0] == 'L' && ascii_buffer[1] == 'E' && ascii_buffer[2] == 'D' && ascii_buffer[3] == ':')
    {
        // Parse hex value
        uint8_t value = 0;
        char h1 = ascii_buffer[4];
        char h2 = ascii_buffer[5];

        if (h1 >= '0' && h1 <= '9')
            value = (h1 - '0') << 4;
        else if (h1 >= 'A' && h1 <= 'F')
            value = (h1 - 'A' + 10) << 4;

        if (h2 >= '0' && h2 <= '9')
            value |= (h2 - '0');
        else if (h2 >= 'A' && h2 <= 'F')
            value |= (h2 - 'A' + 10);

        PORTB = value;

        uart_write('O');
        uart_write('K');
        uart_write('\r');
        uart_write('\n');
    }
    else if (ascii_buffer[0] == 'O' && ascii_buffer[1] == 'N')
    {
        PORTB = 0xFF;
        uart_write('O');
        uart_write('K');
        uart_write('\r');
        uart_write('\n');
    }
    else if (ascii_buffer[0] == 'O' && ascii_buffer[1] == 'F' && ascii_buffer[2] == 'F')
    {
        PORTB = 0x00;
        uart_write('O');
        uart_write('K');
        uart_write('\r');
        uart_write('\n');
    }
    else
    {
        uart_write('E');
        uart_write('R');
        uart_write('R');
        uart_write('\r');
        uart_write('\n');
    }

    ascii_index = 0;
}

void demo_02_ascii_protocol(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei();

    uart_write('R');
    uart_write('E');
    uart_write('A');
    uart_write('D');
    uart_write('Y');
    uart_write('\r');
    uart_write('\n');

    while (1)
    {
        if (uart_available())
        {
            char c = uart_read();

            if (c == '\r' || c == '\n')
            {
                if (ascii_index > 0)
                    process_ascii_command();
            }
            else if (ascii_index < ASCII_BUFFER_SIZE - 1)
            {
                ascii_buffer[ascii_index++] = c;
            }
        }
    }
}

// ============================================================================
// Demo 3: Error Detection Test
// ============================================================================
/*
 * Test checksum error detection
 * Intentionally corrupt some packets
 */

void demo_03_error_detection(void)
{
    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei();

    uint8_t test_counter = 0;

    while (1)
    {
        _delay_ms(1000);

        // Send good packet
        send_packet(CMD_PING, 1, &test_counter);

        _delay_ms(100);

        // Send bad packet (wrong checksum)
        uart_write(PROTO_START);
        uart_write(CMD_PING);
        uart_write(1);
        uart_write(test_counter);
        uart_write(0xFF); // Wrong checksum!
        uart_write(PROTO_END);

        test_counter++;
        PORTB = test_counter;
    }
}

// ============================================================================
// MAIN - Select Demo
// ============================================================================

int main(void)
{
    // CHOOSE ONE DEMO TO RUN:

    // demo_01_command_server();    // Binary protocol
    demo_02_ascii_protocol(); // ASCII protocol
    // demo_03_error_detection();   // Test checksums

    return 0;
}
