/*
 * ==============================================================================
 * ESP-01 / BLUETOOTH MODULE - DUAL UART COMMUNICATION
 * ==============================================================================
 * PROJECT: Bluethooth (ESP-01/Bluetooth AT Command Interface)
 * MCU: ATmega128
 * F_CPU: 16 MHz
 *
 * HARDWARE CONNECTIONS:
 * - UART0 (PE0/PE1): ESP-01 or Bluetooth module (115200 baud)
 *   PE0 (RXD0) ← ESP TX
 *   PE1 (TXD0) → ESP RX (use voltage divider: 5V → 3.3V)
 * - UART1 (PD2/PD3): Debug/Monitor COM port (9600 baud)
 *   PD2 (RXD1) ← PC TX
 *   PD3 (TXD1) → PC RX
 *
 * LEARNING OBJECTIVES:
 * 1. Dual UART configuration and management
 * 2. AT command protocol and state machine
 * 3. Non-blocking UART with ring buffers
 * 4. Command parsing and response handling
 * 5. Timeout and retry mechanisms
 * 6. Cross-UART bridging and monitoring
 *
 * FEATURES:
 * - AT command sequencer with timeouts
 * - Transparent bridge mode (UART1 ↔ UART0)
 * - Auto-initialization sequence
 * - Debug output to UART1
 * - Response parser with pattern matching
 * - WiFi/Bluetooth connection management
 *
 * ==============================================================================
 */

#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * CONFIGURATION
 * ============================================================================ */

#define ESP_BAUD 115200UL // ESP-01/Bluetooth module baud rate
#define DEBUG_BAUD 9600UL // Debug monitor baud rate

#define ESP_RX_BUFFER_SIZE 128
#define ESP_TX_BUFFER_SIZE 64
#define DEBUG_TX_BUFFER_SIZE 128

#define CMD_TIMEOUT_MS 2000 // Command response timeout
#define TICK_MS 10          // Timer tick interval

/* ============================================================================
 * UART0 BUFFERS (ESP-01 / Bluetooth)
 * ============================================================================ */

volatile char esp_rx_buffer[ESP_RX_BUFFER_SIZE];
volatile uint8_t esp_rx_head = 0;
volatile uint8_t esp_rx_tail = 0;

volatile char esp_tx_buffer[ESP_TX_BUFFER_SIZE];
volatile uint8_t esp_tx_head = 0;
volatile uint8_t esp_tx_tail = 0;
volatile uint8_t esp_tx_count = 0;

/* ============================================================================
 * UART1 BUFFERS (Debug/Monitor)
 * ============================================================================ */

volatile char debug_tx_buffer[DEBUG_TX_BUFFER_SIZE];
volatile uint8_t debug_tx_head = 0;
volatile uint8_t debug_tx_tail = 0;
volatile uint8_t debug_tx_count = 0;

/* ============================================================================
 * STATE MACHINE
 * ============================================================================ */

typedef enum
{
    STATE_INIT,
    STATE_WAIT_READY,
    STATE_TEST_AT,
    STATE_CHECK_VERSION,
    STATE_SET_MODE,
    STATE_CONNECT_AP,
    STATE_CONNECTED,
    STATE_BRIDGE_MODE,
    STATE_ERROR
} esp_state_t;

volatile esp_state_t esp_state = STATE_INIT;
volatile uint16_t timeout_counter = 0;
volatile uint16_t systick_ms = 0;

/* ============================================================================
 * RESPONSE PARSER
 * ============================================================================ */

char response_line[ESP_RX_BUFFER_SIZE];
uint8_t response_index = 0;
uint8_t response_ready = 0;

/* ============================================================================
 * TIMER0 - SYSTEM TICK (1ms)
 * ============================================================================ */

void timer0_init(void)
{
    // CTC mode, prescaler 64
    // OCR0 = F_CPU / (prescaler * desired_freq) - 1
    // For 1kHz (1ms): OCR0 = 16000000 / (64 * 1000) - 1 = 249
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00); // CTC, prescaler 64
    OCR0 = 249;
    TIMSK |= (1 << OCIE0); // Enable compare match interrupt
}

ISR(TIMER0_COMP_vect)
{
    systick_ms++;
    if (timeout_counter > 0)
    {
        timeout_counter--;
    }
}

/* ============================================================================
 * UART0 INITIALIZATION (ESP-01 / Bluetooth)
 * ============================================================================ */

void uart0_init(void)
{
    uint16_t ubrr = (F_CPU / (16UL * ESP_BAUD)) - 1;

    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;

    // Enable RX, TX, and RX Complete interrupt
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

    // 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

/* ============================================================================
 * UART1 INITIALIZATION (Debug/Monitor)
 * ============================================================================ */

void uart1_init(void)
{
    uint16_t ubrr = (F_CPU / (16UL * DEBUG_BAUD)) - 1;

    UBRR1H = (uint8_t)(ubrr >> 8);
    UBRR1L = (uint8_t)ubrr;

    // Enable RX, TX
    UCSR1B = (1 << RXEN1) | (1 << TXEN1);

    // 8 data bits, 1 stop bit, no parity
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}

/* ============================================================================
 * UART0 INTERRUPT SERVICE ROUTINES (ESP)
 * ============================================================================ */

ISR(USART0_RX_vect)
{
    char received = UDR0;

    // Store in circular buffer
    uint8_t next_head = (esp_rx_head + 1) % ESP_RX_BUFFER_SIZE;
    if (next_head != esp_rx_tail)
    {
        esp_rx_buffer[esp_rx_head] = received;
        esp_rx_head = next_head;
    }
}

ISR(USART0_UDRE_vect)
{
    if (esp_tx_count > 0)
    {
        UDR0 = esp_tx_buffer[esp_tx_tail];
        esp_tx_tail = (esp_tx_tail + 1) % ESP_TX_BUFFER_SIZE;
        esp_tx_count--;
    }
    else
    {
        // Buffer empty, disable UDRE interrupt
        UCSR0B &= ~(1 << UDRIE0);
    }
}

/* ============================================================================
 * UART1 INTERRUPT SERVICE ROUTINE (Debug TX only)
 * ============================================================================ */

ISR(USART1_UDRE_vect)
{
    if (debug_tx_count > 0)
    {
        UDR1 = debug_tx_buffer[debug_tx_tail];
        debug_tx_tail = (debug_tx_tail + 1) % DEBUG_TX_BUFFER_SIZE;
        debug_tx_count--;
    }
    else
    {
        UCSR1B &= ~(1 << UDRIE1);
    }
}

/* ============================================================================
 * UART0 FUNCTIONS (ESP)
 * ============================================================================ */

uint8_t uart0_available(void)
{
    return (esp_rx_head != esp_rx_tail);
}

char uart0_getc(void)
{
    while (!uart0_available())
        ;

    char c = esp_rx_buffer[esp_rx_tail];
    esp_rx_tail = (esp_rx_tail + 1) % ESP_RX_BUFFER_SIZE;
    return c;
}

void uart0_putc(char c)
{
    // Wait if buffer full
    while (esp_tx_count >= ESP_TX_BUFFER_SIZE)
        ;

    cli();
    esp_tx_buffer[esp_tx_head] = c;
    esp_tx_head = (esp_tx_head + 1) % ESP_TX_BUFFER_SIZE;
    esp_tx_count++;
    sei();

    // Enable UDRE interrupt
    UCSR0B |= (1 << UDRIE0);
}

void uart0_puts(const char *str)
{
    while (*str)
    {
        uart0_putc(*str++);
    }
}

/* ============================================================================
 * UART1 FUNCTIONS (Debug)
 * ============================================================================ */

void uart1_putc(char c)
{
    // Wait if buffer full
    while (debug_tx_count >= DEBUG_TX_BUFFER_SIZE)
        ;

    cli();
    debug_tx_buffer[debug_tx_head] = c;
    debug_tx_head = (debug_tx_head + 1) % DEBUG_TX_BUFFER_SIZE;
    debug_tx_count++;
    sei();

    // Enable UDRE interrupt
    UCSR1B |= (1 << UDRIE1);
}

void uart1_puts(const char *str)
{
    while (*str)
    {
        uart1_putc(*str++);
    }
}

void uart1_puts_P(const char *str)
{
    char c;
    while ((c = pgm_read_byte(str++)))
    {
        uart1_putc(c);
    }
}

uint8_t uart1_available(void)
{
    return (UCSR1A & (1 << RXC1));
}

char uart1_getc(void)
{
    while (!uart1_available())
        ;
    return UDR1;
}

/* ============================================================================
 * ESP COMMAND FUNCTIONS
 * ============================================================================ */

void esp_send_cmd(const char *cmd)
{
    uart1_puts("[CMD] ");
    uart1_puts(cmd);
    uart1_puts("\r\n");

    uart0_puts(cmd);
    uart0_puts("\r\n");

    timeout_counter = CMD_TIMEOUT_MS;
    response_index = 0;
    response_ready = 0;
}

uint8_t esp_read_line(void)
{
    while (uart0_available())
    {
        char c = uart0_getc();

        // Echo to debug UART
        uart1_putc(c);

        if (c == '\n')
        {
            if (response_index > 0 && response_line[response_index - 1] == '\r')
            {
                response_index--;
            }
            response_line[response_index] = '\0';
            response_ready = 1;
            response_index = 0;
            return 1;
        }
        else if (response_index < ESP_RX_BUFFER_SIZE - 1)
        {
            response_line[response_index++] = c;
        }
    }
    return 0;
}

uint8_t esp_wait_for_response(const char *expected)
{
    while (timeout_counter > 0)
    {
        if (esp_read_line() && response_ready)
        {
            response_ready = 0;
            if (strstr(response_line, expected) != NULL)
            {
                return 1;
            }
        }
    }
    return 0;
}

/* ============================================================================
 * STATE MACHINE IMPLEMENTATION
 * ============================================================================ */

void esp_state_machine(void)
{
    switch (esp_state)
    {
    case STATE_INIT:
        uart1_puts_P(PSTR("\r\n=== ESP-01/Bluetooth Init ===\r\n"));
        _delay_ms(100);
        esp_state = STATE_TEST_AT;
        break;

    case STATE_WAIT_READY:
        uart1_puts_P(PSTR("[STATE] Waiting for module ready...\r\n"));
        _delay_ms(1000);
        esp_state = STATE_TEST_AT;
        break;

    case STATE_TEST_AT:
        uart1_puts_P(PSTR("[STATE] Testing AT...\r\n"));
        esp_send_cmd("AT");
        if (esp_wait_for_response("OK"))
        {
            uart1_puts_P(PSTR("[OK] AT responded\r\n"));
            esp_state = STATE_CHECK_VERSION;
        }
        else
        {
            uart1_puts_P(PSTR("[TIMEOUT] No ESP-01 detected\r\n"));
            uart1_puts_P(PSTR("[INFO] Entering bridge mode for manual testing\r\n"));
            _delay_ms(500);
            esp_state = STATE_BRIDGE_MODE;
        }
        break;

    case STATE_CHECK_VERSION:
        uart1_puts_P(PSTR("[STATE] Checking version...\r\n"));
        esp_send_cmd("AT+GMR");
        if (esp_wait_for_response("OK"))
        {
            uart1_puts_P(PSTR("[OK] Version check complete\r\n"));
            esp_state = STATE_SET_MODE;
        }
        else
        {
            uart1_puts_P(PSTR("[TIMEOUT] Skipping...\r\n"));
            esp_state = STATE_SET_MODE;
        }
        break;

    case STATE_SET_MODE:
        uart1_puts_P(PSTR("[STATE] Setting mode...\r\n"));
        esp_send_cmd("AT+CWMODE=1");
        if (esp_wait_for_response("OK"))
        {
            uart1_puts_P(PSTR("[OK] Mode set to Station\r\n"));
            esp_state = STATE_CONNECTED;
        }
        else
        {
            uart1_puts_P(PSTR("[TIMEOUT] Entering bridge mode anyway\r\n"));
            esp_state = STATE_BRIDGE_MODE;
        }
        break;

    case STATE_CONNECT_AP:
        // Optional: Add WiFi connection logic here
        // esp_send_cmd("AT+CWJAP=\"SSID\",\"PASSWORD\"");
        uart1_puts_P(PSTR("[SKIP] AP connection (manual)\r\n"));
        esp_state = STATE_CONNECTED;
        break;

    case STATE_CONNECTED:
        uart1_puts_P(PSTR("\r\n[READY] Module initialized!\r\n"));
        uart1_puts_P(PSTR("Commands available:\r\n"));
        uart1_puts_P(PSTR("  'b' - Enter bridge mode\r\n"));
        uart1_puts_P(PSTR("  't' - Test AT\r\n"));
        uart1_puts_P(PSTR("  'r' - Reset module\r\n"));
        uart1_puts_P(PSTR("  'i' - Info\r\n"));

        // Wait for user command
        while (1)
        {
            if (uart1_available())
            {
                char cmd = uart1_getc();
                if (cmd == 'b' || cmd == 'B')
                {
                    esp_state = STATE_BRIDGE_MODE;
                    break;
                }
                else if (cmd == 't' || cmd == 'T')
                {
                    esp_send_cmd("AT");
                    esp_wait_for_response("OK");
                }
                else if (cmd == 'r' || cmd == 'R')
                {
                    esp_send_cmd("AT+RST");
                    uart1_puts_P(PSTR("[RESET] Waiting for ready...\r\n"));
                    _delay_ms(2000);
                    esp_state = STATE_TEST_AT;
                    break;
                }
                else if (cmd == 'i' || cmd == 'I')
                {
                    esp_send_cmd("AT+CIFSR");
                    esp_wait_for_response("OK");
                }
            }

            // Also check ESP responses
            if (uart0_available())
            {
                uart1_putc(uart0_getc());
            }
        }
        break;

    case STATE_BRIDGE_MODE:
        uart1_puts_P(PSTR("\r\n=== BRIDGE MODE ACTIVE ===\r\n"));
        uart1_puts_P(PSTR("UART0 <-> UART1 transparent bridge\r\n"));
        uart1_puts_P(PSTR("Press Ctrl+C three times to exit\r\n\r\n"));

        uint8_t ctrl_c_count = 0;

        while (1)
        {
            // UART1 → UART0 (PC to ESP)
            if (uart1_available())
            {
                char c = uart1_getc();
                if (c == 0x03)
                { // Ctrl+C
                    ctrl_c_count++;
                    if (ctrl_c_count >= 3)
                    {
                        uart1_puts_P(PSTR("\r\n[EXIT] Bridge mode\r\n"));
                        esp_state = STATE_CONNECTED;
                        break;
                    }
                }
                else
                {
                    ctrl_c_count = 0;
                    uart0_putc(c);
                }
            }

            // UART0 → UART1 (ESP to PC)
            if (uart0_available())
            {
                uart1_putc(uart0_getc());
            }
        }
        break;

    case STATE_ERROR:
        uart1_puts_P(PSTR("[ERROR] Fatal error, restarting...\r\n"));
        _delay_ms(2000);
        esp_state = STATE_INIT;
        break;
    }
}

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */

int main(void)
{
    // Initialize I/O ports
    DDRA = 0x00;
    PORTA = 0xFF;
    DDRB = 0xFF;
    PORTB = 0x00;

    // Initialize UARTs
    uart0_init(); // ESP-01 on UART0 (PE0/PE1)
    uart1_init(); // Debug on UART1 (PD2/PD3)

    // Initialize timer
    timer0_init();

    // Enable global interrupts
    sei();

    // Startup message
    _delay_ms(100);
    uart1_puts_P(PSTR("\r\n\r\n"));
    uart1_puts_P(PSTR("========================================\r\n"));
    uart1_puts_P(PSTR("  ESP-01/Bluetooth AT Interface\r\n"));
    uart1_puts_P(PSTR("  ATmega128 Dual UART System\r\n"));
    uart1_puts_P(PSTR("========================================\r\n"));
    uart1_puts_P(PSTR("UART0 (PE0/PE1): ESP-01 @ 115200 baud\r\n"));
    uart1_puts_P(PSTR("UART1 (PD2/PD3): Debug  @ 9600 baud\r\n"));
    uart1_puts_P(PSTR("========================================\r\n\r\n"));

    // Run state machine once to initialize
    esp_state_machine();

    // Main loop - simple bridge mode
    uart1_puts_P(PSTR("\r\n[BRIDGE] Direct UART bridge active\r\n"));
    uart1_puts_P(PSTR("Type commands, they forward to UART0\r\n\r\n"));

    while (1)
    {
        // UART1 → UART0 (PC to ESP)
        if (uart1_available())
        {
            char c = uart1_getc();
            uart0_putc(c);
            uart1_putc(c); // Echo
        }

        // UART0 → UART1 (ESP to PC)
        if (uart0_available())
        {
            char c = uart0_getc();
            uart1_putc(c);
        }
    }

    return 0;
}
