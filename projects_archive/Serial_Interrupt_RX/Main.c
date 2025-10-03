/*
 * ===========================================================================
 * SERIAL INTERRUPT RX - Interrupt-Based Receive
 * ATmega128 @ 16MHz, UART0 @ 9600 baud
 * ===========================================================================
 *
 * FOCUS: Using interrupts for UART reception instead of polling
 *
 * WHY INTERRUPTS FOR SERIAL?
 * - Non-blocking: Main loop can do other work
 * - No data loss: ISR captures data immediately
 * - Efficient: CPU only works when data arrives
 * - Real-time: Instant response to incoming data
 *
 * INTERRUPT-BASED UART:
 * - RXCIE0 bit enables RX Complete Interrupt
 * - ISR(USART0_RX_vect) automatically called when byte received
 * - Read UDR0 in ISR to get the received byte
 * - Store in buffer or process immediately
 *
 * LEARNING OBJECTIVES:
 * 1. Configure UART for interrupt mode
 * 2. Write UART RX interrupt service routine
 * 3. Handle received data safely
 * 4. Implement simple command processing
 * 5. Understand buffering concepts
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
void uart_transmit(uint8_t data);
void uart_print(const char *str);

// ============================================================================
// SHARED VARIABLES AND ISR
// ============================================================================
// All demos share this ISR but use different modes
// Only ONE demo should run at a time!

volatile uint8_t rx_char = 0;
volatile uint8_t rx_flag = 0;
volatile uint8_t demo_mode = 1;

// Command parser variables (demo 3)
#define CMD_BUFFER_SIZE 16
volatile char cmd_buffer[CMD_BUFFER_SIZE];
volatile uint8_t cmd_index = 0;
volatile uint8_t cmd_ready = 0;

// Numeric input variables (demo 4)
volatile uint16_t numeric_value = 0;
volatile uint8_t digit_count = 0;
volatile uint8_t value_ready = 0;

// UART RX Complete Interrupt - handles all demo modes
ISR(USART0_RX_vect)
{
    char c = UDR0; // Read received byte

    if (demo_mode == 1 || demo_mode == 2)
    {
        // Demo 1 & 2: Simple character receive
        rx_char = c;
        rx_flag = 1;
    }
    else if (demo_mode == 3)
    {
        // Demo 3: Command parser
        if (c == '\r' || c == '\n')
        {
            // Command complete
            cmd_buffer[cmd_index] = '\0';
            cmd_ready = 1;
            cmd_index = 0;
        }
        else if (cmd_index < CMD_BUFFER_SIZE - 1)
        {
            // Add to buffer and echo
            cmd_buffer[cmd_index++] = c;
            uart_transmit(c);
        }
    }
    else if (demo_mode == 4)
    {
        // Demo 4: Numeric input
        uart_transmit(c); // Echo

        if (c >= '0' && c <= '9')
        {
            // Add digit
            numeric_value = numeric_value * 10 + (c - '0');
            digit_count++;
        }
        else if (c == '\r' || c == '\n')
        {
            // Value complete
            if (digit_count > 0)
            {
                value_ready = 1;
                digit_count = 0;
            }
        }
        else if (c == 'c' || c == 'C')
        {
            // Clear
            numeric_value = 0;
            digit_count = 0;
            uart_print("\r\nCleared\r\n");
        }
    }
}

// ============================================================================
// UART BASIC FUNCTIONS
// ============================================================================

void uart_init(void)
{
    // 9600 baud @ 16MHz: UBRR = (16000000/16/9600) - 1 = 103
    UBRR0H = 0;
    UBRR0L = 103;

    // Enable RX, TX, and RX Complete Interrupt
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

    // 8-bit data, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(uint8_t data)
{
    // Wait for transmit buffer empty
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = data;
}

void uart_print(const char *str)
{
    while (*str)
    {
        uart_transmit(*str++);
    }
}

// ============================================================================
// Demo 1: Basic RX Interrupt - Echo Characters
// ============================================================================
/*
 * Simplest interrupt-based receive
 * - ISR stores received character in rx_char
 * - Sets rx_flag when new character available
 * - Main loop echoes character when flag set
 *
 * NOTICE: Main loop is free to do other work between characters!
 */

void demo_01_basic_rx_interrupt(void)
{
    demo_mode = 1;
    rx_flag = 0;

    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei(); // Enable global interrupts

    uart_print("UART RX Interrupt Demo\r\n");
    uart_print("Type characters to echo\r\n\r\n");

    while (1)
    {
        if (rx_flag)
        {
            rx_flag = 0; // Clear flag

            // Echo the character
            uart_transmit(rx_char);

            // Display on LEDs
            PORTB = rx_char;
        }

        // Main loop can do other work here!
        // For example: reading sensors, updating displays, etc.
    }
}

// ============================================================================
// Demo 2: Character Processing - LED Control
// ============================================================================
/*
 * Process received characters to control LEDs
 * Commands:
 * '1'-'8': Turn on LED 0-7
 * '0': All LEDs off
 * '+': All LEDs on
 */

void demo_02_led_control(void)
{
    demo_mode = 2;
    rx_flag = 0;

    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei();

    uart_print("LED Control via UART\r\n");
    uart_print("1-8: Turn on LED 0-7\r\n");
    uart_print("0: All off\r\n");
    uart_print("+: All on\r\n\r\n");

    while (1)
    {
        if (rx_flag)
        {
            rx_flag = 0;

            // Process command
            if (rx_char >= '1' && rx_char <= '8')
            {
                // Turn on specific LED
                uint8_t led_num = rx_char - '1';
                PORTB |= (1 << led_num);

                uart_print("LED ");
                uart_transmit(rx_char);
                uart_print(" ON\r\n");
            }
            else if (rx_char == '0')
            {
                // All LEDs off
                PORTB = 0x00;
                uart_print("All LEDs OFF\r\n");
            }
            else if (rx_char == '+')
            {
                // All LEDs on
                PORTB = 0xFF;
                uart_print("All LEDs ON\r\n");
            }
            else
            {
                uart_print("Unknown command\r\n");
            }
        }
    }
}

// ============================================================================
// Demo 3: Simple Command Parser
// ============================================================================
/*
 * Build command string until ENTER pressed
 * Then parse and execute the command
 *
 * Commands:
 * ON    - Turn on all LEDs
 * OFF   - Turn off all LEDs
 * BLINK - Blink LEDs 5 times
 */

void process_command(void)
{
    uart_print("\r\n");

    // Simple command parser
    if (cmd_buffer[0] == 'O' && cmd_buffer[1] == 'N')
    {
        PORTB = 0xFF;
        uart_print("LEDs ON\r\n");
    }
    else if (cmd_buffer[0] == 'O' && cmd_buffer[1] == 'F' && cmd_buffer[2] == 'F')
    {
        PORTB = 0x00;
        uart_print("LEDs OFF\r\n");
    }
    else if (cmd_buffer[0] == 'B' && cmd_buffer[1] == 'L' && cmd_buffer[2] == 'I' && cmd_buffer[3] == 'N' && cmd_buffer[4] == 'K')
    {
        uart_print("Blinking...\r\n");
        for (uint8_t i = 0; i < 5; i++)
        {
            PORTB = 0xFF;
            _delay_ms(200);
            PORTB = 0x00;
            _delay_ms(200);
        }
    }
    else
    {
        uart_print("Unknown: ");
        uart_print((char *)cmd_buffer);
        uart_print("\r\n");
    }

    uart_print("> ");
}

void demo_03_command_parser(void)
{
    demo_mode = 3;
    cmd_index = 0;
    cmd_ready = 0;

    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei();

    uart_print("\r\n=== Command Parser Demo ===\r\n");
    uart_print("Commands:\r\n");
    uart_print("  ON    - Turn on LEDs\r\n");
    uart_print("  OFF   - Turn off LEDs\r\n");
    uart_print("  BLINK - Blink LEDs\r\n");
    uart_print("> ");

    while (1)
    {
        if (cmd_ready)
        {
            cmd_ready = 0;
            process_command();
        }
    }
}

// ============================================================================
// Demo 4: Numeric Value Input
// ============================================================================
/*
 * Read numeric value and use it to set LED pattern
 * Example: "127" sets PORTB = 127
 *
 * Commands:
 * 0-255 + ENTER: Set LED pattern
 * 'C': Clear and start over
 */

void demo_04_numeric_input(void)
{
    demo_mode = 4;
    numeric_value = 0;
    digit_count = 0;
    value_ready = 0;

    DDRB = 0xFF;
    PORTB = 0x00;

    uart_init();
    sei();

    uart_print("Numeric Input Demo\r\n");
    uart_print("Enter 0-255, press ENTER\r\n");
    uart_print("'C' to clear\r\n\r\n");

    while (1)
    {
        if (value_ready)
        {
            value_ready = 0;

            if (numeric_value <= 255)
            {
                PORTB = (uint8_t)numeric_value;

                uart_print("\r\nValue: ");

                // Print number
                char num_str[4];
                uint8_t i = 0;
                uint16_t temp = numeric_value;

                if (temp == 0)
                {
                    uart_transmit('0');
                }
                else
                {
                    while (temp > 0)
                    {
                        num_str[i++] = '0' + (temp % 10);
                        temp /= 10;
                    }

                    // Print in reverse
                    while (i > 0)
                    {
                        uart_transmit(num_str[--i]);
                    }
                }

                uart_print(" (0x");

                // Print hex
                uint8_t hex_val = (uint8_t)numeric_value;
                char hex_chars[] = "0123456789ABCDEF";
                uart_transmit(hex_chars[hex_val >> 4]);
                uart_transmit(hex_chars[hex_val & 0x0F]);

                uart_print(")\r\n");
            }
            else
            {
                uart_print("\r\nValue too large! (max 255)\r\n");
            }

            // Reset
            numeric_value = 0;
        }
    }
}

// ============================================================================
// MAIN - Select Demo
// ============================================================================

int main(void)
{
    // CHOOSE ONE DEMO TO RUN:

    // demo_01_basic_rx_interrupt();  // Basic echo with interrupt
    // demo_02_led_control();         // Control LEDs with commands
    demo_03_command_parser(); // Multi-character commands
    // demo_04_numeric_input();       // Enter numeric values

    return 0;
}
