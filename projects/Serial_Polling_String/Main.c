/*
 * Serial Polling String - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Understand UART string communication
 * - Learn polling-based string transmission
 * - Practice string handling in embedded C
 * - Master bidirectional communication
 *
 * HARDWARE SETUP:
 * - Connect UART pins (TXD1/RXD1) to PC or terminal
 * - Optional: LEDs on PORTB for visual feedback
 */

#include "config.h"

int main(void)
{
    // Initialize system components
    init_devices(); // Initialize all peripherals

    // Initialize UART for communication
    Uart1_init(); // 9600 baud serial communication

    // Send welcome message
    puts_USART1("\r\n=== Serial String Communication Demo ===\r\n");
    puts_USART1("Type messages and press Enter\r\n");
    puts_USART1("Messages will be echoed back\r\n");
    puts_USART1("Ready to receive...\r\n\r\n");

    char received_char;
    char input_buffer[80];
    uint8_t buffer_index = 0;

    while (1)
    {
        // Check if character available
        if (Uart1_is_available())
        {
            // Read received character
            received_char = getc_USART1();

            // Echo character back
            putc_USART1(received_char);

            // Handle different characters
            if (received_char == '\r' || received_char == '\n')
            {
                // End of string - process it
                input_buffer[buffer_index] = '\0';

                // Send response
                puts_USART1("\r\nYou sent: \"");
                puts_USART1(input_buffer);
                puts_USART1("\"\r\n");

                // Visual feedback - show string length on LEDs
                PORTB = buffer_index;

                // Reset buffer
                buffer_index = 0;
                puts_USART1("Ready for next message: ");
            }
            else if (received_char == 8 || received_char == 127) // Backspace
            {
                if (buffer_index > 0)
                {
                    buffer_index--;
                    puts_USART1(" \b"); // Erase character on terminal
                }
            }
            else if (buffer_index < 79) // Regular character
            {
                input_buffer[buffer_index++] = received_char;
            }
        }

        // Small delay to prevent overwhelming the UART
        _delay_ms(10);
    }

    return 0;
}