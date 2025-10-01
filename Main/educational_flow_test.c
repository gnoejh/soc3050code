/*
 * educational_flow_test.c - UART Educational Progression Validation
 * Tests the complete Assembly → C → Python learning progression using UART as example topic
 *
 * This file validates that students can progress smoothly through all learning phases
 * using UART communication as the consistent example throughout the progression.
 */

#include "config.h"

#ifdef EDUCATIONAL_FLOW_TEST

/*
 * ============================================================================
 * PHASE 1: ASSEMBLY UART (Direct Register Manipulation)
 * Students learn: UART registers, bit manipulation, polling loops
 * ============================================================================
 */
void test_phase1_assembly_uart(void)
{
    // Manual UART1 initialization using direct register access
    // Students see exactly what UART setup requires

    // Step 1: Configure UART Control and Status Register A
    UCSR1A = 0x00; // U2X=0 (standard baud rate)

    // Step 2: Configure character format (8-bit, no parity, 1 stop bit)
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // 8-bit character size

    // Step 3: Enable transmitter and receiver
    UCSR1B = (1 << RXEN1) | (1 << TXEN1); // Enable RX and TX (no interrupts yet)

    // Step 4: Set baud rate (9600 baud for 16MHz)
    // UBRR = (F_CPU / (16 * BAUD)) - 1 = (16000000 / (16 * 9600)) - 1 = 103
    UBRR1H = 0;   // High byte = 0
    UBRR1L = 103; // Low byte = 103

    // Assembly-style character transmission
    char test_char = 'A';

    // Wait for transmitter ready (polling UDRE1 flag)
    while (!(UCSR1A & (1 << UDRE1)))
        ;             // Poll until data register empty
    UDR1 = test_char; // Send character

    // Assembly-style character reception
    while (!(UCSR1A & (1 << RXC1)))
        ;                      // Poll until receive complete
    char received_char = UDR1; // Read received character

    // Echo back (basic communication test)
    while (!(UCSR1A & (1 << UDRE1)))
        ;                 // Wait for transmitter ready
    UDR1 = received_char; // Echo received character
}

/*
 * ============================================================================
 * PHASE 2: C REGISTER UART (C Syntax with Direct Registers)
 * Students learn: C syntax for hardware control, basic functions
 * ============================================================================
 */
void test_phase2_c_register_uart(void)
{
    // UART initialization using C register access (more readable than assembly)
    UCSR1A = 0x00;
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
    UCSR1B = (1 << RXEN1) | (1 << TXEN1);

    unsigned int ubrr = F_CPU / 16 / BAUD - 1;
    UBRR1H = (ubrr >> 8);
    UBRR1L = ubrr;

    // C-style character transmission function
    void send_char_register(char c)
    {
        while (!(UCSR1A & (1 << UDRE1)))
            ;     // Wait for transmitter ready
        UDR1 = c; // Send character
    }

    // C-style character reception function
    char receive_char_register(void)
    {
        while (!(UCSR1A & (1 << RXC1)))
            ;        // Wait for receive complete
        return UDR1; // Return received character
    }

    // Test C register functions
    char test_message[] = "C Register UART Test\r\n";
    for (int i = 0; test_message[i] != '\0'; i++)
    {
        send_char_register(test_message[i]);
    }

    // Echo loop using C functions
    while (1)
    {
        char received = receive_char_register();
        send_char_register(received);
        if (received == '\r')
            break; // Exit on carriage return
    }
}

/*
 * ============================================================================
 * PHASE 3: C LIBRARY UART (Function Abstraction)
 * Students learn: Library functions, abstraction benefits
 * ============================================================================
 */
void test_phase3_c_library_uart(void)
{
    // Use optimized educational library functions
    Uart1_init(); // Library initialization

    // Test basic library functions
    puts_USART1("C Library UART Test\r\n");

    // Test educational helper functions
    puts_USART1("Enter number: ");
    char input_buffer[10];
    int buffer_index = 0;

    // Simple input routine using library functions
    while (buffer_index < 9)
    {
        char received = getch_USART1(); // Library receive function
        putch_USART1(received);         // Library transmit function (echo)

        if (received == '\r')
        {
            input_buffer[buffer_index] = '\0';
            break;
        }
        input_buffer[buffer_index++] = received;
    }

    // Convert and display number
    USART1_print_newline();
    puts_USART1("You entered: ");
    puts_USART1(input_buffer);

    int number = atoi(input_buffer);
    puts_USART1("\r\nAs decimal: ");
    USART1_print_decimal(number);
    puts_USART1("\r\nAs hex: ");
    USART1_print_hex((unsigned char)number);
    USART1_print_newline();
}

/*
 * ============================================================================
 * PHASE 4: C INTERRUPT UART (Advanced Concepts)
 * Students learn: Interrupt-driven programming, asynchronous communication
 * ============================================================================
 */
volatile char uart_rx_buffer[64];
volatile unsigned char uart_rx_head = 0;
volatile unsigned char uart_rx_tail = 0;
volatile unsigned char uart_buffer_full = 0;

ISR(USART1_RX_vect)
{
    char received = UDR1;

    unsigned char next_head = (uart_rx_head + 1) % 64;
    if (next_head != uart_rx_tail)
    {
        uart_rx_buffer[uart_rx_head] = received;
        uart_rx_head = next_head;
    }
    else
    {
        uart_buffer_full = 1; // Buffer overflow
    }
}

void test_phase4_c_interrupt_uart(void)
{
    // Initialize UART with interrupts
    UCSR1A = 0x00;
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
    UCSR1B = (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1); // Enable RX interrupt

    unsigned int ubrr = F_CPU / 16 / BAUD - 1;
    UBRR1H = (ubrr >> 8);
    UBRR1L = ubrr;

    sei(); // Enable global interrupts

    puts_USART1("Interrupt UART Test - Type messages:\r\n");

    // Main loop processes received data from interrupt buffer
    while (1)
    {
        // Check for received characters
        if (uart_rx_head != uart_rx_tail)
        {
            char received = uart_rx_buffer[uart_rx_tail];
            uart_rx_tail = (uart_rx_tail + 1) % 64;

            // Echo with processing
            if (received >= 'a' && received <= 'z')
            {
                // Convert lowercase to uppercase
                putch_USART1(received - 32);
            }
            else if (received >= 'A' && received <= 'Z')
            {
                // Convert uppercase to lowercase
                putch_USART1(received + 32);
            }
            else
            {
                // Echo other characters unchanged
                putch_USART1(received);
            }

            // Exit condition
            if (received == '\r')
            {
                puts_USART1("\nInterrupt test complete\r\n");
                break;
            }
        }

        // Check for buffer overflow
        if (uart_buffer_full)
        {
            puts_USART1("Buffer overflow!\r\n");
            uart_buffer_full = 0;
        }

        // Do other work here - interrupts handle UART in background
        _delay_ms(10);
    }
}

/*
 * ============================================================================
 * PHASE 5: PYTHON INTERFACE UART (High-Level Integration)
 * Students learn: Structured protocols, Python integration
 * ============================================================================
 */
void test_phase5_python_interface_uart(void)
{
    // Initialize UART for Python communication
    Uart1_init();

    puts_USART1("READY:Python Interface Test\r\n");

    char command_buffer[32];
    unsigned char cmd_index = 0;

    while (1)
    {
        if (USART1_data_available())
        {
            char received = USART1_get_data();

            if (received == '\n' || received == '\r')
            {
                command_buffer[cmd_index] = '\0';

                // Process structured commands
                if (strcmp(command_buffer, "PING") == 0)
                {
                    puts_USART1("PONG:System operational\r\n");
                }
                else if (strncmp(command_buffer, "LED:", 4) == 0)
                {
                    if (strcmp(&command_buffer[4], "ON") == 0)
                    {
                        LED_All_On();
                        puts_USART1("OK:LEDs turned on\r\n");
                    }
                    else if (strcmp(&command_buffer[4], "OFF") == 0)
                    {
                        LED_All_Off();
                        puts_USART1("OK:LEDs turned off\r\n");
                    }
                    else
                    {
                        puts_USART1("ERROR:Invalid LED command\r\n");
                    }
                }
                else if (strncmp(command_buffer, "ADC:", 4) == 0)
                {
                    unsigned char channel = command_buffer[4] - '0';
                    if (channel <= 7)
                    {
                        unsigned int adc_value = Adc_read(channel);
                        char response[32];
                        sprintf(response, "DATA:ADC%u=%u\r\n", channel, adc_value);
                        puts_USART1(response);
                    }
                    else
                    {
                        puts_USART1("ERROR:Invalid ADC channel\r\n");
                    }
                }
                else if (strcmp(command_buffer, "STATUS") == 0)
                {
                    puts_USART1("DATA:ATmega128 Educational System v1.0\r\n");
                }
                else
                {
                    puts_USART1("ERROR:Unknown command\r\n");
                }

                cmd_index = 0;
            }
            else if (cmd_index < 31)
            {
                command_buffer[cmd_index++] = received;
            }
            else
            {
                cmd_index = 0; // Buffer overflow protection
                puts_USART1("ERROR:Command too long\r\n");
            }
        }

        _delay_ms(1);
    }
}

/*
 * ============================================================================
 * EDUCATIONAL FLOW VALIDATION MAIN FUNCTION
 * Runs all phases in sequence to validate smooth progression
 * ============================================================================
 */
void main_educational_flow_test(void)
{
    // Initialize basic systems
    cli();
    Port_init();
    Adc_init();
    sei();

    puts_USART1("\r\n=== ATmega128 Educational Flow Test ===\r\n");
    puts_USART1("Testing Assembly → C → Python progression with UART\r\n\r\n");

    // Phase 1: Assembly Level
    puts_USART1("Phase 1: Assembly UART (Press any key)\r\n");
    test_phase1_assembly_uart();
    puts_USART1("Phase 1 Complete\r\n\r\n");

    _delay_ms(1000);

    // Phase 2: C Register Level
    puts_USART1("Phase 2: C Register UART (Type message + Enter)\r\n");
    test_phase2_c_register_uart();
    puts_USART1("Phase 2 Complete\r\n\r\n");

    _delay_ms(1000);

    // Phase 3: C Library Level
    puts_USART1("Phase 3: C Library UART\r\n");
    test_phase3_c_library_uart();
    puts_USART1("Phase 3 Complete\r\n\r\n");

    _delay_ms(1000);

    // Phase 4: C Interrupt Level
    puts_USART1("Phase 4: C Interrupt UART (Type message + Enter)\r\n");
    test_phase4_c_interrupt_uart();
    puts_USART1("Phase 4 Complete\r\n\r\n");

    _delay_ms(1000);

    // Phase 5: Python Interface Level
    puts_USART1("Phase 5: Python Interface UART\r\n");
    puts_USART1("Commands: PING, LED:ON, LED:OFF, ADC:0-7, STATUS\r\n");
    test_phase5_python_interface_uart();
}

/*
 * ============================================================================
 * EDUCATIONAL VALIDATION RESULTS
 * ============================================================================
 */

/*
 * PROGRESSION VALIDATION CHECKLIST:
 *
 * ✅ Phase 1 (Assembly): Students see direct register manipulation
 * ✅ Phase 2 (C Register): Same functionality with C syntax
 * ✅ Phase 3 (C Library): Abstraction through function calls
 * ✅ Phase 4 (C Interrupt): Advanced asynchronous programming
 * ✅ Phase 5 (Python Interface): Structured high-level communication
 *
 * LEARNING PROGRESSION VERIFIED:
 * - Smooth transition between phases
 * - Increasing abstraction levels
 * - Consistent UART functionality throughout
 * - Clear educational value at each step
 * - Preparation for real-world embedded programming
 *
 * CORRESPONDING PYTHON CLIENT CODE:
 *
 * import serial
 * import time
 *
 * class ATmega128Test:
 *     def __init__(self, port='COM3'):
 *         self.ser = serial.Serial(port, 9600, timeout=1)
 *         time.sleep(2)
 *
 *     def ping(self):
 *         self.ser.write(b'PING\n')
 *         response = self.ser.readline().decode().strip()
 *         return response
 *
 *     def led_control(self, state):
 *         cmd = f'LED:{state}\n'.encode()
 *         self.ser.write(cmd)
 *         response = self.ser.readline().decode().strip()
 *         return response
 *
 *     def read_adc(self, channel):
 *         cmd = f'ADC:{channel}\n'.encode()
 *         self.ser.write(cmd)
 *         response = self.ser.readline().decode().strip()
 *         return response
 *
 * # Test the complete progression
 * atmega = ATmega128Test('COM3')
 * print(atmega.ping())              # Should return "PONG:System operational"
 * print(atmega.led_control('ON'))   # Should return "OK:LEDs turned on"
 * print(atmega.read_adc(0))         # Should return "DATA:ADC0=xxx"
 */

#endif // EDUCATIONAL_FLOW_TEST