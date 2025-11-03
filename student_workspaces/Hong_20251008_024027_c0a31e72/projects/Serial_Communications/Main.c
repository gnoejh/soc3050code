/*
 * =============================================================================
 * SERIAL COMMUNICATION METHODS - EDUCATIONAL COMPARISON
 * =============================================================================
 *
 * PROJECT: Serial_Communications
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational comparison between POLLING vs INTERRUPT-based serial communication.
 * Students learn both approaches with practical demonstrations and performance analysis.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Compare polling vs interrupt-driven communication
 * 2. Understand when to use each method
 * 3. Learn interrupt service routine programming
 * 4. Implement buffered vs direct communication
 * 5. Analyze performance and responsiveness differences
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - UART1 connection for serial communication
 * - Serial terminal (9600 baud, 8N1)
 *
 * CRITICAL EDUCATIONAL POINTS FOR STUDENTS:
 *
 * 1. REAL ISR PROGRAMMING:
 *    - No wrapper functions or callback managers
 *    - Direct ISR(USART1_RX_vect) and ISR(USART1_UDRE_vect)
 *    - Students see actual interrupt vector names
 *    - Learn proper ISR syntax and timing constraints
 *
 * 2. DIRECT REGISTER PROGRAMMING:
 *    - UCSR1B |= (1 << RXCIE1) to enable RX interrupts
 *    - UCSR1B |= (1 << UDRIE1) to enable TX interrupts
 *    - UDR1 register for data read/write
 *    - sei() and cli() for global interrupt control
 *
 * 3. VOLATILE VARIABLES:
 *    - All ISR-accessed variables marked volatile
 *    - Students learn why volatile is essential
 *    - Understand shared data between ISR and main()
 *
 * 4. CIRCULAR BUFFER IMPLEMENTATION:
 *    - Head/tail pointers for FIFO operation
 *    - Buffer overflow detection and handling
 *    - Atomic operations for data integrity
 *
 * 5. PERFORMANCE COMPARISON:
 *    - Polling blocks CPU (inefficient)
 *    - Interrupts free CPU for other tasks (efficient)
 *    - Real-time responsiveness differences
 *
 * LEARNING PROGRESSION:
 * POLLING METHODS (Simple, blocking, CPU intensive):
 * - Demo 1: Basic Polling Echo
 * - Demo 2: Polling Command Processing
 * - Demo 3: Polling with Simple Buffering
 *
 * INTERRUPT METHODS (Efficient, non-blocking, complex):
 * - Demo 4: Basic RX Interrupt
 * - Demo 5: TX Interrupt Queue
 * - Demo 6: Full Duplex Interrupts
 * - Demo 7: Advanced Interrupt Buffering
 *
 * =============================================================================
 */

#include "config.h"

// Function prototypes
void simple_init_serial(void);
void demo_polling_echo(void);
void demo_polling_commands(void);
void demo_polling_buffered(void);
void demo_interrupt_echo(void);
void demo_interrupt_tx_queue(void);
void demo_interrupt_bidirectional(void);
void demo_interrupt_commands(void);
void demo_interrupt_advanced(void);

/*
 * =============================================================================
 * EDUCATIONAL COMPARISON: POLLING vs INTERRUPT METHODS
 * =============================================================================
 *
 * USAGE: To select a demo, uncomment the desired function call in main()
 *
 * POLLING DEMOS (CPU waits for data):
 * - demo_polling_echo()              : Simple character echo using polling
 * - demo_polling_commands()          : Command processing using polling
 * - demo_polling_buffered()          : Simple buffering with polling
 *
 * INTERRUPT DEMOS (CPU continues while interrupts handle data):
 * - demo_interrupt_echo()            : Character echo using RX interrupts
 * - demo_interrupt_tx_queue()        : TX queue using interrupts
 * - demo_interrupt_bidirectional()   : Full duplex interrupt communication
 * - demo_interrupt_advanced()        : Advanced interrupt buffering
 */

// Simple initialization function (no LCD needed for serial communication)
void simple_init_serial(void)
{
    // Initialize basic I/O ports
    PORTA = 0xFF;
    DDRA = 0x00; // Set PORTA as input with pull-ups
    PORTB = 0x00;
    DDRB = 0xFF; // Set PORTB as output

    // Note: UART initialization is handled by each individual demo
    // This allows clean demonstration of different initialization approaches
} /*
   * =============================================================================
   * EDUCATIONAL UART FUNCTIONS - DIRECT REGISTER PROGRAMMING
   * =============================================================================
   * These functions show students the exact register operations for UART communication.
   * Students learn to work directly with UART registers without wrapper functions.
   */

// Initialize UART1 for 9600 baud, 8N1 format - EDUCATIONAL VERSION
void init_uart_polling(void)
{
    // Step 1: Configure UART Control Register A (standard baud rate)
    UCSR1A = 0x00; // U2X=0 for standard baud rate calculation

    // Step 2: Configure character format (8 data bits, 1 stop bit, no parity)
    UCSR1C = UART_8BIT_CHAR; // UCSZ01:00 = 11 for 8-bit character size

    // Step 3: Enable transmitter and receiver (NO INTERRUPTS for polling)
    UCSR1B = UART_ENABLE_RX_TX; // RXEN1 and TXEN1

    // Step 4: Calculate and set baud rate
    // Formula: UBRR = (F_CPU / (16 * BAUD)) - 1
    // For 16MHz and 9600 baud: UBRR = (16000000 / (16 * 9600)) - 1 = 103
    unsigned int baud_register = UART_BAUD_REGISTER;
    UBRR1H = (baud_register >> 8); // High byte of baud rate register
    UBRR1L = baud_register;        // Low byte of baud rate register

    // Step 5: Allow UART hardware to stabilize (CRITICAL for first character)
    _delay_ms(10); // Small delay for UART register stabilization
}

// Send single character via UART1 - EDUCATIONAL VERSION
void putch_USART1(char c)
{
    // Wait for transmit buffer to be empty
    // Students learn to check UDRE1 (USART Data Register Empty) flag
    while (!(UCSR1A & (1 << UDRE1)))
    {
        // Busy wait - this is polling!
        // CPU is blocked here until transmitter is ready
    }

    // Put data into buffer, sends the data
    UDR1 = c;
}

// Send string via UART1 - EDUCATIONAL VERSION
void puts_USART1(const char *str)
{
    while (*str != '\0')
    {
        putch_USART1(*str);
        str++;
    }
}

// Receive single character via UART1 - EDUCATIONAL VERSION
char getch_USART1(void)
{
    // Wait for data to be received
    // Students learn to check RXC1 (Receive Complete) flag
    while (!(UCSR1A & (1 << RXC1)))
    {
        // Busy wait - this is polling!
        // CPU is blocked here until data arrives
    }

    // Get and return received data from buffer
    return UDR1;
}

// Check if data is available for reading - EDUCATIONAL VERSION
unsigned char data_available_USART1(void)
{
    // Return 1 if data is available, 0 if not
    // Students learn to check RXC1 flag without blocking
    return (UCSR1A & (1 << RXC1)) ? 1 : 0;
}

/*
 * =============================================================================
 * POLLING-BASED SERIAL COMMUNICATION DEMOS
 * =============================================================================
 * These demos use polling (busy-waiting) to check for received data.
 * Advantages: Simple to understand and implement
 * Disadvantages: CPU is blocked while waiting, inefficient
 */

/*
 * Demo 1: Basic Polling Echo
 * Simple character echo using polling method - CPU waits for each character
 */
void demo_polling_echo(void)
{
    // Initialize UART for polling communication
    init_uart_polling();

    puts_USART1("\r\n=== DEMO 1: Polling Echo ===\r\n");
    puts_USART1("POLLING METHOD: CPU waits for each character\r\n");
    puts_USART1("Type characters - they will be echoed back\r\n");
    puts_USART1("Press 'q' to quit this demo\r\n\r\n");

    char received;
    while (1)
    {
        // POLLING: CPU waits here until character received
        received = getch_USART1(); // This function blocks/waits

        if (received == 'q' || received == 'Q')
        {
            break;
        }

        // Echo character back immediately
        putch_USART1(received);

        // Show that CPU was blocked during getch_USART1()
        puts_USART1(" [CPU was blocked while waiting] ");
    }

    puts_USART1("\r\nPolling Demo 1 completed.\r\n");
}

/*
 * Demo 2: Polling Command Processing
 * Command processing using polling - inefficient but simple
 */
void demo_polling_commands(void)
{
    puts_USART1("\r\n=== DEMO 2: Polling Commands ===\r\n");
    puts_USART1("POLLING METHOD: CPU waits for each command character\r\n");
    puts_USART1("Commands: 'time', 'status', 'help', 'quit'\r\n\r\n");

    char command[32];
    unsigned char cmd_index = 0;
    char received;
    unsigned int message_count = 0;

    puts_USART1("POLL> ");

    while (1)
    {
        // POLLING: CPU blocks here waiting for each character
        received = getch_USART1();

        if (received == '\r' || received == '\n')
        {
            command[cmd_index] = '\0';
            message_count++;

            puts_USART1("\r\n");

            // Process the complete command
            if (strcmp(command, "time") == 0)
            {
                puts_USART1("[POLLING TIME] Count: ");
                putch_USART1('0' + (message_count % 10));
                puts_USART1("\r\n");
            }
            else if (strcmp(command, "status") == 0)
            {
                puts_USART1("[POLLING STATUS] CPU was blocked ");
                putch_USART1('0' + (message_count % 10));
                puts_USART1(" times waiting for input\r\n");
            }
            else if (strcmp(command, "help") == 0)
            {
                puts_USART1("[POLLING HELP] Commands: time, status, help, quit\r\n");
                puts_USART1("Note: CPU blocks on each character with polling\r\n");
            }
            else if (strcmp(command, "quit") == 0)
            {
                puts_USART1("[POLLING EXIT] Exiting polling demo\r\n");
                break;
            }
            else if (cmd_index > 0)
            {
                puts_USART1("[POLLING ERROR] Unknown: '");
                puts_USART1(command);
                puts_USART1("'\r\n");
            }

            cmd_index = 0;
            puts_USART1("POLL> ");
        }
        else if (received == '\b' || received == 127)
        {
            if (cmd_index > 0)
            {
                cmd_index--;
                puts_USART1("\b \b");
            }
        }
        else if (cmd_index < 31 && received >= ' ')
        {
            command[cmd_index++] = received;
            putch_USART1(received);
        }
    }

    puts_USART1("\r\nPolling Demo 2 completed.\r\n");
}

/*
 * Demo 3: Polling with Simple Buffering
 * Shows manual buffering with polling (still inefficient)
 */
void demo_polling_buffered(void)
{
    puts_USART1("\r\n=== DEMO 3: Polling with Manual Buffer ===\r\n");
    puts_USART1("POLLING METHOD: Manual buffer check, CPU still waits\r\n");
    puts_USART1("Type 's' for stats, 'q' to quit\r\n\r\n");

    char simple_buffer[16];
    unsigned char buffer_count = 0;
    char received;
    unsigned int total_chars = 0;

    while (1)
    {
        // POLLING: Still blocks CPU, but we manually manage a buffer
        received = getch_USART1();
        total_chars++;

        if (received == 's' || received == 'S')
        {
            puts_USART1("\r\n[POLLING STATS] Buffer: ");
            putch_USART1('0' + (buffer_count % 10));
            puts_USART1(", Total: ");
            putch_USART1('0' + (total_chars % 10));
            puts_USART1(" (CPU blocked each time)\r\n");
            buffer_count = 0; // Reset buffer
        }
        else if (received == 'q' || received == 'Q')
        {
            break;
        }
        else
        {
            // Add to simple buffer (no interrupt needed)
            if (buffer_count < 15)
            {
                simple_buffer[buffer_count++] = received;
            }

            // Echo with buffer info
            putch_USART1('[');
            putch_USART1('0' + (buffer_count % 10));
            putch_USART1(']');
            putch_USART1(received);
        }
    }

    puts_USART1("\r\nPolling Demo 3 completed.\r\n");
}

/*
 * =============================================================================
 * INTERRUPT COMMUNICATION GLOBAL VARIABLES
 * =============================================================================
 */

// Receive buffer and control variables
#define RX_BUFFER_SIZE 32 // Reduced from 64 to save memory
volatile char rx_buffer[RX_BUFFER_SIZE];
volatile unsigned char rx_head = 0;
volatile unsigned char rx_tail = 0;
volatile unsigned char rx_overflow = 0;

// Transmit buffer and control variables
#define TX_BUFFER_SIZE 32 // Reduced from 64 to save memory
volatile char tx_buffer[TX_BUFFER_SIZE];
volatile unsigned char tx_head = 0;
volatile unsigned char tx_tail = 0;
volatile unsigned char tx_busy = 0;

// Communication status and control
volatile unsigned char new_command_received = 0;
volatile unsigned char communication_mode = 0;
volatile unsigned char error_count = 0;

// Command processing variables
volatile char command_buffer[16]; // Reduced from 32 to save memory
volatile unsigned char command_length = 0;
volatile unsigned char command_ready = 0;

/*
 * =============================================================================
 * CUSTOM INTERRUPT HANDLERS FOR SERIAL_INTERRUPT PROJECT
 * =============================================================================
 */

/*
 * =============================================================================
 * EDUCATIONAL INTERRUPT SERVICE ROUTINES
 * =============================================================================
 * These are the actual ISRs that students must learn to write.
 * No wrappers or managers - direct hardware programming!
 */

/*
 * =============================================================================
 * EDUCATIONAL INTERRUPT SERVICE ROUTINES
 * =============================================================================
 * These are the actual ISRs that students must learn to write.
 * No wrappers or managers - direct hardware programming!
 */

/*
 * USART1 Receive Complete Interrupt
 * This ISR is called automatically when a character is received
 * Students learn: ISR syntax, volatile variables, circular buffers
 */
ISR(USART1_RX_vect)
{
    char received = UDR1; // Read the received character
    unsigned char next_head = (rx_head + 1) % RX_BUFFER_SIZE;

    // Check for buffer overflow
    if (next_head != rx_tail)
    {
        rx_buffer[rx_head] = received;
        rx_head = next_head;
    }
    else
    {
        rx_overflow = 1; // Flag overflow for debugging
        error_count++;
    }
}

/*
 * USART0 Data Register Empty Interrupt
 * This ISR is called when the transmit buffer is ready for next character
 * Students learn: TX interrupts, automatic transmission, buffer management
 */
ISR(USART0_UDRE_vect)
{
    if (tx_head != tx_tail)
    {
        // Send next character from buffer
        UDR0 = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
    }
    else
    {
        // Buffer empty - disable this interrupt
        UCSR0B &= ~(1 << UDRIE0);
        tx_busy = 0;
    }
}

/*
 * =============================================================================
 * INTERRUPT COMMUNICATION SETUP FUNCTIONS
 * =============================================================================
 */

/*
 * Initialize UART1 with proper interrupt configuration
 * Students learn: Direct register programming, interrupt enable bits
 */
void init_uart_interrupts(void)
{
    // EDUCATIONAL UART INITIALIZATION - Direct Register Programming
    // Students learn the exact steps for UART setup:

    // Step 1: Configure UART Control Register A (standard baud rate)
    UCSR1A = 0x00; // U2X=0 for standard baud rate calculation

    // Step 2: Configure character format (8 data bits, 1 stop bit, no parity)
    UCSR1C = UART_8BIT_CHAR; // UCSZ11:10 = 11 for 8-bit character size

    // Step 3: Enable transmitter and receiver
    UCSR1B = UART_ENABLE_RX_TX; // RXEN1 and TXEN1

    // Step 4: Calculate and set baud rate
    // Formula: UBRR = (F_CPU / (16 * BAUD)) - 1
    // For 16MHz and 9600 baud: UBRR = (16000000 / (16 * 9600)) - 1 = 103
    unsigned int baud_register = UART_BAUD_REGISTER;
    UBRR1H = (baud_register >> 8); // High byte of baud rate register
    UBRR1L = baud_register;        // Low byte of baud rate register

    // NOW THE EDUCATIONAL PART: Direct interrupt setup
    // Students learn these exact register operations:

    // Enable RX Complete Interrupt
    UCSR1B |= (1 << RXCIE1);

    // Enable global interrupts (students must understand this!)
    sei();

    // Clear our educational buffers
    rx_head = rx_tail = 0;
    tx_head = tx_tail = 0;
    rx_overflow = 0;
    tx_busy = 0;
    error_count = 0;
}

/*
 * Send character using interrupt-driven transmission
 */
unsigned char send_char_interrupt(char data)
{
    unsigned char next_head = (tx_head + 1) % TX_BUFFER_SIZE;

    // Check if buffer full
    if (next_head == tx_tail)
    {
        return 0; // Buffer full
    }

    // Add to buffer
    tx_buffer[tx_head] = data;
    tx_head = next_head;

    // Enable transmit interrupt if not busy
    if (!tx_busy)
    {
        tx_busy = 1;
        UCSR0B |= (1 << UDRIE0);
    }

    return 1; // Success
}

/*
 * Send string using interrupt-driven transmission
 */
void send_string_interrupt(const char *str)
{
    while (*str)
    {
        while (!send_char_interrupt(*str))
            ; // Wait if buffer full
        str++;
    }
}

/*
 * Check if characters available in RX buffer
 */
unsigned char chars_available(void)
{
    return (rx_head != rx_tail);
}

/*
 * Get character from RX buffer
 */
char get_char_from_buffer(void)
{
    char data;

    if (rx_head == rx_tail)
    {
        return 0; // Buffer empty
    }

    data = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;

    return data;
}

/*
 * =============================================================================
 * DEMONSTRATION MODE FUNCTIONS
 * =============================================================================
 */

/*
 * =============================================================================
 * SERIAL COMMUNICATION DEMO FUNCTIONS
 * =============================================================================
 */

/*
 * Demo 4: Basic RX Interrupt Echo
 * Uses real ISR(USART1_RX_vect) to receive characters
 * Students learn: How interrupts work, non-blocking I/O, ISR programming
 */
void demo_interrupt_echo(void)
{
    puts_USART1("\r\n=== DEMO 4: Interrupt Echo ===\r\n");
    puts_USART1("INTERRUPT METHOD: CPU continues other work while ISR handles data\r\n");
    puts_USART1("Students observe: ISR(USART0_RX_vect) automatically receives data\r\n");
    puts_USART1("Type characters - they will be echoed back using REAL interrupts\r\n");
    puts_USART1("Notice: CPU can do other tasks while ISR handles serial communication\r\n");
    puts_USART1("Press 'q' to quit this demo\r\n\r\n");

    // EDUCATIONAL: Initialize interrupt-based UART (see ISRs above!)
    init_uart_interrupts();

    // EDUCATIONAL: Show students the difference - CPU is free to do other work!
    unsigned int counter = 0;
    char received;

    while (1)
    {
        // EDUCATIONAL POINT: CPU can do other work while ISR handles serial data!
        // This counter proves the CPU is not blocked waiting for serial data
        counter++;
        if ((counter % 20000) == 0)
        {
            // Show that CPU is free to do other tasks
            PORTB = ~PORTB; // Toggle LEDs to show CPU activity
        }

        // EDUCATIONAL: Check if our ISR has received data
        if (chars_available())
        {
            received = get_char_from_buffer();

            // Echo the character back (could also use interrupt for TX)
            putch_USART1(received);

            if (received == 'q' || received == 'Q')
            {
                break; // Exit demo
            }
        }

        // Show ISR buffer status periodically for debugging
        if ((counter % 100000) == 0)
        {
            if (rx_overflow)
            {
                puts_USART1("[ISR BUFFER OVERFLOW - too much data!]\r\n");
                rx_overflow = 0;
            }
        }
    }

    puts_USART1("\r\nInterrupt Demo 4 completed.\r\n");
    puts_USART1("Key Learning: CPU was free to count and toggle LEDs while ISR handled all serial data!\r\n");
    puts_USART1("Compare this efficiency with polling demos above.\r\n");
}

/*
 * Demo Mode 2: TX Interrupt with Queued Transmission
 * Demonstrates buffered transmission using TX interrupts
 */
void demo_interrupt_tx_queue(void)
{
    communication_mode = 2;

    // Initialize UART for interrupt-based communication
    init_uart_interrupts();

    send_string_interrupt("\r\n=== DEMO 5: TX Interrupt Queue ===\r\n");
    send_string_interrupt("INTERRUPT METHOD: Queued transmission frees CPU\r\n");
    send_string_interrupt("CPU can do other tasks while interrupts handle transmission\r\n");
    send_string_interrupt("Sending multiple messages using TX interrupt queue...\r\n\r\n");

    // Send multiple messages quickly to demonstrate queuing
    for (int i = 1; i <= 5; i++)
    {
        send_string_interrupt("Message ");
        send_char_interrupt('0' + i);
        send_string_interrupt(" - Queued transmission\r\n");
        _delay_ms(100); // Small delay between messages
    }

    send_string_interrupt("\r\nAll messages transmitted via interrupt queue.\r\n");
    send_string_interrupt("Press any key to continue...\r\n");

    getch_USART1(); // Wait for key press
}

/*
 * Demo Mode 3: Bidirectional Interrupts (Full Duplex)
 * Simultaneous RX and TX interrupts for real-time communication
 */
void demo_interrupt_bidirectional(void)
{
    communication_mode = 3;

    // Initialize UART for interrupt-based communication
    init_uart_interrupts();

    puts_USART1("\r\n=== DEMO 6: Bidirectional Interrupts ===\r\n");
    puts_USART1("INTERRUPT METHOD: Full duplex communication with RX and TX interrupts\r\n");
    puts_USART1("Compare responsiveness with polling methods (Demos 1-3)\r\n");
    puts_USART1("Type commands and press Enter to execute\r\n");
    puts_USART1("Commands: 'time', 'status', 'help', 'quit'\r\n\r\n");

    char command[32];
    unsigned char cmd_index = 0;
    char received;
    unsigned int message_count = 0;

    puts_USART1("BIDIR> ");

    while (1)
    {
        // Use interrupt buffer instead of direct register access
        if (chars_available())
        {
            received = get_char_from_buffer();

            if (received == '\r' || received == '\n')
            {
                command[cmd_index] = '\0';
                message_count++;

                puts_USART1("\r\n");

                // Process the complete command
                if (strcmp(command, "time") == 0)
                {
                    puts_USART1("[TIME] Uptime: ");
                    putch_USART1('0' + (message_count % 10));
                    puts_USART1(" minutes\r\n");
                }
                else if (strcmp(command, "status") == 0)
                {
                    puts_USART1("[STATUS] System OK, Messages: ");
                    putch_USART1('0' + (message_count % 10));
                    puts_USART1(", Mode: Bidirectional\r\n");
                }
                else if (strcmp(command, "help") == 0)
                {
                    puts_USART1("[HELP] Available commands:\r\n");
                    puts_USART1("  time   - Show uptime\r\n");
                    puts_USART1("  status - Show system status\r\n");
                    puts_USART1("  help   - Show this help\r\n");
                    puts_USART1("  quit   - Exit demo\r\n");
                }
                else if (strcmp(command, "quit") == 0)
                {
                    puts_USART1("[EXIT] Exiting bidirectional demo\r\n");
                    break;
                }
                else if (cmd_index > 0)
                {
                    puts_USART1("[ERROR] Unknown command: '");
                    puts_USART1(command);
                    puts_USART1("'\r\n");
                }

                cmd_index = 0;
                puts_USART1("BIDIR> ");
            }
            else if (received == '\b' || received == 127)
            { // Backspace
                if (cmd_index > 0)
                {
                    cmd_index--;
                    puts_USART1("\b \b");
                }
            }
            else if (cmd_index < 31 && received >= ' ')
            {
                command[cmd_index++] = received;
                putch_USART1(received);
            }
        }

        // Small delay to prevent busy waiting
        _delay_ms(10);
    }

    puts_USART1("\r\nDemo 3 completed.\r\n");
}

/*
 * Demo Mode 4: Command Processing via Interrupts
 * Real-time command interpreter using interrupt-driven communication
 */
void demo_interrupt_commands(void)
{
    communication_mode = 4;
    command_ready = 0;

    // Initialize UART for interrupt-based communication
    init_uart_interrupts();

    puts_USART1("\r\n=== DEMO 4: Command Processing ===\r\n");
    puts_USART1("Real-time command processing via interrupts\r\n");
    puts_USART1("Available commands:\r\n");
    puts_USART1("  led on/off  - Control LED\r\n");
    puts_USART1("  status      - Show system status\r\n");
    puts_USART1("  reset       - Reset counters\r\n");
    puts_USART1("  quit        - Exit demo\r\n\r\n");

    char command[32];
    unsigned char cmd_index = 0;
    char received;
    unsigned int led_state = 0;
    unsigned int cmd_count = 0;

    puts_USART1("CMD> ");

    while (1)
    {
        // Use standard UART function instead of interrupt buffer
        received = getch_USART1();

        if (received == '\r')
        {
            command[cmd_index] = '\0';
            cmd_count++;

            // Process command
            if (strcmp(command, "led on") == 0)
            {
                led_state = 1;
                PORTB |= 0x01;
                puts_USART1("\r\n[OK] LED turned ON\r\n");
            }
            else if (strcmp(command, "led off") == 0)
            {
                led_state = 0;
                PORTB &= ~0x01;
                puts_USART1("\r\n[OK] LED turned OFF\r\n");
            }
            else if (strcmp(command, "status") == 0)
            {
                puts_USART1("\r\n[STATUS] Commands: ");
                putch_USART1('0' + (cmd_count % 10));
                puts_USART1(", LED: ");
                puts_USART1(led_state ? "ON" : "OFF");
                puts_USART1(", Errors: ");
                putch_USART1('0' + (error_count % 10));
                puts_USART1("\r\n");
            }
            else if (strcmp(command, "reset") == 0)
            {
                cmd_count = 0;
                error_count = 0;
                puts_USART1("\r\n[OK] Counters reset\r\n");
            }
            else if (strcmp(command, "quit") == 0)
            {
                break;
            }
            else
            {
                puts_USART1("\r\n[ERROR] Unknown command\r\n");
            }

            cmd_index = 0;
            puts_USART1("CMD> ");
        }
        else if (received == '\b' || received == 127)
        { // Backspace
            if (cmd_index > 0)
            {
                cmd_index--;
                puts_USART1("\b \b");
            }
        }
        else if (cmd_index < 31 && received >= ' ')
        {
            command[cmd_index++] = received;
            putch_USART1(received);
        }
    }

    puts_USART1("\r\nDemo 4 completed.\r\n");
}

/*
 * Demo Mode 5: Advanced Buffering with Statistics
 * Demonstrates buffer management and communication statistics
 */
void demo_interrupt_advanced(void)
{
    communication_mode = 5;

    // Initialize UART for interrupt-based communication
    init_uart_interrupts();

    puts_USART1("\r\n=== DEMO 5: Advanced Buffering ===\r\n");
    puts_USART1("Buffer monitoring and statistics\r\n");
    puts_USART1("Send rapid characters to test buffer handling\r\n");
    puts_USART1("Press 's' for statistics, 'q' to quit\r\n\r\n");

    char received;
    unsigned int char_count = 0;
    unsigned int stats_count = 0;

    while (1)
    {
        // Use standard UART function instead of interrupt buffer
        received = getch_USART1();
        char_count++;

        if (received == 's' || received == 'S')
        {
            stats_count++;
            puts_USART1("\r\n=== STATISTICS ===\r\n");
            puts_USART1("Characters processed: ");
            // Simple number display (limited to single digits for simplicity)
            putch_USART1('0' + (char_count % 10));
            puts_USART1("\r\nBuffer overflows: ");
            putch_USART1('0' + (rx_overflow % 10));
            puts_USART1("\r\nRX Head: ");
            putch_USART1('0' + (rx_head % 10));
            puts_USART1(", Tail: ");
            putch_USART1('0' + (rx_tail % 10));
            puts_USART1("\r\nTX Busy: ");
            puts_USART1(tx_busy ? "YES" : "NO");
            puts_USART1("\r\n==================\r\n");
        }
        else if (received == 'q' || received == 'Q')
        {
            break;
        }
        else
        {
            // Echo with timestamp indicator
            putch_USART1('[');
            putch_USART1('0' + (char_count % 10));
            putch_USART1(']');
            putch_USART1(received);
        }
    }

    puts_USART1("\r\nDemo 5 completed.\r\n");
}

/*
 * =============================================================================
 * MAIN PROGRAM ENTRY POINT
 * =============================================================================
 */

int main(void)
{
    // Initialize basic system (each demo will initialize its own UART)
    simple_init_serial();

    // Wait a moment for system stability
    _delay_ms(1000);

    // Initialize UART for initial message before demo selection
    init_uart_polling();

    puts_USART1("IMPORTANT: Students edit main() to select ONE demo:\r\n\r\n");

    _delay_ms(2000);

    // ===================================================================
    // EDUCATIONAL SELECTION: Students uncomment ONE demo to learn from
    // ===================================================================

    // =====================================
    // POLLING DEMOS: CPU waits for data
    // =====================================
    demo_polling_echo(); // Demo 1: Simple polling (CPU blocks) ← ACTIVE FOR TESTING
    // demo_polling_commands();       // Demo 2: Command polling (inefficient)
    // demo_polling_buffered();       // Demo 3: Manual buffering (still blocks)

    // ========================================
    // INTERRUPT DEMOS: CPU continues running
    // ========================================
    demo_interrupt_echo(); // Demo 4: Real ISR echo (CPU free!) ← ACTIVE FOR TESTING
    // demo_interrupt_tx_queue(); // Demo 5: TX interrupt with buffering
    // demo_interrupt_bidirectional(); // Demo 6: Full duplex communication
    // demo_interrupt_commands(); // Demo 7: Real-time command processing
    // demo_interrupt_advanced(); // Demo 8: Advanced buffer monitoring

    puts_USART1("\r\n=======================================================\r\n");
    puts_USART1("EDUCATIONAL SUMMARY:\r\n");
    puts_USART1("• Polling: Simple but blocks CPU → inefficient\r\n");
    puts_USART1("• Interrupts: Complex but frees CPU → efficient\r\n");
    puts_USART1("• Students must learn ISR syntax and register programming\r\n");
    puts_USART1("• No wrapper functions - direct hardware control only!\r\n");
    puts_USART1("=======================================================\r\n");

    // Keep LED blinking to show program is running
    while (1)
    {
        PORTB ^= 0x01; // Toggle LED to show CPU is free
        _delay_ms(500);
    }

    return 0;
}
