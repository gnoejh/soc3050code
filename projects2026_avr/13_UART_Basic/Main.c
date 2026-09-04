/*
 * ==============================================================================
 * SERIAL COMMUNICATION - DEMO CODE
 * ==============================================================================
 * PROJECT: Serial_Communications
 * See Slide.md for complete UART theory and register details
 *
 * DEMOS - 2×3 Matrix (POLLING vs INTERRUPT):
 * 1. Polling Character Echo - Basic getch/putch
 * 2. Polling Word Echo - Manual buffering, space delimiter
 * 3. Polling Sentence Echo - Line buffering, command parsing
 * 4. Interrupt Character Echo - ISR RX/TX, CPU free
 * 5. Interrupt Word Echo - Circular buffer, ISR buffering
 * 6. Interrupt Sentence Echo - Full duplex ISR, command protocol
 * ==============================================================================
 */

#include "config.h"
#include <avr/pgmspace.h> // For PROGMEM string storage in flash

// Forward declaration for UART functions
void putch_USART1(char c);

// Helper function to send strings from flash memory
void puts_USART1_P(const char *str)
{
    char c;
    while ((c = pgm_read_byte(str++)))
    {
        putch_USART1(c);
    }
}

// Function prototypes for 6 educational demos (2×3 matrix)
void simple_init_serial(void);

// POLLING METHOD DEMOS (Left column: Simple but CPU-blocking)
void demo_polling_char_echo(void);     // Demo 1: Single character echo (polling)
void demo_polling_word_echo(void);     // Demo 2: Word echo with space delimiter (polling)
void demo_polling_sentence_echo(void); // Demo 3: Sentence echo with line delimiter (polling)

// INTERRUPT METHOD DEMOS (Right column: Complex but CPU-efficient)
void demo_interrupt_char_echo(void);     // Demo 4: Single character echo (interrupt)
void demo_interrupt_word_echo(void);     // Demo 5: Word echo with circular buffer (interrupt)
void demo_interrupt_sentence_echo(void); // Demo 6: Sentence echo with full duplex (interrupt)

/*
 * =============================================================================
 * DEMO SELECTION GUIDE - 2×3 PEDAGOGICAL MATRIX
 * =============================================================================
 *
 * USAGE: In main(), uncomment ONE demo to run. Compare across rows or columns:
 *
 * ┌──────────────────────────────────────────────────────────────────┐
 * │                  POLLING (Simple/Blocking)                       │
 * ├──────────────────────────────────────────────────────────────────┤
 * │ Demo 1: demo_polling_char_echo()                                 │
 * │         Single character echo - learn basic getch/putch          │
 * │         Shows how CPU blocks waiting for each character          │
 * │                                                                  │
 * │ Demo 2: demo_polling_word_echo()                                 │
 * │         Word echo (space-delimited) - learn manual buffering     │
 * │         Shows polling with simple parsing logic                  │
 * │                                                                  │
 * │ Demo 3: demo_polling_sentence_echo()                             │
 * │         Sentence echo (line-delimited) - learn protocol design   │
 * │         Shows polling with command processing                    │
 * └──────────────────────────────────────────────────────────────────┘
 *
 * ┌──────────────────────────────────────────────────────────────────┐
 * │                INTERRUPT (Efficient/Non-blocking)                │
 * ├──────────────────────────────────────────────────────────────────┤
 * │ Demo 4: demo_interrupt_char_echo()                               │
 * │         Single character echo - learn ISR programming            │
 * │         Shows how CPU continues other work during RX/TX          │
 * │         Compare with Demo 1 to see efficiency difference!        │
 * │                                                                  │
 * │ Demo 5: demo_interrupt_word_echo()                               │
 * │         Word echo with ISR - learn circular buffer management    │
 * │         Shows interrupt-driven parsing and buffering             │
 * │         Compare with Demo 2 to see CPU freedom!                  │
 * │                                                                  │
 * │ Demo 6: demo_interrupt_sentence_echo()                           │
 * │         Sentence echo with ISR - learn full duplex protocol      │
 * │         Shows advanced ISR with command processing               │
 * │         Compare with Demo 3 to see true non-blocking I/O!        │
 * └──────────────────────────────────────────────────────────────────┘
 *
 * LEARNING STRATEGY:
 * - Horizontal comparison: Compare Demo 1 vs Demo 4 (same granularity)
 * - Vertical progression: Go from Demo 1 → Demo 2 → Demo 3 (increasing complexity)
 * - Full mastery: Understand all 6 demos and their trade-offs
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
    // Step 1: Configure UART Control Register A (U2X=1 for better accuracy)
    UCSR1A = UART_U2X_ENABLE; // U2X=1 for double-speed mode

    // Step 2: Configure character format (8 data bits, No parity, 1 stop bit = 8N1)
    UCSR1C = UART_8BIT_CHAR; // UCSZ11:10 = 11 for 8-bit character size

    // Step 3: Enable transmitter and receiver (NO INTERRUPTS for polling)
    UCSR1B = UART_ENABLE_RX_TX; // RXEN1 and TXEN1

    // Step 4: Calculate and set baud rate
    // Formula with U2X=1: UBRR = (F_CPU / (8 * BAUD)) - 1
    // For 16MHz and 9600 baud: UBRR = (16000000 / (8 * 9600)) - 1 = 207
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
 * =============================================================================
 * DEMO 1: POLLING CHARACTER ECHO
 * =============================================================================
 * DATA GRANULARITY: Single character
 * METHOD: Polling (CPU blocks on each character)
 *
 * LEARNING FOCUS:
 * - Basic UART functions: getch_USART1() and putch_USART1()
 * - Understanding CPU blocking during polling
 * - Simple echo pattern: receive → send back
 *
 * COMPARE WITH: Demo 4 (interrupt character echo) to see CPU efficiency difference
 */
void demo_polling_char_echo(void)
{
    // Initialize UART for polling communication
    init_uart_polling();

    // Store strings in flash memory to save SRAM
    puts_USART1_P(PSTR("\r\n=== DEMO 1: Polling Char Echo ===\r\n"));
    puts_USART1_P(PSTR("Polling: CPU blocks. Type chars, press 'q' to quit.\r\n\r\n"));

    unsigned int char_count = 0;
    char received;

    while (1)
    {
        // EDUCATIONAL POINT: CPU is BLOCKED here waiting for character
        // No other work can be done during this wait!
        received = getch_USART1(); // <-- CPU BLOCKS HERE
        char_count++;

        // Exit condition
        if (received == 'q' || received == 'Q')
        {
            break;
        }

        // Echo character back (also blocks during transmission)
        putch_USART1(received); // <-- CPU BLOCKS HERE TOO

        // Show statistics periodically
        if ((char_count % 10) == 0)
        {
            puts_USART1(" [");
            putch_USART1('0' + (char_count / 10) % 10);
            putch_USART1('0' + char_count % 10);
            puts_USART1(" chars, CPU blocked every time]");
        }
    }

    puts_USART1("\r\n\r\n[DEMO 1 COMPLETE]\r\n");
    puts_USART1("Total characters echoed: ");
    putch_USART1('0' + (char_count / 10) % 10);
    putch_USART1('0' + char_count % 10);
    puts_USART1("\r\nCPU was blocked ");
    putch_USART1('0' + (char_count / 10) % 10);
    putch_USART1('0' + char_count % 10);
    puts_USART1(" times waiting for I/O\r\n");
    puts_USART1("Compare this with Demo 4 (interrupt method)!\r\n\r\n");
}

/*
 * =============================================================================
 * DEMO 2: POLLING WORD ECHO
 * =============================================================================
 * DATA GRANULARITY: Word (space-delimited)
 * METHOD: Polling with manual buffer
 *
 * LEARNING FOCUS:
 * - Manual buffer management for word assembly
 * - Character-by-character polling with parsing
 * - Space as word delimiter
 * - CPU still blocks but now processes complete words
 *
 * COMPARE WITH: Demo 5 (interrupt word echo) to see buffer efficiency
 */
void demo_polling_word_echo(void)
{
    puts_USART1_P(PSTR("\r\n=== DEMO 2: Polling Word Echo ===\r\n"));
    puts_USART1_P(PSTR("Polling: words echo on space. Type 'quit' to exit.\r\n\r\n"));

    char word_buffer[32];
    unsigned char word_index = 0;
    char received;
    unsigned int word_count = 0;

    while (1)
    {
        // POLLING: CPU blocks waiting for each character
        received = getch_USART1(); // <-- CPU BLOCKS

        // Echo the character for user feedback
        putch_USART1(received);

        // Check for word delimiter (space or enter)
        if (received == ' ' || received == '\r' || received == '\n')
        {
            if (word_index > 0)
            {
                word_buffer[word_index] = '\0'; // Null-terminate the word
                word_count++;

                // Check for quit command
                if (strcmp(word_buffer, "quit") == 0)
                {
                    puts_USART1("\r\n[Exiting Demo 2]\r\n");
                    break;
                }

                // Echo the complete word back
                puts_USART1(" → ECHO: [");
                puts_USART1(word_buffer);
                puts_USART1("] ");

                // Show statistics
                if ((word_count % 5) == 0)
                {
                    puts_USART1(" (");
                    putch_USART1('0' + (word_count / 10) % 10);
                    putch_USART1('0' + word_count % 10);
                    puts_USART1(" words, CPU blocked for each char)");
                }

                puts_USART1("\r\n");

                // Reset buffer for next word
                word_index = 0;
            }
        }
        // Handle backspace
        else if (received == '\b' || received == 127)
        {
            if (word_index > 0)
            {
                word_index--;
                puts_USART1(" \b"); // Erase character from screen
            }
        }
        // Add character to word buffer
        else if (word_index < 31 && received >= ' ')
        {
            word_buffer[word_index++] = received;
        }
    }

    puts_USART1("\r\n[DEMO 2 COMPLETE]\r\n");
    puts_USART1("Total words echoed: ");
    putch_USART1('0' + (word_count / 10) % 10);
    putch_USART1('0' + word_count % 10);
    puts_USART1("\r\nCPU blocked on every character, echoed complete words\r\n");
    puts_USART1("Compare this with Demo 5 (interrupt word echo)!\r\n\r\n");
}

/*
 * =============================================================================
 * DEMO 3: POLLING SENTENCE ECHO
 * =============================================================================
 * DATA GRANULARITY: Sentence (line-delimited, multi-word)
 * METHOD: Polling with line buffering
 *
 * LEARNING FOCUS:
 * - Line buffering (collecting until Enter pressed)
 * - Multi-word parsing and protocol design
 * - Complete line echo pattern
 * - CPU blocked but processes full sentences
 *
 * COMPARE WITH: Demo 6 (interrupt sentence echo) for full duplex efficiency
 */
void demo_polling_sentence_echo(void)
{
    puts_USART1_P(PSTR("\r\n=== DEMO 3: Polling Sentence Echo ===\r\n"));
    puts_USART1_P(PSTR("Polling: sentences echo on Enter. Type 'quit' to exit.\r\n\r\n"));

    char line_buffer[64];
    unsigned char line_index = 0;
    char received;
    unsigned int line_count = 0;

    puts_USART1("Type sentence> ");

    while (1)
    {
        // POLLING: CPU blocks waiting for each character
        received = getch_USART1(); // <-- CPU BLOCKS

        // Echo character for user feedback
        putch_USART1(received);

        // Check for line delimiter (Enter key)
        if (received == '\r' || received == '\n')
        {
            if (line_index > 0)
            {
                line_buffer[line_index] = '\0'; // Null-terminate
                line_count++;

                // Check for quit command
                if (strcmp(line_buffer, "quit") == 0)
                {
                    puts_USART1("\r\n[Exiting Demo 3]\r\n");
                    break;
                }

                // Echo the complete sentence back
                puts_USART1("\r\n→ SENTENCE ECHO: \"");
                puts_USART1(line_buffer);
                puts_USART1("\"\r\n");

                // Show statistics every 3 sentences
                if ((line_count % 3) == 0)
                {
                    puts_USART1("   [");
                    putch_USART1('0' + (line_count / 10) % 10);
                    putch_USART1('0' + line_count % 10);
                    puts_USART1(" sentences, ");
                    putch_USART1('0' + (line_index / 10) % 10);
                    putch_USART1('0' + line_index % 10);
                    puts_USART1(" chars, CPU blocked on each]\r\n");
                }

                puts_USART1("Type sentence> ");

                // Reset buffer for next sentence
                line_index = 0;
            }
        }
        // Handle backspace
        else if (received == '\b' || received == 127)
        {
            if (line_index > 0)
            {
                line_index--;
                puts_USART1(" \b"); // Erase character from screen
            }
        }
        // Add character to line buffer
        else if (line_index < 63 && received >= ' ')
        {
            line_buffer[line_index++] = received;
        }
        // Buffer overflow warning
        else if (line_index >= 63)
        {
            puts_USART1("\r\n[BUFFER FULL - Press Enter]\r\n");
        }
    }

    puts_USART1("\r\n[DEMO 3 COMPLETE]\r\n");
    puts_USART1("Total sentences echoed: ");
    putch_USART1('0' + (line_count / 10) % 10);
    putch_USART1('0' + line_count % 10);
    puts_USART1("\r\nCPU blocked for every character, echoed complete sentences\r\n");
    puts_USART1("This is the most common polling pattern for command-line interfaces\r\n");
    puts_USART1("Compare this with Demo 6 (interrupt sentence echo)!\r\n\r\n");
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
 * USART1 Data Register Empty Interrupt
 * This ISR is called when the transmit buffer is ready for next character
 * Students learn: TX interrupts, automatic transmission, buffer management
 */
ISR(USART1_UDRE_vect)
{
    if (tx_head != tx_tail)
    {
        // Send next character from buffer
        UDR1 = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
    }
    else
    {
        // Buffer empty - disable this interrupt
        UCSR1B &= ~(1 << UDRIE1);
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

    // Step 1: Configure UART Control Register A (U2X=1 for better accuracy)
    UCSR1A = UART_U2X_ENABLE; // U2X=1 for double-speed mode

    // Step 2: Configure character format (8 data bits, No parity, 1 stop bit = 8N1)
    UCSR1C = UART_8BIT_CHAR; // UCSZ11:10 = 11 for 8-bit character size

    // Step 3: Enable transmitter and receiver
    UCSR1B = UART_ENABLE_RX_TX; // RXEN1 and TXEN1

    // Step 4: Calculate and set baud rate
    // Formula with U2X=1: UBRR = (F_CPU / (8 * BAUD)) - 1
    // For 16MHz and 9600 baud: UBRR = (16000000 / (8 * 9600)) - 1 = 207
    unsigned int baud_register = UART_BAUD_REGISTER;
    UBRR1H = (baud_register >> 8); // High byte of baud rate register
    UBRR1L = baud_register;        // Low byte of baud rate register

    // Step 5: Allow UART hardware to stabilize (CRITICAL for first character)
    _delay_ms(10); // Small delay for UART register stabilization

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
        UCSR1B |= (1 << UDRIE1);
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
 * =============================================================================
 * DEMO 4: INTERRUPT CHARACTER ECHO
 * =============================================================================
 * DATA GRANULARITY: Single character
 * METHOD: Interrupt-driven with ISR (CPU non-blocking)
 *
 * LEARNING FOCUS:
 * - Real ISR programming: ISR(USART1_RX_vect) and ISR(USART1_UDRE_vect)
 * - Circular buffer for RX and TX
 * - CPU continues other work while ISRs handle I/O
 * - Volatile variables for ISR-main communication
 *
 * COMPARE WITH: Demo 1 (polling character echo) to see CPU freedom!
 */
void demo_interrupt_char_echo(void)
{
    // EDUCATIONAL: Initialize interrupt-based UART (see ISRs above!)
    init_uart_interrupts();

    // Send initial messages using polling (before interrupts fully active)
    puts_USART1_P(PSTR("\r\n=== DEMO 4: Interrupt Char Echo ===\r\n"));
    puts_USART1_P(PSTR("Interrupt: CPU free! ISRs handle I/O. Press 'q' to quit.\r\n\r\n"));

    _delay_ms(100); // Let initial messages complete

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

            // Echo the character back using interrupt-driven TX
            while (!send_char_interrupt(received))
                ; // Wait if TX buffer full

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
    puts_USART1("Key Learning: CPU was free to count and toggle LEDs while ISRs handled all serial data!\r\n");
    puts_USART1("Compare this efficiency with polling demos above.\r\n");
}

/*
 * =============================================================================
 * DEMO 5: INTERRUPT WORD ECHO
 * =============================================================================
 * DATA GRANULARITY: Word (space-delimited)
 * METHOD: Interrupt-driven with ISR circular buffer
 *
 * LEARNING FOCUS:
 * - Word assembly in ISR buffer
 * - Space delimiter parsing with interrupts
 * - Non-blocking word collection
 * - Compare efficiency with Demo 2 (polling word echo)
 *
 * COMPARE WITH: Demo 2 (polling word echo) to see buffer management difference
 */
void demo_interrupt_word_echo(void)
{
    // Initialize UART for interrupt-based communication
    init_uart_interrupts();

    puts_USART1_P(PSTR("\r\n=== DEMO 5: Interrupt Word Echo ===\r\n"));
    puts_USART1_P(PSTR("Interrupt: words via ISR. Type 'quit' to exit.\r\n\r\n"));

    _delay_ms(100); // Let messages transmit

    char word_buffer[32];
    unsigned char word_index = 0;
    char received;
    unsigned int word_count = 0;
    unsigned int cpu_counter = 0;

    while (1)
    {
        // EDUCATIONAL: CPU is FREE to do other work!
        cpu_counter++;
        if ((cpu_counter % 50000) == 0)
        {
            // This proves CPU is not blocked!
            // In polling, this would never execute while waiting for input
        }

        // Check ISR buffer (non-blocking!)
        if (chars_available())
        {
            received = get_char_from_buffer();

            // Echo character back via ISR
            send_char_interrupt(received);

            // Word delimiter check
            if (received == ' ' || received == '\r' || received == '\n')
            {
                if (word_index > 0)
                {
                    word_buffer[word_index] = '\0';
                    word_count++;

                    // Check quit command
                    if (strcmp(word_buffer, "quit") == 0)
                    {
                        send_string_interrupt("\r\n[Exiting Demo 5]\r\n");
                        break;
                    }

                    // Echo complete word
                    send_string_interrupt(" → ECHO: [");
                    send_string_interrupt(word_buffer);
                    send_string_interrupt("]");

                    if ((word_count % 5) == 0)
                    {
                        send_string_interrupt(" (");
                        send_char_interrupt('0' + (word_count / 10) % 10);
                        send_char_interrupt('0' + word_count % 10);
                        send_string_interrupt(" words, CPU was FREE!)");
                    }
                    send_string_interrupt("\r\n");

                    word_index = 0;
                }
            }
            // Backspace
            else if (received == '\b' || received == 127)
            {
                if (word_index > 0)
                {
                    word_index--;
                    send_string_interrupt(" \b");
                }
            }
            // Add to buffer
            else if (word_index < 31 && received >= ' ')
            {
                word_buffer[word_index++] = received;
            }
        }

        // Small delay (CPU still free during this!)
        _delay_ms(5);
    }

    send_string_interrupt("\r\n[DEMO 5 COMPLETE]\r\n");
    send_string_interrupt("Words echoed: ");
    send_char_interrupt('0' + (word_count / 10) % 10);
    send_char_interrupt('0' + word_count % 10);
    send_string_interrupt("\r\nISRs handled ALL I/O, CPU was free!\r\n");
    send_string_interrupt("Compare with Demo 2 (polling word echo)!\r\n\r\n");
}

/*
 * =============================================================================
 * DEMO 6: INTERRUPT SENTENCE ECHO
 * =============================================================================
 * DATA GRANULARITY: Sentence (line-delimited, multi-word)
 * METHOD: Interrupt full duplex with command protocol
 *
 * LEARNING FOCUS:
 * - Full duplex ISR communication
 * - Line buffering with interrupts
 * - Command protocol design with ISR
 * - Complete non-blocking sentence processing
 *
 * COMPARE WITH: Demo 3 (polling sentence echo) for maximum efficiency gain
 */
void demo_interrupt_sentence_echo(void)
{
    // Initialize UART for interrupt-based communication
    init_uart_interrupts();

    puts_USART1_P(PSTR("\r\n=== DEMO 6: Interrupt Sentence Echo ===\r\n"));
    puts_USART1_P(PSTR("Interrupt: sentences via ISR. Type 'quit' to exit.\r\n\r\n"));

    _delay_ms(100);

    char line_buffer[64];
    unsigned char line_index = 0;
    char received;
    unsigned int line_count = 0;
    unsigned int cpu_counter = 0;

    send_string_interrupt("Type sentence> ");

    while (1)
    {
        // EDUCATIONAL: CPU is FREE to do other work!
        cpu_counter++;
        if ((cpu_counter % 50000) == 0)
        {
            // CPU continues working while ISRs handle I/O
        }

        // Check ISR buffer (non-blocking!)
        if (chars_available())
        {
            received = get_char_from_buffer();

            // Echo via ISR
            send_char_interrupt(received);

            // Line delimiter check
            if (received == '\r' || received == '\n')
            {
                if (line_index > 0)
                {
                    line_buffer[line_index] = '\0';
                    line_count++;

                    // Check quit
                    if (strcmp(line_buffer, "quit") == 0)
                    {
                        send_string_interrupt("\r\n[Exiting Demo 6]\r\n");
                        break;
                    }

                    // Echo complete sentence
                    send_string_interrupt("\r\n→ SENTENCE ECHO: \"");
                    send_string_interrupt(line_buffer);
                    send_string_interrupt("\"\r\n");

                    if ((line_count % 3) == 0)
                    {
                        send_string_interrupt("   [");
                        send_char_interrupt('0' + (line_count / 10) % 10);
                        send_char_interrupt('0' + line_count % 10);
                        send_string_interrupt(" sentences, CPU was FREE!]\r\n");
                    }

                    send_string_interrupt("Type sentence> ");
                    line_index = 0;
                }
            }
            // Backspace
            else if (received == '\b' || received == 127)
            {
                if (line_index > 0)
                {
                    line_index--;
                    send_string_interrupt(" \b");
                }
            }
            // Add to buffer
            else if (line_index < 63 && received >= ' ')
            {
                line_buffer[line_index++] = received;
            }
            // Buffer full
            else if (line_index >= 63)
            {
                send_string_interrupt("\r\n[BUFFER FULL - Press Enter]\r\n");
            }
        }

        // Small delay (CPU still free!)
        _delay_ms(5);
    }

    send_string_interrupt("\r\n[DEMO 6 COMPLETE]\r\n");
    send_string_interrupt("Sentences echoed: ");
    send_char_interrupt('0' + (line_count / 10) % 10);
    send_char_interrupt('0' + line_count % 10);
    send_string_interrupt("\r\nFull duplex ISR: Maximum efficiency!\r\n");
    send_string_interrupt("Compare with Demo 3 (polling sentence)!\r\n\r\n");
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

    puts_USART1_P(PSTR("\r\n=== SERIAL COMMUNICATION - 2x3 MATRIX ===\r\n"));
    puts_USART1_P(PSTR("Edit main() to uncomment ONE demo.\r\n\r\n"));

    _delay_ms(1000);

    // ====================================================================
    // 2×3 EDUCATIONAL MATRIX: Select ONE demo to run
    // ====================================================================
    //
    // ┌──────────────────────────────────────────────────────────────┐
    // │ POLLING METHOD (Simple, CPU blocks)                         │
    // ├──────────────────────────────────────────────────────────────┤
    // │ Demo 1: Character Echo    → demo_polling_char_echo()        │
    // │ Demo 2: Word Echo          → demo_polling_word_echo()        │
    // │ Demo 3: Sentence Echo      → demo_polling_sentence_echo()    │
    // └──────────────────────────────────────────────────────────────┘
    //
    // ┌──────────────────────────────────────────────────────────────┐
    // │ INTERRUPT METHOD (Complex, CPU free)                        │
    // ├──────────────────────────────────────────────────────────────┤
    // │ Demo 4: Character Echo ISR → demo_interrupt_char_echo()      │
    // │ Demo 5: Word Echo ISR      → demo_interrupt_word_echo()      │
    // │ Demo 6: Sentence Echo ISR  → demo_interrupt_sentence_echo()  │
    // └──────────────────────────────────────────────────────────────┘

    // ========== POLLING TRACK (Character → Word → Sentence) ==========
    // demo_polling_char_echo(); // Demo 1: Polling character echo ← ACTIVE
    // demo_polling_word_echo();      // Demo 2: Polling word echo
    // demo_polling_sentence_echo();  // Demo 3: Polling sentence echo

    // ======== INTERRUPT TRACK (Character → Word → Sentence) =========
    // demo_interrupt_char_echo(); // Demo 4: Interrupt character echo
    // demo_interrupt_word_echo(); // Demo 5: Interrupt word echo (NEW!)
    demo_interrupt_sentence_echo(); // Demo 6: Interrupt sentence echo

    puts_USART1_P(PSTR("\r\n=== SUMMARY ===\r\n"));
    puts_USART1_P(PSTR("Polling: Simple but blocks CPU\r\n"));
    puts_USART1_P(PSTR("Interrupt: Complex but CPU-efficient\r\n"));
    puts_USART1_P(PSTR("Learn 1-3 first, then 4-6. Compare pairs.\r\n"));

    // Keep LED blinking to show program is running
    while (1)
    {
        PORTB ^= 0x01; // Toggle LED to show CPU is free
        _delay_ms(500);
    }

    return 0;
}
