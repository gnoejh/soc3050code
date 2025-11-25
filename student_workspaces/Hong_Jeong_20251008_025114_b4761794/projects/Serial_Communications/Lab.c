/*
 * =============================================================================
 * SERIAL COMMUNICATION METHODS - HANDS-ON LAB EXERCISES
 * =============================================================================
 *
 * PROJECT: Serial_Communications Lab
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Hands-on laboratory exercises for students to practice and implement
 * both polling and interrupt-based serial communication methods.
 * Students will complete progressively challenging tasks to master both approaches.
 *
 * LAB OBJECTIVES:
 * 1. Compare polling vs interrupt communication methods
 * 2. Implement basic and advanced interrupt service routines
 * 3. Create buffered communication systems
 * 4. Build command processing applications
 * 5. Design real-time embedded communication systems
 *
 * LAB STRUCTURE:
 * - Exercise 1: Polling Communication Basics
 * - Exercise 2: Basic RX Interrupt Implementation
 * - Exercise 3: TX Buffer and Interrupt System
 * - Exercise 4: Advanced Command Parser
 * - Exercise 5: Real-time Data Logger
 * - Exercise 6: Performance Comparison Analysis
 *
 * INSTRUCTIONS:
 * Complete each exercise by filling in the TODO sections.
 * Test your implementation before moving to the next exercise.
 * Use the provided helper functions and follow the coding style.
 *
 * =============================================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include "../../shared_libs/_init.h"
#include "../../shared_libs/_uart.h"
#include "../../shared_libs/_port.h"

/*
 * =============================================================================
 * LAB CONFIGURATION AND GLOBAL VARIABLES
 * =============================================================================
 */

// Buffer sizes for lab exercises
#define LAB_RX_BUFFER_SIZE 32
#define LAB_TX_BUFFER_SIZE 32
#define LAB_CMD_BUFFER_SIZE 16

// Lab global variables (students will work with these)
volatile char lab_rx_buffer[LAB_RX_BUFFER_SIZE];
volatile unsigned char lab_rx_head = 0;
volatile unsigned char lab_rx_tail = 0;
volatile unsigned char lab_rx_count = 0;

volatile char lab_tx_buffer[LAB_TX_BUFFER_SIZE];
volatile unsigned char lab_tx_head = 0;
volatile unsigned char lab_tx_tail = 0;
volatile unsigned char lab_tx_busy = 0;

// Command processing variables
volatile char lab_command[LAB_CMD_BUFFER_SIZE];
volatile unsigned char lab_cmd_ready = 0;
volatile unsigned char lab_cmd_length = 0;

// Lab status and statistics
volatile unsigned int lab_char_count = 0;
volatile unsigned int lab_cmd_count = 0;
volatile unsigned char lab_errors = 0;

// Exercise selection
unsigned char current_exercise = 0;

/*
 * =============================================================================
 * HELPER FUNCTIONS (PROVIDED FOR STUDENTS)
 * =============================================================================
 */

/*
 * Initialize UART for lab exercises
 */
void lab_init_uart(void)
{
    Uart1_init();
    // Students will add interrupt enable code in exercises
    sei();
}

/*
 * Simple delay for lab exercises
 */
void lab_delay_ms(unsigned int ms)
{
    while (ms--)
    {
        _delay_ms(1);
    }
}

/*
 * Helper function to send a string (polling mode for setup messages)
 */
void lab_send_string(const char *str)
{
    puts_USART1((char *)str);
}

/*
 * Helper function to send a character (polling mode for setup messages)
 */
void lab_send_char(char c)
{
    putch_USART1(c);
}

/*
 * Convert single digit to character
 */
char lab_digit_to_char(unsigned char digit)
{
    return '0' + (digit % 10);
}

/*
 * =============================================================================
 * EXERCISE 1: BASIC RX INTERRUPT (STARTER CODE PROVIDED)
 * =============================================================================
 * OBJECTIVE: Understand basic interrupt service routine implementation
 * TASK: Complete the RX interrupt service routine to echo characters
 */

/*
 * TODO for Students: Complete this RX interrupt service routine
 *
 * REQUIREMENTS:
 * 1. Read character from UDR1
 * 2. Echo the character back immediately
 * 3. Count total characters received
 * 4. Handle special case: if 'q' is received, set a flag
 */
ISR(USART1_RX_vect)
{
    /* STUDENT TODO: Read the received character */
    char received_char = UDR1; // This line is provided as an example

    /* STUDENT TODO: Increment character counter */
    lab_char_count++;

    /* STUDENT TODO: Echo the character back
     * HINT: Check if UDRE1 bit is set in UCSR1A, then write to UDR1 */
    while (!(UCSR1A & (1 << UDRE1)))
        ;                 // Wait for transmit buffer ready
    UDR1 = received_char; // Echo back

    /* STUDENT TODO: Handle special commands */
    if (received_char == 'q' || received_char == 'Q')
    {
        // Set a flag or perform special action
        lab_cmd_ready = 1;
    }

    /* BONUS TODO: Add a visual indicator (LED toggle) when character received */
    // HINT: Toggle PORTB bit 0
    PORTB ^= 0x01;
}

/*
 * Exercise 1 main function
 */
void lab_exercise_1_basic_rx_interrupt(void)
{
    lab_send_string("\r\n=== LAB EXERCISE 1: Basic RX Interrupt ===\r\n");
    lab_send_string("OBJECTIVE: Implement basic RX interrupt with echo\r\n");
    lab_send_string("\r\nYOUR TASK:\r\n");
    lab_send_string("1. Complete the ISR(USART1_RX_vect) function above\r\n");
    lab_send_string("2. Make it echo received characters\r\n");
    lab_send_string("3. Count total characters received\r\n");
    lab_send_string("4. Handle 'q' as quit command\r\n");
    lab_send_string("\r\nTEST: Type characters and they should echo back\r\n");
    lab_send_string("Press 'q' to finish this exercise\r\n\r\n");

    /* STUDENT TODO: Enable RX interrupt */
    // HINT: Set RXCIE1 bit in UCSR1B register
    UCSR1B |= (1 << RXCIE1);

    lab_cmd_ready = 0;
    lab_char_count = 0;

    // Main loop - wait for quit command
    while (!lab_cmd_ready)
    {
        lab_delay_ms(100);

        // Optional: Display character count every few seconds
        // Students can implement this as bonus
    }

    lab_send_string("\r\n\r\nEXERCISE 1 COMPLETED!\r\n");
    lab_send_string("Characters processed: ");
    lab_send_char(lab_digit_to_char(lab_char_count));
    lab_send_string("\r\n");
}

/*
 * =============================================================================
 * EXERCISE 2: TX BUFFER IMPLEMENTATION
 * =============================================================================
 * OBJECTIVE: Implement interrupt-driven transmission with buffering
 * TASK: Complete the TX interrupt and buffer management functions
 */

/*
 * TODO for Students: Complete this TX interrupt service routine
 *
 * REQUIREMENTS:
 * 1. Check if there are characters in the TX buffer
 * 2. Send the next character from buffer
 * 3. Update buffer tail pointer
 * 4. Disable interrupt when buffer is empty
 */
ISR(USART1_UDRE_vect)
{
    /* STUDENT TODO: Implement TX interrupt */

    // Check if buffer has data
    if (lab_tx_head != lab_tx_tail)
    {
        /* STUDENT TODO:
         * 1. Send character from lab_tx_buffer[lab_tx_tail]
         * 2. Increment lab_tx_tail (with wraparound)
         * 3. Update lab_tx_tail = (lab_tx_tail + 1) % LAB_TX_BUFFER_SIZE
         */

        // Your code here:
        UDR1 = lab_tx_buffer[lab_tx_tail];
        lab_tx_tail = (lab_tx_tail + 1) % LAB_TX_BUFFER_SIZE;
    }
    else
    {
        /* STUDENT TODO:
         * 1. Disable UDRE interrupt (clear UDRIE1 bit in UCSR1B)
         * 2. Set lab_tx_busy = 0
         */

        // Your code here:
        UCSR1B &= ~(1 << UDRIE1);
        lab_tx_busy = 0;
    }
}

/*
 * TODO for Students: Complete this function to add character to TX buffer
 *
 * PARAMETERS: c - character to send
 * RETURNS: 1 if successful, 0 if buffer full
 */
unsigned char lab_send_char_buffered(char c)
{
    /* STUDENT TODO: Implement buffered character transmission */

    // Calculate next head position
    unsigned char next_head = (lab_tx_head + 1) % LAB_TX_BUFFER_SIZE;

    /* STUDENT TODO:
     * 1. Check if buffer is full (next_head == lab_tx_tail)
     * 2. If full, return 0
     * 3. If not full:
     *    - Add character to lab_tx_buffer[lab_tx_head]
     *    - Update lab_tx_head = next_head
     *    - If not busy, start transmission (enable UDRIE1, set lab_tx_busy = 1)
     * 4. Return 1 for success
     */

    // Your code here:
    if (next_head == lab_tx_tail)
    {
        return 0; // Buffer full
    }

    lab_tx_buffer[lab_tx_head] = c;
    lab_tx_head = next_head;

    if (!lab_tx_busy)
    {
        lab_tx_busy = 1;
        UCSR1B |= (1 << UDRIE1);
    }

    return 1; // Success
}

/*
 * TODO for Students: Complete this function to send string using buffer
 */
void lab_send_string_buffered(const char *str)
{
    /* STUDENT TODO: Send each character of string using lab_send_char_buffered() */

    // Your code here:
    while (*str)
    {
        while (!lab_send_char_buffered(*str))
        {
            // Wait if buffer full
            lab_delay_ms(1);
        }
        str++;
    }
}

/*
 * Exercise 2 main function
 */
void lab_exercise_2_tx_buffer(void)
{
    lab_send_string("\r\n=== LAB EXERCISE 2: TX Buffer Implementation ===\r\n");
    lab_send_string("OBJECTIVE: Implement interrupt-driven transmission\r\n");
    lab_send_string("\r\nYOUR TASKS:\r\n");
    lab_send_string("1. Complete ISR(USART1_UDRE_vect) function\r\n");
    lab_send_string("2. Complete lab_send_char_buffered() function\r\n");
    lab_send_string("3. Complete lab_send_string_buffered() function\r\n");
    lab_send_string("\r\nTEST: Multiple messages will be sent rapidly\r\n");
    lab_send_string("All should appear smoothly without blocking\r\n\r\n");

    // Initialize TX buffer
    lab_tx_head = lab_tx_tail = 0;
    lab_tx_busy = 0;

    lab_delay_ms(2000);

    // Test the buffered transmission
    lab_send_string_buffered("Testing buffered TX...\r\n");
    lab_send_string_buffered("Message 1: Quick transmission\r\n");
    lab_send_string_buffered("Message 2: Non-blocking send\r\n");
    lab_send_string_buffered("Message 3: Buffer management\r\n");
    lab_send_string_buffered("Message 4: Interrupt-driven\r\n");
    lab_send_string_buffered("Message 5: All messages queued!\r\n");

    // Wait for all transmission to complete
    while (lab_tx_busy)
    {
        lab_delay_ms(10);
    }

    lab_send_string("\r\nEXERCISE 2 COMPLETED!\r\n");
    lab_send_string("Press any key to continue...\r\n");
    getch_USART1();
}

/*
 * =============================================================================
 * EXERCISE 3: COMMAND PARSER IMPLEMENTATION
 * =============================================================================
 * OBJECTIVE: Build a real-time command processing system
 * TASK: Implement command parsing and execution
 */

/*
 * TODO for Students: Modify the RX ISR to build commands
 * This will replace the Exercise 1 ISR when Exercise 3 is active
 */
void lab_exercise_3_rx_isr_command_mode(void)
{
    /* This function shows what students should implement in the RX ISR for Exercise 3 */

    /* STUDENT TODO: In the actual RX ISR, add command building logic:
     *
     * 1. Read character from UDR1
     * 2. If character is '\r' (Enter):
     *    - Null-terminate the command
     *    - Set lab_cmd_ready = 1
     *    - Reset lab_cmd_length = 0
     * 3. If character is printable and lab_cmd_length < LAB_CMD_BUFFER_SIZE-1:
     *    - Add to lab_command[lab_cmd_length]
     *    - Increment lab_cmd_length
     * 4. Echo the character back
     */
}

/*
 * TODO for Students: Complete this function to process commands
 */
void lab_process_command(void)
{
    /* STUDENT TODO: Implement command processing */

    if (!lab_cmd_ready)
        return;

    lab_cmd_count++;

    /* STUDENT TODO: Process different commands
     *
     * Commands to implement:
     * "led on"    - Turn on LED (set PORTB bit 0)
     * "led off"   - Turn off LED (clear PORTB bit 0)
     * "status"    - Show system status
     * "count"     - Show command count
     * "reset"     - Reset counters
     * "help"      - Show available commands
     *
     * Use strcmp() to compare lab_command with command strings
     * Use lab_send_string_buffered() to send responses
     */

    // Your code here:
    if (strcmp((char *)lab_command, "led on") == 0)
    {
        PORTB |= 0x01;
        lab_send_string_buffered("[OK] LED ON\r\n");
    }
    else if (strcmp((char *)lab_command, "led off") == 0)
    {
        PORTB &= ~0x01;
        lab_send_string_buffered("[OK] LED OFF\r\n");
    }
    else if (strcmp((char *)lab_command, "status") == 0)
    {
        lab_send_string_buffered("[STATUS] Commands: ");
        lab_send_char_buffered(lab_digit_to_char(lab_cmd_count));
        lab_send_string_buffered(", Chars: ");
        lab_send_char_buffered(lab_digit_to_char(lab_char_count));
        lab_send_string_buffered("\r\n");
    }
    else if (strcmp((char *)lab_command, "count") == 0)
    {
        lab_send_string_buffered("[COUNT] ");
        lab_send_char_buffered(lab_digit_to_char(lab_cmd_count));
        lab_send_string_buffered(" commands processed\r\n");
    }
    else if (strcmp((char *)lab_command, "reset") == 0)
    {
        lab_cmd_count = 0;
        lab_char_count = 0;
        lab_send_string_buffered("[OK] Counters reset\r\n");
    }
    else if (strcmp((char *)lab_command, "help") == 0)
    {
        lab_send_string_buffered("[HELP] Commands: led on/off, status, count, reset, help, quit\r\n");
    }
    else if (strcmp((char *)lab_command, "quit") == 0)
    {
        lab_cmd_ready = 2; // Special value to exit
        return;
    }
    else
    {
        lab_send_string_buffered("[ERROR] Unknown command\r\n");
    }

    lab_cmd_ready = 0; // Clear command ready flag
}

/*
 * Exercise 3 main function
 */
void lab_exercise_3_command_parser(void)
{
    lab_send_string("\r\n=== LAB EXERCISE 3: Command Parser ===\r\n");
    lab_send_string("OBJECTIVE: Build real-time command processing\r\n");
    lab_send_string("\r\nYOUR TASKS:\r\n");
    lab_send_string("1. Modify RX ISR to build command strings\r\n");
    lab_send_string("2. Complete lab_process_command() function\r\n");
    lab_send_string("3. Implement command parsing and responses\r\n");
    lab_send_string("\r\nAVAILABLE COMMANDS:\r\n");
    lab_send_string("  led on/off - Control LED\r\n");
    lab_send_string("  status     - Show system status\r\n");
    lab_send_string("  count      - Show command count\r\n");
    lab_send_string("  reset      - Reset counters\r\n");
    lab_send_string("  help       - Show this help\r\n");
    lab_send_string("  quit       - Exit exercise\r\n");
    lab_send_string("\r\nType commands and press Enter:\r\n");
    lab_send_string("CMD> ");

    // Initialize command processing
    lab_cmd_ready = 0;
    lab_cmd_length = 0;
    lab_cmd_count = 0;

    // Main command processing loop
    while (lab_cmd_ready != 2)
    { // 2 means quit command
        lab_process_command();

        if (lab_cmd_ready == 0)
        {
            // Command was processed, show prompt again
            lab_send_string_buffered("CMD> ");
        }

        lab_delay_ms(10);
    }

    lab_send_string_buffered("\r\nEXERCISE 3 COMPLETED!\r\n");
}

/*
 * =============================================================================
 * EXERCISE 4: DATA LOGGER IMPLEMENTATION
 * =============================================================================
 * OBJECTIVE: Create a simple data logging system
 * TASK: Log received data with timestamps and statistics
 */

// Data logging variables
volatile unsigned int lab_data_log[16]; // Simple data log
volatile unsigned char lab_log_index = 0;
volatile unsigned int lab_timestamp = 0;

/*
 * TODO for Students: Complete this data logging function
 */
void lab_log_data(unsigned char data)
{
    /* STUDENT TODO: Implement data logging
     *
     * REQUIREMENTS:
     * 1. Store data in lab_data_log array
     * 2. Include timestamp information
     * 3. Handle array wraparound
     * 4. Update lab_log_index
     */

    // Your code here:
    lab_data_log[lab_log_index] = (lab_timestamp << 8) | data;
    lab_log_index = (lab_log_index + 1) % 16;
    lab_timestamp++;
}

/*
 * TODO for Students: Complete this function to display logged data
 */
void lab_show_data_log(void)
{
    /* STUDENT TODO: Display the data log
     *
     * Show the last 16 entries with their timestamps
     * Format: "[timestamp] data_value"
     */

    lab_send_string_buffered("\r\n=== DATA LOG ===\r\n");

    // Your code here:
    for (int i = 0; i < 16; i++)
    {
        unsigned char idx = (lab_log_index + i) % 16;
        unsigned int entry = lab_data_log[idx];
        unsigned char data = entry & 0xFF;
        unsigned char time = (entry >> 8) & 0xFF;

        lab_send_string_buffered("[");
        lab_send_char_buffered(lab_digit_to_char(time));
        lab_send_string_buffered("] ");
        lab_send_char_buffered(data);
        lab_send_string_buffered("\r\n");
    }

    lab_send_string_buffered("================\r\n");
}

/*
 * Exercise 4 main function
 */
void lab_exercise_4_data_logger(void)
{
    lab_send_string("\r\n=== LAB EXERCISE 4: Data Logger ===\r\n");
    lab_send_string("OBJECTIVE: Create interrupt-driven data logging\r\n");
    lab_send_string("\r\nYOUR TASKS:\r\n");
    lab_send_string("1. Complete lab_log_data() function\r\n");
    lab_send_string("2. Complete lab_show_data_log() function\r\n");
    lab_send_string("3. Modify RX ISR to log all received data\r\n");
    lab_send_string("\r\nTEST: Type characters, press 's' to show log\r\n");
    lab_send_string("Press 'q' to finish this exercise\r\n\r\n");

    // Initialize data logging
    lab_log_index = 0;
    lab_timestamp = 0;
    lab_cmd_ready = 0;

    char received;
    while (!lab_cmd_ready)
    {
        // Simulate receiving data (students should modify RX ISR instead)
        if (chars_available())
        {                                      // This function would need to be implemented
            received = get_char_from_buffer(); // This function would need to be implemented

            lab_log_data(received);

            if (received == 's' || received == 'S')
            {
                lab_show_data_log();
            }
            else if (received == 'q' || received == 'Q')
            {
                lab_cmd_ready = 1;
            }
            else
            {
                // Echo the character
                lab_send_char_buffered(received);
            }
        }

        lab_delay_ms(10);
    }

    lab_send_string_buffered("\r\nFinal data log:\r\n");
    lab_show_data_log();
    lab_send_string_buffered("EXERCISE 4 COMPLETED!\r\n");
}

/*
 * =============================================================================
 * EXERCISE 5: MINI TERMINAL (CHALLENGE EXERCISE)
 * =============================================================================
 * OBJECTIVE: Create a complete mini terminal system
 * TASK: Combine all previous concepts into a functional terminal
 */

/*
 * TODO for Students: Complete this challenge exercise
 * This is the most advanced exercise combining all previous concepts
 */
void lab_exercise_5_mini_terminal(void)
{
    lab_send_string("\r\n=== LAB EXERCISE 5: Mini Terminal (Challenge) ===\r\n");
    lab_send_string("OBJECTIVE: Create a complete terminal system\r\n");
    lab_send_string("\r\nYOUR CHALLENGE:\r\n");
    lab_send_string("Combine concepts from all previous exercises to create\r\n");
    lab_send_string("a fully functional mini terminal with:\r\n");
    lab_send_string("1. Command history\r\n");
    lab_send_string("2. Backspace support\r\n");
    lab_send_string("3. Multiple command types\r\n");
    lab_send_string("4. Error handling\r\n");
    lab_send_string("5. Status displays\r\n");
    lab_send_string("\r\nThis is your chance to be creative!\r\n");
    lab_send_string("Press any key to start...\r\n");

    getch_USART1();

    /* STUDENT TODO: Implement complete mini terminal */

    lab_send_string("CHALLENGE EXERCISE 5 COMPLETED!\r\n");
    lab_send_string("Congratulations on completing all lab exercises!\r\n");
}

/*
 * =============================================================================
 * LAB MENU SYSTEM
 * =============================================================================
 */

/*
 * Display lab menu
 */
void lab_show_menu(void)
{
    lab_send_string("\r\n");
    lab_send_string("==========================================\r\n");
    lab_send_string("  SERIAL INTERRUPT COMMUNICATION LAB\r\n");
    lab_send_string("==========================================\r\n");
    lab_send_string("Hands-on exercises for interrupt-driven\r\n");
    lab_send_string("serial communication programming\r\n");
    lab_send_string("\r\n");
    lab_send_string("Select lab exercise:\r\n");
    lab_send_string("  1 - Basic RX Interrupt (Starter)\r\n");
    lab_send_string("  2 - TX Buffer Implementation\r\n");
    lab_send_string("  3 - Command Parser\r\n");
    lab_send_string("  4 - Data Logger\r\n");
    lab_send_string("  5 - Mini Terminal (Challenge)\r\n");
    lab_send_string("  h - Help and Instructions\r\n");
    lab_send_string("  q - Quit Lab\r\n");
    lab_send_string("\r\n");
    lab_send_string("Enter choice: ");
}

/*
 * Display lab help
 */
void lab_show_help(void)
{
    lab_send_string("\r\n=== LAB INSTRUCTIONS ===\r\n");
    lab_send_string("\r\n");
    lab_send_string("GETTING STARTED:\r\n");
    lab_send_string("1. Read the exercise objectives carefully\r\n");
    lab_send_string("2. Find the TODO sections in the code\r\n");
    lab_send_string("3. Complete the required functions\r\n");
    lab_send_string("4. Test your implementation\r\n");
    lab_send_string("5. Move to the next exercise\r\n");
    lab_send_string("\r\n");
    lab_send_string("CODING GUIDELINES:\r\n");
    lab_send_string("- Follow the provided coding style\r\n");
    lab_send_string("- Use the helper functions provided\r\n");
    lab_send_string("- Test each function before proceeding\r\n");
    lab_send_string("- Ask for help if you get stuck\r\n");
    lab_send_string("\r\n");
    lab_send_string("DEBUGGING TIPS:\r\n");
    lab_send_string("- Use the LED for visual feedback\r\n");
    lab_send_string("- Add debug messages to understand flow\r\n");
    lab_send_string("- Check interrupt enable bits\r\n");
    lab_send_string("- Verify buffer management logic\r\n");
    lab_send_string("\r\n");
    lab_send_string("Press any key to continue...\r\n");

    getch_USART1();
}

/*
 * =============================================================================
 * MAIN LAB PROGRAM
 * =============================================================================
 */

int main(void)
{
    char choice;

    // Initialize system
    init_devices();
    lab_init_uart();

    // Configure LED pin for lab exercises
    DDRB |= 0x01;   // PB0 as output for LED
    PORTB &= ~0x01; // LED off initially

    lab_send_string("\r\n\r\n");
    lab_send_string("===========================================\r\n");
    lab_send_string("  ATmega128 Serial Interrupt Lab\r\n");
    lab_send_string("  SOC 3050 - Embedded Systems and IoT\r\n");
    lab_send_string("===========================================\r\n");
    lab_send_string("Welcome to the hands-on lab exercises!\r\n");
    lab_send_string("Complete each exercise to master interrupt\r\n");
    lab_send_string("driven serial communication.\r\n");

    lab_delay_ms(2000);

    while (1)
    {
        lab_show_menu();
        choice = getch_USART1();
        lab_send_char(choice); // Echo choice

        switch (choice)
        {
        case '1':
            current_exercise = 1;
            lab_exercise_1_basic_rx_interrupt();
            break;

        case '2':
            current_exercise = 2;
            lab_exercise_2_tx_buffer();
            break;

        case '3':
            current_exercise = 3;
            lab_exercise_3_command_parser();
            break;

        case '4':
            current_exercise = 4;
            lab_exercise_4_data_logger();
            break;

        case '5':
            current_exercise = 5;
            lab_exercise_5_mini_terminal();
            break;

        case 'h':
        case 'H':
            lab_show_help();
            break;

        case 'q':
        case 'Q':
            lab_send_string("\r\n\r\nExiting Serial Interrupt Lab.\r\n");
            lab_send_string("Great work completing the exercises!\r\n");
            lab_send_string("You've mastered interrupt-driven communication!\r\n");
            return 0;

        default:
            lab_send_string("\r\n\r\nInvalid choice. Please try again.\r\n");
            lab_delay_ms(1000);
            break;
        }

        lab_send_string("\r\nPress any key to return to lab menu...\r\n");
        getch_USART1();
    }

    return 0;
}
