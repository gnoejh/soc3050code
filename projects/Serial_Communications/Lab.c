/*
 * =============================================================================
 * SERIAL COMMUNICATION LAB - INTERRUPT-BASED Q&A SYSTEM WITH LCD
 * =============================================================================
 *
 * PROJECT: Serial_Communications - Lab Exercise
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Hands-on laboratory exercise for interrupt-based serial communication.
 * Students implement an interactive Q&A system that displays questions and
 * answers on the GLCD display while communicating via serial monitor.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master interrupt-driven UART communication (ISR programming)
 * 2. Integrate GLCD display with serial communication
 * 3. Implement command parsing and protocol design
 * 4. Display student information (name, ID) on LCD
 * 5. Practice real-time multi-peripheral coordination
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - UART1 connection for VS Code serial monitor (9600 baud, 8N1)
 * - KS0108 Graphic LCD (128x64 pixels)
 * - Serial connection via USB-TTL adapter
 *
 * LAB REQUIREMENTS:
 * 1. Display student name and ID on LCD at startup
 * 2. Receive questions via serial monitor
 * 3. Display questions on LCD
 * 4. Accept answers via serial input
 * 5. Display answers on LCD
 * 6. Use interrupt-based communication (no polling!)
 * 7. Handle multiple questions in sequence
 *
 * COMMUNICATION PROTOCOL:
 * - Question format: "Q: <question text>"
 * - Answer format:   "A: <answer text>"
 * - Command format:  "CMD:<command>"
 * - Special commands: "CLEAR", "RESET", "INFO"
 *
 * =============================================================================
 * STUDENT INFORMATION SECTION
 * =============================================================================
 * TODO: Students must fill in their information below
 */

// ========== STUDENT INFORMATION - FILL THIS IN! ==========
#define STUDENT_NAME "Hong Gil Dong" // TODO: Enter your name
#define STUDENT_ID "2025123456"      // TODO: Enter your student ID
#define LAB_DATE "2025-10-21"        // TODO: Today's date
// =========================================================

/*
 * =============================================================================
 * SYSTEM INCLUDES AND CONFIGURATION
 * =============================================================================
 */

#include "config.h"
#include <avr/pgmspace.h> // For PROGMEM strings in flash
#include <string.h>       // For string operations

// Type definition for byte (used by GLCD library)
typedef unsigned char byte;

// Include GLCD library functions
extern void glcd_port_init(void);
extern void lcd_init(void);
extern void lcd_clear(void);
extern void lcd_xy(byte x, byte y);
extern void lcd_char(byte character);
extern void lcd_string(byte x, byte y, char *string);
extern void GLCD_2DigitDecimal(unsigned char number);
extern void GLCD_3DigitDecimal(unsigned int number);
extern void GLCD_4DigitDecimal(unsigned int number);

/*
 * =============================================================================
 * LED INDICATORS CONFIGURATION
 * =============================================================================
 * Visual feedback for communication events on PORTB LEDs
 */

#define LED_RX_PIN 0   // LED0 on PORTB - blinks on UART RX
#define LED_TX_PIN 1   // LED1 on PORTB - blinks on UART TX
#define LED_ACTIVITY 2 // LED2 on PORTB - blinks on command processing
#define LED_ERROR 3    // LED3 on PORTB - blinks on error

#define LED_RX_ON() (PORTB |= (1 << LED_RX_PIN))
#define LED_RX_OFF() (PORTB &= ~(1 << LED_RX_PIN))
#define LED_RX_TOGGLE() (PORTB ^= (1 << LED_RX_PIN))

#define LED_TX_ON() (PORTB |= (1 << LED_TX_PIN))
#define LED_TX_OFF() (PORTB &= ~(1 << LED_TX_PIN))
#define LED_TX_TOGGLE() (PORTB ^= (1 << LED_TX_PIN))

#define LED_ACTIVITY_ON() (PORTB |= (1 << LED_ACTIVITY))
#define LED_ACTIVITY_OFF() (PORTB &= ~(1 << LED_ACTIVITY))
#define LED_ACTIVITY_TOGGLE() (PORTB ^= (1 << LED_ACTIVITY))

#define LED_ERROR_ON() (PORTB |= (1 << LED_ERROR))
#define LED_ERROR_OFF() (PORTB &= ~(1 << LED_ERROR))
#define LED_ERROR_TOGGLE() (PORTB ^= (1 << LED_ERROR))

/*
 * =============================================================================
 * CIRCULAR BUFFER CONFIGURATION
 * =============================================================================
 */

#define RX_BUFFER_SIZE 128 // Receive buffer size
#define TX_BUFFER_SIZE 128 // Transmit buffer size
#define CMD_BUFFER_SIZE 64 // Command buffer size

// Receive buffer (for incoming serial data)
volatile char rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;
volatile uint8_t rx_overflow = 0;

// Transmit buffer (for outgoing serial data)
volatile char tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_head = 0;
volatile uint8_t tx_tail = 0;
volatile uint8_t tx_busy = 0;

// Command processing
char command_buffer[CMD_BUFFER_SIZE];
uint8_t command_index = 0;
uint8_t command_ready = 0;

// Question and answer storage
char question_buffer[64];
char answer_buffer[64];
uint8_t question_count = 0;
uint8_t answer_count = 0;

// System state
typedef enum
{
    STATE_IDLE,
    STATE_WAITING_QUESTION,
    STATE_WAITING_ANSWER,
    STATE_PROCESSING
} SystemState;

volatile SystemState system_state = STATE_IDLE;

/*
 * =============================================================================
 * UART CONTROL REGISTER DEFINITIONS (EDUCATIONAL REFERENCE)
 * =============================================================================
 *
 * UCSR1A - Control and Status Register A
 * ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
 * │ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
 * ├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
 * │Name │RXC1 │TXC1 │UDRE1│ FE1 │DOR1 │UPE1 │U2X1 │MPCM1│
 * └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
 *
 * UCSR1B - Control and Status Register B
 * ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐
 * │ Bit │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │
 * ├─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┼─────┤
 * │Name │RXCIE│TXCIE│UDRIE│RXEN1│TXEN1│UCSZ │ RXB8│ TXB8│
 * └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘
 */

/*
 * =============================================================================
 * INTERRUPT SERVICE ROUTINES - STUDENTS STUDY THESE!
 * =============================================================================
 */

/*
 * USART1 Receive Complete Interrupt
 * Called automatically when a character is received
 * Students learn: ISR syntax, circular buffer management, volatile variables
 * LED Indicator: LED0 (RX) toggles on each received character
 */
ISR(USART1_RX_vect)
{
    char received = UDR1; // Read received character (clears RXC1 flag)
    uint8_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;

    // Visual feedback - toggle RX LED
    LED_RX_TOGGLE();

    // Check for buffer overflow
    if (next_head != rx_tail)
    {
        rx_buffer[rx_head] = received;
        rx_head = next_head;
    }
    else
    {
        rx_overflow = 1;    // Flag overflow for error handling
        LED_ERROR_TOGGLE(); // Visual error indication
    }
}

/*
 * USART1 Data Register Empty Interrupt
 * Called when UDR1 is ready for next character
 * Students learn: TX interrupt management, buffer empty handling
 * LED Indicator: LED1 (TX) toggles on each transmitted character
 */
ISR(USART1_UDRE_vect)
{
    if (tx_head != tx_tail)
    {
        // Send next character from buffer
        UDR1 = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;

        // Visual feedback - toggle TX LED
        LED_TX_TOGGLE();
    }
    else
    {
        // Buffer empty - disable interrupt to prevent infinite ISR calls!
        UCSR1B &= ~(1 << UDRIE1);
        tx_busy = 0;
    }
}

/*
 * =============================================================================
 * UART INITIALIZATION AND COMMUNICATION FUNCTIONS
 * =============================================================================
 */

/*
 * Initialize UART1 with interrupt support
 * Baud rate: 9600, Format: 8N1, Mode: Interrupt-driven
 */
void init_uart_lab(void)
{
    // Step 1: Enable double-speed mode (U2X=1) for better baud rate accuracy
    UCSR1A = (1 << U2X1);

    // Step 2: Configure 8-bit data, no parity, 1 stop bit (8N1)
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);

    // Step 3: Calculate baud rate for 9600 @ 16MHz with U2X=1
    // UBRR = (F_CPU / (8 * BAUD)) - 1 = (16000000 / (8 * 9600)) - 1 = 207
    uint16_t ubrr = 207;
    UBRR1H = (ubrr >> 8);
    UBRR1L = ubrr;

    // Step 4: Enable RX Complete Interrupt, RX, and TX
    UCSR1B = (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);

    // Step 5: Enable global interrupts
    sei();

    // Allow UART hardware to stabilize
    _delay_ms(10);
}

/*
 * Send single character via interrupt-driven transmission
 * Returns: 1 on success, 0 if buffer full
 */
uint8_t uart_putchar(char c)
{
    uint8_t next_head = (tx_head + 1) % TX_BUFFER_SIZE;

    // Check if buffer full
    if (next_head == tx_tail)
    {
        return 0; // Buffer full
    }

    // Add to transmit buffer
    tx_buffer[tx_head] = c;
    tx_head = next_head;

    // Enable TX interrupt if not already transmitting
    if (!tx_busy)
    {
        tx_busy = 1;
        UCSR1B |= (1 << UDRIE1);
    }

    return 1; // Success
}

/*
 * Send null-terminated string via interrupt-driven transmission
 */
void uart_puts(const char *str)
{
    while (*str)
    {
        while (!uart_putchar(*str))
            ; // Wait if buffer full
        str++;
    }
}

/*
 * Send string from flash memory (PROGMEM)
 */
void uart_puts_P(const char *str)
{
    char c;
    while ((c = pgm_read_byte(str++)))
    {
        while (!uart_putchar(c))
            ;
    }
}

/*
 * Check if characters available in receive buffer
 */
uint8_t uart_available(void)
{
    return (rx_head != rx_tail);
}

/*
 * Get character from receive buffer
 * Returns: character if available, 0 if buffer empty
 */
char uart_getchar(void)
{
    if (rx_head == rx_tail)
    {
        return 0; // Buffer empty
    }

    char data = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;

    return data;
}

/*
 * =============================================================================
 * LCD DISPLAY FUNCTIONS
 * =============================================================================
 */

/*
 * Display student information on LCD (name and ID)
 * This must be called at startup to satisfy lab requirements
 */
void lcd_show_student_info(void)
{
    lcd_clear();

    // Line 0: Display "Student Info"
    lcd_string(0, 0, "Student Info:");

    // Line 2: Display name
    lcd_string(0, 2, "Name: ");
    lcd_string(6, 2, STUDENT_NAME);

    // Line 3: Display ID
    lcd_string(0, 3, "ID: ");
    lcd_string(4, 3, STUDENT_ID);

    // Line 5: Display date
    lcd_string(0, 5, "Date: ");
    lcd_string(6, 5, LAB_DATE);

    // Line 7: Display ready message
    lcd_string(0, 7, "Ready for Q&A");
}

/*
 * Display question on LCD
 * Format: "Q1: <question text>"
 */
void lcd_show_question(uint8_t q_num, const char *question)
{
    lcd_clear();

    // Line 0: "Question #:"
    lcd_string(0, 0, "Question ");
    lcd_xy(9, 0);
    GLCD_2DigitDecimal(q_num);
    lcd_string(11, 0, ":");

    // Line 2-4: Question text (wrap if needed)
    uint8_t len = strlen(question);
    uint8_t line = 2;
    uint8_t col = 0;

    for (uint8_t i = 0; i < len && line < 6; i++)
    {
        if (col >= 20 || question[i] == '\n')
        {
            line++;
            col = 0;
            if (question[i] == '\n')
                continue;
        }

        lcd_xy(col, line);
        lcd_char(question[i]);
        col++;
    }

    // Line 6: Waiting message
    lcd_string(0, 6, "Waiting answer...");
}

/*
 * Display answer on LCD below question
 */
void lcd_show_answer(const char *answer)
{
    // Line 6: "Answer:"
    lcd_string(0, 6, "Answer:");

    // Line 7: Answer text
    uint8_t len = strlen(answer);
    for (uint8_t i = 0; i < len && i < 20; i++)
    {
        lcd_xy(i, 7);
        lcd_char(answer[i]);
    }
}

/*
 * Display statistics on LCD
 */
void lcd_show_stats(uint8_t questions, uint8_t answers)
{
    lcd_clear();
    lcd_string(0, 0, "Session Stats:");

    lcd_string(0, 2, "Questions: ");
    lcd_xy(11, 2);
    GLCD_2DigitDecimal(questions);

    lcd_string(0, 3, "Answers: ");
    lcd_xy(9, 3);
    GLCD_2DigitDecimal(answers);

    lcd_string(0, 5, "Status: Complete");
}

/*
 * =============================================================================
 * COMMAND PROCESSING FUNCTIONS
 * =============================================================================
 */

/*
 * Process received command from serial input
 * Handles: Questions (Q: ...), Answers (A: ...), Commands (CMD:...)
 * LED Indicator: LED2 (ACTIVITY) blinks on command processing
 */
void process_command(const char *cmd)
{
    // Toggle activity LED to show command processing
    LED_ACTIVITY_TOGGLE();

    // Check for question (Q: format)
    if (strncmp(cmd, "Q:", 2) == 0 || strncmp(cmd, "q:", 2) == 0)
    {
        question_count++;
        strncpy(question_buffer, cmd + 2, sizeof(question_buffer) - 1);
        question_buffer[sizeof(question_buffer) - 1] = '\0';

        // Display question on LCD
        lcd_show_question(question_count, question_buffer);

        // Echo to serial
        uart_puts("\r\n>>> Question ");
        uart_putchar('0' + (question_count / 10));
        uart_putchar('0' + (question_count % 10));
        uart_puts(" received: ");
        uart_puts(question_buffer);
        uart_puts("\r\n>>> Please enter answer (A: <your answer>)\r\n");

        system_state = STATE_WAITING_ANSWER;
        return;
    }

    // Check for answer (A: format)
    if (strncmp(cmd, "A:", 2) == 0 || strncmp(cmd, "a:", 2) == 0)
    {
        if (system_state == STATE_WAITING_ANSWER)
        {
            answer_count++;
            strncpy(answer_buffer, cmd + 2, sizeof(answer_buffer) - 1);
            answer_buffer[sizeof(answer_buffer) - 1] = '\0';

            // Display answer on LCD (below question)
            lcd_show_answer(answer_buffer);

            // Echo to serial
            uart_puts("\r\n>>> Answer recorded: ");
            uart_puts(answer_buffer);
            uart_puts("\r\n>>> Send next question or type CMD:STATS\r\n");

            system_state = STATE_WAITING_QUESTION;
        }
        else
        {
            LED_ERROR_TOGGLE(); // Visual error indication
            uart_puts("\r\n>>> ERROR: No question pending. Send Q: first!\r\n");
        }
        return;
    }

    // Check for special commands
    if (strncmp(cmd, "CMD:", 4) == 0 || strncmp(cmd, "cmd:", 4) == 0)
    {
        const char *subcmd = cmd + 4;

        // CMD:CLEAR - Clear LCD
        if (strcmp(subcmd, "CLEAR") == 0 || strcmp(subcmd, "clear") == 0)
        {
            lcd_clear();
            uart_puts("\r\n>>> LCD cleared\r\n");
        }
        // CMD:INFO - Show student info
        else if (strcmp(subcmd, "INFO") == 0 || strcmp(subcmd, "info") == 0)
        {
            lcd_show_student_info();
            uart_puts("\r\n>>> Student Info displayed on LCD\r\n");
            uart_puts("    Name: " STUDENT_NAME "\r\n");
            uart_puts("    ID: " STUDENT_ID "\r\n");
        }
        // CMD:STATS - Show statistics
        else if (strcmp(subcmd, "STATS") == 0 || strcmp(subcmd, "stats") == 0)
        {
            lcd_show_stats(question_count, answer_count);
            uart_puts("\r\n>>> Session Statistics:\r\n");
            uart_puts("    Questions: ");
            uart_putchar('0' + (question_count / 10));
            uart_putchar('0' + (question_count % 10));
            uart_puts("\r\n    Answers: ");
            uart_putchar('0' + (answer_count / 10));
            uart_putchar('0' + (answer_count % 10));
            uart_puts("\r\n");
        }
        // CMD:RESET - Reset session
        else if (strcmp(subcmd, "RESET") == 0 || strcmp(subcmd, "reset") == 0)
        {
            question_count = 0;
            answer_count = 0;
            lcd_show_student_info();
            uart_puts("\r\n>>> Session reset. Ready for new Q&A\r\n");
            system_state = STATE_WAITING_QUESTION;
        }
        // CMD:HELP - Show help
        else if (strcmp(subcmd, "HELP") == 0 || strcmp(subcmd, "help") == 0)
        {
            uart_puts_P(PSTR("\r\n=== Lab Command Reference ===\r\n"));
            uart_puts_P(PSTR("Q: <text>       - Send question\r\n"));
            uart_puts_P(PSTR("A: <text>       - Send answer\r\n"));
            uart_puts_P(PSTR("CMD:INFO        - Show student info\r\n"));
            uart_puts_P(PSTR("CMD:STATS       - Show statistics\r\n"));
            uart_puts_P(PSTR("CMD:CLEAR       - Clear LCD\r\n"));
            uart_puts_P(PSTR("CMD:RESET       - Reset session\r\n"));
            uart_puts_P(PSTR("CMD:HELP        - This help\r\n"));
            uart_puts_P(PSTR("==============================\r\n"));
        }
        else
        {
            uart_puts("\r\n>>> Unknown command. Try CMD:HELP\r\n");
            LED_ERROR_TOGGLE(); // Visual error indication
        }
        return;
    }

    // Unknown format - error
    LED_ERROR_TOGGLE(); // Visual error indication
    uart_puts("\r\n>>> ERROR: Invalid format. Use Q: or A: or CMD:\r\n");
}

/*
 * =============================================================================
 * SYSTEM INITIALIZATION
 * =============================================================================
 */

void system_init(void)
{
    // Initialize ports
    PORTA = 0xFF;
    DDRA = 0x00; // Input with pull-ups
    PORTB = 0x00;
    DDRB = 0xFF; // Output - LEDs on PORTB

    // Flash all LEDs briefly to show system startup
    PORTB = 0x0F; // Turn on LED0-3
    _delay_ms(200);
    PORTB = 0x00; // Turn off all LEDs
    _delay_ms(100);

    // Initialize GLCD
    glcd_port_init();
    lcd_init();
    lcd_clear();

    // Initialize UART with interrupts
    init_uart_lab();

    // Display student information
    lcd_show_student_info();

    // Send welcome message to serial
    _delay_ms(500); // Wait for LCD to display

    uart_puts_P(PSTR("\r\n"));
    uart_puts_P(PSTR("=============================================\r\n"));
    uart_puts_P(PSTR("  Serial Communication Lab - Q&A System\r\n"));
    uart_puts_P(PSTR("  SOC 3050 - Embedded Systems Lab\r\n"));
    uart_puts_P(PSTR("=============================================\r\n"));
    uart_puts_P(PSTR("\r\n"));
    uart_puts_P(PSTR("Student: " STUDENT_NAME "\r\n"));
    uart_puts_P(PSTR("ID: " STUDENT_ID "\r\n"));
    uart_puts_P(PSTR("Date: " LAB_DATE "\r\n"));
    uart_puts_P(PSTR("\r\n"));
    uart_puts_P(PSTR("Lab Features:\r\n"));
    uart_puts_P(PSTR("- Interrupt-based serial communication\r\n"));
    uart_puts_P(PSTR("- GLCD display integration\r\n"));
    uart_puts_P(PSTR("- Real-time Q&A system\r\n"));
    uart_puts_P(PSTR("- LED indicators (RX/TX/Activity/Error)\r\n"));
    uart_puts_P(PSTR("\r\n"));
    uart_puts_P(PSTR("LED Indicators:\r\n"));
    uart_puts_P(PSTR("  LED0 (PB0) - RX activity\r\n"));
    uart_puts_P(PSTR("  LED1 (PB1) - TX activity\r\n"));
    uart_puts_P(PSTR("  LED2 (PB2) - Command processing\r\n"));
    uart_puts_P(PSTR("  LED3 (PB3) - Errors\r\n"));
    uart_puts_P(PSTR("\r\n"));
    uart_puts_P(PSTR("Type CMD:HELP for command list\r\n"));
    uart_puts_P(PSTR("Ready for questions!\r\n"));
    uart_puts_P(PSTR("\r\n> "));

    system_state = STATE_WAITING_QUESTION;
}

/*
 * =============================================================================
 * MAIN PROGRAM
 * =============================================================================
 */

int main(void)
{
    // Initialize all systems
    system_init();

    char received;
    uint16_t idle_counter = 0;

    // Main loop - interrupt-driven I/O, non-blocking
    while (1)
    {
        // Process received characters from serial
        if (uart_available())
        {
            received = uart_getchar();
            idle_counter = 0; // Reset idle counter

            // Echo character back
            uart_putchar(received);

            // Check for line ending (Enter key)
            if (received == '\r' || received == '\n')
            {
                if (command_index > 0)
                {
                    // Null-terminate command
                    command_buffer[command_index] = '\0';

                    // Process the command
                    uart_puts("\r\n"); // New line
                    process_command(command_buffer);
                    uart_puts("> "); // Prompt

                    // Reset command buffer
                    command_index = 0;
                }
            }
            // Handle backspace
            else if (received == '\b' || received == 127)
            {
                if (command_index > 0)
                {
                    command_index--;
                    uart_puts(" \b"); // Erase character
                }
            }
            // Add character to buffer
            else if (command_index < CMD_BUFFER_SIZE - 1 && received >= ' ')
            {
                command_buffer[command_index++] = received;
            }
        }

        // Idle activity - blink LED4 (higher bit) to show CPU is free
        // Note: LED0-3 are used for RX/TX/Activity/Error indicators
        idle_counter++;
        if (idle_counter > 50000)
        {
            PORTB ^= (1 << 4); // Toggle LED4 (PB4) to show system is idle/running
            idle_counter = 0;
        }

        // Check for buffer overflow
        if (rx_overflow)
        {
            LED_ERROR_TOGGLE(); // Visual indication of overflow
            uart_puts_P(PSTR("\r\n>>> WARNING: RX buffer overflow!\r\n> "));
            rx_overflow = 0;
        }

        // Small delay (CPU is still free for other tasks!)
        _delay_us(100);
    }

    return 0;
}

/*
 * =============================================================================
 * LAB EXERCISE NOTES FOR STUDENTS
 * =============================================================================
 *
 * TESTING PROCEDURE:
 *
 * 1. INITIAL SETUP:
 *    - Open VS Code
 *    - Open Serial Monitor (Baud: 9600, 8N1)
 *    - Program ATmega128 with this code
 *    - Verify student info displays on LCD and serial
 *    - Observe LED startup sequence (all LEDs flash briefly)
 *
 * 2. LED INDICATOR TEST:
 *    - LED0 (PB0): Should toggle rapidly when typing (RX activity)
 *    - LED1 (PB1): Should toggle when receiving responses (TX activity)
 *    - LED2 (PB2): Should toggle when commands are processed
 *    - LED3 (PB3): Should toggle on errors (invalid commands, etc.)
 *    - LED4 (PB4): Should blink slowly showing CPU idle time
 *
 * 3. BASIC Q&A TEST:
 *    - Type: Q: What is an ISR?
 *    - Press Enter
 *    - Verify question appears on LCD
 *    - Observe LED0 (RX) and LED2 (Activity) toggling
 *    - Type: A: Interrupt Service Routine
 *    - Press Enter
 *    - Verify answer appears on LCD below question
 *    - Observe LED indicators showing communication
 *
 * 4. COMMAND TESTS:
 *    - Type: CMD:STATS (should show question/answer count)
 *    - Type: CMD:INFO (should show student info again)
 *    - Type: CMD:CLEAR (should clear LCD)
 *    - Type: CMD:HELP (should show command reference)
 *
 * 5. MULTIPLE Q&A TEST:
 *    - Send several questions (Q1, Q2, Q3...)
 *    - Answer each one
 *    - Check CMD:STATS shows correct counts
 *
 * 6. ERROR HANDLING TEST:
 *    - Try sending A: without Q: first (should show error + LED3 toggles)
 *    - Try invalid command format (should show error + LED3 toggles)
 *    - Type very long question (should handle gracefully)
 *    - Watch for RX buffer overflow warning (LED3 + serial message)
 *
 * LED INDICATOR SUMMARY:
 * ┌──────┬─────────┬──────────────────────────────────────────┐
 * │ LED  │  Pin    │ Indication                               │
 * ├──────┼─────────┼──────────────────────────────────────────┤
 * │ LED0 │ PB0     │ RX Activity (ISR(USART1_RX_vect))        │
 * │ LED1 │ PB1     │ TX Activity (ISR(USART1_UDRE_vect))      │
 * │ LED2 │ PB2     │ Command Processing (process_command())   │
 * │ LED3 │ PB3     │ Errors (invalid input, overflow, etc.)   │
 * │ LED4 │ PB4     │ Idle/Running indicator (slow blink)      │
 * └──────┴─────────┴──────────────────────────────────────────┘
 *
 * GRADING CRITERIA:
 * □ Student name and ID displayed on LCD at startup
 * □ Interrupt-based communication (no polling in main loop)
 * □ Questions received and displayed on LCD
 * □ Answers received and displayed on LCD
 * □ Commands processed correctly (INFO, STATS, etc.)
 * □ LED indicators working correctly (RX/TX/Activity/Error)
 * □ LED0 toggles on character reception
 * □ LED1 toggles on character transmission
 * □ LED2 toggles on command processing
 * □ LED3 toggles on errors
 * □ Clean code with proper comments
 * □ Error handling for invalid input
 * □ CPU remains free (LED4 slow blink proves non-blocking)
 *
 * LEARNING OUTCOMES:
 * - Understand ISR programming for UART
 * - Master circular buffer management
 * - Integrate multiple peripherals (UART + GLCD + GPIO LEDs)
 * - Design command protocols for embedded systems
 * - Practice real-time system programming
 * - Use visual feedback (LEDs) for debugging and status indication
 *
 * =============================================================================
 */
