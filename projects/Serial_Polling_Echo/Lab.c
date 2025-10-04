/*
 * =============================================================================
 * SERIAL POLLING ECHO - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master UART serial communication fundamentals
 * DURATION: 60 minutes
 * DIFFICULTY: Beginner
 *
 * STUDENTS WILL:
 * - Configure UART for serial communication
 * - Implement polling-based character transmission/reception
 * - Create echo and command processing systems
 * - Debug serial communication issues
 * - Build interactive serial applications
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - Serial connection (USB-UART or RS232)
 * - 4 LEDs for status indication
 * - 4 push buttons for commands
 * - Terminal emulator (9600 baud, 8N1)
 *
 * LAB STRUCTURE:
 * - Exercise 1: Basic character echo (15 min)
 * - Exercise 2: String echo and buffering (20 min)
 * - Exercise 3: Command processing system (15 min)
 * - Exercise 4: Interactive terminal application (10 min)
 *
 * =============================================================================
 */

#include "config.h"

// Lab configuration
#define BUFFER_SIZE 64
#define MAX_COMMANDS 8

// Global variables for lab exercises
uint16_t lab_score = 0;
char input_buffer[BUFFER_SIZE];
uint8_t buffer_index = 0;
uint16_t char_count = 0;
uint16_t echo_count = 0;

/*
 * =============================================================================
 * HELPER FUNCTIONS
 * =============================================================================
 */

void clear_buffer(void)
{
    for (uint8_t i = 0; i < BUFFER_SIZE; i++)
    {
        input_buffer[i] = '\0';
    }
    buffer_index = 0;
}

void send_prompt(void)
{
    puts_USART1("Serial> ");
}

void blink_led(uint8_t led_num, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        PORTB |= (1 << led_num);
        _delay_ms(200);
        PORTB &= ~(1 << led_num);
        _delay_ms(200);
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 1: BASIC CHARACTER ECHO (15 minutes)
 * =============================================================================
 * OBJECTIVE: Learn fundamental UART polling operations
 * DIFFICULTY: ★☆☆☆☆ (Very Easy)
 */

void lab_ex1_simple_echo(void)
{
    /*
     * CHALLENGE: Echo every character received back to sender
     * TASK: Implement character-by-character echo with visual feedback
     * LEARNING: UART polling, character I/O, real-time response
     */

    puts_USART1("\\r\\n=== Lab 1: Simple Character Echo ===\\r\\n");
    puts_USART1("Type characters - they will be echoed back\\r\\n");
    puts_USART1("Press 'Q' to quit this exercise\\r\\n\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "CHAR ECHO MODE");
    lcd_string(1, 0, "Type on terminal");

    char_count = 0;

    while (1)
    {
        // Check if character received
        if (UCSR1A & (1 << RXC1))
        {
            char received_char = UDR1; // Read character
            char_count++;

            // Echo character back
            while (!(UCSR1A & (1 << UDRE1)))
                ;                 // Wait for transmit ready
            UDR1 = received_char; // Send character back

            // Visual feedback on LCD
            char display_msg[20];
            sprintf(display_msg, "Char: '%c' (%d)",
                    (received_char >= 32 && received_char <= 126) ? received_char : '?',
                    received_char);
            lcd_string(3, 0, display_msg);

            sprintf(display_msg, "Count: %d", char_count);
            lcd_string(4, 0, display_msg);

            // LED feedback based on character type
            if (received_char >= '0' && received_char <= '9')
            {
                blink_led(0, 1); // Numbers - LED 0
            }
            else if (received_char >= 'A' && received_char <= 'Z')
            {
                blink_led(1, 1); // Uppercase - LED 1
            }
            else if (received_char >= 'a' && received_char <= 'z')
            {
                blink_led(2, 1); // Lowercase - LED 2
            }
            else
            {
                blink_led(3, 1); // Other - LED 3
            }

            // Exit condition
            if (received_char == 'Q' || received_char == 'q')
            {
                puts_USART1("\\r\\nCharacter echo exercise complete!\\r\\n");
                break;
            }
        }

        _delay_ms(10); // Small delay to prevent overwhelming
    }

    char summary[50];
    sprintf(summary, "Total characters echoed: %d\\r\\n", char_count);
    puts_USART1(summary);

    if (char_count >= 10)
    {
        lab_score += 100;
        puts_USART1("✓ Character echo mastered!\\r\\n");
    }
}

void lab_ex1_echo_with_formatting(void)
{
    /*
     * CHALLENGE: Echo with character analysis and formatting
     * TASK: Enhance echo to show character codes and properties
     * LEARNING: Character encoding, formatting, data analysis
     */

    puts_USART1("\\r\\n=== Lab 1.2: Enhanced Echo ===\\r\\n");
    puts_USART1("Type characters for detailed analysis\\r\\n");
    puts_USART1("Press ESC to quit this exercise\\r\\n\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "ENHANCED ECHO");
    lcd_string(1, 0, "Char analysis");

    while (1)
    {
        if (UCSR1A & (1 << RXC1))
        {
            char ch = UDR1;

            // Detailed character analysis
            char analysis[80];
            sprintf(analysis, "Received: '%c' (ASCII %d, 0x%02X) - ",
                    (ch >= 32 && ch <= 126) ? ch : '?', ch, ch);
            puts_USART1(analysis);

            // Character classification
            if (ch >= '0' && ch <= '9')
            {
                puts_USART1("DIGIT\\r\\n");
            }
            else if (ch >= 'A' && ch <= 'Z')
            {
                puts_USART1("UPPERCASE\\r\\n");
            }
            else if (ch >= 'a' && ch <= 'z')
            {
                puts_USART1("lowercase\\r\\n");
            }
            else if (ch == ' ')
            {
                puts_USART1("SPACE\\r\\n");
            }
            else if (ch == '\\r')
            {
                puts_USART1("CARRIAGE RETURN\\r\\n");
            }
            else if (ch == '\\n')
            {
                puts_USART1("LINE FEED\\r\\n");
            }
            else if (ch == 27)
            { // ESC
                puts_USART1("ESCAPE - EXITING\\r\\n");
                break;
            }
            else
            {
                puts_USART1("SPECIAL\\r\\n");
            }

            // LCD display
            char lcd_msg[20];
            sprintf(lcd_msg, "ASCII: %3d (0x%02X)", ch, ch);
            lcd_string(3, 0, lcd_msg);

            echo_count++;
            sprintf(lcd_msg, "Analyzed: %d", echo_count);
            lcd_string(4, 0, lcd_msg);
        }

        _delay_ms(10);
    }

    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: STRING ECHO AND BUFFERING (20 minutes)
 * =============================================================================
 * OBJECTIVE: Handle complete strings and implement buffering
 * DIFFICULTY: ★★☆☆☆ (Easy)
 */

void lab_ex2_line_echo(void)
{
    /*
     * CHALLENGE: Buffer complete lines and echo them
     * TASK: Implement line-based input with Enter key detection
     * LEARNING: String buffering, line termination, overflow handling
     */

    puts_USART1("\\r\\n=== Lab 2: Line Echo ===\\r\\n");
    puts_USART1("Type complete lines and press Enter\\r\\n");
    puts_USART1("Type 'exit' to quit this exercise\\r\\n\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "LINE ECHO MODE");
    lcd_string(1, 0, "Type and press Enter");

    send_prompt();
    clear_buffer();
    uint8_t line_count = 0;

    while (1)
    {
        if (UCSR1A & (1 << RXC1))
        {
            char ch = UDR1;

            // Echo character immediately for visual feedback
            putch_USART1(ch);

            if (ch == '\\r' || ch == '\\n')
            {
                // End of line - process the buffer
                puts_USART1("\\r\\nYou typed: '");
                puts_USART1(input_buffer);
                puts_USART1("'\\r\\n");

                line_count++;

                // Check for exit command
                if (strcmp(input_buffer, "exit") == 0)
                {
                    puts_USART1("Line echo exercise complete!\\r\\n");
                    break;
                }

                // Display line statistics
                char stats[50];
                sprintf(stats, "Length: %d characters\\r\\n", strlen(input_buffer));
                puts_USART1(stats);

                // LCD update
                char lcd_msg[20];
                sprintf(lcd_msg, "Lines: %d", line_count);
                lcd_string(3, 0, lcd_msg);

                sprintf(lcd_msg, "Length: %d", strlen(input_buffer));
                lcd_string(4, 0, lcd_msg);

                // Reset buffer and show new prompt
                clear_buffer();
                puts_USART1("\\r\\n");
                send_prompt();
            }
            else if (ch == 8 || ch == 127) // Backspace or DEL
            {
                if (buffer_index > 0)
                {
                    buffer_index--;
                    input_buffer[buffer_index] = '\\0';
                    puts_USART1("\\b \\b"); // Erase character on terminal
                }
            }
            else if (buffer_index < (BUFFER_SIZE - 1))
            {
                // Add character to buffer
                input_buffer[buffer_index] = ch;
                buffer_index++;
                input_buffer[buffer_index] = '\\0'; // Keep null-terminated

                // Show current buffer on LCD
                char truncated[17]; // LCD width limit
                strncpy(truncated, input_buffer, 16);
                truncated[16] = '\\0';
                lcd_string(5, 0, truncated);
            }
            else
            {
                // Buffer overflow
                puts_USART1("\\r\\n[BUFFER FULL - LINE TOO LONG]\\r\\n");
                clear_buffer();
                send_prompt();
            }
        }

        _delay_ms(5);
    }

    char summary[50];
    sprintf(summary, "Total lines processed: %d\\r\\n", line_count);
    puts_USART1(summary);

    if (line_count >= 3)
    {
        lab_score += 150;
        puts_USART1("✓ Line echo mastered!\\r\\n");
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 3: COMMAND PROCESSING (15 minutes)
 * =============================================================================
 * OBJECTIVE: Build a command interpreter system
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex3_command_processor(void)
{
    /*
     * CHALLENGE: Create a command-line interface
     * TASK: Parse commands and execute appropriate actions
     * LEARNING: String parsing, command dispatch, system control
     */

    puts_USART1("\\r\\n=== Lab 3: Command Processor ===\\r\\n");
    puts_USART1("Available commands:\\r\\n");
    puts_USART1("  help    - Show this help\\r\\n");
    puts_USART1("  led <n> - Toggle LED n (0-3)\\r\\n");
    puts_USART1("  status  - Show system status\\r\\n");
    puts_USART1("  count   - Show character count\\r\\n");
    puts_USART1("  clear   - Clear counters\\r\\n");
    puts_USART1("  quit    - Exit command mode\\r\\n\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "COMMAND MODE");
    lcd_string(1, 0, "Enter commands");

    send_prompt();
    clear_buffer();
    uint8_t commands_executed = 0;

    while (1)
    {
        if (UCSR1A & (1 << RXC1))
        {
            char ch = UDR1;
            putch_USART1(ch); // Echo

            if (ch == '\\r' || ch == '\\n')
            {
                puts_USART1("\\r\\n");

                if (strlen(input_buffer) > 0)
                {
                    commands_executed++;

                    // Parse and execute commands
                    if (strcmp(input_buffer, "help") == 0)
                    {
                        puts_USART1("Commands: help, led <n>, status, count, clear, quit\\r\\n");
                    }
                    else if (strncmp(input_buffer, "led ", 4) == 0)
                    {
                        char led_num = input_buffer[4];
                        if (led_num >= '0' && led_num <= '3')
                        {
                            uint8_t led = led_num - '0';
                            PORTB ^= (1 << led); // Toggle LED
                            char msg[30];
                            sprintf(msg, "LED %d toggled\\r\\n", led);
                            puts_USART1(msg);
                        }
                        else
                        {
                            puts_USART1("Invalid LED number (0-3)\\r\\n");
                        }
                    }
                    else if (strcmp(input_buffer, "status") == 0)
                    {
                        char status[60];
                        sprintf(status, "Score: %d, Commands: %d, Chars: %d\\r\\n",
                                lab_score, commands_executed, char_count);
                        puts_USART1(status);
                    }
                    else if (strcmp(input_buffer, "count") == 0)
                    {
                        char count_msg[40];
                        sprintf(count_msg, "Character count: %d\\r\\n", char_count);
                        puts_USART1(count_msg);
                    }
                    else if (strcmp(input_buffer, "clear") == 0)
                    {
                        char_count = 0;
                        echo_count = 0;
                        puts_USART1("Counters cleared\\r\\n");
                    }
                    else if (strcmp(input_buffer, "quit") == 0)
                    {
                        puts_USART1("Exiting command mode\\r\\n");
                        break;
                    }
                    else
                    {
                        puts_USART1("Unknown command. Type 'help' for commands\\r\\n");
                    }

                    // Update LCD
                    char lcd_msg[20];
                    sprintf(lcd_msg, "Commands: %d", commands_executed);
                    lcd_string(3, 0, lcd_msg);
                }

                clear_buffer();
                send_prompt();
            }
            else if (ch == 8 || ch == 127) // Backspace
            {
                if (buffer_index > 0)
                {
                    buffer_index--;
                    input_buffer[buffer_index] = '\\0';
                    puts_USART1("\\b \\b");
                }
            }
            else if (buffer_index < (BUFFER_SIZE - 1))
            {
                input_buffer[buffer_index] = ch;
                buffer_index++;
                input_buffer[buffer_index] = '\\0';
                char_count++;
            }
        }

        _delay_ms(5);
    }

    char summary[50];
    sprintf(summary, "Commands executed: %d\\r\\n", commands_executed);
    puts_USART1(summary);

    if (commands_executed >= 5)
    {
        lab_score += 150;
        puts_USART1("✓ Command processing mastered!\\r\\n");
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 4: INTERACTIVE TERMINAL (10 minutes)
 * =============================================================================
 * OBJECTIVE: Create a complete terminal application
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex4_interactive_terminal(void)
{
    /*
     * CHALLENGE: Build a full-featured terminal interface
     * TASK: Combine all previous concepts into a complete application
     * LEARNING: Application architecture, user experience, system integration
     */

    puts_USART1("\\r\\n=== Lab 4: Interactive Terminal ===\\r\\n");
    puts_USART1("\\r\\n┌─────────────────────────────────────┐\\r\\n");
    puts_USART1("│       ATmega128 Serial Terminal     │\\r\\n");
    puts_USART1("│                                     │\\r\\n");
    puts_USART1("│  Commands:                          │\\r\\n");
    puts_USART1("│    echo <text>  - Echo text         │\\r\\n");
    puts_USART1("│    uptime       - Show uptime       │\\r\\n");
    puts_USART1("│    ledshow      - LED light show    │\\r\\n");
    puts_USART1("│    stats        - Show statistics   │\\r\\n");
    puts_USART1("│    exit         - Exit terminal     │\\r\\n");
    puts_USART1("└─────────────────────────────────────┘\\r\\n\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "INTERACTIVE TERM");
    lcd_string(1, 0, "Full featured");

    uint32_t start_time = 0; // Simple uptime counter
    uint8_t session_commands = 0;

    send_prompt();
    clear_buffer();

    while (1)
    {
        start_time++; // Increment uptime

        if (UCSR1A & (1 << RXC1))
        {
            char ch = UDR1;
            putch_USART1(ch);

            if (ch == '\\r' || ch == '\\n')
            {
                puts_USART1("\\r\\n");

                if (strlen(input_buffer) > 0)
                {
                    session_commands++;

                    if (strncmp(input_buffer, "echo ", 5) == 0)
                    {
                        puts_USART1("Echo: ");
                        puts_USART1(&input_buffer[5]);
                        puts_USART1("\\r\\n");
                    }
                    else if (strcmp(input_buffer, "uptime") == 0)
                    {
                        char uptime_msg[50];
                        sprintf(uptime_msg, "Uptime: %ld seconds\\r\\n", start_time / 100);
                        puts_USART1(uptime_msg);
                    }
                    else if (strcmp(input_buffer, "ledshow") == 0)
                    {
                        puts_USART1("LED light show starting...\\r\\n");
                        for (uint8_t i = 0; i < 8; i++)
                        {
                            for (uint8_t led = 0; led < 4; led++)
                            {
                                PORTB |= (1 << led);
                                _delay_ms(100);
                                PORTB &= ~(1 << led);
                            }
                        }
                        puts_USART1("Light show complete!\\r\\n");
                    }
                    else if (strcmp(input_buffer, "stats") == 0)
                    {
                        char stats[100];
                        sprintf(stats, "Session Statistics:\\r\\n");
                        puts_USART1(stats);
                        sprintf(stats, "  Commands: %d\\r\\n", session_commands);
                        puts_USART1(stats);
                        sprintf(stats, "  Characters: %d\\r\\n", char_count);
                        puts_USART1(stats);
                        sprintf(stats, "  Score: %d points\\r\\n", lab_score);
                        puts_USART1(stats);
                    }
                    else if (strcmp(input_buffer, "exit") == 0)
                    {
                        puts_USART1("Terminal session ended. Goodbye!\\r\\n");
                        break;
                    }
                    else
                    {
                        puts_USART1("Unknown command. Available: echo, uptime, ledshow, stats, exit\\r\\n");
                    }

                    // Update LCD
                    char lcd_msg[20];
                    sprintf(lcd_msg, "Cmds: %d", session_commands);
                    lcd_string(3, 0, lcd_msg);

                    sprintf(lcd_msg, "Up: %lds", start_time / 100);
                    lcd_string(4, 0, lcd_msg);
                }

                clear_buffer();
                send_prompt();
            }
            else if (ch == 8 || ch == 127)
            {
                if (buffer_index > 0)
                {
                    buffer_index--;
                    input_buffer[buffer_index] = '\\0';
                    puts_USART1("\\b \\b");
                }
            }
            else if (buffer_index < (BUFFER_SIZE - 1))
            {
                input_buffer[buffer_index] = ch;
                buffer_index++;
                input_buffer[buffer_index] = '\\0';
                char_count++;
            }
        }

        _delay_ms(10);
    }

    if (session_commands >= 3)
    {
        lab_score += 200;
        puts_USART1("✓ Interactive terminal mastered!\\r\\n");
    }
}

/*
 * =============================================================================
 * LAB MAIN PROGRAM - EXERCISE SELECTION
 * =============================================================================
 */

void show_lab_menu(void)
{
    puts_USART1("\\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("    SERIAL POLLING ECHO - LAB EXERCISES      \\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("1. Basic Character Echo                      \\r\\n");
    puts_USART1("2. String Echo and Buffering                 \\r\\n");
    puts_USART1("3. Command Processing System                 \\r\\n");
    puts_USART1("4. Interactive Terminal Application          \\r\\n");
    puts_USART1("                                              \\r\\n");
    puts_USART1("0. Run All Exercises                         \\r\\n");
    puts_USART1("X. Exit Lab                                   \\r\\n");
    puts_USART1("==============================================\\r\\n");
    char score_msg[50];
    sprintf(score_msg, "Current Score: %d points\\r\\n", lab_score);
    puts_USART1(score_msg);
    puts_USART1("Select exercise (1-4, 0, X): ");
}

int main(void)
{
    init_devices();

    // Initialize LEDs as outputs
    DDRB |= 0x0F;   // PB0-PB3 as outputs for LEDs
    PORTB &= ~0x0F; // Turn off all LEDs

    puts_USART1("\\r\\n*** SERIAL POLLING ECHO LAB SESSION ***\\r\\n");
    puts_USART1("Welcome to hands-on UART communication!\\r\\n");
    puts_USART1("Ensure terminal is set to 9600 baud, 8N1\\r\\n");

    lcd_clear();
    lcd_string(1, 0, "SERIAL COMM LAB");
    lcd_string(2, 0, "UART Echo & Commands");
    lcd_string(4, 0, "Use Serial Menu");

    while (1)
    {
        show_lab_menu();
        char choice = getch_USART1();
        putch_USART1(choice);
        putch_USART1('\\r');
        putch_USART1('\\n');

        switch (choice)
        {
        case '1':
            lab_ex1_simple_echo();
            lab_ex1_echo_with_formatting();
            break;

        case '2':
            lab_ex2_line_echo();
            break;

        case '3':
            lab_ex3_command_processor();
            break;

        case '4':
            lab_ex4_interactive_terminal();
            break;

        case '0':
            puts_USART1("\\r\\n*** RUNNING ALL EXERCISES ***\\r\\n");
            lab_ex1_simple_echo();
            lab_ex1_echo_with_formatting();
            lab_ex2_line_echo();
            lab_ex3_command_processor();
            lab_ex4_interactive_terminal();

            char final_buffer[80];
            sprintf(final_buffer, "\\r\\n*** ALL EXERCISES COMPLETE! ***\\r\\nFinal Score: %d points\\r\\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\\r\\nExiting lab. Great work on serial communication!\\r\\n");
            puts_USART1("Remember: UART settings must match on both ends!\\r\\n");
            lcd_clear();
            lcd_string(2, 0, "LAB COMPLETE!");
            char exit_score[30];
            sprintf(exit_score, "Score: %d pts", lab_score);
            lcd_string(3, 0, exit_score);
            while (1)
                ;

        default:
            puts_USART1("Invalid choice. Please try again.\\r\\n");
        }

        puts_USART1("\\r\\nPress any key to continue...\\r\\n");
        getch_USART1();
    }

    return 0;
}