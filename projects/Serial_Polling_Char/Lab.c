/*
 * =============================================================================
 * SERIAL COMMUNICATION - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master UART communication through practical exercises
 * DURATION: 90 minutes
 * DIFFICULTY: Intermediate
 *
 * STUDENTS WILL:
 * - Build echo servers and clients
 * - Implement command parsers
 * - Create binary communication protocols
 * - Design packet framing systems
 * - Test multi-device communication
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - UART connection (USB-TTL or RS232)
 * - Optional: Second ATmega128 for device-to-device
 *
 * =============================================================================
 */

#include "config.h"

// Lab configuration
#define BUFFER_SIZE 128
#define MAX_COMMANDS 10
#define PACKET_START 0xAA
#define PACKET_END 0x55

// Global variables
uint16_t lab_score = 0;
char rx_buffer[BUFFER_SIZE];
uint8_t rx_index = 0;
uint32_t bytes_tx = 0;
uint32_t bytes_rx = 0;
uint16_t packets_sent = 0;
uint16_t packets_received = 0;

/*
 * =============================================================================
 * LAB EXERCISE 1: ECHO SYSTEMS (15 minutes)
 * =============================================================================
 * OBJECTIVE: Understand basic UART echo and character handling
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex1_simple_echo(void)
{
    /*
     * CHALLENGE: Create a character echo server
     * TASK: Echo back every character received
     * LEARNING: Basic UART TX/RX operations
     */

    puts_USART1("\r\n=== Lab 1.1: Simple Echo Server ===\r\n");
    puts_USART1("Type characters - they will be echoed back\r\n");
    puts_USART1("Press ESC to exit\r\n\r\n");

    uint16_t char_count = 0;

    while (1)
    {
        if (UCSR1A & (1 << RXC1)) // Character received
        {
            char c = UDR1;
            char_count++;

            // Exit on ESC
            if (c == 27) // ESC key
            {
                break;
            }

            // Echo character
            putch_USART1(c);

            // Show count every 10 characters
            if (char_count % 10 == 0)
            {
                char buffer[40];
                sprintf(buffer, " [%u chars]\r\n", char_count);
                puts_USART1(buffer);
            }
        }
    }

    char summary[60];
    sprintf(summary, "\r\nEcho complete! Total characters: %u\r\n", char_count);
    puts_USART1(summary);

    lab_score += 50;
}

void lab_ex1_uppercase_converter(void)
{
    /*
     * CHALLENGE: Echo with transformation
     * TASK: Convert lowercase to uppercase
     * LEARNING: Character manipulation
     */

    puts_USART1("\r\n=== Lab 1.2: Uppercase Converter ===\r\n");
    puts_USART1("Type text - lowercase will be converted to UPPERCASE\r\n");
    puts_USART1("Press ESC to exit\r\n\r\n");

    uint16_t converted = 0;

    while (1)
    {
        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;

            if (c == 27)
                break; // ESC

            // Convert to uppercase
            if (c >= 'a' && c <= 'z')
            {
                c = c - 32;
                converted++;
            }

            putch_USART1(c);
        }
    }

    char summary[60];
    sprintf(summary, "\r\n\r\nConverted %u characters to uppercase\r\n", converted);
    puts_USART1(summary);

    lab_score += 50;
}

void lab_ex1_line_echo(void)
{
    /*
     * CHALLENGE: Line-based echo with editing
     * TASK: Buffer a line, support backspace, echo on Enter
     * LEARNING: Line buffering and editing
     */

    puts_USART1("\r\n=== Lab 1.3: Line Echo with Editing ===\r\n");
    puts_USART1("Type a line and press Enter to echo\r\n");
    puts_USART1("Backspace works! Type 'quit' to exit\r\n\r\n");

    uint16_t lines = 0;

    while (1)
    {
        rx_index = 0;
        puts_USART1("> ");

        // Read a line
        while (1)
        {
            if (UCSR1A & (1 << RXC1))
            {
                char c = UDR1;

                if (c == '\r' || c == '\n') // Enter
                {
                    rx_buffer[rx_index] = '\0';
                    puts_USART1("\r\n");
                    break;
                }
                else if (c == 8 || c == 127) // Backspace or DEL
                {
                    if (rx_index > 0)
                    {
                        rx_index--;
                        puts_USART1("\b \b"); // Erase character on screen
                    }
                }
                else if (rx_index < BUFFER_SIZE - 1)
                {
                    rx_buffer[rx_index++] = c;
                    putch_USART1(c); // Echo
                }
            }
        }

        // Check for quit
        if (strcmp(rx_buffer, "quit") == 0)
        {
            break;
        }

        // Echo the line
        puts_USART1("Echo: ");
        puts_USART1(rx_buffer);
        puts_USART1("\r\n");

        lines++;
    }

    char summary[50];
    sprintf(summary, "\r\nProcessed %u lines\r\n", lines);
    puts_USART1(summary);

    lab_score += 75;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: COMMAND PARSING (20 minutes)
 * =============================================================================
 * OBJECTIVE: Implement command-line interface
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

typedef struct
{
    char command[20];
    uint8_t arg_count;
    int16_t args[4];
} Command;

Command parse_command(char *input)
{
    Command cmd = {0};
    char *token;

    // Get command name
    token = strtok(input, " ");
    if (token != NULL)
    {
        strncpy(cmd.command, token, 19);

        // Get arguments
        while ((token = strtok(NULL, " ")) != NULL && cmd.arg_count < 4)
        {
            cmd.args[cmd.arg_count++] = atoi(token);
        }
    }

    return cmd;
}

void lab_ex2_calculator(void)
{
    /*
     * CHALLENGE: Command-line calculator
     * TASK: Parse and execute math commands
     * LEARNING: Command parsing and execution
     */

    puts_USART1("\r\n=== Lab 2.1: Command-Line Calculator ===\r\n");
    puts_USART1("Commands:\r\n");
    puts_USART1("  add <a> <b>      - Add two numbers\r\n");
    puts_USART1("  sub <a> <b>      - Subtract\r\n");
    puts_USART1("  mul <a> <b>      - Multiply\r\n");
    puts_USART1("  div <a> <b>      - Divide\r\n");
    puts_USART1("  quit             - Exit\r\n\r\n");

    uint16_t calculations = 0;

    while (1)
    {
        rx_index = 0;
        puts_USART1("calc> ");

        // Read command line
        while (1)
        {
            if (UCSR1A & (1 << RXC1))
            {
                char c = UDR1;

                if (c == '\r' || c == '\n')
                {
                    rx_buffer[rx_index] = '\0';
                    puts_USART1("\r\n");
                    break;
                }
                else if (c == 8 || c == 127)
                {
                    if (rx_index > 0)
                    {
                        rx_index--;
                        puts_USART1("\b \b");
                    }
                }
                else if (rx_index < BUFFER_SIZE - 1)
                {
                    rx_buffer[rx_index++] = c;
                    putch_USART1(c);
                }
            }
        }

        // Parse command
        Command cmd = parse_command(rx_buffer);

        // Execute command
        char result[80];

        if (strcmp(cmd.command, "quit") == 0)
        {
            break;
        }
        else if (strcmp(cmd.command, "add") == 0 && cmd.arg_count == 2)
        {
            int16_t sum = cmd.args[0] + cmd.args[1];
            sprintf(result, "Result: %d + %d = %d\r\n", cmd.args[0], cmd.args[1], sum);
            puts_USART1(result);
            calculations++;
        }
        else if (strcmp(cmd.command, "sub") == 0 && cmd.arg_count == 2)
        {
            int16_t diff = cmd.args[0] - cmd.args[1];
            sprintf(result, "Result: %d - %d = %d\r\n", cmd.args[0], cmd.args[1], diff);
            puts_USART1(result);
            calculations++;
        }
        else if (strcmp(cmd.command, "mul") == 0 && cmd.arg_count == 2)
        {
            int16_t prod = cmd.args[0] * cmd.args[1];
            sprintf(result, "Result: %d * %d = %d\r\n", cmd.args[0], cmd.args[1], prod);
            puts_USART1(result);
            calculations++;
        }
        else if (strcmp(cmd.command, "div") == 0 && cmd.arg_count == 2)
        {
            if (cmd.args[1] != 0)
            {
                int16_t quot = cmd.args[0] / cmd.args[1];
                int16_t rem = cmd.args[0] % cmd.args[1];
                sprintf(result, "Result: %d / %d = %d remainder %d\r\n",
                        cmd.args[0], cmd.args[1], quot, rem);
                puts_USART1(result);
                calculations++;
            }
            else
            {
                puts_USART1("Error: Division by zero!\r\n");
            }
        }
        else
        {
            puts_USART1("Unknown command or wrong arguments\r\n");
        }
    }

    char summary[60];
    sprintf(summary, "\r\nCalculations performed: %u\r\n", calculations);
    puts_USART1(summary);

    lab_score += 100;
}

void lab_ex2_led_controller(void)
{
    /*
     * CHALLENGE: LED control via serial commands
     * TASK: Parse commands to control PORTB LEDs
     * LEARNING: Hardware control via serial interface
     */

    puts_USART1("\r\n=== Lab 2.2: LED Controller ===\r\n");
    puts_USART1("Commands:\r\n");
    puts_USART1("  led <num> on     - Turn LED on (0-7)\r\n");
    puts_USART1("  led <num> off    - Turn LED off\r\n");
    puts_USART1("  led all on       - All LEDs on\r\n");
    puts_USART1("  led all off      - All LEDs off\r\n");
    puts_USART1("  quit             - Exit\r\n\r\n");

    // Configure PORTB for LEDs
    DDRB = 0xFF;
    PORTB = 0xFF; // All off (active low)

    while (1)
    {
        rx_index = 0;
        puts_USART1("led> ");

        // Read command
        while (1)
        {
            if (UCSR1A & (1 << RXC1))
            {
                char c = UDR1;

                if (c == '\r' || c == '\n')
                {
                    rx_buffer[rx_index] = '\0';
                    puts_USART1("\r\n");
                    break;
                }
                else if (c == 8 || c == 127)
                {
                    if (rx_index > 0)
                    {
                        rx_index--;
                        puts_USART1("\b \b");
                    }
                }
                else if (rx_index < BUFFER_SIZE - 1)
                {
                    rx_buffer[rx_index++] = c;
                    putch_USART1(c);
                }
            }
        }

        // Parse and execute
        if (strcmp(rx_buffer, "quit") == 0)
        {
            break;
        }
        else if (strncmp(rx_buffer, "led ", 4) == 0)
        {
            char *args = rx_buffer + 4;

            if (strncmp(args, "all on", 6) == 0)
            {
                PORTB = 0x00; // All on
                puts_USART1("All LEDs ON\r\n");
            }
            else if (strncmp(args, "all off", 7) == 0)
            {
                PORTB = 0xFF; // All off
                puts_USART1("All LEDs OFF\r\n");
            }
            else
            {
                // Parse "led <num> on/off"
                char *num_str = strtok(args, " ");
                char *state = strtok(NULL, " ");

                if (num_str != NULL && state != NULL)
                {
                    uint8_t led_num = atoi(num_str);

                    if (led_num < 8)
                    {
                        if (strcmp(state, "on") == 0)
                        {
                            PORTB &= ~(1 << led_num);
                            sprintf(rx_buffer, "LED %u ON\r\n", led_num);
                            puts_USART1(rx_buffer);
                        }
                        else if (strcmp(state, "off") == 0)
                        {
                            PORTB |= (1 << led_num);
                            sprintf(rx_buffer, "LED %u OFF\r\n", led_num);
                            puts_USART1(rx_buffer);
                        }
                    }
                }
            }
        }
        else
        {
            puts_USART1("Unknown command\r\n");
        }
    }

    PORTB = 0xFF; // All off
    puts_USART1("\r\nLED controller complete!\r\n");

    lab_score += 125;
}

/*
 * =============================================================================
 * LAB EXERCISE 3: BINARY PROTOCOLS (20 minutes)
 * =============================================================================
 * OBJECTIVE: Design and implement binary communication
 * DIFFICULTY: ★★★★☆ (Advanced)
 */

typedef struct
{
    uint8_t start;
    uint8_t type;
    uint8_t length;
    uint8_t data[16];
    uint8_t checksum;
    uint8_t end;
} Packet;

uint8_t calculate_checksum(uint8_t *data, uint8_t length)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        sum ^= data[i];
    }
    return sum;
}

void lab_ex3_packet_protocol(void)
{
    /*
     * CHALLENGE: Implement packet-based protocol
     * TASK: Send/receive packets with checksums
     * LEARNING: Framing, error detection, binary protocols
     */

    puts_USART1("\r\n=== Lab 3.1: Packet Protocol ===\r\n");
    puts_USART1("Packet format: [START][TYPE][LEN][DATA...][CHK][END]\r\n");
    puts_USART1("Commands:\r\n");
    puts_USART1("  send <type> <data>  - Send packet\r\n");
    puts_USART1("  receive             - Wait for packet\r\n");
    puts_USART1("  quit                - Exit\r\n\r\n");

    uint16_t packets_ok = 0;
    uint16_t packets_error = 0;

    while (1)
    {
        rx_index = 0;
        puts_USART1("packet> ");

        // Read command
        while (1)
        {
            if (UCSR1A & (1 << RXC1))
            {
                char c = UDR1;

                if (c == '\r' || c == '\n')
                {
                    rx_buffer[rx_index] = '\0';
                    puts_USART1("\r\n");
                    break;
                }
                else if (c == 8 || c == 127)
                {
                    if (rx_index > 0)
                    {
                        rx_index--;
                        puts_USART1("\b \b");
                    }
                }
                else if (rx_index < BUFFER_SIZE - 1)
                {
                    rx_buffer[rx_index++] = c;
                    putch_USART1(c);
                }
            }
        }

        if (strcmp(rx_buffer, "quit") == 0)
        {
            break;
        }
        else if (strncmp(rx_buffer, "send ", 5) == 0)
        {
            // Parse send command
            char *args = rx_buffer + 5;
            char *type_str = strtok(args, " ");
            char *data_str = strtok(NULL, "");

            if (type_str != NULL && data_str != NULL)
            {
                Packet pkt;
                pkt.start = PACKET_START;
                pkt.type = atoi(type_str);
                pkt.length = strlen(data_str);

                if (pkt.length > 16)
                    pkt.length = 16;

                memcpy(pkt.data, data_str, pkt.length);
                pkt.checksum = calculate_checksum(pkt.data, pkt.length);
                pkt.end = PACKET_END;

                // Send packet
                putch_USART1(pkt.start);
                putch_USART1(pkt.type);
                putch_USART1(pkt.length);
                for (uint8_t i = 0; i < pkt.length; i++)
                {
                    putch_USART1(pkt.data[i]);
                }
                putch_USART1(pkt.checksum);
                putch_USART1(pkt.end);

                char msg[80];
                sprintf(msg, "Sent packet: Type=%u, Len=%u, Checksum=0x%02X\r\n",
                        pkt.type, pkt.length, pkt.checksum);
                puts_USART1(msg);

                packets_sent++;
            }
        }
        else if (strcmp(rx_buffer, "receive") == 0)
        {
            puts_USART1("Waiting for packet (10 sec timeout)...\r\n");

            uint16_t timeout = 0;
            uint8_t state = 0;
            Packet pkt = {0};
            uint8_t data_idx = 0;

            while (timeout < 10000) // 10 second timeout
            {
                if (UCSR1A & (1 << RXC1))
                {
                    uint8_t byte = UDR1;

                    switch (state)
                    {
                    case 0: // Wait for START
                        if (byte == PACKET_START)
                        {
                            pkt.start = byte;
                            state = 1;
                        }
                        break;

                    case 1: // Get TYPE
                        pkt.type = byte;
                        state = 2;
                        break;

                    case 2: // Get LENGTH
                        pkt.length = byte;
                        if (pkt.length > 16)
                            pkt.length = 16;
                        data_idx = 0;
                        state = 3;
                        break;

                    case 3: // Get DATA
                        pkt.data[data_idx++] = byte;
                        if (data_idx >= pkt.length)
                        {
                            state = 4;
                        }
                        break;

                    case 4: // Get CHECKSUM
                        pkt.checksum = byte;
                        state = 5;
                        break;

                    case 5: // Get END
                        pkt.end = byte;

                        // Validate packet
                        uint8_t calc_chk = calculate_checksum(pkt.data, pkt.length);

                        if (pkt.end == PACKET_END && calc_chk == pkt.checksum)
                        {
                            puts_USART1("✓ Packet received successfully!\r\n");
                            char msg[100];
                            sprintf(msg, "  Type: %u\r\n  Length: %u\r\n  Data: ",
                                    pkt.type, pkt.length);
                            puts_USART1(msg);

                            for (uint8_t i = 0; i < pkt.length; i++)
                            {
                                putch_USART1(pkt.data[i]);
                            }

                            sprintf(msg, "\r\n  Checksum: 0x%02X (valid)\r\n", pkt.checksum);
                            puts_USART1(msg);

                            packets_ok++;
                        }
                        else
                        {
                            puts_USART1("✗ Packet error! Invalid checksum or framing\r\n");
                            packets_error++;
                        }

                        goto receive_done;
                    }
                }

                _delay_ms(1);
                timeout++;
            }

            puts_USART1("Timeout - no packet received\r\n");

        receive_done:;
        }
    }

    char summary[100];
    sprintf(summary, "\r\nPacket Statistics:\r\n  Sent: %u\r\n  Received OK: %u\r\n  Errors: %u\r\n",
            packets_sent, packets_ok, packets_error);
    puts_USART1(summary);

    lab_score += 150;
}

void lab_ex3_data_transfer(void)
{
    /*
     * CHALLENGE: High-speed data transfer test
     * TASK: Send large amount of data, measure throughput
     * LEARNING: Performance testing, flow control
     */

    puts_USART1("\r\n=== Lab 3.2: Data Transfer Test ===\r\n");
    puts_USART1("Testing UART throughput...\r\n\r\n");

    // Test 1: Send 1KB of data
    puts_USART1("Test 1: Sending 1KB of sequential data\r\n");

    uint32_t start_time = 0; // Would use Timer for real timing

    for (uint16_t i = 0; i < 1024; i++)
    {
        putch_USART1(i & 0xFF);
    }

    puts_USART1("✓ 1KB sent\r\n\r\n");

    // Test 2: Echo test
    puts_USART1("Test 2: Echo test (type 100 chars, will be echoed)\r\n");

    uint16_t echoed = 0;
    while (echoed < 100)
    {
        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            putch_USART1(c);
            echoed++;
        }
    }

    puts_USART1("\r\n✓ 100 chars echoed\r\n\r\n");

    // Test 3: Pattern test
    puts_USART1("Test 3: Pattern test (alternating 0x55/0xAA)\r\n");

    for (uint16_t i = 0; i < 256; i++)
    {
        putch_USART1(i % 2 ? 0xAA : 0x55);
    }

    puts_USART1("✓ 256 bytes pattern sent\r\n\r\n");

    puts_USART1("Data transfer tests complete!\r\n");
    puts_USART1("At 9600 baud: ~960 bytes/sec theoretical\r\n");

    lab_score += 100;
}

/*
 * =============================================================================
 * LAB MENU SYSTEM
 * =============================================================================
 */

void print_lab_menu(void)
{
    puts_USART1("\r\n");
    puts_USART1("========================================\r\n");
    puts_USART1("  SERIAL COMMUNICATION - LAB EXERCISES\r\n");
    puts_USART1("========================================\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 1: Echo Systems\r\n");
    puts_USART1("  1. Simple Echo Server\r\n");
    puts_USART1("  2. Uppercase Converter\r\n");
    puts_USART1("  3. Line Echo with Editing\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 2: Command Parsing\r\n");
    puts_USART1("  4. Command-Line Calculator\r\n");
    puts_USART1("  5. LED Controller\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 3: Binary Protocols\r\n");
    puts_USART1("  6. Packet Protocol\r\n");
    puts_USART1("  7. Data Transfer Test\r\n");
    puts_USART1("\r\n");
    puts_USART1("  0. Run All Exercises\r\n");
    puts_USART1("  X. Exit Lab\r\n");
    puts_USART1("\r\n");
    char score_str[40];
    sprintf(score_str, "Current Score: %u points\r\n\r\n", lab_score);
    puts_USART1(score_str);
    puts_USART1("Select exercise (1-7, 0, X): ");
}

int main(void)
{
    // Initialize system
    init_devices();
    Uart1_init();

    _delay_ms(100);

    puts_USART1("\r\n\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("*  ATmega128 SERIAL COMMUNICATION LAB          *\r\n");
    puts_USART1("*  Hands-On UART Exercises                     *\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("\r\n");
    puts_USART1("Welcome to the Serial Communication Lab!\r\n");
    puts_USART1("Master UART through practical exercises.\r\n");

    while (1)
    {
        print_lab_menu();

        char choice = getch_USART1();
        putch_USART1(choice);
        putch_USART1('\r');
        putch_USART1('\n');

        switch (choice)
        {
        case '1':
            lab_ex1_simple_echo();
            break;
        case '2':
            lab_ex1_uppercase_converter();
            break;
        case '3':
            lab_ex1_line_echo();
            break;
        case '4':
            lab_ex2_calculator();
            break;
        case '5':
            lab_ex2_led_controller();
            break;
        case '6':
            lab_ex3_packet_protocol();
            break;
        case '7':
            lab_ex3_data_transfer();
            break;

        case '0':
            puts_USART1("\r\n*** RUNNING ALL EXERCISES ***\r\n");
            lab_ex1_simple_echo();
            lab_ex1_uppercase_converter();
            lab_ex1_line_echo();
            lab_ex2_calculator();
            lab_ex2_led_controller();
            lab_ex3_packet_protocol();
            lab_ex3_data_transfer();

            char final_buffer[80];
            sprintf(final_buffer, "\r\n*** ALL EXERCISES COMPLETE! ***\r\nFinal Score: %u points\r\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\r\nExiting lab. Great work!\r\n");
            while (1)
                ;

        default:
            puts_USART1("Invalid choice. Please try again.\r\n");
        }

        puts_USART1("\r\nPress any key to continue...\r\n");
        getch_USART1();
    }

    return 0;
}
