/*
 * =============================================================================
 * SERIAL POLLING STRING - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master string-based UART communication and text processing
 * DURATION: 70 minutes
 * DIFFICULTY: Intermediate
 *
 * STUDENTS WILL:
 * - Implement string transmission and reception via UART
 * - Create command parsing and response systems
 * - Build text-based communication protocols
 * - Handle variable-length messages and buffering
 * - Implement error detection and message validation
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - UART connection (USB-to-serial or direct connection)
 * - Terminal software (PuTTY, Arduino IDE Serial Monitor, etc.)
 * - Status LEDs for communication indication
 * - Optional: Second microcontroller for bidirectional communication
 * - Optional: External EEPROM for message logging
 *
 * COMMUNICATION FEATURES:
 * - String-based command interface
 * - Message framing and termination
 * - Command parsing and parameter extraction
 * - Response generation and formatting
 * - Buffer management for variable-length data
 *
 * LAB STRUCTURE:
 * - Exercise 1: Basic string transmission and reception (20 min)
 * - Exercise 2: Command parsing and execution system (20 min)
 * - Exercise 3: Bidirectional communication protocol (20 min)
 * - Exercise 4: Advanced messaging with validation (10 min)
 *
 * =============================================================================
 */

#include "config.h"

// String communication configuration
#define MAX_COMMAND_LENGTH 64
#define MAX_RESPONSE_LENGTH 128
#define MAX_PARAMETERS 8
#define COMMAND_BUFFER_SIZE 256
#define MESSAGE_TIMEOUT 5000 // ms

// Communication status indicators
#define TX_LED_PIN 6    // PB6 - Transmission indicator
#define RX_LED_PIN 7    // PB7 - Reception indicator
#define ERROR_LED_PIN 5 // PB5 - Error indicator

// Message framing characters
#define MSG_START_CHAR '<'
#define MSG_END_CHAR '>'
#define PARAM_SEPARATOR ','
#define CMD_TERMINATOR '\r'

// Command definitions
#define CMD_HELLO "HELLO"
#define CMD_STATUS "STATUS"
#define CMD_SET "SET"
#define CMD_GET "GET"
#define CMD_RESET "RESET"
#define CMD_HELP "HELP"
#define CMD_ECHO "ECHO"
#define CMD_VERSION "VERSION"

// Lab session variables
uint16_t lab_score = 0;
uint32_t messages_sent = 0;
uint32_t messages_received = 0;
uint16_t command_executions = 0;
uint16_t parsing_errors = 0;

// Communication buffers
char rx_buffer[COMMAND_BUFFER_SIZE];
char tx_buffer[MAX_RESPONSE_LENGTH];
char command_buffer[MAX_COMMAND_LENGTH];
char parameters[MAX_PARAMETERS][16];
uint8_t rx_index = 0;
uint8_t param_count = 0;

// System state variables
uint16_t system_temperature = 250; // 25.0°C
uint8_t system_status = 1;         // 1 = OK, 0 = Error
uint16_t sensor_threshold = 500;
char device_name[20] = "ATmega128-Lab";

/*
 * =============================================================================
 * STRING COMMUNICATION FUNCTIONS
 * =============================================================================
 */

void comm_init(void)
{
    // Configure communication indicator LEDs
    DDRB |= (1 << TX_LED_PIN) | (1 << RX_LED_PIN) | (1 << ERROR_LED_PIN);

    // Initialize all LEDs off
    PORTB &= ~((1 << TX_LED_PIN) | (1 << RX_LED_PIN) | (1 << ERROR_LED_PIN));

    // Clear communication buffers
    memset(rx_buffer, 0, COMMAND_BUFFER_SIZE);
    memset(tx_buffer, 0, MAX_RESPONSE_LENGTH);
    memset(command_buffer, 0, MAX_COMMAND_LENGTH);

    rx_index = 0;
    param_count = 0;
}

void tx_led_flash(void)
{
    PORTB |= (1 << TX_LED_PIN);
    _delay_ms(100);
    PORTB &= ~(1 << TX_LED_PIN);
}

void rx_led_flash(void)
{
    PORTB |= (1 << RX_LED_PIN);
    _delay_ms(100);
    PORTB &= ~(1 << RX_LED_PIN);
}

void error_led_flash(void)
{
    PORTB |= (1 << ERROR_LED_PIN);
    _delay_ms(300);
    PORTB &= ~(1 << ERROR_LED_PIN);
}

void send_string(char *str)
{
    tx_led_flash();
    puts_USART1(str);
    messages_sent++;
}

void send_formatted_response(char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(tx_buffer, MAX_RESPONSE_LENGTH, format, args);
    va_end(args);

    send_string(tx_buffer);
}

uint8_t receive_string_timeout(char *buffer, uint16_t max_length, uint16_t timeout_ms)
{
    uint16_t elapsed = 0;
    uint8_t index = 0;
    char received_char;

    memset(buffer, 0, max_length);

    while (elapsed < timeout_ms && index < max_length - 1)
    {
        if (USART1_Data_Ready())
        {
            received_char = getch_USART1();
            rx_led_flash();

            if (received_char == '\r' || received_char == '\n')
            {
                buffer[index] = '\0';
                messages_received++;
                return 1; // Success
            }
            else if (received_char >= 32 && received_char <= 126)
            {
                buffer[index++] = received_char;
                putch_USART1(received_char); // Echo character
            }
        }

        _delay_ms(1);
        elapsed++;
    }

    buffer[index] = '\0';
    return 0; // Timeout or buffer full
}

uint8_t parse_command(char *input_string)
{
    // Clear previous command and parameters
    memset(command_buffer, 0, MAX_COMMAND_LENGTH);
    param_count = 0;

    for (uint8_t i = 0; i < MAX_PARAMETERS; i++)
    {
        memset(parameters[i], 0, 16);
    }

    // Skip leading whitespace
    while (*input_string == ' ' || *input_string == '\t')
    {
        input_string++;
    }

    // Extract command (first word)
    uint8_t cmd_index = 0;
    while (*input_string && *input_string != ' ' && *input_string != PARAM_SEPARATOR &&
           cmd_index < MAX_COMMAND_LENGTH - 1)
    {
        command_buffer[cmd_index++] = toupper(*input_string++);
    }
    command_buffer[cmd_index] = '\0';

    // Extract parameters
    while (*input_string && param_count < MAX_PARAMETERS)
    {
        // Skip separators and whitespace
        while (*input_string == ' ' || *input_string == PARAM_SEPARATOR || *input_string == '\t')
        {
            input_string++;
        }

        if (*input_string == '\0')
            break;

        // Extract parameter
        uint8_t param_index = 0;
        while (*input_string && *input_string != ' ' && *input_string != PARAM_SEPARATOR &&
               param_index < 15)
        {
            parameters[param_count][param_index++] = *input_string++;
        }
        parameters[param_count][param_index] = '\0';

        if (param_index > 0)
        {
            param_count++;
        }
    }

    return (cmd_index > 0) ? 1 : 0;
}

void execute_command(void)
{
    command_executions++;

    if (strcmp(command_buffer, CMD_HELLO) == 0)
    {
        send_formatted_response("Hello from %s! System ready.\r\n", device_name);
    }
    else if (strcmp(command_buffer, CMD_STATUS) == 0)
    {
        send_formatted_response("Status: %s, Temp: %d.%d°C, Threshold: %d\r\n",
                                system_status ? "OK" : "ERROR",
                                system_temperature / 10, system_temperature % 10,
                                sensor_threshold);
    }
    else if (strcmp(command_buffer, CMD_SET) == 0)
    {
        if (param_count >= 2)
        {
            if (strcmp(parameters[0], "TEMP") == 0)
            {
                system_temperature = atoi(parameters[1]);
                send_formatted_response("Temperature set to %d.%d°C\r\n",
                                        system_temperature / 10, system_temperature % 10);
            }
            else if (strcmp(parameters[0], "THRESHOLD") == 0)
            {
                sensor_threshold = atoi(parameters[1]);
                send_formatted_response("Threshold set to %d\r\n", sensor_threshold);
            }
            else if (strcmp(parameters[0], "NAME") == 0)
            {
                strncpy(device_name, parameters[1], 19);
                device_name[19] = '\0';
                send_formatted_response("Device name set to %s\r\n", device_name);
            }
            else
            {
                send_string("ERROR: Unknown parameter\r\n");
                parsing_errors++;
            }
        }
        else
        {
            send_string("ERROR: SET command requires parameter and value\r\n");
            parsing_errors++;
        }
    }
    else if (strcmp(command_buffer, CMD_GET) == 0)
    {
        if (param_count >= 1)
        {
            if (strcmp(parameters[0], "TEMP") == 0)
            {
                send_formatted_response("TEMP=%d.%d\r\n",
                                        system_temperature / 10, system_temperature % 10);
            }
            else if (strcmp(parameters[0], "THRESHOLD") == 0)
            {
                send_formatted_response("THRESHOLD=%d\r\n", sensor_threshold);
            }
            else if (strcmp(parameters[0], "NAME") == 0)
            {
                send_formatted_response("NAME=%s\r\n", device_name);
            }
            else
            {
                send_string("ERROR: Unknown parameter\r\n");
                parsing_errors++;
            }
        }
        else
        {
            send_string("ERROR: GET command requires parameter\r\n");
            parsing_errors++;
        }
    }
    else if (strcmp(command_buffer, CMD_RESET) == 0)
    {
        system_temperature = 250;
        sensor_threshold = 500;
        strcpy(device_name, "ATmega128-Lab");
        system_status = 1;
        send_string("System reset to defaults\r\n");
    }
    else if (strcmp(command_buffer, CMD_ECHO) == 0)
    {
        send_string("ECHO: ");
        for (uint8_t i = 0; i < param_count; i++)
        {
            send_string(parameters[i]);
            if (i < param_count - 1)
                send_string(" ");
        }
        send_string("\r\n");
    }
    else if (strcmp(command_buffer, CMD_VERSION) == 0)
    {
        send_string("ATmega128 String Communication Lab v1.0\r\n");
    }
    else if (strcmp(command_buffer, CMD_HELP) == 0)
    {
        send_string("Available commands:\r\n");
        send_string("  HELLO - Greeting message\r\n");
        send_string("  STATUS - System status\r\n");
        send_string("  SET <param> <value> - Set parameter\r\n");
        send_string("  GET <param> - Get parameter value\r\n");
        send_string("  RESET - Reset to defaults\r\n");
        send_string("  ECHO <text> - Echo back text\r\n");
        send_string("  VERSION - Show version info\r\n");
        send_string("  HELP - This help message\r\n");
    }
    else
    {
        send_formatted_response("ERROR: Unknown command '%s'. Type HELP for commands.\r\n", command_buffer);
        parsing_errors++;
        error_led_flash();
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 1: BASIC STRING TRANSMISSION (20 minutes)
 * =============================================================================
 * OBJECTIVE: Implement fundamental string communication
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex1_string_transmission(void)
{
    /*
     * CHALLENGE: Send and receive complete strings via UART
     * TASK: Implement string transmission with proper termination
     * LEARNING: String handling, UART buffering, message framing
     */

    puts_USART1("\r\n=== Lab 1: String Transmission ===\r\n");
    puts_USART1("Testing basic string communication\r\n");

    lcd_clear();
    lcd_string(0, 0, "STRING COMM LAB");
    lcd_string(1, 0, "Basic transmission");

    comm_init();

    // Test 1: Send predefined strings
    puts_USART1("Test 1: Sending predefined messages\r\n");
    lcd_string(3, 0, "Sending messages");

    char test_messages[][50] = {
        "Hello, World!",
        "ATmega128 String Communication Test",
        "This is message number 3",
        "Testing special chars: !@#$%^&*()",
        "Final test message"};

    for (uint8_t i = 0; i < 5; i++)
    {
        char msg_header[60];
        sprintf(msg_header, "Message %d: %s\r\n", i + 1, test_messages[i]);
        send_string(msg_header);

        char lcd_msg[20];
        sprintf(lcd_msg, "Sent msg %d/5", i + 1);
        lcd_string(4, 0, lcd_msg);

        _delay_ms(1000);
    }

    // Test 2: Interactive string reception
    puts_USART1("\r\nTest 2: String reception test\r\n");
    puts_USART1("Please type a message and press Enter:\r\n");
    lcd_string(3, 0, "Type a message");
    lcd_string(4, 0, "Press Enter");

    char received_message[64];
    if (receive_string_timeout(received_message, 64, 15000))
    {
        char response[80];
        sprintf(response, "Received: '%s' (length: %d)\r\n",
                received_message, strlen(received_message));
        send_string(response);

        lcd_string(5, 0, "Message received!");
        lab_score += 100;
    }
    else
    {
        send_string("Reception timeout or buffer full\r\n");
        lcd_string(5, 0, "Reception timeout");
    }

    _delay_ms(2000);
}

void lab_ex1_message_formatting(void)
{
    /*
     * CHALLENGE: Create formatted message output functions
     * TASK: Build dynamic string formatting and response generation
     * LEARNING: String formatting, variable substitution, message templates
     */

    puts_USART1("\r\n=== Lab 1.2: Message Formatting ===\r\n");
    puts_USART1("Testing formatted message generation\r\n");

    lcd_clear();
    lcd_string(0, 0, "MESSAGE FORMAT");
    lcd_string(1, 0, "Dynamic generation");

    // Test sensor data formatting
    puts_USART1("Generating sensor data reports...\r\n");

    for (uint8_t reading = 0; reading < 8; reading++)
    {
        // Simulate sensor readings
        uint16_t temperature = 200 + reading * 25 + (rand() % 20);
        uint16_t humidity = 400 + reading * 15 + (rand() % 30);
        uint16_t pressure = 1000 + (rand() % 50);

        // Format comprehensive sensor report
        send_formatted_response("SENSOR_REPORT:%d,TEMP=%d.%d,HUMID=%d.%d,PRESS=%d\r\n",
                                reading + 1,
                                temperature / 10, temperature % 10,
                                humidity / 10, humidity % 10,
                                pressure);

        // Format human-readable version
        send_formatted_response("Reading %d: Temperature %d.%d°C, Humidity %d.%d%%, Pressure %d hPa\r\n",
                                reading + 1,
                                temperature / 10, temperature % 10,
                                humidity / 10, humidity % 10,
                                pressure);

        char lcd_reading[20];
        sprintf(lcd_reading, "Reading %d/8", reading + 1);
        lcd_string(3, 0, lcd_reading);

        char lcd_temp[20];
        sprintf(lcd_temp, "T:%d.%d H:%d.%d",
                temperature / 10, temperature % 10,
                humidity / 10, humidity % 10);
        lcd_string(4, 0, lcd_temp);

        _delay_ms(1500);
    }

    // Test error message formatting
    puts_USART1("\r\nTesting error message formatting...\r\n");

    char error_conditions[][30] = {
        "Temperature too high",
        "Sensor disconnected",
        "Communication timeout",
        "Invalid parameter"};

    for (uint8_t err = 0; err < 4; err++)
    {
        send_formatted_response("ERROR:%d,%s,TIMESTAMP=%ld\r\n",
                                err + 100, error_conditions[err], messages_sent);

        char lcd_error[20];
        sprintf(lcd_error, "Error %d/4", err + 1);
        lcd_string(5, 0, lcd_error);

        _delay_ms(800);
    }

    char summary[60];
    sprintf(summary, "Message formatting complete: %ld sent\r\n", messages_sent);
    send_string(summary);

    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: COMMAND PARSING SYSTEM (20 minutes)
 * =============================================================================
 * OBJECTIVE: Build a comprehensive command-line interface
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex2_command_interface(void)
{
    /*
     * CHALLENGE: Create a complete command parsing and execution system
     * TASK: Implement command recognition, parameter extraction, and execution
     * LEARNING: String parsing, command interfaces, parameter handling
     */

    puts_USART1("\r\n=== Lab 2: Command Interface ===\r\n");
    puts_USART1("Building command parsing system\r\n");

    lcd_clear();
    lcd_string(0, 0, "COMMAND INTERFACE");
    lcd_string(1, 0, "Parser & executor");

    // Display available commands
    send_string("\r\n*** ATmega128 Command Interface ***\r\n");
    send_string("Type 'HELP' for available commands\r\n");
    send_string("Command> ");

    uint8_t demo_commands = 0;
    char demo_command_list[][30] = {
        "HELLO",
        "STATUS",
        "SET TEMP 275",
        "GET TEMP",
        "SET NAME TestDevice",
        "GET NAME",
        "ECHO Hello World",
        "VERSION",
        "RESET",
        "HELP"};

    while (demo_commands < 10)
    {
        char input_command[64];

        // For demonstration, use predefined commands
        strcpy(input_command, demo_command_list[demo_commands]);
        send_formatted_response("%s\r\n", input_command);

        char lcd_cmd[20];
        sprintf(lcd_cmd, "Cmd %d: %s", demo_commands + 1,
                demo_commands < 7 ? demo_command_list[demo_commands] : "...");
        lcd_string(3, 0, lcd_cmd);

        if (parse_command(input_command))
        {
            char parse_msg[80];
            sprintf(parse_msg, "Parsed: CMD='%s', PARAMS=%d\r\n", command_buffer, param_count);
            send_string(parse_msg);

            // Show extracted parameters
            if (param_count > 0)
            {
                send_string("Parameters: ");
                for (uint8_t p = 0; p < param_count; p++)
                {
                    send_formatted_response("'%s'", parameters[p]);
                    if (p < param_count - 1)
                        send_string(", ");
                }
                send_string("\r\n");
            }

            // Execute the command
            execute_command();

            char lcd_status[20];
            sprintf(lcd_status, "Exec: %d errors", parsing_errors);
            lcd_string(4, 0, lcd_status);
        }
        else
        {
            send_string("ERROR: Failed to parse command\r\n");
            parsing_errors++;
            error_led_flash();
        }

        send_string("\r\nCommand> ");
        demo_commands++;
        _delay_ms(2000);
    }

    char final_stats[80];
    sprintf(final_stats, "\r\nCommand demo complete: %d executed, %d errors\r\n",
            command_executions, parsing_errors);
    send_string(final_stats);

    lcd_string(5, 0, "Commands complete");

    if (parsing_errors <= 2)
    {
        lab_score += 200;
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 3: BIDIRECTIONAL COMMUNICATION (20 minutes)
 * =============================================================================
 * OBJECTIVE: Implement robust bidirectional messaging
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex3_bidirectional_protocol(void)
{
    /*
     * CHALLENGE: Create a structured communication protocol
     * TASK: Implement message framing, acknowledgments, and error handling
     * LEARNING: Communication protocols, message validation, error recovery
     */

    puts_USART1("\r\n=== Lab 3: Bidirectional Protocol ===\r\n");
    puts_USART1("Implementing structured messaging protocol\r\n");

    lcd_clear();
    lcd_string(0, 0, "BIDIRECTIONAL");
    lcd_string(1, 0, "Protocol & framing");

    // Protocol demonstration with framed messages
    send_string("\r\n*** Structured Messaging Protocol ***\r\n");
    send_string("Format: <COMMAND,PARAM1,PARAM2,...>\r\n");
    send_string("Acknowledgment: ACK or NAK\r\n\r\n");

    // Simulate structured message exchange
    typedef struct
    {
        uint8_t message_id;
        char command[16];
        char parameters[64];
        uint8_t acknowledged;
    } protocol_message_t;

    protocol_message_t messages[] = {
        {1, "SENSOR_REQ", "TEMP,HUMID", 0},
        {2, "CONFIG_SET", "THRESHOLD,600", 0},
        {3, "DATA_LOG", "START,60", 0},
        {4, "ALERT_SUB", "HIGH_TEMP,LOW_BATT", 0},
        {5, "STATUS_REQ", "", 0}};

    uint8_t total_messages = 5;
    uint8_t successful_acks = 0;

    for (uint8_t msg = 0; msg < total_messages; msg++)
    {
        // Send framed message
        if (strlen(messages[msg].parameters) > 0)
        {
            send_formatted_response("<%d,%s,%s>\r\n",
                                    messages[msg].message_id,
                                    messages[msg].command,
                                    messages[msg].parameters);
        }
        else
        {
            send_formatted_response("<%d,%s>\r\n",
                                    messages[msg].message_id,
                                    messages[msg].command);
        }

        char lcd_msg[20];
        sprintf(lcd_msg, "Msg %d: %s", msg + 1, messages[msg].command);
        lcd_string(3, 0, lcd_msg);

        // Simulate response processing
        _delay_ms(500);

        // Generate appropriate response based on command
        if (strcmp(messages[msg].command, "SENSOR_REQ") == 0)
        {
            send_formatted_response("<ACK,%d,TEMP=25.3,HUMID=65.2>\r\n", messages[msg].message_id);
            messages[msg].acknowledged = 1;
            successful_acks++;
        }
        else if (strcmp(messages[msg].command, "CONFIG_SET") == 0)
        {
            send_formatted_response("<ACK,%d,THRESHOLD_SET>\r\n", messages[msg].message_id);
            messages[msg].acknowledged = 1;
            successful_acks++;
        }
        else if (strcmp(messages[msg].command, "DATA_LOG") == 0)
        {
            send_formatted_response("<ACK,%d,LOGGING_STARTED>\r\n", messages[msg].message_id);
            messages[msg].acknowledged = 1;
            successful_acks++;
        }
        else if (strcmp(messages[msg].command, "ALERT_SUB") == 0)
        {
            send_formatted_response("<ACK,%d,SUBSCRIBED>\r\n", messages[msg].message_id);
            messages[msg].acknowledged = 1;
            successful_acks++;
        }
        else if (strcmp(messages[msg].command, "STATUS_REQ") == 0)
        {
            send_formatted_response("<ACK,%d,STATUS=OK,UPTIME=%ld>\r\n",
                                    messages[msg].message_id, messages_sent);
            messages[msg].acknowledged = 1;
            successful_acks++;
        }
        else
        {
            send_formatted_response("<NAK,%d,UNKNOWN_COMMAND>\r\n", messages[msg].message_id);
        }

        char lcd_ack[20];
        sprintf(lcd_ack, "ACK: %d/%d", successful_acks, msg + 1);
        lcd_string(4, 0, lcd_ack);

        _delay_ms(1000);
    }

    // Protocol statistics
    send_string("\r\n=== Protocol Statistics ===\r\n");
    send_formatted_response("Messages sent: %d\r\n", total_messages);
    send_formatted_response("Acknowledged: %d\r\n", successful_acks);
    send_formatted_response("Success rate: %d%%\r\n", (successful_acks * 100) / total_messages);

    char lcd_stats[20];
    sprintf(lcd_stats, "Success: %d%%", (successful_acks * 100) / total_messages);
    lcd_string(5, 0, lcd_stats);

    if (successful_acks >= 4)
    {
        lab_score += 200;
    }

    _delay_ms(3000);
}

/*
 * =============================================================================
 * LAB EXERCISE 4: ADVANCED MESSAGING WITH VALIDATION (10 minutes)
 * =============================================================================
 * OBJECTIVE: Implement message validation and error correction
 * DIFFICULTY: ★★★★★ (Hard)
 */

void lab_ex4_message_validation(void)
{
    /*
     * CHALLENGE: Add checksums and validation to messages
     * TASK: Implement message integrity checking and error recovery
     * LEARNING: Data integrity, checksums, error detection and correction
     */

    puts_USART1("\r\n=== Lab 4: Message Validation ===\r\n");
    puts_USART1("Implementing message integrity checking\r\n");

    lcd_clear();
    lcd_string(0, 0, "MESSAGE VALIDATION");
    lcd_string(1, 0, "Checksums & integrity");

    // Simple checksum calculation function
    uint8_t calculate_checksum(char *message)
    {
        uint8_t checksum = 0;
        while (*message)
        {
            checksum ^= *message++;
        }
        return checksum;
    }

    // Test messages with checksums
    char test_messages[][60] = {
        "HELLO,WORLD",
        "SENSOR,DATA,25.3,65.2",
        "CONFIG,THRESHOLD,500",
        "ERROR,INVALID,PARAMETER",
        "STATUS,OK,READY"};

    uint8_t message_count = 5;
    uint8_t validation_passed = 0;

    send_string("Testing message validation with checksums:\r\n\r\n");

    for (uint8_t msg = 0; msg < message_count; msg++)
    {
        // Calculate checksum
        uint8_t checksum = calculate_checksum(test_messages[msg]);

        // Send message with checksum
        send_formatted_response("MSG:%s,CHK:%02X\r\n", test_messages[msg], checksum);

        char lcd_msg[20];
        sprintf(lcd_msg, "Msg %d: CHK=%02X", msg + 1, checksum);
        lcd_string(3, 0, lcd_msg);

        // Simulate received message validation
        _delay_ms(300);

        // Verify checksum (simulate perfect transmission)
        uint8_t received_checksum = calculate_checksum(test_messages[msg]);

        if (received_checksum == checksum)
        {
            send_formatted_response("VALID: Message %d integrity confirmed\r\n", msg + 1);
            validation_passed++;
        }
        else
        {
            send_formatted_response("ERROR: Message %d checksum mismatch\r\n", msg + 1);
        }

        char lcd_valid[20];
        sprintf(lcd_valid, "Valid: %d/%d", validation_passed, msg + 1);
        lcd_string(4, 0, lcd_valid);

        _delay_ms(800);
    }

    // Test corrupted message detection
    send_string("\r\nTesting corrupted message detection:\r\n");

    char corrupted_msg[] = "SENSOR,DATA,25.3,65.2";
    uint8_t original_checksum = calculate_checksum(corrupted_msg);

    // Introduce corruption
    corrupted_msg[10] = 'X'; // Corrupt one character
    uint8_t corrupted_checksum = calculate_checksum(corrupted_msg);

    send_formatted_response("Original: SENSOR,DATA,25.3,65.2 (CHK:%02X)\r\n", original_checksum);
    send_formatted_response("Received: %s (CHK:%02X)\r\n", corrupted_msg, corrupted_checksum);

    if (original_checksum != corrupted_checksum)
    {
        send_string("DETECTED: Message corruption identified!\r\n");
        lcd_string(5, 0, "Corruption detected");
        validation_passed++;
    }
    else
    {
        send_string("FAILED: Corruption not detected\r\n");
        lcd_string(5, 0, "Detection failed");
    }

    char final_validation[60];
    sprintf(final_validation, "\r\nValidation complete: %d/%d tests passed\r\n",
            validation_passed, message_count + 1);
    send_string(final_validation);

    if (validation_passed >= message_count)
    {
        lab_score += 150;
    }

    _delay_ms(2000);
}

/*
 * =============================================================================
 * LAB MAIN PROGRAM - EXERCISE SELECTION
 * =============================================================================
 */

void show_lab_menu(void)
{
    puts_USART1("\r\n");
    puts_USART1("==============================================\r\n");
    puts_USART1("   SERIAL POLLING STRING - LAB EXERCISES    \r\n");
    puts_USART1("==============================================\r\n");
    puts_USART1("1. Basic String Transmission & Reception    \r\n");
    puts_USART1("2. Command Parsing & Execution System       \r\n");
    puts_USART1("3. Bidirectional Communication Protocol     \r\n");
    puts_USART1("4. Advanced Message Validation              \r\n");
    puts_USART1("                                              \r\n");
    puts_USART1("0. Run All Exercises                         \r\n");
    puts_USART1("X. Exit Lab                                   \r\n");
    puts_USART1("==============================================\r\n");
    char score_msg[50];
    sprintf(score_msg, "Current Score: %d points\r\n", lab_score);
    puts_USART1(score_msg);
    char comm_stats[60];
    sprintf(comm_stats, "Stats: %ld sent, %ld received, %d commands\r\n",
            messages_sent, messages_received, command_executions);
    puts_USART1(comm_stats);
    char error_stats[50];
    sprintf(error_stats, "Errors: %d parsing errors\r\n", parsing_errors);
    puts_USART1(error_stats);
    puts_USART1("Select exercise (1-4, 0, X): ");
}

int main(void)
{
    init_devices();

    puts_USART1("\r\n*** SERIAL POLLING STRING LAB SESSION ***\r\n");
    puts_USART1("Welcome to string-based UART communication!\r\n");
    puts_USART1("This lab covers string handling, command parsing, and protocols\r\n");
    puts_USART1("Ensure UART connection is working properly\r\n");

    lcd_clear();
    lcd_string(1, 0, "STRING COMM LAB");
    lcd_string(2, 0, "UART messaging");
    lcd_string(4, 0, "Use Serial Menu");

    while (1)
    {
        show_lab_menu();
        char choice = getch_USART1();
        putch_USART1(choice);
        putch_USART1('\r');
        putch_USART1('\n');

        switch (choice)
        {
        case '1':
            lab_ex1_string_transmission();
            lab_ex1_message_formatting();
            break;

        case '2':
            lab_ex2_command_interface();
            break;

        case '3':
            lab_ex3_bidirectional_protocol();
            break;

        case '4':
            lab_ex4_message_validation();
            break;

        case '0':
            puts_USART1("\r\n*** RUNNING ALL EXERCISES ***\r\n");
            lab_ex1_string_transmission();
            lab_ex1_message_formatting();
            lab_ex2_command_interface();
            lab_ex3_bidirectional_protocol();
            lab_ex4_message_validation();

            char final_buffer[80];
            sprintf(final_buffer, "\r\n*** ALL EXERCISES COMPLETE! ***\r\nFinal Score: %d points\r\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\r\nExiting lab. Great work on string communication!\r\n");
            puts_USART1("Remember: String protocols are powerful for human-readable interfaces!\r\n");
            lcd_clear();
            lcd_string(2, 0, "LAB COMPLETE!");
            char exit_score[30];
            sprintf(exit_score, "Score: %d pts", lab_score);
            lcd_string(3, 0, exit_score);
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