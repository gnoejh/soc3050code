/*
 * Serial General Word - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Learn word-based communication protocols
 * - Practice command parsing and response generation
 * - Understand structured message formats
 * - Master bidirectional protocol implementation
 *
 * HARDWARE SETUP:
 * - UART for word-based command communication
 * - LEDs for command execution visualization
 * - Optional: Multiple devices for network communication
 */

#include "config.h"

#define MAX_WORD_LENGTH 16
#define MAX_COMMAND_ARGS 4

// Command structure
typedef struct
{
    char command[MAX_WORD_LENGTH];
    char args[MAX_COMMAND_ARGS][MAX_WORD_LENGTH];
    uint8_t arg_count;
} command_t;

// Function to parse received words into command structure
uint8_t parse_command(char *input, command_t *cmd)
{
    // Simple word parsing (space-separated)
    char *token = strtok(input, " ");
    if (token == NULL)
        return 0;

    strcpy(cmd->command, token);
    cmd->arg_count = 0;

    while ((token = strtok(NULL, " ")) != NULL && cmd->arg_count < MAX_COMMAND_ARGS)
    {
        strcpy(cmd->args[cmd->arg_count], token);
        cmd->arg_count++;
    }

    return 1;
}

// Function to execute parsed commands
void execute_command(command_t *cmd)
{
    char response[80];

    if (strcmp(cmd->command, "LED") == 0)
    {
        if (cmd->arg_count >= 2)
        {
            uint8_t led_num = atoi(cmd->args[0]);
            if (strcmp(cmd->args[1], "ON") == 0)
            {
                PORTB &= ~(1 << led_num);
                sprintf(response, "ACK LED %u ON\r\n", led_num);
            }
            else if (strcmp(cmd->args[1], "OFF") == 0)
            {
                PORTB |= (1 << led_num);
                sprintf(response, "ACK LED %u OFF\r\n", led_num);
            }
            else
            {
                sprintf(response, "ERROR Invalid LED state\r\n");
            }
        }
        else
        {
            sprintf(response, "ERROR LED command needs 2 args\r\n");
        }
    }
    else if (strcmp(cmd->command, "READ") == 0)
    {
        if (cmd->arg_count >= 1)
        {
            uint8_t channel = atoi(cmd->args[0]);
            uint16_t value = Adc_read_ch(channel);
            sprintf(response, "DATA %u %u\r\n", channel, value);
        }
        else
        {
            sprintf(response, "ERROR READ needs channel arg\r\n");
        }
    }
    else if (strcmp(cmd->command, "STATUS") == 0)
    {
        sprintf(response, "STATUS OK PORTB=0x%02X\r\n", PORTB);
    }
    else
    {
        sprintf(response, "ERROR Unknown command: %s\r\n", cmd->command);
    }

    puts_USART1(response);
}

int main(void)
{
    init_devices();
    Adc_init();
    Uart1_init();

    puts_USART1("Serial General Word Protocol Started\r\n");
    puts_USART1("Commands: LED <num> <ON|OFF>, READ <channel>, STATUS\r\n");

    DDRB = 0xFF;
    PORTB = 0xFF;

    char input_buffer[80];
    uint8_t buffer_index = 0;
    command_t current_command;

    while (1)
    {
        if (is_USART1_received())
        {
            char received_char = get_USART1();

            if (received_char == '\r' || received_char == '\n')
            {
                if (buffer_index > 0)
                {
                    input_buffer[buffer_index] = '\0';

                    if (parse_command(input_buffer, &current_command))
                    {
                        execute_command(&current_command);
                    }
                    else
                    {
                        puts_USART1("ERROR Invalid command format\r\n");
                    }

                    buffer_index = 0;
                }
            }
            else if (buffer_index < sizeof(input_buffer) - 1)
            {
                input_buffer[buffer_index++] = received_char;
                put_USART1(received_char); // Echo
            }
        }

        _delay_ms(10);
    }

    return 0;
}
neral Word
        *ATmega128 Educational Framework
            *
                *This project demonstrates general word processing
                    *through serial communication.
                        * /

#include "config.h"

    int main(void)
{
    main_serial_general_word();

    return 0;
}