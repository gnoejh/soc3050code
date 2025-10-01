/*
 * main_python_interface.c - Python Communication Interface Example
 * Demonstrates structured communication between ATmega128 and Python
 *
 * LEARNING OBJECTIVES:
 * 1. Understand structured data communication protocols
 * 2. Learn JSON-like message formatting for microcontrollers
 * 3. Bridge embedded systems to high-level Python programming
 * 4. Demonstrate the complete Assembly→C→Python progression
 */

#include "config.h"

#ifdef PYTHON_INTERFACE_EXAMPLE

#include <stdio.h>
#include <string.h>

/*
 * Communication Protocol Design
 * Simple command/response protocol for educational use
 */

// Command format: "CMD:parameter\n"
// Response format: "STATUS:value\n" or "ERROR:message\n"

/*
 * Command Definitions
 * These commands can be sent from Python to control the ATmega128
 */
#define CMD_LED_ON "LED_ON"
#define CMD_LED_OFF "LED_OFF"
#define CMD_LED_PATTERN "LED_PATTERN"
#define CMD_LED_BLINK "LED_BLINK"
#define CMD_READ_ADC "READ_ADC"
#define CMD_READ_BUTTON "READ_BUTTON"
#define CMD_BEEP "BEEP"
#define CMD_PLAY_NOTE "PLAY_NOTE"
#define CMD_GET_STATUS "GET_STATUS"
#define CMD_RESET "RESET"

/*
 * Response Definitions
 */
#define RESP_OK "OK"
#define RESP_ERROR "ERROR"
#define RESP_STATUS "STATUS"
#define RESP_DATA "DATA"

/*
 * Global variables for communication
 */
char command_buffer[64];         // Buffer for incoming commands
char response_buffer[64];        // Buffer for outgoing responses
unsigned char buffer_index = 0;  // Current position in command buffer
unsigned char command_ready = 0; // Flag indicating complete command received

/*
 * Communication Functions
 */

/*
 * Send formatted response to Python
 */
void send_response(const char *type, const char *message)
{
    sprintf(response_buffer, "%s:%s\n", type, message);
    puts_USART1(response_buffer);
}

/*
 * Send error message to Python
 */
void send_error(const char *error_message)
{
    send_response(RESP_ERROR, error_message);
}

/*
 * Send OK response to Python
 */
void send_ok(void)
{
    send_response(RESP_OK, "Command executed");
}

/*
 * Send data response to Python
 */
void send_data(const char *data)
{
    send_response(RESP_DATA, data);
}

/*
 * Parse incoming command from serial buffer
 * Commands are terminated by newline character
 */
void process_serial_input(void)
{
    if (USART1_data_available())
    {
        unsigned char received_char = USART1_get_data();

        if (received_char == '\n' || received_char == '\r')
        {
            // End of command - null terminate and set flag
            command_buffer[buffer_index] = '\0';
            command_ready = 1;
            buffer_index = 0;
        }
        else if (buffer_index < sizeof(command_buffer) - 1)
        {
            // Add character to buffer
            command_buffer[buffer_index] = received_char;
            buffer_index++;
        }
        else
        {
            // Buffer overflow - reset
            buffer_index = 0;
            send_error("Command too long");
        }
    }
}

/*
 * Parse command and parameter from command string
 * Format: "COMMAND:parameter"
 */
void parse_command(char *cmd_str, char *command, char *parameter)
{
    char *colon_pos = strchr(cmd_str, ':');

    if (colon_pos != NULL)
    {
        // Split at colon
        *colon_pos = '\0';                // Null terminate command part
        strcpy(command, cmd_str);         // Copy command
        strcpy(parameter, colon_pos + 1); // Copy parameter
    }
    else
    {
        // No parameter
        strcpy(command, cmd_str);
        parameter[0] = '\0';
    }
}

/*
 * Execute parsed command with parameter
 */
void execute_command(const char *command, const char *parameter)
{
    char data_buffer[32];
    unsigned int adc_value;
    unsigned char button_state;
    unsigned int frequency;
    unsigned char pattern;

    // LED Control Commands
    if (strcmp(command, CMD_LED_ON) == 0)
    {
        LED_All_On();
        send_ok();
    }
    else if (strcmp(command, CMD_LED_OFF) == 0)
    {
        LED_All_Off();
        send_ok();
    }
    else if (strcmp(command, CMD_LED_PATTERN) == 0)
    {
        pattern = (unsigned char)atoi(parameter);
        LED_Set_Pattern(pattern);
        send_ok();
    }
    else if (strcmp(command, CMD_LED_BLINK) == 0)
    {
        // Blink LEDs specified number of times
        unsigned char blink_count = (unsigned char)atoi(parameter);
        if (blink_count == 0)
            blink_count = 3; // Default

        for (unsigned char i = 0; i < blink_count; i++)
        {
            LED_All_On();
            _delay_ms(200);
            LED_All_Off();
            _delay_ms(200);
        }
        send_ok();
    }

    // Sensor Reading Commands
    else if (strcmp(command, CMD_READ_ADC) == 0)
    {
        unsigned char channel = (unsigned char)atoi(parameter);
        if (channel > 7)
        {
            send_error("Invalid ADC channel (0-7)");
        }
        else
        {
            adc_value = Adc_read(channel);
            sprintf(data_buffer, "ADC%u:%u", channel, adc_value);
            send_data(data_buffer);
        }
    }
    else if (strcmp(command, CMD_READ_BUTTON) == 0)
    {
        button_state = Button_Read_All();
        sprintf(data_buffer, "BUTTONS:%u", button_state);
        send_data(data_buffer);
    }

    // Audio Output Commands
    else if (strcmp(command, CMD_BEEP) == 0)
    {
        Buzzer_beep();
        send_ok();
    }
    else if (strcmp(command, CMD_PLAY_NOTE) == 0)
    {
        frequency = (unsigned int)atoi(parameter);
        if (frequency > 0 && frequency < 5000)
        {
            Buzzer_tone(frequency, 500); // Play for 500ms
            send_ok();
        }
        else
        {
            send_error("Invalid frequency (1-4999 Hz)");
        }
    }

    // System Commands
    else if (strcmp(command, CMD_GET_STATUS) == 0)
    {
        sprintf(data_buffer, "ATMEGA128_READY:F_CPU=%lu,BAUD=%u", F_CPU, BAUD);
        send_data(data_buffer);
    }
    else if (strcmp(command, CMD_RESET) == 0)
    {
        send_ok();
        _delay_ms(100);
        // Trigger watchdog reset or jump to reset vector
        asm volatile("jmp 0");
    }

    // Unknown Command
    else
    {
        sprintf(data_buffer, "Unknown command: %s", command);
        send_error(data_buffer);
    }
}

/*
 * Main communication loop
 */
void main_python_interface(void)
{
    char command[32];
    char parameter[32];

    // Initialize all systems
    init_devices();

    // Send startup message to Python
    send_response(RESP_STATUS, "ATmega128 Python Interface Ready");

    // Main command processing loop
    while (1)
    {
        // Check for incoming serial data
        process_serial_input();

        // Process complete commands
        if (command_ready)
        {
            parse_command(command_buffer, command, parameter);
            execute_command(command, parameter);
            command_ready = 0;
        }

        // Small delay to prevent overwhelming the serial port
        _delay_ms(10);
    }
}

/*
 * ============================================================================
 * PYTHON INTEGRATION EXAMPLES
 * These show corresponding Python code for communication
 * ============================================================================
 */

/*
 * PYTHON CLIENT EXAMPLE:
 *
 * import serial
 * import time
 * import json
 *
 * class ATmega128Interface:
 *     def __init__(self, port='COM3', baud=9600):
 *         self.ser = serial.Serial(port, baud, timeout=1)
 *         time.sleep(2)  # Wait for Arduino reset
 *
 *     def send_command(self, command, parameter=""):
 *         if parameter:
 *             cmd_str = f"{command}:{parameter}\n"
 *         else:
 *             cmd_str = f"{command}\n"
 *
 *         self.ser.write(cmd_str.encode())
 *         response = self.ser.readline().decode().strip()
 *         return self.parse_response(response)
 *
 *     def parse_response(self, response):
 *         if ':' in response:
 *             type_part, data_part = response.split(':', 1)
 *             return {'type': type_part, 'data': data_part}
 *         return {'type': 'UNKNOWN', 'data': response}
 *
 *     def led_on(self):
 *         return self.send_command('LED_ON')
 *
 *     def led_off(self):
 *         return self.send_command('LED_OFF')
 *
 *     def led_pattern(self, pattern):
 *         return self.send_command('LED_PATTERN', str(pattern))
 *
 *     def read_adc(self, channel):
 *         response = self.send_command('READ_ADC', str(channel))
 *         if response['type'] == 'DATA':
 *             # Parse "ADC0:512" format
 *             parts = response['data'].split(':')
 *             return int(parts[1])
 *         return None
 *
 *     def read_voltage(self, channel):
 *         adc_value = self.read_adc(channel)
 *         if adc_value is not None:
 *             return (adc_value * 5.0) / 1023.0
 *         return None
 *
 *     def beep(self):
 *         return self.send_command('BEEP')
 *
 *     def play_note(self, frequency):
 *         return self.send_command('PLAY_NOTE', str(frequency))
 *
 * # Usage example:
 * atmega = ATmega128Interface('COM3')
 * atmega.led_on()
 * voltage = atmega.read_voltage(0)
 * print(f"Voltage on ADC0: {voltage:.2f}V")
 * atmega.beep()
 */

/*
 * FLASK WEB SERVER EXAMPLE:
 *
 * from flask import Flask, render_template, request, jsonify
 * import threading
 * import time
 *
 * app = Flask(__name__)
 * atmega = ATmega128Interface('COM3')
 *
 * @app.route('/')
 * def dashboard():
 *     return render_template('dashboard.html')
 *
 * @app.route('/led/<action>')
 * def led_control(action):
 *     if action == 'on':
 *         result = atmega.led_on()
 *     elif action == 'off':
 *         result = atmega.led_off()
 *     else:
 *         return jsonify({'error': 'Invalid action'})
 *
 *     return jsonify(result)
 *
 * @app.route('/sensors')
 * def read_sensors():
 *     sensors = {}
 *     for channel in range(4):  # Read first 4 ADC channels
 *         voltage = atmega.read_voltage(channel)
 *         sensors[f'adc{channel}'] = voltage
 *
 *     return jsonify(sensors)
 *
 * @app.route('/beep')
 * def beep():
 *     result = atmega.beep()
 *     return jsonify(result)
 *
 * if __name__ == '__main__':
 *     app.run(debug=True, host='0.0.0.0', port=5000)
 */

/*
 * DATA LOGGING EXAMPLE:
 *
 * import csv
 * import time
 * from datetime import datetime
 *
 * def log_sensor_data(duration_minutes=60):
 *     atmega = ATmega128Interface('COM3')
 *
 *     with open('sensor_log.csv', 'w', newline='') as csvfile:
 *         writer = csv.writer(csvfile)
 *         writer.writerow(['timestamp', 'adc0', 'adc1', 'adc2', 'adc3'])
 *
 *         start_time = time.time()
 *         while time.time() - start_time < duration_minutes * 60:
 *             timestamp = datetime.now().isoformat()
 *             readings = [timestamp]
 *
 *             for channel in range(4):
 *                 voltage = atmega.read_voltage(channel)
 *                 readings.append(voltage)
 *
 *             writer.writerow(readings)
 *             csvfile.flush()  # Ensure data is written
 *
 *             time.sleep(1)  # Log every second
 *
 * # Start logging
 * log_sensor_data(60)  # Log for 60 minutes
 */

#endif // PYTHON_INTERFACE_EXAMPLE