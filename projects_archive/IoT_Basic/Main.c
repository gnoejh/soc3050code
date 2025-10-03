/*
 * IoT Basic - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Understand IoT fundamentals and connectivity
 * - Learn sensor data collection and transmission
 * - Practice device identification and status reporting
 * - Master basic IoT communication protocols
 *
 * HARDWARE SETUP:
 * - Sensors for data collection (temperature, light, etc.)
 * - UART for IoT gateway communication
 * - LEDs for device status indication
 * - Optional: WiFi module for wireless connectivity
 */

#include "config.h"

// IoT device information
#define DEVICE_ID "ATM128_001"
#define FIRMWARE_VERSION "1.0.0"
#define SENSOR_COUNT 3

// IoT message structure
typedef struct
{
    char device_id[16];
    uint32_t timestamp;
    uint8_t sensor_count;
    uint16_t sensor_values[SENSOR_COUNT];
    uint8_t battery_level;
    uint8_t signal_strength;
} iot_message_t;

// Function to collect sensor data
void collect_sensor_data(iot_message_t *msg)
{
    // Simulate various sensor readings
    msg->sensor_values[0] = Adc_read_ch(0); // Temperature
    msg->sensor_values[1] = Adc_read_ch(1); // Light level
    msg->sensor_values[2] = Adc_read_ch(2); // Humidity

    // Simulate battery and signal status
    msg->battery_level = 85;   // 85% battery
    msg->signal_strength = 75; // 75% signal
}

// Function to format IoT message as JSON
void format_iot_message(iot_message_t *msg, char *output)
{
    sprintf(output,
            "{\"device_id\":\"%s\",\"timestamp\":%lu,\"sensors\":[%u,%u,%u],\"battery\":%u,\"signal\":%u}\r\n",
            msg->device_id,
            msg->timestamp,
            msg->sensor_values[0],
            msg->sensor_values[1],
            msg->sensor_values[2],
            msg->battery_level,
            msg->signal_strength);
}

// Function to send IoT data
void send_iot_data(iot_message_t *msg)
{
    char json_buffer[200];
    format_iot_message(msg, json_buffer);

    puts_USART1("IoT-DATA: ");
    puts_USART1(json_buffer);

    // Visual feedback for data transmission
    PORTB = 0x00; // All LEDs on during transmission
    _delay_ms(100);
    PORTB = 0xFF; // LEDs off
}

// Function to handle IoT device status
void report_device_status()
{
    char status_buffer[150];
    sprintf(status_buffer,
            "DEVICE-STATUS: {\"id\":\"%s\",\"version\":\"%s\",\"uptime\":%u,\"sensors\":%u}\r\n",
            DEVICE_ID,
            FIRMWARE_VERSION,
            (uint16_t)(0), // Simplified uptime
            SENSOR_COUNT);

    puts_USART1(status_buffer);
}

// Function to simulate IoT command processing
void process_iot_commands()
{
    if (is_USART1_received())
    {
        char received = get_USART1();

        switch (received)
        {
        case 'S': // Status request
            puts_USART1("IoT-CMD: Status requested\r\n");
            report_device_status();
            break;

        case 'R': // Reset request
            puts_USART1("IoT-CMD: Reset requested\r\n");
            // Simulate reset sequence
            for (uint8_t i = 0; i < 5; i++)
            {
                PORTB = 0x55;
                _delay_ms(200);
                PORTB = 0xAA;
                _delay_ms(200);
            }
            PORTB = 0xFF;
            puts_USART1("IoT-STATUS: Device reset complete\r\n");
            break;

        case 'C': // Configuration request
            puts_USART1("IoT-CMD: Configuration requested\r\n");
            puts_USART1("IoT-CONFIG: {\"interval\":5000,\"sensors\":3,\"format\":\"json\"}\r\n");
            break;

        default:
            puts_USART1("IoT-ERROR: Unknown command\r\n");
            break;
        }
    }
}

int main(void)
{
    // Initialize system components
    init_devices();
    Adc_init();
    Uart1_init();

    puts_USART1("IoT Basic Device Starting...\r\n");
    puts_USART1("Device ID: ");
    puts_USART1(DEVICE_ID);
    puts_USART1("\r\n");
    puts_USART1("Firmware: ");
    puts_USART1(FIRMWARE_VERSION);
    puts_USART1("\r\n");
    puts_USART1("Commands: S=Status, R=Reset, C=Config\r\n");

    DDRB = 0xFF;
    PORTB = 0xFF;

    iot_message_t device_msg;
    strcpy(device_msg.device_id, DEVICE_ID);
    device_msg.sensor_count = SENSOR_COUNT;
    device_msg.timestamp = 0;

    uint16_t data_interval_counter = 0;

    // Device startup sequence
    puts_USART1("IoT-STATUS: Device online and ready\r\n");

    while (1)
    {
        data_interval_counter++;

        // Send IoT data every 50 cycles (approximately 5 seconds)
        if (data_interval_counter >= 50)
        {
            device_msg.timestamp++;

            collect_sensor_data(&device_msg);
            send_iot_data(&device_msg);

            data_interval_counter = 0;

            // Status indication
            char status[40];
            sprintf(status, "IoT-INFO: Data sent at T+%lu\r\n", device_msg.timestamp);
            puts_USART1(status);
        }

        // Process incoming IoT commands
        process_iot_commands();

        // Heartbeat LED
        PORTB ^= (1 << 7); // Toggle LED 7 as heartbeat

        _delay_ms(100);
    }

    return 0;
}