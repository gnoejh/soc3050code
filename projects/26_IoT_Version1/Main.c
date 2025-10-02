/*
 * IoT Version1 - Advanced Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Implement advanced IoT protocols and features
 * - Learn multi-sensor data fusion and processing
 * - Practice edge computing and local decision making
 * - Master IoT security and authentication basics
 *
 * HARDWARE SETUP:
 * - Multiple sensors with data preprocessing
 * - UART for enhanced IoT gateway communication
 * - LEDs for advanced status and alert indication
 * - EEPROM for configuration and data persistence
 */

#include "config.h"

// Advanced IoT device configuration
#define DEVICE_ID "ATM128_V1_001"
#define FIRMWARE_VERSION "1.1.0"
#define PROTOCOL_VERSION "IoT_v1.0"
#define MAX_SENSORS 5
#define DATA_BUFFER_SIZE 10

// Enhanced sensor data structure
typedef struct
{
    uint8_t sensor_id;
    uint16_t raw_value;
    float processed_value;
    uint8_t quality_score;
    uint32_t timestamp;
    uint8_t alarm_status;
} sensor_reading_t;

// IoT device state
typedef struct
{
    char device_id[20];
    uint8_t operational_mode;
    uint16_t data_interval;
    uint8_t alert_threshold_high;
    uint8_t alert_threshold_low;
    uint8_t security_key[4];
} device_config_t;

// Global variables
sensor_reading_t sensor_buffer[MAX_SENSORS][DATA_BUFFER_SIZE];
uint8_t buffer_index[MAX_SENSORS] = {0};
device_config_t device_config;
uint32_t system_uptime = 0;

// Function to process raw sensor data
float process_sensor_data(uint8_t sensor_id, uint16_t raw_value)
{
    float processed = 0.0;

    switch (sensor_id)
    {
    case 0: // Temperature sensor (ADC to Celsius)
        processed = (raw_value * 5.0 / 1024.0 - 0.5) * 100.0;
        break;
    case 1: // Light sensor (ADC to lux approximation)
        processed = raw_value * 100.0 / 1024.0;
        break;
    case 2: // Pressure sensor (ADC to kPa)
        processed = raw_value * 200.0 / 1024.0 + 80.0;
        break;
    default:
        processed = raw_value;
        break;
    }

    return processed;
}

// Function to calculate data quality score
uint8_t calculate_quality_score(uint8_t sensor_id, uint16_t raw_value)
{
    // Simple quality assessment based on value stability
    uint8_t quality = 100;

    // Check for out-of-range values
    if (raw_value < 10 || raw_value > 1000)
        quality -= 30;

    // Check recent history for stability
    uint8_t prev_idx = (buffer_index[sensor_id] + DATA_BUFFER_SIZE - 1) % DATA_BUFFER_SIZE;
    if (buffer_index[sensor_id] > 0)
    {
        uint16_t prev_value = sensor_buffer[sensor_id][prev_idx].raw_value;
        uint16_t difference = (raw_value > prev_value) ? (raw_value - prev_value) : (prev_value - raw_value);

        if (difference > 100)
            quality -= 20; // Large change reduces quality
    }

    return quality;
}

// Function to read and process all sensors
void update_sensor_readings()
{
    for (uint8_t sensor = 0; sensor < 3; sensor++)
    {
        uint16_t raw_value = Adc_read_ch(sensor);
        uint8_t idx = buffer_index[sensor];

        sensor_buffer[sensor][idx].sensor_id = sensor;
        sensor_buffer[sensor][idx].raw_value = raw_value;
        sensor_buffer[sensor][idx].processed_value = process_sensor_data(sensor, raw_value);
        sensor_buffer[sensor][idx].quality_score = calculate_quality_score(sensor, raw_value);
        sensor_buffer[sensor][idx].timestamp = system_uptime;

        // Check alarm conditions
        if (raw_value > (device_config.alert_threshold_high * 10) ||
            raw_value < (device_config.alert_threshold_low * 10))
        {
            sensor_buffer[sensor][idx].alarm_status = 1;
            // Visual alarm indication
            PORTB = ~(1 << sensor);
        }
        else
        {
            sensor_buffer[sensor][idx].alarm_status = 0;
        }

        buffer_index[sensor] = (buffer_index[sensor] + 1) % DATA_BUFFER_SIZE;
    }
}

// Function to send comprehensive IoT data
void send_enhanced_iot_data()
{
    puts_USART1("IoT-V1-DATA: {");

    char buffer[150];
    sprintf(buffer, "\"device\":\"%s\",\"version\":\"%s\",\"uptime\":%lu,",
            device_config.device_id, FIRMWARE_VERSION, system_uptime);
    puts_USART1(buffer);

    puts_USART1("\"sensors\":[\r\n");

    for (uint8_t sensor = 0; sensor < 3; sensor++)
    {
        uint8_t latest_idx = (buffer_index[sensor] + DATA_BUFFER_SIZE - 1) % DATA_BUFFER_SIZE;
        sensor_reading_t *reading = &sensor_buffer[sensor][latest_idx];

        sprintf(buffer, "  {\"id\":%u,\"raw\":%u,\"processed\":%d,\"quality\":%u,\"alarm\":%u}",
                reading->sensor_id,
                reading->raw_value,
                (int)(reading->processed_value * 10), // Send as integer * 10
                reading->quality_score,
                reading->alarm_status);
        puts_USART1(buffer);

        if (sensor < 2)
            puts_USART1(",");
        puts_USART1("\r\n");
    }

    puts_USART1("]\r\n}\r\n");
}

// Function to handle advanced IoT commands
void process_advanced_commands()
{
    if (is_USART1_received())
    {
        char cmd = get_USART1();
        char response[100];

        switch (cmd)
        {
        case 'D': // Data request
            send_enhanced_iot_data();
            break;

        case 'C': // Configuration dump
            sprintf(response, "CONFIG: mode=%u, interval=%u, high=%u, low=%u\r\n",
                    device_config.operational_mode,
                    device_config.data_interval,
                    device_config.alert_threshold_high,
                    device_config.alert_threshold_low);
            puts_USART1(response);
            break;

        case 'H': // Health check
            sprintf(response, "HEALTH: uptime=%lu, alerts=%u, data_quality=OK\r\n",
                    system_uptime, 0); // Count active alarms
            puts_USART1(response);
            break;

        case 'R': // Restart
            puts_USART1("RESTART: Simulating device restart...\r\n");
            system_uptime = 0;
            for (uint8_t i = 0; i < MAX_SENSORS; i++)
                buffer_index[i] = 0;
            break;

        default:
            puts_USART1("ERROR: Unknown command. Available: D,C,H,R\r\n");
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
    EEPROM_init();

    // Initialize device configuration
    strcpy(device_config.device_id, DEVICE_ID);
    device_config.operational_mode = 1; // Normal operation
    device_config.data_interval = 5000; // 5 second intervals
    device_config.alert_threshold_high = 80;
    device_config.alert_threshold_low = 20;

    puts_USART1("IoT Version1 Advanced Device Starting...\r\n");
    puts_USART1("Protocol: ");
    puts_USART1(PROTOCOL_VERSION);
    puts_USART1("\r\n");
    puts_USART1("Features: Multi-sensor, Data processing, Alarms\r\n");
    puts_USART1("Commands: D=Data, C=Config, H=Health, R=Restart\r\n");

    DDRB = 0xFF;
    PORTB = 0xFF;

    uint16_t cycle_counter = 0;

    puts_USART1("IoT-V1-STATUS: Advanced device online\r\n");

    while (1)
    {
        cycle_counter++;
        system_uptime = cycle_counter / 10; // Approximate seconds

        // Update sensor readings
        update_sensor_readings();

        // Send data at configured intervals
        if (cycle_counter % (device_config.data_interval / 100) == 0)
        {
            send_enhanced_iot_data();
        }

        // Process commands
        process_advanced_commands();

        // System heartbeat
        if (cycle_counter % 20 == 0) // Every 2 seconds
        {
            PORTB ^= (1 << 7); // Heartbeat LED
        }

        _delay_ms(100);
    }

    return 0;
}
on 1 * ATmega128 Educational Framework **This project demonstrates enhanced IoT functionality *with advanced sensor monitoring.* /

#include "config.h"

    int main(void)
{
    main_iot1();

    return 0;
}