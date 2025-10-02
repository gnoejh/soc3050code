/*
 * Lab Serial - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Practice lab-style data acquisition via UART
 * - Learn structured data logging and analysis
 * - Understand sensor data formatting and transmission
 * - Master real-time monitoring and control
 *
 * HARDWARE SETUP:
 * - Multiple sensors connected for data collection
 * - UART for data transmission to PC/logger
 * - LEDs for system status indication
 */

#include "config.h"

// Data logging structure
typedef struct
{
    uint16_t timestamp;
    uint8_t sensor_id;
    uint16_t value;
    uint8_t status;
} sensor_data_t;

// Function to simulate sensor readings
uint16_t read_sensor_channel(uint8_t channel)
{
    // Simulate different sensor types
    switch (channel)
    {
    case 0:
        return Adc_read_ch(0); // Temperature sensor
    case 1:
        return Adc_read_ch(1); // Light sensor
    case 2:
        return Adc_read_ch(2); // Pressure sensor
    default:
        return 0;
    }
}

// Function to format and send sensor data
void log_sensor_data(uint8_t sensor_id, uint16_t value)
{
    static uint16_t timestamp = 0;
    timestamp++;

    char buffer[80];
    sprintf(buffer, "DATA,%u,%u,%u,OK\r\n", timestamp, sensor_id, value);
    puts_USART1(buffer);

    // Visual feedback
    PORTB = ~(1 << sensor_id);
    _delay_ms(100);
    PORTB = 0xFF;
}

int main(void)
{
    // Initialize system components
    init_devices();
    Adc_init();
    Uart1_init();

    puts_USART1("Lab Serial Data Acquisition Started\r\n");
    puts_USART1("Format: DATA,timestamp,sensor_id,value,status\r\n");

    DDRB = 0xFF;
    PORTB = 0xFF;

    uint8_t current_sensor = 0;

    while (1)
    {
        // Read and log data from each sensor
        for (uint8_t sensor = 0; sensor < 3; sensor++)
        {
            uint16_t reading = read_sensor_channel(sensor);
            log_sensor_data(sensor, reading);
            _delay_ms(500);
        }

        _delay_ms(2000); // 2 second interval between cycles
    }

    return 0;
}