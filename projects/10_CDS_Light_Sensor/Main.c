/*
 * CDS Light Sensor - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Understand photoresistor operation
 * - Learn analog sensor interfacing
 * - Practice threshold-based control
 * - Master sensor data processing
 *
 * HARDWARE SETUP:
 * - Connect CDS photoresistor to ADC1 (PA1)
 * - Use voltage divider with 10kΩ resistor
 * - LEDs on PORTB for light level indication
 * - UART for data logging
 */

#include "config.h"

int main(void)
{
    // Initialize system components
    init_devices(); // Initialize all peripherals

    // Initialize UART for data output
    Uart1_init(); // 9600 baud serial communication

    // Send startup message
    puts_USART1("CDS Light Sensor Started\r\n");
    puts_USART1("Reading light levels from ADC1...\r\n");

    uint16_t light_value;
    char buffer[60];
    uint8_t light_level;

    while (1)
    {
        // Read light sensor value from ADC1 (PA1)
        light_value = Adc_read_ch(1);

        // Convert to light level (0-8 scale)
        light_level = light_value >> 7; // Convert 10-bit to 3-bit

        // Display light level on LEDs (bar graph)
        PORTB = ~((1 << light_level) - 1); // Active LOW LEDs

        // Send data via UART
        sprintf(buffer, "Light: %u, Level: %u/8, LEDs: 0x%02X\r\n",
                light_value, light_level, PORTB);
        puts_USART1(buffer);

        // Classify light conditions
        if (light_value < 200)
        {
            puts_USART1("Status: DARK\r\n");
        }
        else if (light_value < 600)
        {
            puts_USART1("Status: DIM\r\n");
        }
        else if (light_value < 900)
        {
            puts_USART1("Status: BRIGHT\r\n");
        }
        else
        {
            puts_USART1("Status: VERY BRIGHT\r\n");
        }

        // Wait before next reading
        _delay_ms(500);
    }

    return 0;
}