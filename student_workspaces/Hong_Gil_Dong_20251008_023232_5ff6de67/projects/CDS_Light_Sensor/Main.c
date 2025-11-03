/*
 * =============================================================================
 * LIGHT SENSOR INTERFACE - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: CDS_Light_Sensor
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of photoresistor-based light sensing systems.
 * Students learn analog sensor interfacing and environmental monitoring.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master photoresistor operation and characteristics
 * 2. Learn voltage divider circuits for sensors
 * 3. Practice threshold-based control systems
 * 4. Implement sensor calibration techniques
 * 5. Process and filter analog sensor data
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - CDS photoresistor with voltage divider circuit
 * - 10kΩ reference resistor for voltage division
 * - LEDs on PORTB for light level indication
 * - Serial connection for data logging (9600 baud)
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Basic Light Level Reading
 * - Demo 2: Threshold-Based LED Control
 * - Demo 3: Multi-Level Light Classification
 * - Demo 4: Sensor Calibration and Scaling
 * - Demo 5: Environmental Monitoring System
 *
 * =============================================================================
 */
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