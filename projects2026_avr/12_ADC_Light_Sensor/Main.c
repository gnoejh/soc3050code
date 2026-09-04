/*
 * ==============================================================================
 * CDS LIGHT SENSOR - DEMO CODE (REFINED)
 * ==============================================================================
 * PROJECT: CDS_Light_Sensor
 * See Slide.md for complete theory and technical details
 *
 * DEMOS: Light sensor reading, ADC conversion, ambient light detection
 *
 * EDUCATIONAL NOTE: Using enhanced ADC library for stable light readings
 * OLD APPROACH: Simple Adc_read_ch() with fluctuating values
 * NEW APPROACH: Read_Adc_Averaged() for smooth, reliable light detection
 * ==============================================================================
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
        // Using averaging for stable, flicker-free readings
        light_value = Read_Adc_Averaged(1, 16); // 16 samples for smooth detection

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
