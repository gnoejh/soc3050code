/*
 * ==============================================================================
 * ACCELEROMETER - DEMO CODE (REFINED)
 * ==============================================================================
 * PROJECT: Accelerometer
 * See Slide.md for complete theory and technical details
 *
 * DEMOS: Accelerometer interfacing, axis reading, motion detection
 *
 * EDUCATIONAL NOTE: Using enhanced ADC library for noise-free readings
 * OLD APPROACH: Simple Adc_read_ch() with noisy data
 * NEW APPROACH: Read_Adc_Averaged() for stable, reliable measurements
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
    puts_USART1("3-Axis Accelerometer Started\r\n");
    puts_USART1("Reading X, Y, Z acceleration values...\r\n");

    uint16_t x_axis, y_axis, z_axis;
    uint16_t x_prev = 512, y_prev = 512, z_prev = 512; // Assume 512 = center
    char buffer[80];
    uint8_t motion_detected = 0;

    while (1)
    {
        // Read accelerometer values from ADC channels 2, 3, 4
        // Using averaging for stable, noise-free readings
        x_axis = Read_Adc_Averaged(2, 8); // X-axis (8 samples)
        y_axis = Read_Adc_Averaged(3, 8); // Y-axis
        z_axis = Read_Adc_Averaged(4, 8); // Z-axis

        // Detect motion (simple threshold-based)
        if (abs(x_axis - x_prev) > 50 ||
            abs(y_axis - y_prev) > 50 ||
            abs(z_axis - z_prev) > 50)
        {
            motion_detected = 1;
            PORTB = 0x00; // Turn on all LEDs (active LOW)
        }
        else
        {
            motion_detected = 0;
            PORTB = 0xFF; // Turn off all LEDs
        }

        // Send data via UART
        sprintf(buffer, "X:%u Y:%u Z:%u Motion:%s\r\n",
                x_axis, y_axis, z_axis,
                motion_detected ? "YES" : "NO");
        puts_USART1(buffer);

        // Analyze orientation (simplified)
        if (z_axis > 700)
        {
            puts_USART1("Orientation: FACE UP\r\n");
        }
        else if (z_axis < 300)
        {
            puts_USART1("Orientation: FACE DOWN\r\n");
        }
        else if (x_axis > 700)
        {
            puts_USART1("Orientation: TILTED RIGHT\r\n");
        }
        else if (x_axis < 300)
        {
            puts_USART1("Orientation: TILTED LEFT\r\n");
        }
        else
        {
            puts_USART1("Orientation: LEVEL\r\n");
        }

        // Store previous values for motion detection
        x_prev = x_axis;
        y_prev = y_axis;
        z_prev = z_axis;

        // Wait before next reading
        _delay_ms(200);
    }

    return 0;
}
