/*
 * =============================================================================
 * ANALOG-TO-DIGITAL CONVERSION - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: ADC_Basic
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of ATmega128 ADC operations and analog signal processing.
 * Students learn analog-to-digital conversion concepts and sensor interfacing.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master ADC configuration and reference selection
 * 2. Learn analog sensor interfacing techniques
 * 3. Practice data acquisition and processing
 * 4. Understand resolution and precision concepts
 * 5. Implement real-time monitoring systems
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - Potentiometer on ADC0 for voltage input
 * - LCD display for value visualization
 * - Serial connection for data logging (9600 baud)
 * - Optional: Multiple analog sensors
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Basic ADC Configuration
 * - Demo 2: Single Channel Reading
 * - Demo 3: Multi-Channel Sampling
 * - Demo 4: Continuous Monitoring
 * - Demo 5: Sensor Calibration
 *
 * =============================================================================
 */

#include "config.h"

int main(void)
{
    // Initialize system components
    init_devices(); // Initialize all peripherals

    // Initialize UART for data output
    Uart1_init(); // 9600 baud serial communication

    // Send startup message
    puts_USART1("ADC Basic Reading Started\r\n");
    puts_USART1("Reading analog values from ADC0...\r\n");

    uint16_t adc_value;
    char buffer[50];

    while (1)
    {
        // Read ADC value from channel 0 (PA0)
        adc_value = Adc_read_ch(0);

        // Convert to string and send via UART
        sprintf(buffer, "ADC Value: %u (0x%03X)\r\n", adc_value, adc_value);
        puts_USART1(buffer);

        // Visual feedback on LEDs (optional)
        PORTB = (uint8_t)(adc_value >> 2); // Display upper 8 bits on LEDs

        // Wait before next reading
        _delay_ms(1000);
    }

    return 0;
}