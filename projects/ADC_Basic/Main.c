/*
 * =============================================================================
 * ADC BASIC - DEMO CODE
 * =============================================================================
 * PROJECT: ADC_Basic
 * See Slide.md for complete ADC theory and register details
 *
 * DEMO: Read analog value from ADC0, display via UART and LEDs
 * - Reads potentiometer on ADC0 (PA0)
 * - Outputs value via UART (9600 baud)
 * - Displays on LEDs (upper 8 bits)
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