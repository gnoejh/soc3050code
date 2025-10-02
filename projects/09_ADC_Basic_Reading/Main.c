/*
 * ADC Basic Reading - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Understand ADC initialization and configuration
 * - Learn analog-to-digital conversion process
 * - Practice reading sensor values
 * - Master UART communication for data output
 *
 * HARDWARE SETUP:
 * - Connect analog sensor to ADC0 (PA0)
 * - Connect UART for serial output
 * - Optional: LEDs on PORTB for visual feedback
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