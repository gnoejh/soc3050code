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
 * DOCUMENTATION REFERENCE:
 * ATmega128 Datasheet: https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/2467S.pdf
 * - ADC section (pages 206-230)
 * - Analog Comparator (pages 231-235)
 * - Internal voltage reference (page 41)
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Basic ADC Configuration
 * - Demo 2: Single Channel Reading
 * - Demo 3: Multi-Channel Sampling
 * - Demo 4: Continuous Monitoring
 * - Demo 5: Sensor Calibration
 *
 * =============================================================================
 * ADC CONTROL REGISTERS - DETAILED REFERENCE FOR STUDENTS
 * =============================================================================
 *
 * REGISTER 1: ADMUX (ADC Multiplexer Selection Register)
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name: REFS1  REFS0  ADLAR   -    MUX4   MUX3   MUX2   MUX1   MUX0
 *
 * REFS1:0 (bits 7-6): Voltage Reference Selection
 *                     00 = AREF pin, internal Vref off
 *                     01 = AVCC with external capacitor on AREF
 *                     10 = Reserved
 *                     11 = Internal 2.56V reference with cap on AREF
 *                     Example: ADMUX = (1<<REFS0); // Use AVCC as reference
 *
 * ADLAR (bit 5): ADC Left Adjust Result
 *                0 = Right adjust (ADCH:ADCL = 00000000:XXXXXXXX, read ADCL first)
 *                1 = Left adjust  (ADCH:ADCL = XXXXXXXX:XX000000, read ADCH for 8-bit)
 *                Example: ADMUX |= (1<<ADLAR); // Left adjust for 8-bit reading
 *
 * MUX4:0 (bits 4-0): Analog Channel Selection (ATmega128 has channels 0-7)
 *                    00000 = ADC0 (PF0)
 *                    00001 = ADC1 (PF1)
 *                    00010 = ADC2 (PF2)
 *                    ...
 *                    00111 = ADC7 (PF7)
 *                    Example: ADMUX = (ADMUX & 0xE0) | 3; // Select ADC3
 *
 * REGISTER 2: ADCSRA (ADC Control and Status Register A)
 *
 *    Bit:   7      6      5      4      3      2      1      0
 *    Name: ADEN   ADSC   ADFR   ADIF   ADIE  ADPS2  ADPS1  ADPS0
 *
 * ADEN (bit 7): ADC Enable
 *               1 = Enable ADC
 *               Example: ADCSRA |= (1<<ADEN); // Turn on ADC
 *
 * ADSC (bit 6): ADC Start Conversion
 *               Write 1 to start conversion, reads 1 while converting, 0 when done
 *               Example: ADCSRA |= (1<<ADSC); // Start conversion
 *               Poll: while (ADCSRA & (1<<ADSC)); // Wait for completion
 *
 * ADFR (bit 5): ADC Free Running Select
 *               1 = Auto trigger mode (continuous conversion)
 *               0 = Single conversion mode
 *               Example: ADCSRA |= (1<<ADFR); // Enable auto-trigger
 *
 * ADIF (bit 4): ADC Interrupt Flag
 *               Set when conversion completes, cleared by writing 1 or reading ADCL/ADCH
 *               Poll: while (!(ADCSRA & (1<<ADIF))); // Wait for conversion
 *
 * ADIE (bit 3): ADC Interrupt Enable
 *               1 = Enable ADC conversion complete interrupt
 *               Example: ADCSRA |= (1<<ADIE); sei(); // Enable interrupt
 *               ISR: ISR(ADC_vect) { result = ADCL; result |= (ADCH<<8); }
 *
 * ADPS2:0 (bits 2-0): ADC Prescaler Select (ADC clock = F_CPU / prescaler)
 *                     ADC needs 50-200 kHz for max resolution (10-bit)
 *                     @ 16MHz: prescaler 128 → 16MHz/128 = 125 kHz (optimal)
 *
 *                     000 = /2    (not recommended, too fast)
 *                     001 = /2
 *                     010 = /4
 *                     011 = /8
 *                     100 = /16
 *                     101 = /32
 *                     110 = /64
 *                     111 = /128  (recommended @ 16MHz)
 *                     Example: ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // /128
 *
 * REGISTER 3: ADCL/ADCH (ADC Data Registers - 16-bit result)
 *
 * Right Adjusted (ADLAR=0):
 *   ADCH:        0  0  0  0  0  0  D9 D8
 *   ADCL:       D7 D6 D5 D4 D3 D2 D1 D0
 *   Read: result = ADCL; result |= (ADCH << 8); // Must read ADCL first!
 *
 * Left Adjusted (ADLAR=1):
 *   ADCH:       D9 D8 D7 D6 D5 D4 D3 D2
 *   ADCL:       D1 D0  0  0  0  0  0  0
 *   Read 8-bit: result = ADCH; // Just read high byte for 8-bit precision
 *
 * CRITICAL READING SEQUENCE:
 * - Right adjusted: MUST read ADCL first, then ADCH (locks data)
 * - Left adjusted: Can read ADCH only for 8-bit result
 * - Reading ADCL unlocks the register for next conversion
 *
 * ADC CONVERSION TIME CALCULATION:
 * Conversion time = 13 ADC clock cycles (first conv: 25 cycles)
 * @ 125 kHz ADC clock: 13 / 125kHz = 104 µs per conversion
 * Max sampling rate: ~9600 samples/second
 *
 * TYPICAL ADC INITIALIZATION @ 16MHz:
 *   ADMUX = (1<<REFS0);              // AVCC reference
 *   ADCSRA = (1<<ADEN)|(1<<ADPS2)|   // Enable ADC
 *            (1<<ADPS1)|(1<<ADPS0);  // Prescaler 128
 *
 * TYPICAL SINGLE CONVERSION:
 *   ADMUX = (ADMUX & 0xE0) | channel; // Select channel
 *   ADCSRA |= (1<<ADSC);              // Start conversion
 *   while (ADCSRA & (1<<ADSC));       // Wait for completion
 *   result = ADCL;                    // Read low byte first!
 *   result |= (ADCH << 8);            // Then high byte
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