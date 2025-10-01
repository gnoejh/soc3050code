/*
 * _adc.c - ATmega128 Analog-to-Digital Converter Library
 * Educational Version for Assembly→C→Python Learning Progression
 *
 * LEARNING OBJECTIVES:
 * 1. Understand ADC register configuration (ADMUX, ADCSRA)
 * 2. Learn voltage reference concepts (AREF, AVCC, Internal)
 * 3. Understand conversion timing and prescaler settings
 * 4. Bridge analog hardware concepts to digital data processing
 *
 * ASSEMBLY EQUIVALENT CONCEPTS:
 * - ADCSRA |= (1<<ADSC)  ≡  LDS R16, ADCSRA; ORI R16, 0x40; STS ADCSRA, R16
 * - Reading ADCL/ADCH    ≡  LDS R16, ADCL; LDS R17, ADCH
 * - Polling ADIF flag    ≡  LDS R16, ADCSRA; SBRS R16, ADIF
 */

#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "_main.h"
#include "_adc.h"

/*
 * ADC Voltage Reference Configuration Constants
 * These help students understand different voltage reference options
 */
#define ADC_AREF_EXTERNAL 0x00  // External AREF pin (precise external reference)
#define ADC_AVCC_REFERENCE 0x40 // AVCC pin (usually +5V, convenient)
#define ADC_INTERNAL_1_1V 0x80  // Internal 1.1V reference (not available on ATmega128)
#define ADC_INTERNAL_2_56V 0xC0 // Internal 2.56V reference (ATmega128 specific)

/*
 * ADC Prescaler Configuration Constants
 * ADC clock must be 50-200kHz for best accuracy
 * For 16MHz system clock: prescaler 128 gives 125kHz ADC clock
 */
#define ADC_PRESCALER_2 0x01   // F_CPU/2   = 8MHz   (too fast)
#define ADC_PRESCALER_4 0x02   // F_CPU/4   = 4MHz   (too fast)
#define ADC_PRESCALER_8 0x03   // F_CPU/8   = 2MHz   (too fast)
#define ADC_PRESCALER_16 0x04  // F_CPU/16  = 1MHz   (too fast)
#define ADC_PRESCALER_32 0x05  // F_CPU/32  = 500kHz (too fast)
#define ADC_PRESCALER_64 0x06  // F_CPU/64  = 250kHz (acceptable)
#define ADC_PRESCALER_128 0x07 // F_CPU/128 = 125kHz (optimal)

/*
 * Adc_init() - Initialize ADC for basic analog input
 *
 * EDUCATIONAL NOTES:
 * - ADC converts analog voltage (0-5V) to digital number (0-1023)
 * - ADMUX register selects input channel and voltage reference
 * - ADCSRA register controls conversion timing and enables ADC
 * - Prescaler sets ADC clock speed for accurate conversion
 *
 * REGISTER CONFIGURATION DETAILS:
 * - ADCSRA: ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
 * - ADMUX:  REFS1 REFS0 ADLAR - MUX3 MUX2 MUX1 MUX0
 */
void Adc_init(void)
{
    // Step 1: Disable ADC during configuration
    // Assembly equivalent: LDI R16, 0x00; STS ADCSRA, R16
    ADCSRA = 0x00; // Disable ADC (ADEN=0)

    // Step 2: Configure input channel and voltage reference
    // Assembly equivalent: LDI R16, 0x00; STS ADMUX, R16
    ADMUX = 0x00; // Channel 0 (ADC0), AREF external reference

    // Step 3: Disable analog comparator (saves power)
    // Assembly equivalent: LDI R16, 0x80; STS ACSR, R16
    ACSR = 0x80; // ACD=1 (Analog Comparator Disable)

    // Step 4: Enable ADC with prescaler for optimal timing
    // Assembly equivalent: LDI R16, 0x87; STS ADCSRA, R16
    ADCSRA = 0x87; // ADEN=1 (enable), prescaler=128
    // Binary: 10000111
    // Bit 7 (ADEN) = 1: ADC Enable
    // Bit 6 (ADSC) = 0: No conversion yet
    // Bit 5 (ADATE) = 0: Single conversion mode
    // Bit 4 (ADIF) = 0: Interrupt flag (cleared)
    // Bit 3 (ADIE) = 0: Interrupt disabled
    // Bits 2:0 = 111: Prescaler 128 (optimal for 16MHz)
}

/*
 * Adc_read() - Read analog value from specified ADC channel
 *
 * EDUCATIONAL NOTES:
 * - Channel parameter selects which analog input pin (0-7)
 * - Function demonstrates polling method for ADC conversion
 * - Returns 10-bit result (0-1023) representing input voltage
 * - Voltage calculation: volts = (adc_value * reference_voltage) / 1024
 *
 * ASSEMBLY EQUIVALENT PROCESS:
 * 1. Set channel in ADMUX
 * 2. Start conversion (ADSC=1)
 * 3. Wait for completion (ADIF=1)
 * 4. Read result from ADCL/ADCH
 */
unsigned int Adc_read(unsigned char channel)
{
    unsigned int adc_result = 0;

    // Step 1: Validate channel number (0-7 for ATmega128)
    if (channel > 7)
    {
        return 0; // Invalid channel returns 0
    }

    // Step 2: Select ADC channel and voltage reference
    // Keep existing reference setting, change only channel
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    // This preserves voltage reference bits (7:4) and sets channel bits (3:0)

    // Step 3: Start ADC conversion
    // Assembly equivalent: LDS R16, ADCSRA; ORI R16, 0x40; STS ADCSRA, R16
    ADCSRA |= (1 << ADSC); // Set ADSC bit to start conversion

    // Step 4: Wait for conversion completion (polling method)
    // Assembly equivalent:
    // wait_loop: LDS R16, ADCSRA; SBRS R16, ADIF; RJMP wait_loop
    while ((ADCSRA & (1 << ADIF)) == 0)
        ; // Poll ADIF flag until conversion complete

    // Step 5: Read ADC result (must read ADCL first, then ADCH)
    // Assembly equivalent: LDS R16, ADCL; LDS R17, ADCH
    adc_result = ADCL;         // Read low byte first (important!)
    adc_result += (ADCH << 8); // Add high byte shifted left 8 positions

    // Step 6: Clear conversion complete flag for next conversion
    ADCSRA |= (1 << ADIF); // Write 1 to clear ADIF flag

    return adc_result; // Return 10-bit result (0-1023)
}

/*
 * Educational Helper Functions
 * These bridge ADC concepts to real-world measurements
 */

/*
 * Adc_read_voltage() - Convert ADC reading to voltage
 * Assumes AVCC reference (5.0V) for educational simplicity
 */
float Adc_read_voltage(unsigned char channel)
{
    unsigned int adc_value;
    float voltage;

    adc_value = Adc_read(channel); // Get digital value (0-1023)

    // Convert to voltage: (ADC / 1024) * reference_voltage
    voltage = (adc_value * 5.0) / 1024.0; // Assume 5V reference

    return voltage; // Return voltage as floating point
}

/*
 * Adc_read_percentage() - Convert ADC reading to percentage (0-100%)
 * Useful for sensors, potentiometers, etc.
 */
unsigned char Adc_read_percentage(unsigned char channel)
{
    unsigned int adc_value;
    unsigned char percentage;

    adc_value = Adc_read(channel); // Get digital value (0-1023)

    // Convert to percentage: (ADC * 100) / 1023
    percentage = (adc_value * 100) / 1023;

    return percentage; // Return 0-100%
}

/*
 * Adc_set_reference() - Change voltage reference for educational experiments
 */
void Adc_set_reference(unsigned char reference)
{
    // Preserve channel selection, change only reference bits
    ADMUX = (ADMUX & 0x0F) | (reference & 0xF0);
}

/*
 * Educational Reference Setting Functions
 */
void Adc_use_external_reference(void)
{
    Adc_set_reference(ADC_AREF_EXTERNAL); // Use external AREF pin
}

void Adc_use_avcc_reference(void)
{
    Adc_set_reference(ADC_AVCC_REFERENCE); // Use AVCC (usually 5V)
}

void Adc_use_internal_reference(void)
{
    Adc_set_reference(ADC_INTERNAL_2_56V); // Use internal 2.56V
}

/*
 * Adc_read_multiple() - Read multiple channels for sensor arrays
 * Educational function showing how to read multiple analog inputs
 */
void Adc_read_multiple(unsigned char *channels, unsigned int *results, unsigned char count)
{
    unsigned char i;

    for (i = 0; i < count; i++)
    {
        if (channels[i] <= 7)
        { // Validate channel
            results[i] = Adc_read(channels[i]);
        }
        else
        {
            results[i] = 0; // Invalid channel
        }
        _delay_ms(1); // Small delay between readings
    }
}

/*
 * EDUCATIONAL PROGRESSION NOTES:
 *
 * 1. ASSEMBLY LEVEL (Direct Register Access):
 *    Students learn: LDS R16, ADCSRA; ORI R16, 0x40; STS ADCSRA, R16
 *    Understanding polling loops and register bit manipulation
 *
 * 2. C ABSTRACTION LEVEL (This Library):
 *    Students learn: adc_value = Adc_read(0); voltage = Adc_read_voltage(0);
 *    Understanding function parameters and return values
 *
 * 3. PYTHON LEVEL (High-Level Interface):
 *    Students learn: voltage = atmega.adc.read_voltage(0)
 *    Understanding object methods and automatic unit conversion
 *
 * PROGRESSION TOPICS:
 * - Analog vs Digital signal concepts
 * - Voltage references and measurement accuracy
 * - Sampling rate and conversion timing
 * - Sensor interfacing and calibration
 * - Data filtering and noise reduction
 */