/*
 * _adc.h - ATmega128 Analog-to-Digital Converter Library Header
 * Educational Version for Assembly→C→Python Learning Progression
 */

#ifndef _ADC_H_
#define _ADC_H_

/*
 * Core ADC Functions
 */
void Adc_init(void);                          // Initialize ADC with optimal settings
unsigned int Adc_read(unsigned char channel); // Read raw ADC value (0-1023)

/*
 * Educational Conversion Functions
 * These help students understand ADC data interpretation
 */
float Adc_read_voltage(unsigned char channel);            // Read as voltage (0.0-5.0V)
unsigned char Adc_read_percentage(unsigned char channel); // Read as percentage (0-100%)

/*
 * Voltage Reference Control Functions
 * Educational functions for understanding different reference voltages
 */
void Adc_set_reference(unsigned char reference); // Set voltage reference
void Adc_use_external_reference(void);           // Use external AREF pin
void Adc_use_avcc_reference(void);               // Use AVCC (usually 5V)
void Adc_use_internal_reference(void);           // Use internal 2.56V

/*
 * Multi-Channel Functions
 * For sensor arrays and multiple analog inputs
 */
void Adc_read_multiple(unsigned char *channels, unsigned int *results, unsigned char count);

/*
 * ADC Channel Constants (ATmega128 has 8 ADC channels)
 */
#define ADC_CHANNEL_0 0 // ADC0 pin
#define ADC_CHANNEL_1 1 // ADC1 pin
#define ADC_CHANNEL_2 2 // ADC2 pin
#define ADC_CHANNEL_3 3 // ADC3 pin
#define ADC_CHANNEL_4 4 // ADC4 pin
#define ADC_CHANNEL_5 5 // ADC5 pin
#define ADC_CHANNEL_6 6 // ADC6 pin
#define ADC_CHANNEL_7 7 // ADC7 pin

/*
 * Voltage Reference Constants
 */
#define ADC_AREF_EXTERNAL 0x00  // External reference on AREF pin
#define ADC_AVCC_REFERENCE 0x40 // AVCC with external capacitor on AREF
#define ADC_INTERNAL_2_56V 0xC0 // Internal 2.56V reference (ATmega128)

/*
 * ADC Resolution and Range Constants
 */
#define ADC_RESOLUTION_BITS 10 // ATmega128 has 10-bit ADC
#define ADC_MAX_VALUE 1023     // Maximum ADC reading (2^10 - 1)
#define ADC_MIN_VALUE 0        // Minimum ADC reading

/*
 * Common Voltage Values for Educational Reference
 */
#define VOLTAGE_5V 5.0    // Standard logic voltage
#define VOLTAGE_3V3 3.3   // Common sensor voltage
#define VOLTAGE_2V56 2.56 // Internal reference voltage
#define VOLTAGE_1V1 1.1   // Low voltage reference

/*
 * Educational Calculation Macros
 * These help students understand ADC mathematics
 */
#define ADC_TO_VOLTAGE_5V(adc) ((adc * 5.0) / 1023.0)   // Convert ADC to 5V scale
#define ADC_TO_VOLTAGE_3V3(adc) ((adc * 3.3) / 1023.0)  // Convert ADC to 3.3V scale
#define ADC_TO_PERCENTAGE(adc) ((adc * 100) / 1023)     // Convert ADC to percentage
#define VOLTAGE_TO_ADC_5V(volt) ((volt * 1023.0) / 5.0) // Convert voltage to ADC value

/*
 * Sensor Interface Constants
 * Common sensor types for educational projects
 */
#define SENSOR_POTENTIOMETER 0    // Variable resistor (0-5V)
#define SENSOR_LIGHT_LDR 1        // Light dependent resistor
#define SENSOR_TEMPERATURE_LM35 2 // Temperature sensor (10mV/°C)
#define SENSOR_MICROPHONE 3       // Audio input
#define SENSOR_ACCELEROMETER_X 4  // X-axis accelerometer
#define SENSOR_ACCELEROMETER_Y 5  // Y-axis accelerometer
#define SENSOR_JOYSTICK_X 6       // Joystick X-axis
#define SENSOR_JOYSTICK_Y 7       // Joystick Y-axis

/*
 * Educational Function Prototypes for Examples
 * These will be implemented in main_xxx.c files
 */
void main_adc_simple_read(void);      // Basic ADC reading example
void main_adc_voltage_display(void);  // Display voltage on serial
void main_adc_percentage_leds(void);  // Show percentage on LEDs
void main_adc_multiple_sensors(void); // Read multiple sensors
void main_adc_data_logging(void);     // Log sensor data over time

/*
 * EDUCATIONAL USAGE EXAMPLES:
 *
 * Basic ADC reading:
 *   Adc_init();
 *   value = Adc_read(0);              // Read channel 0 (0-1023)
 *
 * Voltage measurement:
 *   Adc_init();
 *   voltage = Adc_read_voltage(0);    // Read channel 0 as voltage
 *
 * Percentage reading:
 *   Adc_init();
 *   percent = Adc_read_percentage(0); // Read channel 0 as 0-100%
 *
 * Multiple channels:
 *   unsigned char channels[] = {0, 1, 2};
 *   unsigned int results[3];
 *   Adc_read_multiple(channels, results, 3);
 */

#endif // _ADC_H_