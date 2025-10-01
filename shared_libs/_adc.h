
/*
 * _adc.h - ATmega128 ADC Communication Library Header
 * Educational Version for Assembly→C→Python Learning Progression
 */

#ifndef _ADC_H_
#define _ADC_H_

/*
 * Core ADC Functions - Basic Analog Input
 */
void Adc_init(void);                                 // Initialize ADC for optimal operation
unsigned int Read_Adc_Data(unsigned char adc_input); // Read single ADC conversion

/*
 * Advanced ADC Functions - Enhanced Sensor Interface
 */
unsigned int Read_Adc_Averaged(unsigned char adc_input, unsigned char num_samples); // Noise reduction
unsigned int Read_Adc_Voltage_mV(unsigned char adc_input);                          // Voltage conversion
signed int Read_Temperature_Celsius(unsigned char adc_input);                       // Temperature sensor
unsigned int Read_Light_Level(unsigned char adc_input);                             // Light sensor (0-100%)

/*
 * Multi-Channel and Advanced Functions
 */
void Scan_Adc_Channels(unsigned int *results, unsigned char start_channel, unsigned char num_channels);
unsigned char Adc_Self_Test(void); // ADC functionality verification

/*
 * Interrupt-Based ADC (Advanced Topic)
 */
void Start_Adc_Interrupt(unsigned char adc_input); // Start non-blocking conversion
unsigned char Is_Adc_Complete(void);               // Check conversion status

/*
 * Global Variables for Educational Use
 */
extern volatile unsigned int adc_result;    // Last ADC conversion result
extern volatile unsigned char adc_channel;  // Current ADC channel
extern volatile unsigned char adc_samples;  // Number of samples for averaging
extern unsigned int adc_calibration_offset; // Calibration offset value
extern unsigned int adc_calibration_scale;  // Calibration scale factor

/*
 * ADC Constants for Educational Reference
 */
#define ADC_MAX_VALUE 1023       // Maximum 10-bit ADC value
#define ADC_REFERENCE_AVCC 5000  // AVCC reference in millivolts (5V)
#define ADC_REFERENCE_2_56V 2560 // Internal 2.56V reference in millivolts
#define ADC_LSB_mV_AVCC 4.883f   // LSB value in mV for AVCC reference (5000/1024)
#define ADC_LSB_mV_2_56V 2.5f    // LSB value in mV for 2.56V reference (2560/1024)

/*
 * ADC Channel Definitions for Educational Use
 */
#define ADC_CHANNEL_0 0 // ADC0 - General purpose analog input
#define ADC_CHANNEL_1 1 // ADC1 - General purpose analog input
#define ADC_CHANNEL_2 2 // ADC2 - General purpose analog input
#define ADC_CHANNEL_3 3 // ADC3 - General purpose analog input
#define ADC_CHANNEL_4 4 // ADC4 - General purpose analog input
#define ADC_CHANNEL_5 5 // ADC5 - General purpose analog input
#define ADC_CHANNEL_6 6 // ADC6 - General purpose analog input
#define ADC_CHANNEL_7 7 // ADC7 - General purpose analog input

/*
 * Sensor Interface Constants
 */
#define TEMPERATURE_SENSOR_ADC ADC_CHANNEL_0 // Default channel for temperature sensor
#define LIGHT_SENSOR_ADC ADC_CHANNEL_1       // Default channel for light sensor
#define POTENTIOMETER_ADC ADC_CHANNEL_2      // Default channel for potentiometer
#define VOLTAGE_DIVIDER_ADC ADC_CHANNEL_3    // Default channel for voltage measurement

/*
 * Function Prototypes for Educational Examples
 * These are implemented in separate main_*.c files
 */
void main_adc_basic_reading(void);       // Basic ADC reading example
void main_adc_voltage_measurement(void); // Voltage measurement example
void main_adc_temperature_sensor(void);  // Temperature sensor example
void main_adc_light_sensor(void);        // Light sensor example
void main_adc_multi_channel(void);       // Multi-channel scanning example
void main_adc_interrupt_driven(void);    // Interrupt-driven ADC example

#endif // _ADC_H_
