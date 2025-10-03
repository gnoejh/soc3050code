
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
 * =============================================================================
 * ENHANCED ADC FUNCTIONS - PROFESSIONAL SENSOR INTERFACE
 * =============================================================================
 */

/*
 * Advanced Noise Filtering - Statistical Methods
 */
unsigned int Read_Adc_Median(unsigned char adc_input, unsigned char num_samples); // Median filter (robust)
unsigned int Read_Adc_Moving_Average(unsigned char adc_input);                    // Running average
void Reset_Moving_Average(void);                                                  // Clear averaging buffer

/*
 * Data Range and Statistics
 */
typedef struct
{
    unsigned int min_value;     // Minimum ADC value
    unsigned int max_value;     // Maximum ADC value
    unsigned int current_value; // Current ADC value
    unsigned long sum;          // Sum for averaging
    unsigned int count;         // Number of samples
    unsigned int average;       // Running average
} ADC_Statistics;

void ADC_Init_Statistics(ADC_Statistics *stats);                                                    // Initialize statistics structure
void ADC_Update_Statistics(ADC_Statistics *stats, unsigned int new_value);                          // Update statistics with new reading
void ADC_Get_Statistics(unsigned char adc_input, ADC_Statistics *stats, unsigned char num_samples); // Get complete statistics

/*
 * Threshold and Event Detection
 */
typedef struct
{
    unsigned int low_threshold;   // Lower threshold value
    unsigned int high_threshold;  // Upper threshold value
    unsigned char state;          // Current threshold state
    unsigned char event_occurred; // Flag for threshold crossing
} ADC_Threshold;

void ADC_Set_Threshold(ADC_Threshold *threshold, unsigned int low, unsigned int high);
unsigned char ADC_Check_Threshold(ADC_Threshold *threshold, unsigned int adc_value);      // Check if value crossed threshold
unsigned char ADC_Read_With_Threshold(unsigned char adc_input, ADC_Threshold *threshold); // Read and check threshold

/*
 * Multi-Point Calibration
 */
typedef struct
{
    unsigned int adc_points[10];  // ADC calibration points
    unsigned int real_values[10]; // Corresponding real values
    unsigned char num_points;     // Number of calibration points
} ADC_Calibration;

void ADC_Add_Calibration_Point(ADC_Calibration *cal, unsigned int adc_val, unsigned int real_val);
unsigned int ADC_Apply_Calibration(ADC_Calibration *cal, unsigned int adc_value); // Apply multi-point calibration

/*
 * Data Logging Buffer
 */
#define ADC_LOG_BUFFER_SIZE 64

typedef struct
{
    unsigned int buffer[ADC_LOG_BUFFER_SIZE]; // Circular buffer
    unsigned char head;                       // Buffer head index
    unsigned char tail;                       // Buffer tail index
    unsigned char count;                      // Number of samples in buffer
    unsigned char channel;                    // ADC channel being logged
} ADC_Logger;

void ADC_Logger_Init(ADC_Logger *logger, unsigned char channel);               // Initialize logger
void ADC_Logger_Add_Sample(ADC_Logger *logger, unsigned int sample);           // Add sample to buffer
unsigned char ADC_Logger_Get_Sample(ADC_Logger *logger, unsigned int *sample); // Get oldest sample
unsigned char ADC_Logger_Is_Full(ADC_Logger *logger);                          // Check if buffer full
void ADC_Logger_Clear(ADC_Logger *logger);                                     // Clear buffer

/*
 * Differential Mode Functions
 */
signed int Read_Adc_Differential(unsigned char positive_input, unsigned char negative_input); // Differential reading
unsigned int Read_Adc_Ratiometric(unsigned char signal_input, unsigned char reference_input); // Ratiometric measurement

/*
 * Auto-Ranging Functions
 */
typedef struct
{
    unsigned char current_gain; // Current gain setting (0-3)
    unsigned int scaled_value;  // Value scaled to full range
    unsigned char overrange;    // Overrange flag
    unsigned char underrange;   // Underrange flag
} ADC_AutoRange;

void ADC_AutoRange_Init(ADC_AutoRange *ar);                                  // Initialize auto-ranging
unsigned int ADC_Read_AutoRange(unsigned char adc_input, ADC_AutoRange *ar); // Read with auto-ranging

/*
 * Fast Sampling Functions
 */
void ADC_Fast_Sample_Array(unsigned char adc_input, unsigned int *buffer, unsigned char num_samples); // Fast burst sampling
unsigned int ADC_Get_Sample_Rate_Hz(void);                                                            // Get current sample rate

/*
 * Temperature Sensor Advanced Functions
 */
signed int Read_Temperature_Calibrated(unsigned char adc_input, signed int offset, unsigned int scale); // Calibrated temp
float Read_Temperature_Float(unsigned char adc_input);                                                  // Floating-point temperature

/*
 * Voltage Reference Management
 */
void ADC_Set_Reference(unsigned char ref_type); // Change voltage reference
unsigned int ADC_Measure_VCC_mV(void);          // Measure VCC using internal ref
unsigned int ADC_Measure_Internal_Ref(void);    // Measure internal 1.1V reference

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
