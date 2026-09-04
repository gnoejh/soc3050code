
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
 * =============================================================================
 * ENHANCED ADC FEATURES - Professional Quality Additions
 * =============================================================================
 */

/**
 * @brief Calibrate ADC with known reference voltage
 *
 * @param reference_voltage_mV Known reference voltage in millivolts
 *
 * EDUCATIONAL NOTE: Improves accuracy by correcting for voltage reference variations.
 * Apply known voltage to ADC channel, then call this function.
 */
void ADC_calibrate(unsigned int reference_voltage_mV);

/**
 * @brief Set ADC precision (8-bit or 10-bit)
 *
 * @param bits Precision in bits (8 or 10)
 *
 * EDUCATIONAL NOTE: 8-bit mode is faster, 10-bit gives better resolution.
 * Uses ADLAR bit to select left-adjusted (8-bit) or right-adjusted (10-bit).
 */
void ADC_set_precision(unsigned char bits);

/**
 * @brief Get ADC precision
 *
 * @return Current precision in bits (8 or 10)
 */
unsigned char ADC_get_precision(void);

/**
 * @brief Start free-running (continuous) conversion mode
 *
 * @param channel ADC channel for continuous sampling
 *
 * EDUCATIONAL NOTE: ADC converts continuously without explicit start commands.
 * Useful for high-speed data logging. Use ADC_get_last_result() to read values.
 */
void ADC_start_free_running(unsigned char channel);

/**
 * @brief Stop free-running mode
 */
void ADC_stop_free_running(void);

/**
 * @brief Get last ADC result (from interrupt or free-running mode)
 *
 * @return Last ADC conversion result
 */
unsigned int ADC_get_last_result(void);

/**
 * @brief Read differential input (positive - negative)
 *
 * @param pos_channel Positive channel (0-7)
 * @param neg_channel Negative channel (0-7)
 * @param gain Gain selection (1, 10, or 200)
 * @return Signed differential value (-512 to +511)
 *
 * EDUCATIONAL NOTE: Measures voltage difference between two channels.
 * Useful for bridge sensors, thermocouples, and differential signals.
 * ATmega128 supports differential mode with selectable gain.
 */
int ADC_read_differential(unsigned char pos_channel, unsigned char neg_channel, unsigned char gain);

/**
 * @brief Automatically detect and set voltage reference
 *
 * @return Detected reference voltage in mV (5000 for AVCC, 2560 for internal)
 *
 * EDUCATIONAL NOTE: Attempts to determine which reference is being used.
 * Reads internal bandgap reference and calculates actual Vref.
 */
unsigned int ADC_set_reference_auto(void);

/**
 * @brief Set voltage reference explicitly
 *
 * @param reference Reference type: 0=AREF, 1=AVCC, 2=Internal 2.56V
 */
void ADC_set_reference(unsigned char reference);

/**
 * @brief Get current voltage reference setting
 *
 * @return Reference type (0=AREF, 1=AVCC, 2=Internal 2.56V)
 */
unsigned char ADC_get_reference(void);

/**
 * @brief Reset ADC calibration to default values
 *
 * EDUCATIONAL NOTE: Clears offset and scale to factory defaults.
 */
void ADC_reset_calibration(void);

/**
 * @brief Get ADC sample rate in Hz
 *
 * @return Current sampling rate
 *
 * EDUCATIONAL NOTE: Calculated from prescaler and conversion time.
 */
unsigned int ADC_get_sample_rate_Hz(void);

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

/*
 * =============================================================================
 * BACKWARD COMPATIBILITY LAYER
 * =============================================================================
 * Legacy function names for older projects
 */

/* Legacy function aliases - redirect to new names */
#define Adc_read_ch(ch) Read_Adc_Data(ch)
#define Read_Adc(ch) Read_Adc_Data(ch)

#endif // _ADC_H_
