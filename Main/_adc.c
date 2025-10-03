
/*
 * _adc.c - ATmega128 Educational ADC (Analog-to-Digital Converter) Library
 * Part of Assembly → C → Python Learning Progression
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Understand analog-to-digital conversion concepts
 * 2. Learn ADC register configuration (ADCSRA, ADMUX, ADCL/ADCH)
 * 3. Master different voltage reference options
 * 4. Practice sensor interfacing and calibration
 * 5. Bridge assembly register access to C abstraction
 * 6. Prepare for Python sensor data acquisition
 *
 * ADC OVERVIEW:
 * - ADC = Analog-to-Digital Converter
 * - Converts analog voltage (0V to VREF) to digital value (0 to 1023)
 * - 10-bit resolution = 1024 different values
 * - Multiple input channels (ADC0-ADC7 on ATmega128)
 * - Configurable voltage reference (AREF, AVCC, Internal 2.56V)
 *
 * ATmega128 ADC FEATURES:
 * - 8 multiplexed ADC input channels
 * - 10-bit resolution (0-1023 values)
 * - Configurable prescaler (division factor 2-128)
 * - Multiple voltage reference options
 * - Interrupt-driven or polling operation
 * - Auto-triggering from various sources
 *
 * ASSEMBLY EQUIVALENT CONCEPTS:
 * - ADCSRA = control  ≡  LDI R16, control; STS ADCSRA, R16
 * - Start conversion  ≡  LDI R16, (1<<ADSC); STS ADCSRA, R16
 * - Read result      ≡  LDS R16, ADCL; LDS R17, ADCH
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "_main.h"
#include "_adc.h"

// Only compile ADC functions if not using self-contained assembly example
#ifndef ASSEMBLY_BLINK_BASIC

/*
 * EDUCATIONAL CONSTANTS: ADC Voltage Reference Selection
 * These control the voltage reference used for ADC conversion
 * REFS1:0 bits in ADMUX register determine the reference voltage
 */
#define ADC_VREF_TYPE 0x00 // External AREF pin voltage reference (REFS1:0 = 00)
#define ADC_AVCC_TYPE 0x40 // AVCC (supply voltage) reference (REFS1:0 = 01)
#define ADC_RES_TYPE 0x80  // Reserved - not used (REFS1:0 = 10)
#define ADC_2_56_TYPE 0xC0 // Internal 2.56V reference (REFS1:0 = 11)

/*
 * EDUCATIONAL CONSTANTS: ADC Prescaler Values
 * These control the ADC clock frequency for proper conversion timing
 * ADC clock should be between 50kHz and 200kHz for maximum resolution
 */
#define ADC_PRESCALE_2 0x01	  // F_CPU/2   (8MHz for 16MHz system) - too fast
#define ADC_PRESCALE_4 0x02	  // F_CPU/4   (4MHz for 16MHz system) - too fast
#define ADC_PRESCALE_8 0x03	  // F_CPU/8   (2MHz for 16MHz system) - too fast
#define ADC_PRESCALE_16 0x04  // F_CPU/16  (1MHz for 16MHz system) - too fast
#define ADC_PRESCALE_32 0x05  // F_CPU/32  (500kHz for 16MHz system) - too fast
#define ADC_PRESCALE_64 0x06  // F_CPU/64  (250kHz for 16MHz system) - too fast
#define ADC_PRESCALE_128 0x07 // F_CPU/128 (125kHz for 16MHz system) - optimal

/*
 * EDUCATIONAL VARIABLES
 * Global variables for learning ADC concepts and sensor interfacing
 */
volatile unsigned int adc_result = 0;	   // Last ADC conversion result
volatile unsigned char adc_channel = 0;	   // Current ADC channel
volatile unsigned char adc_samples = 1;	   // Number of samples for averaging
unsigned int adc_calibration_offset = 0;   // Calibration offset value
unsigned int adc_calibration_scale = 1024; // Calibration scale factor

/*
 * EDUCATIONAL FUNCTION: ADC Initialization
 *
 * PURPOSE: Configure ADC for 10-bit conversion with optimal settings
 * LEARNING: Shows complete ADC setup sequence and register configuration
 *
 * REGISTER EXPLANATION:
 * - ADCSRA: ADC Control and Status Register A
 *   ADEN  = ADC Enable (1 = enabled)
 *   ADSC  = ADC Start Conversion (1 = start conversion)
 *   ADATE = ADC Auto Trigger Enable
 *   ADIF  = ADC Interrupt Flag
 *   ADIE  = ADC Interrupt Enable
 *   ADPS2:0 = ADC Prescaler Select (000=div2, 111=div128)
 *
 * - ADMUX: ADC Multiplexer Selection Register
 *   REFS1:0 = Reference Selection (00=AREF, 01=AVCC, 11=2.56V)
 *   ADLAR   = ADC Left Adjust Result (0=right adjust, 1=left adjust)
 *   MUX4:0  = Analog Channel Selection (00000=ADC0, 00001=ADC1, etc.)
 *
 * - ACSR: Analog Comparator Control and Status Register
 *   ACD = Analog Comparator Disable (1 = disable to save power)
 *
 * ASSEMBLY EQUIVALENT:
 * LDI R16, 0x00; STS ADCSRA, R16    ; Disable ADC initially
 * LDI R16, 0x00; STS ADMUX, R16     ; Select ADC0 and AREF
 * LDI R16, 0x80; STS ACSR, R16      ; Disable analog comparator
 * LDI R16, 0x87; STS ADCSRA, R16    ; Enable ADC with prescaler 128
 */
void Adc_init(void)
{
	/*
	 * STEP 1: Disable ADC initially for safe configuration
	 * Clear all ADCSRA bits to ensure clean start
	 */
	ADCSRA = 0x00; // Disable ADC completely

	/*
	 * STEP 2: Configure ADC multiplexer
	 * - Use AVCC as reference voltage (most common for 5V systems)
	 * - Right-adjust result (ADLAR=0) for easy 10-bit reading
	 * - Select ADC0 as default channel
	 */
	ADMUX = ADC_AVCC_TYPE | 0x00; // AVCC reference, right-adjust, ADC0

	/*
	 * STEP 3: Disable analog comparator to save power
	 * The analog comparator shares some circuitry with ADC
	 * Disabling it reduces noise and power consumption
	 */
	ACSR = (1 << ACD); // Analog Comparator Disable

	/*
	 * STEP 4: Enable ADC with optimal prescaler
	 * For 16MHz system clock:
	 * - Prescaler 128 gives 125kHz ADC clock (optimal for accuracy)
	 * - Enable ADC but don't start conversion yet
	 */
	ADCSRA = (1 << ADEN) | ADC_PRESCALE_128;

	/*
	 * STEP 5: Perform dummy conversion to stabilize ADC
	 * First conversion after enabling ADC is often inaccurate
	 * Dummy conversion ensures accurate subsequent readings
	 */
	ADCSRA |= (1 << ADSC); // Start dummy conversion
	while (ADCSRA & (1 << ADSC))
		; // Wait for completion

	/*
	 * EDUCATIONAL NOTE:
	 * ADC is now ready for accurate conversions
	 * - Reference: AVCC (typically 5V)
	 * - Resolution: 10-bit (0-1023)
	 * - Clock: 125kHz (optimal for accuracy)
	 * - Default channel: ADC0
	 */
}

/*
 * EDUCATIONAL FUNCTION: Read ADC Data (Primary Function)
 *
 * PURPOSE: Perform single ADC conversion on specified channel
 * LEARNING: Shows complete ADC conversion sequence and timing
 *
 * PROCESS:
 * 1. Configure ADMUX for desired channel and reference
 * 2. Start conversion by setting ADSC bit
 * 3. Wait for conversion complete (ADSC clears automatically)
 * 4. Read 10-bit result from ADCL and ADCH registers
 *
 * PARAMETERS:
 * adc_input - ADC channel number (0-7 for ADC0-ADC7)
 *
 * RETURNS:
 * 10-bit ADC result (0-1023) representing voltage ratio
 *
 * VOLTAGE CALCULATION:
 * voltage = (adc_result / 1023.0) * reference_voltage
 * For AVCC=5V: voltage = (adc_result / 1023.0) * 5.0
 *
 * ASSEMBLY EQUIVALENT:
 * LDI R16, channel; STS ADMUX, R16      ; Select channel
 * LDI R16, (1<<ADSC); STS ADCSRA, R16   ; Start conversion
 * wait_loop: LDS R16, ADCSRA            ; Wait for completion
 *           SBRC R16, ADSC
 *           RJMP wait_loop
 * LDS R16, ADCL; LDS R17, ADCH          ; Read result
 */
unsigned int Read_Adc_Data(unsigned char adc_input)
{
	unsigned int adc_result_local = 0;

	/*
	 * STEP 1: Configure ADC channel and reference
	 * Preserve reference voltage setting while changing channel
	 * Use AVCC reference for most sensor applications
	 */
	ADMUX = (adc_input & 0x1F) | ADC_AVCC_TYPE; // Select channel, keep AVCC reference

	/*
	 * STEP 2: Ensure ADC is enabled with correct prescaler
	 * This step is usually not needed if Adc_init() was called
	 * But provides safety for standalone function usage
	 */
	ADCSRA = (1 << ADEN) | ADC_PRESCALE_128;

	/*
	 * STEP 3: Start ADC conversion
	 * Setting ADSC bit starts the conversion process
	 * Hardware will clear this bit when conversion is complete
	 */
	ADCSRA |= (1 << ADSC);

	/*
	 * STEP 4: Wait for conversion to complete
	 * ADSC bit remains high during conversion
	 * Typical conversion time: 13 ADC clock cycles
	 * At 125kHz ADC clock: 13/125000 = 104 microseconds
	 */
	while (ADCSRA & (1 << ADSC))
		;

	/*
	 * STEP 5: Read 10-bit conversion result
	 * ADCL must be read first, then ADCH
	 * This ensures atomic reading of the 10-bit value
	 * Result = ADCL + (ADCH << 8)
	 */
	adc_result_local = ADCL;		 // Read low byte first
	adc_result_local += (ADCH << 8); // Add high byte (shifted)

	/*
	 * STEP 6: Store result in global variable for educational access
	 */
	adc_result = adc_result_local;
	adc_channel = adc_input;

	/*
	 * EDUCATIONAL NOTE:
	 * The 10-bit result represents the ratio: Vin/Vref
	 * - 0 corresponds to 0V input
	 * - 1023 corresponds to reference voltage input
	 * - For AVCC=5V: each LSB = 5V/1024 = 4.88mV
	 */
	return adc_result_local;
}

/*
 * EDUCATIONAL FUNCTION: Read ADC with Averaging
 *
 * PURPOSE: Reduce noise by averaging multiple ADC readings
 * LEARNING: Shows noise reduction techniques and statistical processing
 */
unsigned int Read_Adc_Averaged(unsigned char adc_input, unsigned char num_samples)
{
	unsigned long sum = 0;
	unsigned char i;

	/* Take multiple samples and average them */
	for (i = 0; i < num_samples; i++)
	{
		sum += Read_Adc_Data(adc_input);
		_delay_us(100); // Small delay between samples
	}

	return (unsigned int)(sum / num_samples);
}

/*
 * EDUCATIONAL FUNCTION: Convert ADC to Voltage
 *
 * PURPOSE: Convert raw ADC value to actual voltage
 * LEARNING: Shows real-world sensor interfacing and unit conversion
 */
unsigned int Read_Adc_Voltage_mV(unsigned char adc_input)
{
	unsigned int adc_value = Read_Adc_Data(adc_input);

	/* Convert to millivolts assuming AVCC = 5000mV */
	/* Formula: voltage_mV = (adc_value * 5000) / 1024 */
	return (unsigned long)(adc_value * 5000UL) / 1024;
}

/*
 * EDUCATIONAL FUNCTION: Read Temperature Sensor
 *
 * PURPOSE: Demonstrate sensor calibration and linearization
 * LEARNING: Shows practical sensor interfacing
 *
 * NOTE: This assumes a linear temperature sensor like LM35
 * LM35: 10mV per degree Celsius, 0V at 0°C
 */
signed int Read_Temperature_Celsius(unsigned char adc_input)
{
	unsigned int voltage_mV = Read_Adc_Voltage_mV(adc_input);

	/* For LM35: Temperature = voltage_mV / 10 */
	/* Apply calibration offset if needed */
	signed int temperature = (signed int)(voltage_mV / 10) + adc_calibration_offset;

	return temperature;
}

/*
 * EDUCATIONAL FUNCTION: Read Light Sensor (CDS/LDR)
 *
 * PURPOSE: Demonstrate resistive sensor interfacing
 * LEARNING: Shows voltage divider calculations and sensor linearization
 *
 * CIRCUIT: VCC -- [10kΩ] -- ADC_PIN -- [CDS] -- GND
 * Light increases → CDS resistance decreases → Voltage increases
 */
unsigned int Read_Light_Level(unsigned char adc_input)
{
	unsigned int adc_value = Read_Adc_Averaged(adc_input, 8); // Average 8 samples

	/* Convert to percentage (0-100%) */
	/* Higher ADC value = more light */
	return (unsigned long)(adc_value * 100UL) / 1023;
}

/*
 * EDUCATIONAL FUNCTION: ADC Channel Scanning
 *
 * PURPOSE: Read multiple ADC channels sequentially
 * LEARNING: Shows multi-sensor data acquisition
 */
void Scan_Adc_Channels(unsigned int *results, unsigned char start_channel, unsigned char num_channels)
{
	unsigned char i;

	for (i = 0; i < num_channels; i++)
	{
		results[i] = Read_Adc_Data(start_channel + i);
		_delay_ms(1); // Small delay between channels
	}
}

/*
 * EDUCATIONAL FUNCTION: ADC Self-Test
 *
 * PURPOSE: Verify ADC functionality with known references
 * LEARNING: Shows testing and validation procedures
 */
unsigned char Adc_Self_Test(void)
{
	unsigned int test_results[3];

	/* Test with internal references */
	ADMUX = ADC_2_56_TYPE | 0x1E; // 2.56V reference, internal 1.1V
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC))
		;
	test_results[0] = ADCL + (ADCH << 8);

	/* Expected result should be around (1.1V / 2.56V) * 1023 ≈ 440 */
	if (test_results[0] < 400 || test_results[0] > 480)
	{
		return 0; // Self-test failed
	}

	return 1; // Self-test passed
}

/*
 * EDUCATIONAL FUNCTION: ADC Interrupt Handler
 *
 * PURPOSE: Demonstrate interrupt-driven ADC operation
 * LEARNING: Shows non-blocking ADC conversion
 */
volatile unsigned char adc_interrupt_complete = 0;

ISR(ADC_vect)
{
	/* Read conversion result */
	adc_result = ADCL + (ADCH << 8);
	adc_interrupt_complete = 1;
}

/*
 * EDUCATIONAL FUNCTION: Start ADC with Interrupt
 *
 * PURPOSE: Begin non-blocking ADC conversion
 * LEARNING: Shows interrupt-driven programming
 */
void Start_Adc_Interrupt(unsigned char adc_input)
{
	/* Configure channel */
	ADMUX = (adc_input & 0x1F) | ADC_AVCC_TYPE;

	/* Enable ADC interrupt */
	ADCSRA |= (1 << ADIE);

	/* Clear completion flag */
	adc_interrupt_complete = 0;

	/* Start conversion */
	ADCSRA |= (1 << ADSC);
}

/*
 * EDUCATIONAL FUNCTION: Check ADC Interrupt Status
 *
 * PURPOSE: Check if interrupt-driven conversion is complete
 * LEARNING: Shows non-blocking status checking
 */
unsigned char Is_Adc_Complete(void)
{
	return adc_interrupt_complete;
}

/*
 * =============================================================================
 * ENHANCED ADC FUNCTIONS - PROFESSIONAL SENSOR INTERFACE
 * =============================================================================
 */

/*
 * ADVANCED FUNCTION: Median Filter for Noise Reduction
 *
 * PURPOSE: Remove outliers and spikes using median filtering
 * LEARNING: Statistical noise rejection, robust measurement
 * ALGORITHM: Bubble sort to find median value
 */
unsigned int Read_Adc_Median(unsigned char adc_input, unsigned char num_samples)
{
	unsigned int samples[16]; // Support up to 16 samples
	unsigned int temp;
	unsigned char i, j;

	// Limit number of samples
	if (num_samples > 16)
		num_samples = 16;
	if (num_samples < 3)
		num_samples = 3;

	// Collect samples
	for (i = 0; i < num_samples; i++)
	{
		samples[i] = Read_Adc_Data(adc_input);
		_delay_us(100);
	}

	// Bubble sort to find median
	for (i = 0; i < num_samples - 1; i++)
	{
		for (j = 0; j < num_samples - i - 1; j++)
		{
			if (samples[j] > samples[j + 1])
			{
				temp = samples[j];
				samples[j] = samples[j + 1];
				samples[j + 1] = temp;
			}
		}
	}

	// Return median value (middle element)
	return samples[num_samples / 2];
}

/*
 * ADVANCED FUNCTION: Moving Average Filter
 *
 * PURPOSE: Smooth sensor readings with exponential moving average
 * LEARNING: Digital filter implementation, real-time smoothing
 */
#define MOVING_AVG_SIZE 8
static unsigned int moving_avg_buffer[MOVING_AVG_SIZE];
static unsigned char moving_avg_index = 0;
static unsigned char moving_avg_filled = 0;

unsigned int Read_Adc_Moving_Average(unsigned char adc_input)
{
	unsigned long sum = 0;
	unsigned char i, count;

	// Add new sample to buffer
	moving_avg_buffer[moving_avg_index] = Read_Adc_Data(adc_input);
	moving_avg_index = (moving_avg_index + 1) % MOVING_AVG_SIZE;

	if (moving_avg_index == 0)
		moving_avg_filled = 1;

	// Calculate average
	count = moving_avg_filled ? MOVING_AVG_SIZE : moving_avg_index;
	for (i = 0; i < count; i++)
	{
		sum += moving_avg_buffer[i];
	}

	return (unsigned int)(sum / count);
}

void Reset_Moving_Average(void)
{
	moving_avg_index = 0;
	moving_avg_filled = 0;
}

/*
 * ADVANCED FUNCTION: ADC Statistics Collection
 *
 * PURPOSE: Collect comprehensive statistics for sensor analysis
 * LEARNING: Data analysis, min/max tracking, statistical processing
 */
void ADC_Init_Statistics(ADC_Statistics *stats)
{
	stats->min_value = 1023;
	stats->max_value = 0;
	stats->current_value = 0;
	stats->sum = 0;
	stats->count = 0;
	stats->average = 0;
}

void ADC_Update_Statistics(ADC_Statistics *stats, unsigned int new_value)
{
	stats->current_value = new_value;

	// Update min/max
	if (new_value < stats->min_value)
		stats->min_value = new_value;
	if (new_value > stats->max_value)
		stats->max_value = new_value;

	// Update running average
	stats->sum += new_value;
	stats->count++;
	stats->average = (unsigned int)(stats->sum / stats->count);
}

void ADC_Get_Statistics(unsigned char adc_input, ADC_Statistics *stats, unsigned char num_samples)
{
	unsigned char i;

	ADC_Init_Statistics(stats);

	for (i = 0; i < num_samples; i++)
	{
		unsigned int value = Read_Adc_Data(adc_input);
		ADC_Update_Statistics(stats, value);
		_delay_ms(1);
	}
}

/*
 * ADVANCED FUNCTION: Threshold Detection
 *
 * PURPOSE: Detect when sensor values cross thresholds
 * LEARNING: Event detection, hysteresis, state machines
 * APPLICATIONS: Alarm systems, limit switches, range detection
 */
void ADC_Set_Threshold(ADC_Threshold *threshold, unsigned int low, unsigned int high)
{
	threshold->low_threshold = low;
	threshold->high_threshold = high;
	threshold->state = 0;
	threshold->event_occurred = 0;
}

unsigned char ADC_Check_Threshold(ADC_Threshold *threshold, unsigned int adc_value)
{
	unsigned char new_state = threshold->state;

	// Check for threshold crossings with hysteresis
	if (adc_value > threshold->high_threshold)
	{
		new_state = 1; // Above upper threshold
	}
	else if (adc_value < threshold->low_threshold)
	{
		new_state = 0; // Below lower threshold
	}
	// Stay in current state if between thresholds (hysteresis)

	// Detect state change
	if (new_state != threshold->state)
	{
		threshold->event_occurred = 1;
		threshold->state = new_state;
		return 1; // Threshold crossed
	}

	return 0; // No threshold crossing
}

unsigned char ADC_Read_With_Threshold(unsigned char adc_input, ADC_Threshold *threshold)
{
	unsigned int value = Read_Adc_Data(adc_input);
	return ADC_Check_Threshold(threshold, value);
}

/*
 * ADVANCED FUNCTION: Multi-Point Calibration
 *
 * PURPOSE: Linear interpolation between calibration points
 * LEARNING: Sensor linearization, lookup tables, interpolation
 * APPLICATIONS: Non-linear sensors, temperature compensation
 */
void ADC_Add_Calibration_Point(ADC_Calibration *cal, unsigned int adc_val, unsigned int real_val)
{
	if (cal->num_points < 10)
	{
		cal->adc_points[cal->num_points] = adc_val;
		cal->real_values[cal->num_points] = real_val;
		cal->num_points++;
	}
}

unsigned int ADC_Apply_Calibration(ADC_Calibration *cal, unsigned int adc_value)
{
	unsigned char i;

	if (cal->num_points == 0)
		return adc_value;

	// Find calibration points bracketing the ADC value
	for (i = 0; i < cal->num_points - 1; i++)
	{
		if (adc_value >= cal->adc_points[i] && adc_value <= cal->adc_points[i + 1])
		{
			// Linear interpolation
			unsigned int adc_span = cal->adc_points[i + 1] - cal->adc_points[i];
			unsigned int real_span = cal->real_values[i + 1] - cal->real_values[i];
			unsigned int adc_offset = adc_value - cal->adc_points[i];

			return cal->real_values[i] + ((unsigned long)adc_offset * real_span) / adc_span;
		}
	}

	// Outside calibration range - use nearest point
	if (adc_value < cal->adc_points[0])
		return cal->real_values[0];
	else
		return cal->real_values[cal->num_points - 1];
}

/*
 * ADVANCED FUNCTION: Data Logging Buffer
 *
 * PURPOSE: Circular buffer for continuous data logging
 * LEARNING: Ring buffer implementation, data streaming
 * APPLICATIONS: Waveform capture, trend analysis, data recording
 */
void ADC_Logger_Init(ADC_Logger *logger, unsigned char channel)
{
	logger->head = 0;
	logger->tail = 0;
	logger->count = 0;
	logger->channel = channel;
}

void ADC_Logger_Add_Sample(ADC_Logger *logger, unsigned int sample)
{
	logger->buffer[logger->head] = sample;
	logger->head = (logger->head + 1) % ADC_LOG_BUFFER_SIZE;

	if (logger->count < ADC_LOG_BUFFER_SIZE)
	{
		logger->count++;
	}
	else
	{
		// Buffer full - overwrite oldest data
		logger->tail = (logger->tail + 1) % ADC_LOG_BUFFER_SIZE;
	}
}

unsigned char ADC_Logger_Get_Sample(ADC_Logger *logger, unsigned int *sample)
{
	if (logger->count == 0)
		return 0; // Buffer empty

	*sample = logger->buffer[logger->tail];
	logger->tail = (logger->tail + 1) % ADC_LOG_BUFFER_SIZE;
	logger->count--;

	return 1; // Sample retrieved
}

unsigned char ADC_Logger_Is_Full(ADC_Logger *logger)
{
	return (logger->count >= ADC_LOG_BUFFER_SIZE);
}

void ADC_Logger_Clear(ADC_Logger *logger)
{
	logger->head = 0;
	logger->tail = 0;
	logger->count = 0;
}

/*
 * ADVANCED FUNCTION: Differential ADC Reading
 *
 * PURPOSE: Measure voltage difference between two inputs
 * LEARNING: Differential measurement, common-mode rejection
 */
signed int Read_Adc_Differential(unsigned char positive_input, unsigned char negative_input)
{
	unsigned int pos_value = Read_Adc_Averaged(positive_input, 4);
	unsigned int neg_value = Read_Adc_Averaged(negative_input, 4);

	return (signed int)pos_value - (signed int)neg_value;
}

/*
 * ADVANCED FUNCTION: Ratiometric Measurement
 *
 * PURPOSE: Measure ratio independent of supply voltage
 * LEARNING: Ratiometric sensors, supply voltage compensation
 */
unsigned int Read_Adc_Ratiometric(unsigned char signal_input, unsigned char reference_input)
{
	unsigned int signal = Read_Adc_Averaged(signal_input, 4);
	unsigned int reference = Read_Adc_Averaged(reference_input, 4);

	if (reference == 0)
		return 0;

	// Return ratio as percentage (0-100%)
	return (unsigned long)(signal * 100UL) / reference;
}

/*
 * ADVANCED FUNCTION: Auto-Ranging
 *
 * PURPOSE: Automatically adjust gain for optimal resolution
 * LEARNING: Automatic gain control, dynamic range extension
 */
void ADC_AutoRange_Init(ADC_AutoRange *ar)
{
	ar->current_gain = 0;
	ar->scaled_value = 0;
	ar->overrange = 0;
	ar->underrange = 0;
}

unsigned int ADC_Read_AutoRange(unsigned char adc_input, ADC_AutoRange *ar)
{
	unsigned int raw_value = Read_Adc_Data(adc_input);

	// Check for overrange (>90% of full scale)
	if (raw_value > 921)
	{
		ar->overrange = 1;
		ar->underrange = 0;
	}
	// Check for underrange (<10% of full scale)
	else if (raw_value < 102)
	{
		ar->overrange = 0;
		ar->underrange = 1;
	}
	else
	{
		ar->overrange = 0;
		ar->underrange = 0;
	}

	// Scale value based on current gain
	ar->scaled_value = raw_value << ar->current_gain;

	return ar->scaled_value;
}

/*
 * ADVANCED FUNCTION: Fast Burst Sampling
 *
 * PURPOSE: Capture waveforms at maximum ADC speed
 * LEARNING: High-speed acquisition, waveform capture
 */
void ADC_Fast_Sample_Array(unsigned char adc_input, unsigned int *buffer, unsigned char num_samples)
{
	unsigned char i;

	// Configure channel
	ADMUX = (adc_input & 0x1F) | ADC_AVCC_TYPE;

	for (i = 0; i < num_samples; i++)
	{
		// Start conversion
		ADCSRA |= (1 << ADSC);

		// Wait for completion (no delay for maximum speed)
		while (ADCSRA & (1 << ADSC))
			;

		// Read result
		buffer[i] = ADCL + (ADCH << 8);
	}
}

unsigned int ADC_Get_Sample_Rate_Hz(void)
{
	// ADC clock = F_CPU / prescaler
	// Conversion time = 13 ADC clock cycles
	// For F_CPU=7372800Hz, prescaler=128: 7372800/128/13 ≈ 4434 Hz
	return (F_CPU / 128UL / 13UL);
}

/*
 * ADVANCED FUNCTION: Calibrated Temperature Reading
 *
 * PURPOSE: Temperature measurement with user calibration
 * LEARNING: Sensor calibration, offset and scale correction
 */
signed int Read_Temperature_Calibrated(unsigned char adc_input, signed int offset, unsigned int scale)
{
	unsigned int voltage_mV = Read_Adc_Voltage_mV(adc_input);
	signed int temperature = (signed int)(voltage_mV / 10);

	// Apply calibration: temp_calibrated = (temp * scale / 1000) + offset
	temperature = (signed int)(((long)temperature * scale) / 1000) + offset;

	return temperature;
}

float Read_Temperature_Float(unsigned char adc_input)
{
	unsigned int voltage_mV = Read_Adc_Voltage_mV(adc_input);
	return (float)voltage_mV / 10.0f;
}

/*
 * ADVANCED FUNCTION: Voltage Reference Management
 *
 * PURPOSE: Switch between different voltage references
 * LEARNING: Reference voltage selection, measurement accuracy
 */
void ADC_Set_Reference(unsigned char ref_type)
{
	// Preserve channel selection while changing reference
	unsigned char channel = ADMUX & 0x1F;
	ADMUX = ref_type | channel;

	// Wait for reference to stabilize
	_delay_ms(10);

	// Dummy conversion
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC))
		;
}

/*
 * ADVANCED FUNCTION: Measure VCC Using Internal Reference
 *
 * PURPOSE: Measure supply voltage without external reference
 * LEARNING: Self-measurement, bandgap reference
 */
unsigned int ADC_Measure_VCC_mV(void)
{
	// Select internal 2.56V reference and channel 0x1E (1.1V bandgap)
	ADMUX = ADC_2_56_TYPE | 0x1E;
	_delay_ms(10); // Wait for reference to stabilize

	// Perform conversion
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC))
		;
	unsigned int result = ADCL + (ADCH << 8);

	// VCC = (1.1V * 1024) / ADC_result * (2.56V / Vref)
	// Simplified: VCC = 1126400 / result (in mV)
	if (result == 0)
		return 0;
	return (unsigned int)(1126400UL / result);
}

unsigned int ADC_Measure_Internal_Ref(void)
{
	// Measure internal 1.1V reference
	ADMUX = ADC_AVCC_TYPE | 0x1E; // AVCC reference, internal 1.1V
	_delay_ms(10);

	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC))
		;

	return ADCL + (ADCH << 8);
}

#endif // !ASSEMBLY_BLINK_BASIC
