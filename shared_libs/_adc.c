
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
#include <avr/io.h>
#include <avr/interrupt.h>
#ifndef F_CPU
#define F_CPU 7372800UL
#endif
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
 * ENHANCED ADC FUNCTIONS - Professional Quality Additions
 * =============================================================================
 */

/* Internal state variables for enhanced features */
static unsigned char adc_precision_bits = 10; // Default 10-bit precision
static unsigned char adc_reference_type = 1;  // Default AVCC reference
static unsigned char adc_free_running = 0;	  // Free-running mode flag

/*
 * ADC CALIBRATION FUNCTION
 *
 * PURPOSE: Calibrate ADC using known reference voltage
 * LEARNING: Shows calibration techniques for improved accuracy
 *
 * CALIBRATION PROCEDURE:
 * 1. Apply known voltage (e.g., from precision power supply)
 * 2. Read ADC value
 * 3. Calculate calibration factors
 * 4. Apply factors to future readings
 */
void ADC_calibrate(unsigned int reference_voltage_mV)
{
	/* Read current ADC value (assume channel 0 for calibration) */
	unsigned int raw_value = Read_Adc(ADC_CHANNEL_0);

	/* Calculate expected value based on current reference voltage */
	unsigned int expected_value;
	if (adc_reference_type == 1)
	{
		/* AVCC reference (5000mV) */
		expected_value = (reference_voltage_mV * 1023UL) / 5000UL;
	}
	else if (adc_reference_type == 2)
	{
		/* Internal 2.56V reference */
		expected_value = (reference_voltage_mV * 1023UL) / 2560UL;
	}
	else
	{
		/* AREF - cannot auto-calibrate, return */
		return;
	}

	/* Calculate calibration offset and scale */
	if (raw_value > 0)
	{
		/* Offset = difference between expected and actual */
		adc_calibration_offset = (raw_value > expected_value) ? (raw_value - expected_value) : (expected_value - raw_value);

		/* Scale factor = (expected / actual) * 1024 */
		adc_calibration_scale = (expected_value * 1024UL) / raw_value;
	}
}

/*
 * ADC PRECISION CONTROL
 *
 * PURPOSE: Set 8-bit or 10-bit resolution
 * LEARNING: Shows resolution vs speed tradeoff
 *
 * IMPLEMENTATION:
 * - 10-bit: Right-adjusted result (ADLAR=0), read ADCL+ADCH
 * - 8-bit:  Left-adjusted result (ADLAR=1), read ADCH only
 */
void ADC_set_precision(unsigned char bits)
{
	if (bits == 8)
	{
		/* Left-adjust for 8-bit precision */
		ADMUX |= (1 << ADLAR);
		adc_precision_bits = 8;
	}
	else
	{
		/* Right-adjust for 10-bit precision */
		ADMUX &= ~(1 << ADLAR);
		adc_precision_bits = 10;
	}
}

unsigned char ADC_get_precision(void)
{
	return adc_precision_bits;
}

/*
 * FREE-RUNNING (CONTINUOUS) MODE
 *
 * PURPOSE: Continuous ADC conversion without manual triggering
 * LEARNING: Shows auto-triggering and continuous data acquisition
 *
 * OPERATION:
 * - ADC starts new conversion automatically after each completion
 * - Much faster than polling mode
 * - Useful for data logging and signal monitoring
 */
void ADC_start_free_running(unsigned char channel)
{
	/* Select channel */
	ADMUX = (ADMUX & 0xE0) | (channel & 0x1F);

#ifdef __AVR_ATmega128__
	/* ATmega128 uses ADFR (ADC Free Running) bit in ADCSRA */
	ADCSRA |= (1 << ADFR);
#else
	/* ATmega328P/644P use ADATE (Auto Trigger Enable) */
	ADCSRA |= (1 << ADATE);

	/* Set free-run mode (ADTS2:0 = 000 in ADCSRB) */
	ADCSRB &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));
#endif

	/* Enable ADC interrupt */
	ADCSRA |= (1 << ADIE);

	/* Start first conversion */
	ADCSRA |= (1 << ADSC);

	adc_free_running = 1;
}

void ADC_stop_free_running(void)
{
#ifdef __AVR_ATmega128__
	/* ATmega128: Disable ADFR */
	ADCSRA &= ~(1 << ADFR);
#else
	/* ATmega328P/644P: Disable ADATE */
	ADCSRA &= ~(1 << ADATE);
#endif

	/* Disable interrupt */
	ADCSRA &= ~(1 << ADIE);

	adc_free_running = 0;
}

unsigned int ADC_get_last_result(void)
{
	return adc_result; // Global variable updated by ISR
}

/*
 * DIFFERENTIAL ADC READING
 *
 * PURPOSE: Measure voltage difference between two channels
 * LEARNING: Shows differential measurement techniques
 *
 * DIFFERENTIAL CHANNELS (ATmega128):
 * - ADC0-ADC1 with gain 1x, 10x, 200x
 * - ADC2-ADC3 with gain 1x, 10x, 200x
 * - Result is signed: positive if pos > neg, negative otherwise
 */
int ADC_read_differential(unsigned char pos_channel, unsigned char neg_channel, unsigned char gain)
{
	unsigned char mux_setting = 0;

	/* Determine MUX setting for differential input */
	/* Simplified implementation - supports ADC0-ADC1 and ADC2-ADC3 */
	if (pos_channel == 0 && neg_channel == 1)
	{
		if (gain == 1)
			mux_setting = 0x10; // ADC0-ADC1, gain 1x
		else if (gain == 10)
			mux_setting = 0x08; // ADC0-ADC1, gain 10x
		else if (gain == 200)
			mux_setting = 0x09; // ADC0-ADC1, gain 200x
		else
			mux_setting = 0x10; // Default to gain 1x
	}
	else if (pos_channel == 2 && neg_channel == 3)
	{
		if (gain == 1)
			mux_setting = 0x11; // ADC2-ADC3, gain 1x
		else if (gain == 10)
			mux_setting = 0x0A; // ADC2-ADC3, gain 10x
		else if (gain == 200)
			mux_setting = 0x0B; // ADC2-ADC3, gain 200x
		else
			mux_setting = 0x11; // Default to gain 1x
	}
	else
	{
		/* Unsupported channel combination */
		return 0;
	}

	/* Configure for differential mode */
	ADMUX = (ADMUX & 0xE0) | mux_setting;

	/* Start conversion */
	ADCSRA |= (1 << ADSC);

	/* Wait for completion */
	while (ADCSRA & (1 << ADSC))
		;

	/* Read result (10-bit signed) */
	int result = ADCL;
	result |= (ADCH << 8);

	/* Convert to signed value (-512 to +511) */
	if (result & 0x0200)
	{
		/* Negative result - extend sign */
		result |= 0xFC00;
	}

	return result;
}

/*
 * VOLTAGE REFERENCE MANAGEMENT
 *
 * PURPOSE: Configure and detect voltage reference
 * LEARNING: Shows reference voltage selection importance
 */
void ADC_set_reference(unsigned char reference)
{
	/* Clear reference bits */
	ADMUX &= ~((1 << REFS1) | (1 << REFS0));

	switch (reference)
	{
	case 0: /* AREF, internal Vref turned off */
		/* REFS1:0 = 00 - already cleared */
		adc_reference_type = 0;
		break;
	case 1: /* AVCC reference */
		ADMUX |= (1 << REFS0);
		adc_reference_type = 1;
		break;
	case 2: /* Internal 2.56V reference */
		ADMUX |= (1 << REFS1) | (1 << REFS0);
		adc_reference_type = 2;
		break;
	}

	/* Wait for reference to stabilize */
	_delay_ms(10);
}

unsigned char ADC_get_reference(void)
{
	return adc_reference_type;
}

unsigned int ADC_set_reference_auto(void)
{
	/* This is a simplified implementation */
	/* Actual implementation would measure internal bandgap reference */
	/* For now, return the configured reference voltage */

	if (adc_reference_type == 1)
	{
		return 5000; /* AVCC = 5V */
	}
	else if (adc_reference_type == 2)
	{
		return 2560; /* Internal = 2.56V */
	}

	return 0; /* AREF - unknown */
}

/*
 * CALIBRATION MANAGEMENT
 */
void ADC_reset_calibration(void)
{
	adc_calibration_offset = 0;
	adc_calibration_scale = 1024;
}

/*
 * SAMPLE RATE CALCULATION
 *
 * PURPOSE: Calculate ADC sampling rate
 * LEARNING: Shows timing analysis
 *
 * CALCULATION:
 * ADC clock = F_CPU / prescaler
 * Conversion time = 13 ADC clock cycles (first conversion: 25 cycles)
 * Sample rate = ADC clock / 13
 */
unsigned int ADC_get_sample_rate_Hz(void)
{
	/* Assume prescaler = 128 (typical) */
	unsigned long adc_clock = F_CPU / 128;

	/* Conversion time = 13 ADC clock cycles */
	unsigned int sample_rate = adc_clock / 13;

	return sample_rate;
}

#endif // !ASSEMBLY_BLINK_BASIC
