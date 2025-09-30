#include "config.h"

// Only compile this file if any MOTORS demo is enabled
#ifdef MOTORS_FULLSTEP
    #define MOTORS_DEMO_ENABLED
#endif
#ifdef MOTORS_FULLSTEP_INTERRUPT
    #define MOTORS_DEMO_ENABLED
#endif
#ifdef MOTORS_HALFSTEP
    #define MOTORS_DEMO_ENABLED
#endif
#ifdef MOTORS_HALFSTEP_INTERRUPT
    #define MOTORS_DEMO_ENABLED
#endif
#ifdef MOTORS_STEPPER_DEMO
    #define MOTORS_DEMO_ENABLED
#endif
#ifdef MOTORS_SERVO
    #define MOTORS_DEMO_ENABLED
#endif
#ifdef MOTORS_SERVO_ADC
    #define MOTORS_DEMO_ENABLED
#endif
#ifdef MOTORS_SERVO_UART
    #define MOTORS_DEMO_ENABLED
#endif
#ifdef MOTORS_PWM_FAST
    #define MOTORS_DEMO_ENABLED
#endif
#ifdef MOTORS_PWM_PHASECORRECT
    #define MOTORS_DEMO_ENABLED
#endif

#ifdef MOTORS_DEMO_ENABLED

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>

//////////////////////////////////////////////////////////////
// Stepping Motors

#ifdef MOTORS_FULLSTEP
void main_motors_fullstep(void)
{
	DDRD = 0x00; // PORTD as input
	DDRB = 0xFF; // PORTB as output

	#define DELAY_MS 100
	#define ANGLE 100

	const uint8_t step_sequence[] = {0x66, 0xCC, 0x99, 0x33};
	const int num_steps = sizeof(step_sequence) / sizeof(step_sequence[0]);

	int current_step = 0, step_count = 0;

	while (1)
	{
		if ((PIND & (1 << PD0)) == 0)
		{
			PORTB = step_sequence[current_step];
			_delay_ms(DELAY_MS);
			current_step = (current_step + 1) % num_steps;
			step_count++;
		}
		else
		{
			current_step = (current_step == 0) ? num_steps - 1 : current_step - 1;
			PORTB = step_sequence[current_step];
			_delay_ms(DELAY_MS);
			step_count++;
		}

		if (step_count == ANGLE) while (1);
	}
}
#endif

#ifdef MOTORS_HALFSTEP
void main_motors_halfstep(void)
{
	DDRD = 0x00; // PORTD as input
	DDRB = 0xFF; // PORTB as output

	#define DELAY_MS 100
	#define ANGLE 100

	const uint8_t step_sequence[] = {0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09};
	const int num_steps = sizeof(step_sequence) / sizeof(step_sequence[0]);

	int current_step = 0, step_count = 0;

	while (1)
	{
		if ((PIND & (1 << PD0)) == 0)
		{
			PORTB = step_sequence[current_step];
			_delay_ms(DELAY_MS);
			current_step = (current_step + 1) % num_steps;
			step_count++;
		}
		else
		{
			current_step = (current_step == 0) ? num_steps - 1 : current_step - 1;
			PORTB = step_sequence[current_step];
			_delay_ms(DELAY_MS);
			step_count++;
		}

		if (step_count == ANGLE) while (1);
	}
}
#endif

#ifdef MOTORS_FULLSTEP_INTERRUPT
volatile uint8_t direction = 0;

void main_motors_fullstep_interrupt(void)
{
	DDRD &= ~(1 << DDD2); // PD2 as input
	DDRB = 0xFF;          // PORTB as output

	#define DELAY_MS 100
	#define ANGLE 100

	const uint8_t step_sequence[] = {0x66, 0xCC, 0x99, 0x33};
	const int num_steps = sizeof(step_sequence) / sizeof(step_sequence[0]);

	int current_step = 0, step_count = 0;

	EICRA = (1 << ISC01);
	EIMSK = (1 << INT0);
	sei();

	while (1)
	{
		PORTB = step_sequence[current_step];
		_delay_ms(DELAY_MS);
		current_step = (direction == 0) ? (current_step + 1) % num_steps : (current_step == 0) ? num_steps - 1 : current_step - 1;
		step_count++;
		if (step_count == ANGLE) while (1);
	}
}

ISR(INT0_vect)
{
	direction ^= 1;
}
#endif

#ifdef MOTORS_HALFSTEP_INTERRUPT
volatile uint8_t direction = 0;

void main_motors_halfstep_interrupt(void)
{
	DDRD &= ~(1 << DDD2);
	DDRB = 0xFF;

	#define DELAY_MS 100
	#define ANGLE 100

	const uint8_t step_sequence[] = {0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09};
	const int num_steps = sizeof(step_sequence) / sizeof(step_sequence[0]);

	int current_step = 0, step_count = 0;

	EICRA = (1 << ISC01);
	EIMSK = (1 << INT0);
	sei();

	while (1)
	{
		PORTB = step_sequence[current_step];
		_delay_ms(DELAY_MS);
		current_step = (direction == 0) ? (current_step + 1) % num_steps : (current_step == 0) ? num_steps - 1 : current_step - 1;
		step_count++;
		if (step_count == ANGLE) while (1);
	}
}

ISR(INT0_vect)
{
	direction ^= 1;
}
#endif

#ifdef MOTORS_STEPPER_DEMO
void delay_ms(uint16_t ms)
{
	for (uint16_t i = 0; i < ms; i++) _delay_ms(1);
}

void execute_pattern(const uint8_t* sequence, int num_steps, int cycles, uint16_t delay, int direction)
{
	int step_index = (direction == 0) ? 0 : num_steps - 1;

	for (int cycle = 0; cycle < cycles; cycle++)
	{
		for (int i = 0; i < num_steps; i++)
		{
			PORTB = sequence[step_index];
			delay_ms(delay);
			step_index = (direction == 0) ? (step_index + 1) % num_steps : (step_index - 1 + num_steps) % num_steps;
		}
	}
}

void main_motors_stepper_demo(void)
{
	DDRB = 0xFF;

	const uint8_t full_step_sequence[] = {0x66, 0xCC, 0x99, 0x33};
	const uint8_t half_step_sequence[] = {0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09};
	const int num_full_steps = sizeof(full_step_sequence) / sizeof(full_step_sequence[0]);
	const int num_half_steps = sizeof(half_step_sequence) / sizeof(half_step_sequence[0]);

	while (1)
	{
		execute_pattern(full_step_sequence, num_full_steps, 3, 200, 0);
		delay_ms(1000);
		execute_pattern(full_step_sequence, num_full_steps, 3, 100, 1);
		execute_pattern(half_step_sequence, num_half_steps, 5, 300, 0);

		for (int i = 0; i < 10; i++)
		{
			PORTB = half_step_sequence[0];
			delay_ms(200);
			PORTB = half_step_sequence[4];
			delay_ms(200);
		}

		for (uint16_t delay = 300; delay >= 50; delay -= 50)
		{
			execute_pattern(half_step_sequence, num_half_steps, 1, delay, 0);
		}

		for (uint16_t delay = 50; delay <= 300; delay += 50)
		{
			execute_pattern(half_step_sequence, num_half_steps, 1, delay, 1);
		}

		delay_ms(2000);
	}
}
#endif

//////////////////////////////////////////////////////
// DC Motors
/*
-----------------------------------------------
| ATmega128 PWM Control Register Table        |
-----------------------------------------------
| Register  | Bits            | Description                                      |
|-----------|------------------|-------------------------------------------------|
| DDRB      | PB3 (OC0 pin)   | Sets PB3 as output for PWM signal.               |
| TCCR0     | COM01, COM00    | Compare Match Output Mode:                      |
|           |                 | - 00: Normal operation.                         |
|           |                 | - 10: Non-inverting PWM mode (for motors).      |
|           | WGM01, WGM00    | Waveform Generation Mode:                       |
|           |                 | - 00: Normal mode.                              |
|           |                 | - 01: Phase Correct PWM.                        |
|           |                 | - 11: Fast PWM.                                 |
|           | CS02, CS01, CS00| Clock Select (Prescaler):                       |
|           |                 | - 001: No prescaler.                            |
|           |                 | - 010: Prescaler of 8.                          |
|           |                 | - 011: Prescaler of 64.                         |
| OCR0      | 8-bit register  | Sets PWM duty cycle (0-255).                    |
-----------------------------------------------
*/

/* Includes */
#include <avr/io.h>
#include <util/delay.h>

#ifdef MOTORS_PWM_FAST
void main_motors_pwm_fast(void)
{
	// Set PB4 (OC0) as output
	DDRB |= (1 << PB4);

	// Configure Timer0 for Fast PWM, Non-inverting mode, Prescaler = 64
	TCCR0 = (1 << COM01) | (1 << WGM01) | (1 << WGM00) | (1 << CS01) | (1 << CS00);

	while (1)
	{
		// Increment duty cycle from 0 to 255
		for (uint8_t duty_cycle = 0; duty_cycle < 255; duty_cycle++)
		{
			OCR0 = duty_cycle; // Set duty cycle
			_delay_ms(10);    // Add delay for visible change
		}

		// Decrement duty cycle from 255 to 0
		for (uint8_t duty_cycle = 255; duty_cycle > 0; duty_cycle--)
		{
			OCR0 = duty_cycle; // Set duty cycle
			_delay_ms(10);    // Add delay for visible change
		}
	}
}
#endif

#ifdef MOTORS_PWM_PHASECORRECT
void main_motors_pwm_phasecorrect(void)
{
	// Set PB4 (OC0) as output
	DDRB |= (1 << PB4);

	// Configure Timer0 for Phase Correct PWM, Non-inverting mode, Prescaler = 64
	TCCR0 = (1 << COM01) | (1 << WGM00) | (1 << CS01) | (1 << CS00);

	while (1)
	{
		// Increment duty cycle from 0 to 255
		for (uint8_t duty_cycle = 0; duty_cycle < 255; duty_cycle++)
		{
			OCR0 = duty_cycle; // Set duty cycle
			_delay_ms(10);    // Add delay for visible change
		}

		// Decrement duty cycle from 255 to 0
		for (uint8_t duty_cycle = 255; duty_cycle > 0; duty_cycle--)
		{
			OCR0 = duty_cycle; // Set duty cycle
			_delay_ms(10);    // Add delay for visible change
		}
	}
}
#endif


//////////////////////////////////////////////////////
// Servo Motors
/*
--------------------------------------------------------------
| ATmega128 Servo Motor Control Register Table              |
--------------------------------------------------------------
| Register  | Bits            | Description                                      |
|-----------|------------------|-------------------------------------------------|
| DDRB      | PB5 (OC1A pin)  | Sets PB5 as output for PWM signal.               |
| ICR1      | 16-bit register | Sets the TOP value for Timer1, determining PWM frequency. |
| TCCR1A    | COM1A1          | Non-inverting PWM output on OC1A.                |
|           | WGM11           | Selects Fast PWM mode with ICR1 as TOP.          |
| TCCR1B    | WGM12, WGM13    | Selects Fast PWM mode with ICR1 as TOP.          |
|           | CS11            | Prescaler of 8 for Timer1.                       |
| OCR1A     | 16-bit register | Sets duty cycle (pulse width) for OC1A (servo control). |
--------------------------------------------------------------
*/

#ifdef MOTORS_SERVO
#include <avr/io.h>
#include <util/delay.h>

#define PWM_FREQUENCY 50  // Servo PWM frequency in Hz (50Hz)
#define PRESCALER 8       // Prescaler for Timer1

// Initialize PWM for Servo Motor
void pwm_init()
{
	// Set PB1 (OC1A) as output
	DDRB |= (1 << PB5);

	// Set TOP value for Timer1 to generate 50Hz PWM
	ICR1 = F_CPU / (PRESCALER * PWM_FREQUENCY) - 1;

	// Configure Timer1 for Fast PWM, non-inverting mode
	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS11);
}

// Write angle to Servo Motor
void servo_write(uint8_t angle)
{
	uint16_t min_pulse = ICR1 / 20;  // Minimum pulse width (1ms)
	uint16_t max_pulse = ICR1 / 10; // Maximum pulse width (2ms)
	uint16_t pulse_width = min_pulse + ((max_pulse - min_pulse) * angle) / 180;
	OCR1A = pulse_width;            // Set pulse width for the given angle
}

// Main function to demonstrate Servo Motor control
void main_motors_servo(void)
{
	pwm_init();  // Initialize PWM

	while (1)
	{
		// Sweep servo from 0 to 180 degrees
		for (uint8_t angle = 0; angle <= 180; angle += 10)
		{
			servo_write(angle);
			_delay_ms(500);  // Wait for the servo to reach the position
		}

		// Sweep servo back from 180 to 0 degrees
		for (uint8_t angle = 180; angle >= 0; angle -= 10)
		{
			servo_write(angle);
			_delay_ms(500);  // Wait for the servo to reach the position
		}
	}
}
#endif

#endif // MOTORS demos wrapper
