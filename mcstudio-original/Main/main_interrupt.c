#include "config.h"

// Interrupt Vectors for ATmega128 with Descriptions
/*
ISR(INT0_vect)           // External Interrupt Request 0 - triggered by an event on PD2
ISR(INT1_vect)           // External Interrupt Request 1 - triggered by an event on PD3
ISR(INT2_vect)           // External Interrupt Request 2 - triggered by an event on PB2
ISR(INT3_vect)           // External Interrupt Request 3 - triggered by an event on PE4
ISR(INT4_vect)           // External Interrupt Request 4 - triggered by an event on PE5
ISR(INT5_vect)           // External Interrupt Request 5 - triggered by an event on PE6
ISR(INT6_vect)           // External Interrupt Request 6 - triggered by an event on PE7
ISR(INT7_vect)           // External Interrupt Request 7 - triggered by an event on PE3

ISR(TIMER2_COMP_vect)    // Timer/Counter2 Compare Match - triggers when Timer2 reaches OCR2 value
ISR(TIMER2_OVF_vect)     // Timer/Counter2 Overflow - triggers when Timer2 overflows from 0xFF to 0x00
ISR(TIMER1_CAPT_vect)    // Timer/Counter1 Capture Event - triggered by input capture on ICP1 pin
ISR(TIMER1_COMPA_vect)   // Timer/Counter1 Compare Match A - triggered by a match with OCR1A
ISR(TIMER1_COMPB_vect)   // Timer/Counter1 Compare Match B - triggered by a match with OCR1B
ISR(TIMER1_OVF_vect)     // Timer/Counter1 Overflow - triggered when Timer1 overflows from 0xFFFF to 0x0000
ISR(TIMER0_COMP_vect)    // Timer/Counter0 Compare Match - triggers when Timer0 reaches OCR0 value
ISR(TIMER0_OVF_vect)     // Timer/Counter0 Overflow - triggered when Timer0 overflows from 0xFF to 0x00

ISR(SPI_STC_vect)        // SPI Serial Transfer Complete - triggered when an SPI transfer is completed
ISR(USART0_RX_vect)      // USART0 Rx Complete - triggered when USART0 completes receiving a byte
ISR(USART0_UDRE_vect)    // USART0 Data Register Empty - triggered when USART0's data register is empty
ISR(USART0_TX_vect)      // USART0 Tx Complete - triggered when USART0 completes transmitting a byte
ISR(ANALOG_COMP_vect)    // Analog Comparator - triggered when the comparator detects a change
ISR(ADC_vect)            // ADC Conversion Complete - triggered when an ADC conversion is finished
ISR(EE_READY_vect)       // EEPROM Ready - triggered when EEPROM write completes
ISR(TWI_vect)            // 2-Wire Interface (I2C) - triggered by TWI events such as start, stop, and data received/transmitted

ISR(SPM_READY_vect)      // Store Program Memory Ready - indicates readiness for memory operations
ISR(USART1_RX_vect)      // USART1 Rx Complete - triggered when USART1 completes receiving a byte
ISR(USART1_UDRE_vect)    // USART1 Data Register Empty - triggered when USART1's data register is empty
ISR(USART1_TX_vect)      // USART1 Tx Complete - triggered when USART1 completes transmitting a byte
*/



// External Interrupt Configuration Table
/*
| Register | Bits                                                   | Description                                                                                               |
|----------|--------------------------------------------------------|-----------------------------------------------------------------------------------------------------------|
| EICRA    | ISC31, ISC30, ISC21, ISC20, ISC11, ISC10, ISC01, ISC00 | Defines trigger for INT0-INT3: <br> 00: Low-level <br> 10: Falling edge <br> 11: Rising edge              |
| EICRB    | ISC71, ISC70, ISC61, ISC60, ISC51, ISC50, ISC41, ISC40 | Defines trigger for INT4-INT7                                                                             |
| EIMSK    | INT7 to INT0                                           | Enables external interrupts INT0 to INT7                                                                  |
| SREG     | SEI                                                    | Global interrupt enable bit                                                                               |
*/

#ifdef INTERRUPT_EXTERNAL
void main_interrupt_external (void) {
	// Apps
	DDRB = 0xFF;
	PORTB = 0xAA;

	// Interrupt configuration
	EICRA = 1 << ISC01;   // INT0 falling edge
	EICRB = 0x00;
	EIMSK |= 1 << INT0;   // Enable INT0
	sei();                // Enable global interrupt
	
	while (1);
}

ISR(INT0_vect) {
	PORTB = ~PORTB;
}
#endif

// Timer/Counter Interrupt Configuration Table
/*
| Register | Bits                                               | Description                                                                                          |
|----------|----------------------------------------------------|------------------------------------------------------------------------------------------------------|
| TCCR0    | FOC0, WGM00, COM01, COM00, WGM01, CS02, CS01, CS00 | Timer control and prescaler: <br> WGM01=1: CTC Mode <br> CS02-CS00: Prescaler selection               |
| TCNT0    | -                                                  | Timer/Counter initial value (0 to 255)                                                               |
| TIMSK    | OCIE0, TOIE0                                       | Enable Compare Match and Overflow Interrupts                                                         |
| SREG     | SEI                                                | Global interrupt enable bit                                                                          |
*/

#ifdef INTERRUPT_TIMER
void main_interrupt_timer (void) {
	// Apps
	DDRB = 0xFF;
	PORTB = 0xAA;

	// Timer interrupt setup
	TCNT0 = 0x00;                             // Timer initial value
	TCCR0 = (1 << CS02) | (1 << CS01) | (1 << CS00); // Prescaler 1024
	TIMSK |= 1 << TOIE0;                      // Enable overflow interrupt
	sei();                                    // Enable global interrupt

	while (1);
}

ISR(TIMER0_OVF_vect) {
	static int i = 0;
	if (!(i % 10)) {
		PORTB = ~PORTB;
	}
	i++;
	TCNT0 = 0x00;
}
#endif

// Timer Interrupt: CTC Mode Configuration Table
/*
| Register | Bits                           | Description                                                     |
|----------|--------------------------------|-----------------------------------------------------------------|
| TCCR0    | WGM01, CS02, CS01, CS00        | CTC mode (WGM01=1) with prescaler                               |
| TCNT0    | -                              | Timer initial value                                             |
| OCR0     | -                              | Output compare register to set match value                      |
| TIMSK    | OCIE0                          | Enable Compare Match Interrupt                                  |
| SREG     | SEI                            | Global interrupt enable bit                                     |
*/

#ifdef INTERRUPT_TIMER_CTC
void main_interrupt_timer_ctc (void) {
	// Apps
	DDRB = 0xFF;
	PORTB = 0xAA;

	// Timer CTC mode setup
	TCNT0 = 0x00;                             // Timer initial value
	OCR0 = 0xFF;                              // Set match value
	TCCR0 = (1 << WGM01) | (1 << CS02) | (1 << CS01) | (1 << CS00); // CTC mode, clock/1024
	TIMSK = 1 << OCIE0;                       // Enable Compare Match Interrupt
	sei();                                    // Enable global interrupt

	while (1);
}

ISR(TIMER0_COMP_vect) {
	static int i = 0;
	if (!(i % 100)) {
		PORTB = ~PORTB;
	}
	i++;
}
#endif

// Lab: Button Toggle and LCD Display (INT0 Interrupt) Configuration Table
/*
| Register | Bits                         | Description                                           |
|----------|------------------------------|-------------------------------------------------------|
| EICRA    | ISC01                        | Set falling edge for INT0                             |
| EIMSK    | INT0                         | Enable INT0                                           |
| SREG     | SEI                          | Global interrupt enable                               |
*/

#ifdef INTERRUPT_LAB
unsigned char num1, num2;
char result[30];

void main_interrupt_lab (void) {
	init_devices();                         // Initialize LCD
	lcd_clear();
	lcd_string(0, 0, "12345 Hong Jeong");

	// Apps
	DDRB = 0xFF;
	PORTB = 0xAA;

	// INT0 interrupt setup
	EICRA = 1 << ISC01;                     // Falling edge for INT0
	EIMSK = 1 << INT0;                      // Enable INT0
	sei();                                  // Enable global interrupt

	// Timer display logic
	int i = 0, s = 0, m = 0, h = 0;
	char clock[20];
	while (1) {
		_delay_ms(1000);
		i++;
		h = i / 3600;
		m = (i % 3600) / 60;
		s = (i % 3600) % 60;
		sprintf(clock, "%d:%d:%d", h, m, s);
		lcd_string(3, 0, clock);
	}
}

ISR(INT0_vect) {
	// Display random multiplication result
	num1 = rand();
	num2 = rand();
	sprintf(result, "%d * %d = %d", num1, num2, num1 * num2);
	lcd_string(6, 0, result);
	PORTB = ~PORTB;
}
#endif

#ifdef INTERRUPT_EXT_TIMER

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>

// Variables for the clock display
int seconds = 0, minutes = 0, hours = 0;
char clock_display[20];

// Variables for random multiplication result
unsigned char num1, num2;
char result[30];

// Setup External Interrupt on INT0 (PD2)
void setup_external_interrupt(void) {
	DDRD &= ~(1 << PD2);       // Set PD2 as input for external interrupt
	PORTD |= (1 << PD2);       // Enable pull-up resistor on PD2

	EICRA = (1 << ISC01);      // Set INT0 to trigger on falling edge
	EIMSK = (1 << INT0);       // Enable INT0 interrupt
}

// Setup Timer0 Overflow Interrupt for 1-second interval
void setup_timer0_interrupt(void) {
	TCCR0 = (1 << CS02) | (1 << CS00); // Set Timer0 with 1024 prescaler
	TCNT0 = 0;                         // Initialize Timer0 counter
	TIMSK |= (1 << TOIE0);             // Enable Timer0 overflow interrupt
}

// External Interrupt Service Routine for INT0 (Button Press)
ISR(INT0_vect) {
	// Generate random multiplication result
	num1 = rand() % 10;  // Random number between 0 and 9
	num2 = rand() % 10;
	sprintf(result, "%d * %d = %d", num1, num2, num1 * num2);
	lcd_clear();
	lcd_string(6, 0, result); // Display result on LCD

	// Toggle LED on PORTB
	PORTB ^= 0xFF;             // Toggle all bits on PORTB
}

// Timer0 Overflow Interrupt Service Routine (1-second clock update)
ISR(TIMER0_OVF_vect) {
	static int overflow_count = 0;
	overflow_count++;

	if (overflow_count >= 61) { // Approximately 1-second interval
		overflow_count = 0;
		
		// Update clock time
		seconds++;
		if (seconds >= 60) {
			seconds = 0;
			minutes++;
			if (minutes >= 60) {
				minutes = 0;
				hours++;
				if (hours >= 24) hours = 0;
			}
		}

		// Display the updated clock on LCD
		sprintf(clock_display, "%02d:%02d:%02d", hours, minutes, seconds);
		lcd_string(3, 0, clock_display);
	}
}

void main_interrupt_ext_timer(void) {
	// Initialize LCD and display initial message
	init_devices();          // Initialize LCD (assumes lcd_init is defined here)
	lcd_clear();
	lcd_string(0, 0, "12345 Hong Jeong");

	// Setup PORTB as output for LED control
	DDRB = 0xFF;              // Set all PORTB pins as output
	PORTB = 0xAA;             // Initialize PORTB

	// Setup interrupts
	setup_external_interrupt();   // Setup INT0 for button press
	setup_timer0_interrupt();     // Setup Timer0 overflow for 1-second clock

	sei();                    // Enable global interrupts

	// Main loop does nothing, interrupts handle the functionality
	while (1) {
		// Main code can be placed here, if needed
	}
}


#endif