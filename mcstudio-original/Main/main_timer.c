#include "config.h"


/*
// Timer/Counter 0 (8-bit)
| Register | Bit     | Description                     |
|----------|---------|---------------------------------|
| TCCR0    | CS02:00 | Clock source and prescaler     |
|          |         | 000: No clock (stopped)        |
|          |         | 001: No prescaling             |
|          |         | 010: /8, 011: /64              |
|          |         | 100: /256, 101: /1024          |
|          | WGM01:0 | Waveform Generation Mode       |
|          |         | 00: Normal                     |
|          |         | 01: PWM, Phase Correct         |
|          |         | 10: CTC (Clear Timer on Match) |
|          |         | 11: Fast PWM                   |
|          | COM01:0 | Compare Match Output Mode      |

// Timer/Counter 1 (16-bit)
| Register | Bit     | Description                     |
|----------|---------|---------------------------------|
| TCCR1A   | WGM11:0 | Lower bits of Waveform Gen Mode |
|          | COM1A1:0| Compare Match Output (Channel A)|
|          | COM1B1:0| Compare Match Output (Channel B)|
| TCCR1B   | CS12:10 | Clock source and prescaler     |
|          |         | 000: No clock (stopped)        |
|          |         | 001: No prescaling             |
|          |         | 010: /8, 011: /64              |
|          |         | 100: /256, 101: /1024          |
|          |         | 110/111: Ext. clk on T1 (PD6)  |
|          | WGM13:12| Upper bits of Waveform Gen Mode|
|          | ICES1   | Input Capture Edge Select      |
|          | ICNC1   | Input Capture Noise Canceler   |

// Timer/Counter 2 (8-bit)
| Register | Bit     | Description                     |
|----------|---------|---------------------------------|
| TCCR2    | CS22:20 | Clock source and prescaler     |
|          |         | 000: No clock (stopped)        |
|          |         | 001: No prescaling             |
|          |         | 010: /8, 011: /32             |
|          |         | 100: /64, 101: /128           |
|          |         | 110: /256, 111: /1024         |
|          | WGM21:0 | Waveform Generation Mode       |
|          |         | 00: Normal                     |
|          |         | 01: PWM, Phase Correct         |
|          |         | 10: CTC (Clear Timer on Match) |
|          |         | 11: Fast PWM                   |
|          | COM21:0 | Compare Match Output Mode      |

*/

#ifdef TIMER_NORMAL
/* 1. Timer/Counter0 Normal Mode
TIFR:  OCF2|TOV2|ICF1|OCF1B|TOV1|OCF0|TOV0
TCCR0: FOC0|WGM00|COM01|COM00|WGM01|CS02|CS01|CS00
*/

void main_timer_normal (void)
{
	init_devices();
	// Toggle PORTB.7
	DDRB |= 1<<PB7;

	while (1)
	{
		// delay
		for (char i = 0; i < 100; i++){		// delay 100 times
			TCNT0 = 0x00;					// 0~255
			// Set the clock source to internal clock/1024
			TCCR0 |= (1<<CS02) | (0<< CS01) | (1<< CS00); // clock/1024
			while(!(TIFR&(1<<TOV0)));		// wait
			TCCR0 = 0;						// stop
			TIFR |= (1<<TOV0);				// reset TOV
		}
		// application
		PORTB ^= (1<<PB7);	
	}
}
#endif


#ifdef TIMER_COUNTER
void main_timer_counter(void)
{
	init_devices();
	// Configure PORTB.7 as output to toggle an LED
	DDRB |= 1 << PB7;
	//PORTB &= ~(1 << PB7);  // Ensure PB7 is initially low
	DDRD &= ~(1 << PD6);    // Set PD6 as input for external clock from push button (T1)
	PORTD |= (1<<PD6);		// Pull up resistance

	while(1)
	{
		TCNT1 = 0xFFFD;  // Set Timer1 counter close to overflow for quick response
		
		// Configure Timer1 with external clock on T1 (PD6), falling edge
		TCCR1A = 0;  // Set TCCR1A to 0 as no PWM or output compare is used
		TCCR1B = (1 << CS12);  // Set external clock on T1 with falling edge (CS12 = 1, CS11 = 0, CS10 = 0)
		
		while (!(TIFR & (1 << TOV1)));  // Wait for overflow (button press)
		TCCR1B = 0;  // Stop Timer1 after overflow
		TIFR |= (1 << TOV1);  // Clear overflow flag by writing a 1 to TOV1

		// Toggle PORTB.7 to indicate button press
		PORTB ^= (1 << PB7);
	}
}
#endif


#ifdef TIMER_CTC
/* Timer0: CTC Mode */
void main_timer_ctc (void)
{
	// Graphics
	init_devices();
	lcd_string(0,0,"12345 Hong Jeong");
	
	// application
	DDRB |= 1<<PB7;
	
	while (1)
	{	
		// delay
		for (int i=0; i < 100; i++){
			TCCR0 = (1<<WGM01) | (1<<CS02) | (0<< CS01) | (1<< CS00); // CTC and Clock/1024
			OCR0 = 255;						// 0~255
			while(!(TIFR&(1<<OCF0)));	// wait
			TCCR0 = 0;						// stop
			TIFR |= (1<<OCF0);				// reset flag
			lcd_xy(4,0); GLCD_4DigitDecimal(i);
		}
		// application
		PORTB ^= (1<<PB7);
	}
}
#endif


#ifdef TIMER_FASTPWM
/* Timer0: Fast PWM mode (PB4: OC0)*/
void main_timer_fastpwm (void)
{
	// Graphics
	init_devices();
	
	// application
	DDRB = 1<<4; //OC0 PWM output
	
	while(1){
		for (int offset=255; offset >0; offset -= 10)
		{
			// delay
			for (int i=0; i < 100; i++){
				TCCR0 = (1<<WGM01) | (1<<WGM00) | (1<<COM01) | (1<<COM00) | (1<<CS02) | (0<< CS01) | (1<< CS00); // fast PWM, OC0, and Clock/1024
				OCR0 = offset;					// 0~255 input signal
				while((TIFR&(1<<OCF0))==0){}	// wait
				TCCR0 = 0;						// stop
				TIFR |= (1<<OCF0);				// reset flag
			}	
		}
	}
}
#endif

