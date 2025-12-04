/* Serial communication: Loop-back by Interrupt	*/
#define F_CPU 16000000UL
#include <avr/io.h>				//<avr/portpins.h> <avr/iom128.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "_main.h"
#include "_buzzer.h"
#include "_adc.h"
#include "_eeprom.h"
//#include "_init.h"
//#include "_interrupt.h"
#include "_port.h"
#include "_timer2.h"
//#include "_uart.h"
#include "_glcd.h"
#define BAUD 9600

/* Atmega128 board: PD2,3:RXD1,TXD1 */
/* 	UART Registers:
	UCSRA: RXC   TXC   UDRE	
	UCSRB: RXCIE TXCIE UDRIE RXEN TXEN UCSZ2 
	UCSRC: UCSZ1 UCSZ0 
*/	

/* Serial communications UART1 for Atmega128 board: Polling */
/*
void main_serial(void)										
{
	unsigned char data;
	// UART1: 9600 8N1
	UBRR1H = (F_CPU/16/BAUD-1)>>8;		// UBRR
	UBRR1L = F_CPU/16/BAUD-1;		
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);		// receiver and transmitter
	UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);	// 8-bit
	sei();								//never forget
		
	DDRB = 0xFF;
	PORTB = 0xAA;	
	while (1){
		while (!(UCSR1A & (1<<RXC1)));	// receiver polling
		data = UDR1;
		PORTB = ~PORTB;
		_delay_ms(10);
		
		while(!(UCSR1A & (1<<UDRE1)));	// transmitter polling
		UDR1 = data;
		_delay_ms(10);
		PORTB = ~PORTB;
	}	
}
*/


/* Serial communication: Interrupt */

int main_serial (void)
{
	DDRB = 0xFF;
	PORTB = 0xAA;

	// UART1: 9600 8N1
	UBRR1H = (F_CPU/16/BAUD-1)>>8;		// UBRR
	UBRR1L = F_CPU/16/BAUD-1;
	UCSR1B = (1<<RXCIE1)|(1<<TXCIE1)|(1<<RXEN1)|(1<<TXEN1); // receiver/ transmit interrupt
	UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);			// character size 8
	sei();
	
	while(1);
	return 0;
}

		

SIGNAL(USART1_RX_vect)					// Serial ISR
{
	UDR1 = UDR1 + 1;
				
}
SIGNAL(USART1_TX_vect)
{
	PORTB = ~PORTB;
}



