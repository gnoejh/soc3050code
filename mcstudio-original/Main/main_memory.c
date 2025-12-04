#include "config.h"

/*  Accessing Flash Memory */
#ifdef MEMORY_PROGRAM

#include <avr/pgmspace.h>
const unsigned char PROGMEM lookup[] = "0123456789abcdefzhijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!@#$%^&*()_-<>,.~`+";
void main_memory_program(void)
{
	unsigned char a;
	DDRB = 0xFF;
	
	init_devices();
	while (1){
		for (int i=0; i < 128; i++){
			a = pgm_read_byte(&lookup[i]);
			PORTB = ~(a - 0x30);
			_delay_ms(1000);
			lcd_clear();							// clear LCD
			ScreenBuffer_clear();					// clear screen buffer
			lcd_string(0,0,"12345 Hong Jeong");
			lcd_string(2,0, "FLASH Data");
			lcd_xy(4,0);
			lcd_char(a);
		}
	}
}
#endif


/* Accessing EEPROM */
#ifdef MEMORY_EEPROM

#include <avr/eeprom.h>

unsigned char EEMEM *lookup_p;
unsigned char lookup[]="012345";
unsigned char result;

void main_memory_eeprom(void)
{
	DDRD = 0x00;
	init_devices();
	S_Start();
	while (1){
		if((PIND&(1<<0)) == 0){  //if PD0 is LOW
			//eeprom_write_byte(lookup_p,lookup[4]);
			//In_EEPROM_Write(*lookup_p,lookup[5]);
			In_EEPROM_Write(100,lookup[5]);
		}
		else {
			//result = eeprom_read_byte(lookup_p); //read from EEPROM
			//result = In_EEPROM_Read(*lookup_p);
			result = In_EEPROM_Read(100);
			_delay_ms(1000);
			lcd_clear();							// clear LCD
			ScreenBuffer_clear();					// clear screen buffer
			lcd_string(0,0,"12345 Hong Jeong");
			lcd_string(2,0,"EEPROM Data");
			lcd_xy(4,0);
			lcd_char(result);
		}
	}

}
#endif

