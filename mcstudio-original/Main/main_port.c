
#include "config.h"

/* 2. Port Programming: Blinking */
#ifdef PORT_BLINKING

void main_port_blinking(void){
	DDRB = 0xFF;
	PORTB = 0xAA;
	int flag = 0;
	init_devices();
	lcd_clear();
	lcd_string(0,0,"12345 Hong Jeong");
	while(1){
		PORTB = ~PORTB;
		_delay_ms(1000);
		lcd_clear();
		for (int i=0; i<8; i++){
			if(flag) {
				lcd_string(0,0,"12345 Hong Jeong");
				//GLCD_Circle(50,10*i+10,2);
				flag = 0;
			}
			else {
				lcd_string(0,0,"12345 Hong Jeong");
				GLCD_Circle(50,10*i+10,4);
				flag = 1;
			}
			//ScreenBuffer_clear();	
		}
		
	}

}
#endif

/* Port Programming: Rotation */
#ifdef PORT_ROTATION

void main_port_rotation(void){
	DDRB = 0xFF;
	DDRD = 0x00;
	PORTB = 0x7F;
	while (1){
		if (PIND & 0x01){
			PORTB = (PORTB << 1) | (PORTB >> 7);
		}
		else {
			PORTB = (PORTB >> 1) | (PORTB << 7);	
		}
		_delay_ms(1000);
		
	}

}
#endif
