///* Games */
//#define F_CPU 16000000UL
//#include <avr/io.h>				//<avr/portpins.h> <avr/iom128.h>
//#include <avr/interrupt.h>
//#include <util/delay.h>
//#include <stdio.h>
//#include <string.h>
//#include <stdlib.h>
//#include <time.h>
//
//#include "_main.h"
//#include "_buzzer.h"
//#include "_adc.h"
//#include "_eeprom.h"
//#include "_init.h"
//#include "_interrupt.h"
//#include "_port.h"
//#include "_timer2.h"
//#include "_uart.h"
//#include "_glcd.h"
//#include <time.h>
//#include "Template.h"
//
//int main_game0 (void) {
	//init_devices();
	//lcd_clear();
	//lcd_string(1, 0, "Press Start");
	//
	//while (!(PIND & 0xFF)); // Wait for any button to be pressed
//
	//time_t t;
	//srand((unsigned) time(&t)); // Initialize the random seed
//
	//while (1) {
		//int position = rand() % 8; // Assuming 8 buttons for 8 possible positions
		//int x = position * 16; // Adjust if needed for exact screen positioning
		//int y = 64; // Middle of the screen in the y-axis, adjust if necessary
		//GLCD_Circle(x, y, 5);
//
		//_delay_ms(500); // Give player 500ms to react, adjust as needed
//
		//if (PIND & (1 << position)) {
			//S_Start();
			//lcd_clear();
			//lcd_string(1, 0, "Success!");
			//} else {
			//lcd_clear();
			//lcd_string(1, 0, "Try Again!");
		//}
		//
		//_delay_ms(1000); // Delay before next round
	//}
//
	//return 0;
//}
