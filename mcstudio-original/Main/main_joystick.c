
#include "config.h"

#ifdef JOYSTICK
/* Joystick */

unsigned int mData_ADC3 = 0;
unsigned int mData_ADC4 = 0;
	
void main_joystick (void)
{
	init_devices();
	while (1){
		mData_ADC3 = Read_Adc_Data(3) / 16;					// read analog port
		mData_ADC4 = 127 - Read_Adc_Data(4) / 8;			// read analog port
		_delay_ms(200);			
		lcd_clear();				
		ScreenBuffer_clear();			
		lcd_string(0,0,"ADC3,ADC4 JOYSTICK");				// output joystick position
		GLCD_Line(31,0,31,127);   GLCD_Line(0,63,64,63);	// line output horizontal vertical
		GLCD_Circle(mData_ADC3,mData_ADC4,5);				// output position
	}

}
	
#endif	