#include "config.h"

#ifdef ACCELEROMETER
	
void main_accelerometer (void)
{
	unsigned int Data_ADC5 = 0;
	unsigned int Data_ADC6 = 0;
	unsigned int Data_ADC7 = 0;
	init_devices();
	while (1){
		Data_ADC5 = Read_Adc_Data(5)-352+32;	// 아날로그 5번 포트 읽기
		Data_ADC6 = Read_Adc_Data(6)-358+62;	// 아날로그 6번 포트 읽기
		Data_ADC7 = Read_Adc_Data(7);		// 아날로그 7번 포트 읽기
		_delay_ms(300);			// 딜레이 300ms
		
		lcd_clear();				// 그래픽 LCD 클리어
		ScreenBuffer_clear();			// 스크린 버퍼 클리어

		lcd_xy(1,0);	GLCD_4DigitDecimal(Data_ADC5);
		lcd_xy(2,0);	GLCD_4DigitDecimal(Data_ADC6);
		lcd_xy(3,0);	GLCD_4DigitDecimal(Data_ADC7);
		
		lcd_string(0,0,"ADC5,ADC6 Acceleration");	// ADC0 Potentiometer 출력
		GLCD_Line(32,0,31,127);
		GLCD_Line(0,63,63,63);
		GLCD_Circle(Data_ADC5,Data_ADC6,5);
		GLCD_Circle(32,62,5);
	}

}
#endif	