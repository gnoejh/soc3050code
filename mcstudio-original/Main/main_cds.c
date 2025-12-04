#include "config.h"

#ifdef CDS
	
void main_cds (void)
{
	unsigned int Data_ADC2 = 0;
	init_devices();
	while (1){
		Data_ADC2 = Read_Adc_Data(2) / 10;		// 아날로그 0번 포트 읽기
		_delay_ms(200);				// 딜레이 200ms
		
		lcd_clear();				// 그래픽 LCD 클리어
		ScreenBuffer_clear();			// 스크린 버퍼 클리어
		lcd_string(0,0,"ADC2 CDS");			// ADC0 Potentiometer 출력
		GLCD_Circle(35,55,Data_ADC2);		// 라인 게이지 출력
		lcd_xy(1,0); GLCD_4DigitDecimal(Data_ADC2); 	// ADC2의 값을 출력

	}
}
#endif
	