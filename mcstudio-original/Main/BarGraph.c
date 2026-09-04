
#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "_main.h"
#include "_buzzer.h"
#include "_adc.h"
#include "_eeprom.h"
#include "_init.h"
#include "_interrupt.h"
#include "_port.h"
#include "_timer2.h"
#include "_uart.h"
#include "_glcd.h"

unsigned int Uart1_DataView = 0;
unsigned int INT0_DataView = 0;
unsigned int INT1_DataView = 0;
unsigned int Data_ADC0 = 0;

char Dis_Scr_IO_ON[]={"O"};
char Dis_Scr_IO_OFF[]={"X"};
char Dis_Scr1[]={"#$%&'()*+,-./0123456"};
char Dis_Scr2[]={"789:;<=>?@ABCDEFGHIJ"};
char Dis_Scr3[]={"KLMNOPQRSTUVWXYZ[]{}"};

int main(void)
{
	init_devices();	S_Start();
	
	lcd_clear();
	lcd_string(0,0,"====================");
	lcd_string(1,0,"   ATmega128 GLCD   ");
	lcd_string(2,0,Dis_Scr1);
	GLCD_Line(30,10,40,20);				GLCD_Rectangle(30,31,40,41);
	lcd_string(4,5,Dis_Scr_IO_ON);  	GLCD_Circle(35,55,5);
	lcd_xy(4,12); GLCD_4DigitDecimal(1235);
	
	while(1)
	{
		Data_ADC0 = Read_Adc_Data(0);		// 아날로그 0번 포트 읽기
		_delay_ms(200);				// 딜레이 200ms
		lcd_clear();				// 그래픽 LCD 클리어
		ScreenBuffer_clear();			// 스크린 버퍼 클리어
		
		lcd_string(0,0,"ADC0 Potentiometer");		// ADC0 Potentiometer 출력
		GLCD_Rectangle(20,0,30,Data_ADC0);		// 라인 게이지 출력
		lcd_xy(1,0); GLCD_4DigitDecimal(Data_ADC0);	// ADC0의 값을 출력

		
	}
}