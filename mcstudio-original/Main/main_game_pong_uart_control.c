#include "config.h"

#ifdef GAME_PONG_UART_CONTROL
void main_game_pong_uart_control(void)
{
	// random numbers and seed.
	uint16_t u_rand_x = 0;
	uint16_t u_rand_y = 0;
	uint16_t u_rand_r = 0;
	time_t t;
	init_devices();
	lcd_clear();
	srand((unsigned) time(&t));
	while(1){
		u_rand_x = rand()%64;
		u_rand_y = rand()%128;
		u_rand_r = rand()%10;
		GLCD_Circle(u_rand_x, u_rand_y, u_rand_r);
		_delay_ms(100);
	}
}
#endif
