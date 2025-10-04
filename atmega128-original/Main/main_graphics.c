
#include "config.h"

/* Graphics: basics */
#ifdef GRAPHICS_BASICS

/* Graphics: Characters  */
char Dis_Scr_IO_ON1[]={"O"};
char Dis_Scr[]={"#$%&'()*+,-./0123456"};
void main_graphics_basics(void)
{
	init_devices();								// initialize LCD
	S_Start();									// sound
	lcd_clear();
	while(1){
	lcd_string(0,0,"====================");       
	lcd_string(1,0,"   ATmega128 GLCD   ");		
	lcd_string(2,0,Dis_Scr);
	GLCD_Line(30,10,40,20);				GLCD_Rectangle(30,31,40,41);    	
	lcd_string(4,5,Dis_Scr_IO_ON1);  	GLCD_Circle(35,55,5);
	lcd_xy(4,12); GLCD_4DigitDecimal(1235);
	lcd_string(6,0, "SOC3050 Hong Jeong");
	_delay_ms(100);
	}
	
}
#endif

/* Graphics: Random circles 	*/
#ifdef GRAPHICS_RANDOM

void main_graphics_random (void)
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
	
/* 	Graphics: Random movement 	*/
#ifdef GRAPHICS_MOVEMENT

void main_graphics_movement (void)
{
	//random numbers and seed.
	uint16_t u_rand_x = 0, old_x = 0;
	uint16_t u_rand_y = 0, old_y = 0;
	time_t t;
		
	init_devices();
	lcd_clear();
	srand((unsigned) time(&t));
	while(1){
		//S_Star();
		u_rand_x = rand()%64;
		u_rand_y = rand()%128;
		GLCD_Line(old_x, old_y, u_rand_x, u_rand_y);
		old_x = u_rand_x; old_y = u_rand_y;
		_delay_ms(100);
	}
}
#endif

#ifdef GRAPHICS_BOUNCING_BALL

void main_graphics_bouncing_ball (void)
{
	uint16_t x, y, radius = 5;
	int8_t dx = 6, dy = 6;
	time_t t;

	init_devices();
	lcd_clear();

	// Seed the random number generator
	srand((unsigned) time(&t));
	
	// Randomize the initial position of the ball
	x = rand() % (64 - radius * 2) + radius;  // Ensure it starts within the screen bounds
	y = rand() % (128 - radius * 2) + radius;

	while (1) {
		GLCD_Circle(x, y, radius); // Draw ball
		
		_delay_ms(100);
		GLCD_Circle(x, y, radius); // Clear ball by redrawing it (as XOR)
		
		// Update position
		x += dx;
		y += dy;
		
		// Bounce if it hits the edge
		if (x + radius >= 64 || x - radius <= 0) dx = -dx;
		if (y + radius >= 128 || y - radius <= 0) dy = -dy;
	}
}
#endif


#ifdef GRAPHICS_MOVING_SQUARE

void main_graphics_moving_square (void)
{
	uint16_t x = 0;
	
	init_devices();
	lcd_clear();
	
	while (1) {
		GLCD_Rectangle(x, 30, x + 10, 40); // Draw square
		_delay_ms(100);
		GLCD_Rectangle(x, 30, x + 10, 40); // Erase square
		
		x += 8; // Move square to the right
		
		if (x > 64) {  // If it moves out of bounds, reset to the left side
			x = 0;
		}
	}
}
#endif

#ifdef GRAPHICS_SINE_WAVE

void main_graphics_sine_wave (void)
{
	uint16_t x = 0, y = 0;
	uint16_t radius = 0.5;
	
	init_devices();
	lcd_clear();
	
	for (y = 0; y < 128; y++) {
		// Invert the sine wave by negating the sine function
		x = (uint16_t)(32 - 30 * sin(y * M_PI / 64));  // Calculate vertical position (inverted sine wave)

		GLCD_Circle(x, y, radius);  // Draw circle at the (x, y) position based on inverted sine wave
		_delay_ms(50);              // Small delay to visualize the wave pattern
	}
}
#endif


