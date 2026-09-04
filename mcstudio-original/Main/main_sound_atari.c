/*
 * main_sound_atari.c
 *
 * Created: 2024-10-21 오전 12:18:28
 *  Author: hjeong
 */ 



#include "config.h"
#ifdef SOUND_ATARI
void main_sound_atari(void)
{
	// Start sound - rising pitch
	for (int i = 100; i < 800; i += 50)
	{
		Sound(i, 30);
		_delay_ms(50);
	}
	
	_delay_ms(500); // Short delay

	// Falling sound - falling pitch
	for (int i = 800; i > 100; i -= 50)
	{
		Sound(i, 30);
		_delay_ms(50);
	}

	_delay_ms(500); // Short delay

	// Rapid beeps like game points
	for (int i = 0; i < 5; i++)
	{
		Sound(400, 50);
		_delay_ms(100);
		Sound(500, 50);
		_delay_ms(100);
	}

	_delay_ms(500); // Short delay

	// Laser shot sound (quickly rising then falling tone)
	for (int i = 600; i < 1000; i += 20)
	{
		Sound(i, 10);
	}
	for (int i = 1000; i > 600; i -= 20)
	{
		Sound(i, 10);
	}

	_delay_ms(500); // Short delay

	// Long, eerie tone (constant low frequency)
	Sound(150, 1000);
}
#endif