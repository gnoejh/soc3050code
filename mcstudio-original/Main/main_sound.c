/*
 * main_sound.c
 *
 * Created: 2024-10-21 오전 12:12:58
 *  Author: hjeong
 */ 



#include "config.h"

#ifdef SOUND

void main_sound(void)
{
	// Test different sound sequences
	S_Start();      // Play the start sequence
	_delay_ms(1000);  // Delay for 1 second
	
	S_Good();       // Play a 'good' sound
	_delay_ms(1000);  // Delay for 1 second
	
	SError();       // Play an error sound
	_delay_ms(1000);  // Delay for 1 second

	S_Push1();      // Test another sound sequence
	_delay_ms(1000);  // Delay for 1 second
}
#endif