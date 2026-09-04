/*
 * main_sound_twingkle.c
 *
 * Created: 2024-10-21 오전 12:25:35
 *  Author: hjeong
 */ 



#include "config.h"

#ifdef SOUND_TWINGKLE

#define LDo  395  // Low Do note frequency
#define LRe  354  // Low Re note frequency
#define LMi  316  // Low Mi note frequency
#define LFa  298  // Low Fa note frequency
#define LSo  266  // Low So note frequency
#define LLa  237  // Low La note frequency
#define LSi  212  // Low Si note frequency

#define HDo  101  // High Do note frequency
#define HRe  90   // High Re note frequency
#define HMi  80   // High Mi note frequency
#define HFa  76   // High Fa note frequency
#define HSo  68   // High So note frequency
#define HLa  61   // High La note frequency
#define HSi  54   // High Si note frequency

void main_sound_twingkle(void)
{
	// "Twinkle Twinkle Little Star"
	Sound(LDo, 400);  // Twinkle
	Sound(LDo, 400);  // Twinkle
	Sound(HSo, 400);  // Little
	Sound(HSo, 400);  // Star
	Sound(HLa, 400);  // How
	Sound(HLa, 400);  // I
	Sound(HSo, 800);  // Wonder
	
	_delay_ms(400);   // Pause between phrases

	// "What you are"
	Sound(HFa, 400);  // What
	Sound(HFa, 400);  // You
	Sound(HMi, 400);  // Are
	Sound(HMi, 400);  // Above
	Sound(HRe, 400);  // The
	Sound(HRe, 400);  // World
	Sound(HDo, 800);  // So high

	_delay_ms(400);   // Pause between phrases

	// "Like a diamond in the sky"
	Sound(HSo, 400);  // Like
	Sound(HSo, 400);  // A
	Sound(HFa, 400);  // Diamond
	Sound(HFa, 400);  // In
	Sound(HMi, 400);  // The
	Sound(HMi, 400);  // Sky
	Sound(HRe, 800);  // So high

	_delay_ms(400);   // Pause before repeating

	// Repeat: "Twinkle Twinkle Little Star"
	Sound(LDo, 400);  // Twinkle
	Sound(LDo, 400);  // Twinkle
	Sound(HSo, 400);  // Little
	Sound(HSo, 400);  // Star
	Sound(HLa, 400);  // How
	Sound(HLa, 400);  // I
	Sound(HSo, 800);  // Wonder
}
#endif