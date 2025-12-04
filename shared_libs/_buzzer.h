
/*
 * _buzzer.h - ATmega128 Buzzer and Sound Library Header
 * Educational Version for Assembly→C→Python Learning Progression
 */

#ifndef _BUZZER_H_
#define _BUZZER_H_

/*
 * Core Buzzer Functions - Basic Sound Generation
 */
void Buzzer_init(void);                         // Initialize buzzer hardware
void delay_us_Melody(int data);                 // Precise delay for sound timing
void Sound(unsigned int ch, unsigned int time); // Generate square wave sound

/*
 * Advanced Sound Functions - Enhanced Audio Control
 */
void Buzzer_set_volume(unsigned char volume);                           // Set volume (0-100%)
void Buzzer_enable(unsigned char enable);                               // Enable/disable buzzer
void Play_Note_Hz(unsigned int frequency_hz, unsigned int duration_ms); // Play by frequency

/*
 * Pre-defined Sound Effects - User Feedback
 */
void S_Good(void);  // Success/confirmation sound
void SError(void);  // Error/warning sound
void S_Push1(void); // Button press feedback
void S_Start(void); // System startup sequence

/*
 * Level/Achievement Sounds - Game and Application Feedback
 */
void S_S1(void); // Level 1 - four-note ascending
void S_S2(void); // Level 2 - two high notes
void S_S3(void); // Level 3 - single note
void S_S4(void); // Level 4 - three-note sequence
void S_S5(void); // Level 5 - two-note sequence
void S_S6(void); // Level 6 - repeating pattern
void S_S7(void); // Level 7 - octave jump

/*
 * Musical Functions - Educational Music Theory
 */
void S_Star(void);                 // "Twinkle Twinkle" opening
void Play_Scale_Major(void);       // C major scale
void Play_Chord_C_Major(void);     // C major chord arpeggio
void Sound_Test_All_Octaves(void); // Test all frequency ranges

/*
 * Global Variables for Educational Use
 */
extern volatile unsigned char buzzer_enabled; // Global buzzer enable/disable
extern unsigned char current_volume;          // Volume level (0-100%)
extern unsigned char current_tempo;           // Tempo in BPM
extern unsigned int note_gap_us;              // Gap between notes

/*
 * Musical Note Constants - Frequency References
 * Values represent half-periods in microseconds
 */

// Low Octave Notes (Deep bass tones)
#define NOTE_L_C 395 // C3  ≈ 131 Hz
#define NOTE_L_D 354 // D3  ≈ 147 Hz
#define NOTE_L_E 316 // E3  ≈ 165 Hz
#define NOTE_L_F 298 // F3  ≈ 175 Hz
#define NOTE_L_G 266 // G3  ≈ 196 Hz
#define NOTE_L_A 237 // A3  ≈ 220 Hz
#define NOTE_L_B 212 // B3  ≈ 247 Hz

// Base Octave Notes (Mid-range tones)
#define NOTE_C 200 // C4  ≈ 262 Hz (Middle C)
#define NOTE_D 178 // D4  ≈ 294 Hz
#define NOTE_E 159 // E4  ≈ 330 Hz
#define NOTE_F 150 // F4  ≈ 349 Hz
#define NOTE_G 134 // G4  ≈ 392 Hz
#define NOTE_A 112 // A4  ≈ 440 Hz (Concert pitch)
#define NOTE_B 107 // B4  ≈ 494 Hz

// High Octave Notes (Treble tones)
#define NOTE_H_C 101 // C5  ≈ 523 Hz
#define NOTE_H_D 90  // D5  ≈ 587 Hz
#define NOTE_H_E 80  // E5  ≈ 659 Hz
#define NOTE_H_F 76  // F5  ≈ 698 Hz
#define NOTE_H_G 68  // G5  ≈ 784 Hz
#define NOTE_H_A 61  // A5  ≈ 880 Hz
#define NOTE_H_B 54  // B5  ≈ 988 Hz

// Very High Octave
#define NOTE_VH_C 51 // C6  ≈ 1047 Hz

/*
 * Timing Constants for Musical Rhythm
 */
#define BEAT_WHOLE 140   // Whole note duration
#define BEAT_HALF 70     // Half note duration
#define BEAT_QUARTER 35  // Quarter note duration (standard)
#define BEAT_EIGHTH 17   // Eighth note duration
#define BEAT_SIXTEENTH 8 // Sixteenth note duration

#define REST_SHORT 30  // Short rest/pause
#define REST_MEDIUM 50 // Medium rest/pause
#define REST_LONG 65   // Long rest/pause

/*
 * Common Frequencies in Hz for Direct Use
 */
#define FREQ_C3 131    // Low C
#define FREQ_C4 262    // Middle C
#define FREQ_C5 523    // High C
#define FREQ_A4 440    // Concert pitch A
#define FREQ_BEEP 1000 // Standard beep frequency

/*
 * Hardware Configuration Constants
 */
#define BUZZER_PORT PORTG // Buzzer output port
#define BUZZER_PIN 4      // Buzzer pin number
#define BUZZER_DDR DDRG   // Buzzer direction register

/*
 * Function Prototypes for Educational Examples
 * These are implemented in separate main_*.c files
 */
void main_buzzer_basic_tones(void);     // Basic tone generation
void main_buzzer_musical_scale(void);   // Musical scale demo
void main_buzzer_sound_effects(void);   // Sound effects demo
void main_buzzer_melody_player(void);   // Simple melody player
void main_buzzer_frequency_sweep(void); // Frequency sweep demo
void main_buzzer_user_interface(void);  // UI feedback sounds

#endif // _BUZZER_H_
