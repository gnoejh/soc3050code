/*
 * _buzzer.h - ATmega128 Buzzer/Sound Control Library Header
 * Educational Version for Assembly→C→Python Learning Progression
 */

#ifndef _BUZZER_H_
#define _BUZZER_H_

/*
 * Core Buzzer Functions
 */
void Buzzer_init(void);                                             // Initialize buzzer hardware
void Buzzer_tone(unsigned int frequency, unsigned int duration_ms); // Generate tone
void Buzzer_off(void);                                              // Turn buzzer off immediately

/*
 * Educational Helper Functions
 * Pre-defined sounds for easy learning
 */
void Buzzer_beep(void);        // Simple notification beep
void Buzzer_long_beep(void);   // Long alarm beep
void Buzzer_double_beep(void); // Two quick beeps
void Buzzer_ascending(void);   // Rising tone sequence
void Buzzer_descending(void);  // Falling tone sequence
void Buzzer_scale(void);       // C major scale

/*
 * Advanced Educational Functions
 */
void Buzzer_play_melody(unsigned int *frequencies, unsigned int *durations, unsigned char length);
void Buzzer_frequency_sweep(unsigned int start_freq, unsigned int end_freq, unsigned int step);
void Buzzer_morse_dot(void);  // Morse code dot
void Buzzer_morse_dash(void); // Morse code dash

/*
 * Utility Functions
 */
void delay_us(unsigned int microseconds); // Precise microsecond delay

/*
 * Musical Note Frequency Constants (in Hz)
 * Educational set - easy to remember and use
 */

// 4th Octave (Middle range - most common)
#define NOTE_C4 262 // Middle C
#define NOTE_D4 294 // D
#define NOTE_E4 330 // E
#define NOTE_F4 349 // F
#define NOTE_G4 392 // G
#define NOTE_A4 440 // A (concert pitch)
#define NOTE_B4 494 // B
#define NOTE_C5 523 // High C

// 3rd Octave (Lower range)
#define NOTE_C3 131 // Low C
#define NOTE_D3 147 // Low D
#define NOTE_E3 165 // Low E
#define NOTE_F3 175 // Low F
#define NOTE_G3 196 // Low G
#define NOTE_A3 220 // Low A
#define NOTE_B3 247 // Low B

// Special frequencies
#define NOTE_SILENCE 0 // No sound
#define FREQ_1KHZ 1000 // 1 kHz reference
#define FREQ_2KHZ 2000 // 2 kHz reference

/*
 * Rhythm and Timing Constants (in milliseconds)
 * Based on 120 BPM (beats per minute)
 */
#define WHOLE_NOTE 2000    // 2 seconds
#define HALF_NOTE 1000     // 1 second
#define QUARTER_NOTE 500   // 0.5 seconds
#define EIGHTH_NOTE 250    // 0.25 seconds
#define SIXTEENTH_NOTE 125 // 0.125 seconds

// Shorter durations for alerts
#define BEEP_SHORT 100  // Quick beep
#define BEEP_MEDIUM 300 // Medium beep
#define BEEP_LONG 800   // Long beep

/*
 * Hardware Pin Constants
 * Educational reference for understanding connections
 */
#define BUZZER_PORT_NAME 'G' // Port G
#define BUZZER_PIN_NUMBER 4  // Pin 4 (PG4)

/*
 * Educational Melody Arrays
 * Pre-defined melodies for learning exercises
 */

// Simple scale frequencies
extern unsigned int melody_scale_frequencies[];
extern unsigned int melody_scale_durations[];
extern unsigned char melody_scale_length;

// Happy birthday melody (simplified)
extern unsigned int melody_happy_birthday_frequencies[];
extern unsigned int melody_happy_birthday_durations[];
extern unsigned char melody_happy_birthday_length;

// Alert patterns
extern unsigned int melody_alarm_frequencies[];
extern unsigned int melody_alarm_durations[];
extern unsigned char melody_alarm_length;

/*
 * Educational Macros for Frequency Calculation
 * Help students understand the math behind music
 */

// Convert musical note to frequency (A4=440Hz reference)
#define NOTE_TO_FREQ(note_number) (440.0 * pow(2.0, (note_number - 69) / 12.0))

// Convert frequency to period in microseconds
#define FREQ_TO_PERIOD_US(freq) (1000000UL / (freq))

// Convert frequency to half-period for square wave
#define FREQ_TO_HALF_PERIOD_US(freq) (500000UL / (freq))

/*
 * Educational Function Prototypes for Examples
 * These will be implemented in main_xxx.c files
 */
void main_buzzer_simple_beep(void);    // Basic beep example
void main_buzzer_musical_notes(void);  // Play individual notes
void main_buzzer_melody_player(void);  // Play melody from array
void main_buzzer_frequency_test(void); // Test different frequencies
void main_buzzer_morse_code(void);     // Morse code communication
void main_buzzer_alarm_system(void);   // Alarm pattern generator

/*
 * EDUCATIONAL USAGE EXAMPLES:
 *
 * Basic beep:
 *   Buzzer_init();
 *   Buzzer_beep();                    // Simple notification
 *
 * Musical note:
 *   Buzzer_init();
 *   Buzzer_tone(NOTE_A4, QUARTER_NOTE); // Play A4 for quarter note
 *
 * Scale:
 *   Buzzer_init();
 *   Buzzer_scale();                   // Play C major scale
 *
 * Custom melody:
 *   unsigned int freqs[] = {NOTE_C4, NOTE_E4, NOTE_G4};
 *   unsigned int durs[] = {QUARTER_NOTE, QUARTER_NOTE, HALF_NOTE};
 *   Buzzer_play_melody(freqs, durs, 3);
 *
 * Frequency sweep:
 *   Buzzer_frequency_sweep(200, 2000, 100); // 200Hz to 2kHz in 100Hz steps
 */

#endif // _BUZZER_H_