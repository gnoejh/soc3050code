
/*
 * _buzzer.c - ATmega128 Educational Buzzer and Sound Library
 * Part of Assembly → C → Python Learning Progression
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Understand sound generation using digital output
 * 2. Learn frequency and timing concepts in audio
 * 3. Master pulse-width modulation principles
 * 4. Practice musical note representation and scales
 * 5. Bridge assembly bit manipulation to C abstraction
 * 6. Prepare for Python audio synthesis concepts
 *
 * SOUND GENERATION OVERVIEW:
 * - Sound = pressure waves in air at specific frequencies
 * - Digital sound = rapid switching of output pin (square wave)
 * - Frequency = number of cycles per second (Hz)
 * - Musical notes = specific frequencies in harmonic ratios
 * - Buzzer = simple speaker that converts electrical signals to sound
 *
 * FREQUENCY CALCULATIONS:
 * Note frequency determines pitch (higher frequency = higher pitch)
 * Period = 1 / frequency
 * Half-period = time for high or low portion of square wave
 * Delay time = half-period in microseconds
 *
 * ATmega128 SOUND FEATURES:
 * - Any digital output pin can generate square waves
 * - Precise timing using delay functions
 * - Multiple octaves by frequency scaling
 * - Volume control through pulse timing
 * - Complex melodies through note sequences
 *
 * ASSEMBLY EQUIVALENT CONCEPTS:
 * - Pin high/low    ≡  SBI/CBI PORTG, PIN4
 * - Delay loop      ≡  DEC R16; BRNE loop
 * - Function call   ≡  CALL subroutine; RET
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "_main.h"
#include "_buzzer.h"

/*
 * EDUCATIONAL CONSTANTS: Musical Note Frequencies
 * These values represent half-periods in microseconds for each note
 * Lower values = higher frequency = higher pitch
 *
 * FREQUENCY CALCULATION:
 * For a note with frequency F Hz:
 * Period = 1/F seconds = 1,000,000/F microseconds
 * Half-period = 500,000/F microseconds
 *
 * MUSICAL THEORY:
 * - Each octave doubles the frequency
 * - Standard tuning: A4 = 440 Hz
 * - Semitone ratio: 12th root of 2 ≈ 1.059
 */

// Low Octave Notes (L prefix) - Deep bass tones
#define LDo 395 // C3  ≈ 131 Hz (Low C)
#define LRe 354 // D3  ≈ 147 Hz
#define LMi 316 // E3  ≈ 165 Hz
#define LFa 298 // F3  ≈ 175 Hz
#define LSo 266 // G3  ≈ 196 Hz
#define LLa 237 // A3  ≈ 220 Hz
#define LSi 212 // B3  ≈ 247 Hz

// Base Octave Notes (B prefix) - Mid-range tones
#define BDo 200 // C4  ≈ 262 Hz (Middle C)
#define BRe 178 // D4  ≈ 294 Hz
#define BMi 159 // E4  ≈ 330 Hz
#define BFa 150 // F4  ≈ 349 Hz
#define BSo 134 // G4  ≈ 392 Hz
#define BLa 112 // A4  ≈ 440 Hz (Concert pitch)
#define BSi 107 // B4  ≈ 494 Hz

// High Octave Notes (H prefix) - Treble tones
#define HDo 101 // C5  ≈ 523 Hz (High C)
#define HRe 90	// D5  ≈ 587 Hz
#define HMi 80	// E5  ≈ 659 Hz
#define HFa 76	// F5  ≈ 698 Hz
#define HSo 68	// G5  ≈ 784 Hz
#define HLa 61	// A5  ≈ 880 Hz
#define HSi 54	// B5  ≈ 988 Hz

// Very High Octave Notes (h prefix) - Ultra treble
#define hDo 51 // C6  ≈ 1047 Hz (Very high C)

/*
 * EDUCATIONAL CONSTANTS: Rhythm and Timing
 * These control note duration and musical timing
 */
#define RB 50 // Rest/Break - short pause
#define LB 65 // Long Break - extended pause
#define SB 30 // Short Beat - quick note
#define BB 35 // Base Beat - standard note duration

/*
 * EDUCATIONAL CONSTANTS: Hardware Configuration
 * These define the physical connection of the buzzer
 */
#define Buzzer_Port PORTG						 // Port G for buzzer output
#define Buzzer_Pin 4							 // Pin 4 on Port G
#define MelOn SETBIT(Buzzer_Port, Buzzer_Pin)	 // Turn buzzer on (high)
#define MelOff CLEARBIT(Buzzer_Port, Buzzer_Pin) // Turn buzzer off (low)

/*
 * EDUCATIONAL VARIABLES
 * Global variables for sound control and music programming
 */
volatile unsigned char buzzer_enabled = 1; // Global buzzer enable/disable
unsigned char current_volume = 100;		   // Volume level (0-100%)
unsigned char current_tempo = 120;		   // Tempo in BPM (beats per minute)
unsigned int note_gap_us = 50;			   // Gap between notes in microseconds

/*
 * EDUCATIONAL FUNCTION: Precise Microsecond Delay
 *
 * PURPOSE: Provide accurate timing for sound generation
 * LEARNING: Shows custom delay implementation and timing control
 *
 * PARAMETERS:
 * data - delay time in microseconds
 *
 * NOTE: This function provides more precise control than _delay_us()
 * for sound generation where exact timing is critical
 *
 * ASSEMBLY EQUIVALENT:
 * loop: DEC R16        ; Decrement counter
 *       BRNE loop      ; Branch if not zero
 *       NOP            ; Small delay adjustments
 */
void delay_us_Melody(int data)
{
	int i;
	/*
	 * Simple delay loop - each iteration takes approximately 1 microsecond
	 * Actual timing may vary slightly due to loop overhead
	 * For critical applications, assembly or hardware timers would be used
	 */
	for (i = 0; i < data; i++)
	{
		_delay_us(1);
	}
}

/*
 * EDUCATIONAL FUNCTION: Generate Sound (Primary Function)
 *
 * PURPOSE: Generate square wave sound with specified frequency and duration
 * LEARNING: Shows digital sound synthesis and frequency generation
 *
 * PARAMETERS:
 * ch   - duration cycles (higher = longer sound)
 * time - half-period in microseconds (lower = higher pitch)
 *
 * PROCESS:
 * 1. Calculate total number of square wave cycles
 * 2. For each cycle: set pin high, delay, set pin low, delay
 * 3. Continue until desired duration is reached
 *
 * FREQUENCY CALCULATION:
 * Frequency (Hz) = 500,000 / time
 * Duration (ms) = (ch * time * 2) / 1000
 *
 * ASSEMBLY EQUIVALENT:
 * sound_loop:
 *   SBI PORTG, 4      ; Set buzzer pin high
 *   CALL delay        ; Wait half period
 *   CBI PORTG, 4      ; Set buzzer pin low
 *   CALL delay        ; Wait half period
 *   DEC cycles        ; Decrement cycle count
 *   BRNE sound_loop   ; Continue if not zero
 */
void Sound(unsigned int ch, unsigned int time)
{
	unsigned int cycles;

	/* Check if buzzer is enabled */
	if (!buzzer_enabled)
		return;

	/*
	 * Calculate number of square wave cycles needed
	 * ch represents duration multiplier
	 * time affects both pitch and contributes to duration calculation
	 */
	cycles = ch;
	cycles = cycles * 1000 / time; // Scale cycles based on frequency

	/*
	 * Generate square wave for calculated number of cycles
	 * Each cycle = high period + low period = one complete waveform
	 */
	while (cycles--)
	{
		/*
		 * High portion of square wave
		 * Turn buzzer on and wait for half-period
		 */
		MelOn;				   // Set buzzer pin high
		delay_us_Melody(time); // Wait for half-period

		/*
		 * Low portion of square wave
		 * Turn buzzer off and wait for half-period
		 */
		MelOff;				   // Set buzzer pin low
		delay_us_Melody(time); // Wait for half-period
	}

	/*
	 * Add small gap after note to improve clarity
	 * This prevents notes from blending together
	 */
	for (unsigned int i = 0; i < note_gap_us; i++)
	{
		_delay_us(1);
	}
}

/*
 * EDUCATIONAL FUNCTION: Initialize Buzzer
 *
 * PURPOSE: Configure buzzer pin as output
 * LEARNING: Shows hardware initialization for sound output
 */
void Buzzer_init(void)
{
	/*
	 * Configure buzzer pin as output
	 * DDRG controls direction of Port G pins
	 * Setting bit 4 makes PG4 an output pin
	 */
	SETBIT(DDRG, Buzzer_Pin); // Set buzzer pin as output
	MelOff;					  // Ensure buzzer starts in off state
}

/*
 * EDUCATIONAL FUNCTION: Set Buzzer Volume
 *
 * PURPOSE: Control buzzer volume through duty cycle
 * LEARNING: Shows pulse-width modulation concepts
 */
void Buzzer_set_volume(unsigned char volume)
{
	if (volume <= 100)
	{
		current_volume = volume;
	}
}

/*
 * EDUCATIONAL FUNCTION: Enable/Disable Buzzer
 *
 * PURPOSE: Global control of sound output
 * LEARNING: Shows system-level control features
 */
void Buzzer_enable(unsigned char enable)
{
	buzzer_enabled = enable;
	if (!enable)
	{
		MelOff; // Turn off buzzer immediately when disabled
	}
}

/*
 * EDUCATIONAL FUNCTION: Play Note by Frequency
 *
 * PURPOSE: Play note specified by frequency in Hz
 * LEARNING: Shows frequency-to-timing conversion
 */
void Play_Note_Hz(unsigned int frequency_hz, unsigned int duration_ms)
{
	unsigned int half_period_us;
	unsigned int cycles;

	if (frequency_hz == 0)
	{
		/* Rest/silence - just delay */
		for (unsigned int i = 0; i < duration_ms; i++)
		{
			_delay_ms(1);
		}
		return;
	}

	/* Calculate half-period in microseconds */
	half_period_us = 500000UL / frequency_hz;

	/* Calculate number of cycles for desired duration */
	cycles = (unsigned long)duration_ms * frequency_hz / 1000;

	/* Generate sound */
	Sound(cycles * half_period_us / 1000, half_period_us);
}

/*
 * EDUCATIONAL SOUND FUNCTIONS: Pre-defined Sound Effects
 * These demonstrate practical applications of sound generation
 * Each function creates a specific audio pattern for user feedback
 */

/*
 * SUCCESS SOUND: Indicates successful operation
 * Pattern: Low C followed by higher G (rising tone)
 * Use: Task completion, correct input, system ready
 */
void S_Good(void)
{
	Sound(BB, BDo); // Base C note
	Sound(BB, BSo); // Base G note (higher pitch)
}

/*
 * ERROR SOUND: Indicates error or invalid operation
 * Pattern: Short low C (single warning tone)
 * Use: Invalid input, system error, operation failed
 */
void SError(void)
{
	Sound(SB, LDo); // Short low C note
}

/*
 * BUTTON PRESS SOUND: Confirms user input
 * Pattern: High C followed by high E (quick confirmation)
 * Use: Button press feedback, menu selection
 */
void S_Push1(void)
{
	Sound(SB, HDo); // Short high C
	Sound(SB, HMi); // Short high E
}

/*
 * STARTUP SOUND: System initialization sequence
 * Pattern: Ascending scale from high C to very high C
 * Use: System boot, program start, initialization complete
 */
void S_Start(void)
{
	Sound(BB, HDo); // High C
	Sound(SB, HRe); // High D (short)
	Sound(BB, HMi); // High E
	Sound(SB, HFa); // High F (short)
	Sound(BB, HSo); // High G
	Sound(SB, HLa); // High A (short)
	Sound(BB, HSi); // High B
	Sound(SB, hDo); // Very high C (short)
}

/*
 * LEVEL 1 SOUND: Four-note ascending sequence
 * Pattern: Base C-E-G-High C (major chord arpeggio)
 * Use: Game level completion, achievement unlock
 */
void S_S1(void)
{
	Sound(BB, BDo); // Base C
	Sound(BB, BMi); // Base E
	Sound(BB, BSo); // Base G
	Sound(BB, HDo); // High C
}

/*
 * LEVEL 2 SOUND: Two high notes
 * Pattern: High C followed by high E
 * Use: Intermediate achievement, partial completion
 */
void S_S2(void)
{
	Sound(BB, HDo); // High C
	Sound(BB, HMi); // High E
}

/*
 * LEVEL 3 SOUND: Single note
 * Pattern: Low C (simple acknowledgment)
 * Use: Basic confirmation, simple feedback
 */
void S_S3(void)
{
	Sound(BB, LDo); // Low C
}

/*
 * LEVEL 4 SOUND: Three-note sequence
 * Pattern: Base C-E-G (major triad)
 * Use: Moderate achievement, progress indicator
 */
void S_S4(void)
{
	Sound(BB, BDo); // Base C
	Sound(BB, BMi); // Base E
	Sound(BB, BSo); // Base G
}

/*
 * LEVEL 5 SOUND: Two-note sequence
 * Pattern: Base C-E (major third)
 * Use: Minor achievement, step completion
 */
void S_S5(void)
{
	Sound(BB, BDo); // Base C
	Sound(BB, BMi); // Base E
}

/*
 * LEVEL 6 SOUND: Repeating pattern
 * Pattern: High G-C repeated 4 times (alarm-like)
 * Use: Warning, attention required, timer expiry
 */
void S_S6(void)
{
	unsigned char i;
	for (i = 0; i < 4; i++)
	{
		Sound(BB, HSo); // High G
		Sound(BB, HDo); // High C
	}
}

/*
 * LEVEL 7 SOUND: Octave jump
 * Pattern: Base C to High C (octave interval)
 * Use: Major achievement, level completion
 */
void S_S7(void)
{
	Sound(BB, BDo); // Base C
	Sound(BB, HDo); // High C (one octave higher)
}

/*
 * SPECIAL SOUND: "Twinkle Twinkle Little Star" opening
 * Pattern: Classic melody beginning (C-C-G-G-A-A-G)
 * Use: Special events, celebration, demo
 */
void S_Star(void)
{
	Sound(BB, BDo); // C - "Twin-"
	Sound(BB, BDo); // C - "kle"
	Sound(BB, BSo); // G - "twin-"
	Sound(BB, BSo); // G - "kle"
	Sound(BB, BLa); // A - "lit-"
	Sound(BB, BLa); // A - "tle"
	Sound(BB, BSo); // G - "star"
}

/*
 * EDUCATIONAL FUNCTION: Play Musical Scale
 *
 * PURPOSE: Demonstrate complete musical scale
 * LEARNING: Shows musical theory and note relationships
 */
void Play_Scale_Major(void)
{
	/* C Major Scale: C-D-E-F-G-A-B-C */
	Sound(BB, BDo); // C (Do)
	Sound(BB, BRe); // D (Re)
	Sound(BB, BMi); // E (Mi)
	Sound(BB, BFa); // F (Fa)
	Sound(BB, BSo); // G (So)
	Sound(BB, BLa); // A (La)
	Sound(BB, BSi); // B (Ti)
	Sound(BB, HDo); // C (Do) - octave higher
}

/*
 * EDUCATIONAL FUNCTION: Play Chord
 *
 * PURPOSE: Demonstrate harmony by playing notes in sequence
 * LEARNING: Shows musical harmony and chord construction
 */
void Play_Chord_C_Major(void)
{
	/* C Major Chord: C-E-G played as arpeggio */
	Sound(BB, BDo); // C (root)
	Sound(BB, BMi); // E (third)
	Sound(BB, BSo); // G (fifth)
	Sound(BB, HDo); // C (octave)
}

/*
 * EDUCATIONAL FUNCTION: Sound Test Sequence
 *
 * PURPOSE: Test all octaves and demonstrate frequency range
 * LEARNING: Shows complete frequency range of the system
 */
void Sound_Test_All_Octaves(void)
{
	/* Test low octave */
	Sound(SB, LDo);
	Sound(SB, LRe);
	Sound(SB, LMi);

	/* Test base octave */
	Sound(SB, BDo);
	Sound(SB, BRe);
	Sound(SB, BMi);

	/* Test high octave */
	Sound(SB, HDo);
	Sound(SB, HRe);
	Sound(SB, HMi);

	/* Test very high */
	Sound(SB, hDo);
}