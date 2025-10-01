/*
 * _buzzer.c - ATmega128 Buzzer/Sound Control Library
 * Educational Version for Assembly→C→Python Learning Progression
 *
 * LEARNING OBJECTIVES:
 * 1. Understand PWM (Pulse Width Modulation) concepts for sound generation
 * 2. Learn frequency vs musical note relationships
 * 3. Bridge digital timing concepts to audio output
 * 4. Understand bit manipulation for port control
 *
 * ASSEMBLY EQUIVALENT CONCEPTS:
 * - PORTG |= (1<<4)  ≡  SBI PORTG, 4  (set buzzer pin)
 * - PORTG &= ~(1<<4) ≡  CBI PORTG, 4  (clear buzzer pin)
 * - delay loops      ≡  Loop with NOP instructions
 */

#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "_main.h"
#include "_buzzer.h"

/*
 * Buzzer Hardware Configuration
 * Educational constants showing port/pin mapping
 */
#define BUZZER_PORT PORTG                                // Buzzer connected to Port G
#define BUZZER_PIN 4                                     // Pin 4 of Port G (PG4)
#define BUZZER_ON() (BUZZER_PORT |= (1 << BUZZER_PIN))   // Set pin HIGH
#define BUZZER_OFF() (BUZZER_PORT &= ~(1 << BUZZER_PIN)) // Set pin LOW

/*
 * Musical Note Frequencies (in Hz)
 * Simplified set for educational use - easier to remember
 */
#define NOTE_C4 262 // Middle C
#define NOTE_D4 294 // D
#define NOTE_E4 330 // E
#define NOTE_F4 349 // F
#define NOTE_G4 392 // G
#define NOTE_A4 440 // A (concert pitch)
#define NOTE_B4 494 // B
#define NOTE_C5 523 // High C

// Lower octave notes
#define NOTE_C3 131 // Low C
#define NOTE_D3 147 // Low D
#define NOTE_E3 165 // Low E
#define NOTE_F3 175 // Low F
#define NOTE_G3 196 // Low G
#define NOTE_A3 220 // Low A
#define NOTE_B3 247 // Low B

// Timing constants for rhythm
#define WHOLE_NOTE 2000    // 2 seconds
#define HALF_NOTE 1000     // 1 second
#define QUARTER_NOTE 500   // 0.5 seconds
#define EIGHTH_NOTE 250    // 0.25 seconds
#define SIXTEENTH_NOTE 125 // 0.125 seconds

/*
 * Buzzer_init() - Initialize buzzer pin for output
 *
 * EDUCATIONAL NOTES:
 * - Buzzer requires output pin configuration
 * - Pin must be set as OUTPUT in DDR register
 * - Initial state should be OFF (LOW voltage)
 */
void Buzzer_init(void)
{
    // Configure buzzer pin as output
    // Assembly equivalent: SBI DDRG, 4
    DDRG |= (1 << BUZZER_PIN); // Set PG4 as output pin

    // Initialize buzzer to OFF state
    BUZZER_OFF(); // Ensure buzzer starts silent
}

/*
 * delay_us() - Microsecond delay function for precise timing
 * Educational function showing custom delay implementation
 */
void delay_us(unsigned int microseconds)
{
    // Use millisecond delays for simplicity in educational version
    if (microseconds >= 1000)
    {
        _delay_ms(microseconds / 1000);
    }
    else if (microseconds >= 100)
    {
        // For delays >= 100us, use a simple loop
        volatile unsigned int i;
        for (i = 0; i < microseconds * 4; i++)
        {
            asm volatile("nop");
        }
    }
    else
    {
        // For very short delays, just use NOPs
        volatile unsigned int i;
        for (i = 0; i < microseconds; i++)
        {
            asm volatile("nop");
        }
    }
}

/*
 * Buzzer_tone() - Generate tone at specified frequency for specified duration
 *
 * EDUCATIONAL NOTES:
 * - Frequency determines pitch (Hz = cycles per second)
 * - Duration determines how long the tone plays (milliseconds)
 * - Square wave generation: ON-delay-OFF-delay repeat
 * - Period = 1/frequency, half_period = delay between ON/OFF
 *
 * MATH CONCEPTS:
 * - 440Hz tone needs 440 complete cycles per second
 * - Each cycle = HIGH time + LOW time
 * - For 50% duty cycle: HIGH = LOW = 1/(2*frequency) seconds
 */
void Buzzer_tone(unsigned int frequency, unsigned int duration_ms)
{
    unsigned int half_period_us; // Half period in microseconds
    unsigned int cycles;         // Number of cycles to generate
    unsigned int i;

    // Calculate timing for specified frequency
    if (frequency == 0)
    {
        // Silence - just delay
        _delay_ms(duration_ms);
        return;
    }

    // Calculate half period in microseconds
    // half_period = (1 / frequency) / 2 * 1,000,000
    half_period_us = 500000UL / frequency; // 500,000 = 1,000,000 / 2

    // Calculate number of cycles needed for specified duration
    // cycles = frequency * (duration_ms / 1000)
    cycles = (frequency * duration_ms) / 1000;

    // Generate square wave for calculated number of cycles
    for (i = 0; i < cycles; i++)
    {
        BUZZER_ON();              // Turn buzzer ON
        delay_us(half_period_us); // Wait half period
        BUZZER_OFF();             // Turn buzzer OFF
        delay_us(half_period_us); // Wait half period
    }
}

/*
 * Educational Helper Functions
 * These simplify music creation for students
 */

/*
 * Buzzer_beep() - Simple beep sound for notifications
 */
void Buzzer_beep(void)
{
    Buzzer_tone(1000, 200); // 1kHz for 200ms
}

/*
 * Buzzer_long_beep() - Longer beep for alarms
 */
void Buzzer_long_beep(void)
{
    Buzzer_tone(800, 1000); // 800Hz for 1 second
}

/*
 * Buzzer_double_beep() - Two quick beeps
 */
void Buzzer_double_beep(void)
{
    Buzzer_tone(1200, 150); // First beep
    _delay_ms(100);         // Pause
    Buzzer_tone(1200, 150); // Second beep
}

/*
 * Buzzer_ascending() - Ascending tone sequence
 */
void Buzzer_ascending(void)
{
    Buzzer_tone(NOTE_C4, QUARTER_NOTE);
    Buzzer_tone(NOTE_E4, QUARTER_NOTE);
    Buzzer_tone(NOTE_G4, QUARTER_NOTE);
    Buzzer_tone(NOTE_C5, QUARTER_NOTE);
}

/*
 * Buzzer_descending() - Descending tone sequence
 */
void Buzzer_descending(void)
{
    Buzzer_tone(NOTE_C5, QUARTER_NOTE);
    Buzzer_tone(NOTE_G4, QUARTER_NOTE);
    Buzzer_tone(NOTE_E4, QUARTER_NOTE);
    Buzzer_tone(NOTE_C4, QUARTER_NOTE);
}

/*
 * Buzzer_scale() - Play C major scale for education
 */
void Buzzer_scale(void)
{
    Buzzer_tone(NOTE_C4, QUARTER_NOTE);
    Buzzer_tone(NOTE_D4, QUARTER_NOTE);
    Buzzer_tone(NOTE_E4, QUARTER_NOTE);
    Buzzer_tone(NOTE_F4, QUARTER_NOTE);
    Buzzer_tone(NOTE_G4, QUARTER_NOTE);
    Buzzer_tone(NOTE_A4, QUARTER_NOTE);
    Buzzer_tone(NOTE_B4, QUARTER_NOTE);
    Buzzer_tone(NOTE_C5, QUARTER_NOTE);
}

/*
 * Buzzer_play_melody() - Play simple melody from array
 * Educational function showing array processing
 */
void Buzzer_play_melody(unsigned int *frequencies, unsigned int *durations, unsigned char length)
{
    unsigned char i;

    for (i = 0; i < length; i++)
    {
        Buzzer_tone(frequencies[i], durations[i]);
        _delay_ms(50); // Small pause between notes
    }
}

/*
 * Buzzer_off() - Ensure buzzer is off
 * Safety function for educational use
 */
void Buzzer_off(void)
{
    BUZZER_OFF(); // Turn off buzzer immediately
}

/*
 * Educational Frequency Functions
 * These help students understand frequency-to-note conversion
 */

/*
 * Buzzer_frequency_sweep() - Sweep through frequency range
 * Demonstrates frequency vs pitch relationship
 */
void Buzzer_frequency_sweep(unsigned int start_freq, unsigned int end_freq, unsigned int step)
{
    unsigned int freq;

    for (freq = start_freq; freq <= end_freq; freq += step)
    {
        Buzzer_tone(freq, 100); // 100ms per frequency
    }
}

/*
 * Buzzer_morse_dot() - Morse code dot
 */
void Buzzer_morse_dot(void)
{
    Buzzer_tone(800, 200); // Short beep
    _delay_ms(200);        // Pause
}

/*
 * Buzzer_morse_dash() - Morse code dash
 */
void Buzzer_morse_dash(void)
{
    Buzzer_tone(800, 600); // Long beep
    _delay_ms(200);        // Pause
}

/*
 * EDUCATIONAL PROGRESSION NOTES:
 *
 * 1. ASSEMBLY LEVEL (Direct Pin Control):
 *    Students learn: SBI PORTG, 4; delay; CBI PORTG, 4; delay
 *    Understanding direct pin manipulation and timing loops
 *
 * 2. C ABSTRACTION LEVEL (This Library):
 *    Students learn: Buzzer_tone(440, 1000); Buzzer_beep();
 *    Understanding function parameters and frequency concepts
 *
 * 3. PYTHON LEVEL (High-Level Interface):
 *    Students learn: atmega.buzzer.play_note("A4", 1.0)
 *    Understanding musical notation and duration specification
 *
 * PROGRESSION TOPICS:
 * - Digital to analog concepts (PWM, frequency)
 * - Musical theory (frequencies, notes, scales)
 * - Timing and rhythm programming
 * - Array processing for melodies
 * - Audio signal generation principles
 */