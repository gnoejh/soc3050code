/*
 * Sound Twinkle - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Practice musical programming and timing
 * - Learn melody data structures and playback
 * - Understand rhythm and tempo control
 * - Master complex audio sequences
 *
 * HARDWARE SETUP:
 * - Buzzer connected to OC2 pin (PB7) for PWM sound
 * - LEDs on PORTB for musical visualization
 * - Optional: Sheet music display for educational reference
 */

#include "config.h"

// Musical note frequencies (in Hz) - Concert pitch
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_G5 784
#define NOTE_REST 0

// Note durations (in ms)
#define QUARTER_NOTE 500
#define HALF_NOTE 1000
#define WHOLE_NOTE 2000

// Structure for musical notes
typedef struct
{
    uint16_t frequency;
    uint16_t duration;
} musical_note_t;

// Function to generate tone using PWM
void play_note(uint16_t frequency, uint16_t duration_ms)
{
    if (frequency == 0)
    {
        // Rest note - silence
        Buzzer_off();
        _delay_ms(duration_ms);
        return;
    }

    // Configure Timer2 for PWM sound generation
    uint8_t ocr_value = (F_CPU / (2 * 256 * frequency)) - 1;
    OCR2 = ocr_value;

    // Enable PWM output for sound
    Buzzer_on();

    // Play note for specified duration
    _delay_ms(duration_ms);

    // Brief pause between notes for clarity
    Buzzer_off();
    _delay_ms(50);
}

// Function to create visual effects during music
void music_visualization(uint8_t note_index)
{
    // Create different LED patterns for different parts of the song
    if (note_index < 8)
    {
        // First line - ascending pattern
        PORTB = ~(1 << (note_index % 8));
    }
    else if (note_index < 16)
    {
        // Second line - descending pattern
        PORTB = ~(1 << (7 - (note_index % 8)));
    }
    else
    {
        // Final lines - alternating pattern
        PORTB = (note_index % 2) ? 0x55 : 0xAA;
    }
}

// Twinkle Twinkle Little Star melody
void play_twinkle_twinkle()
{
    puts_USART1("Playing 'Twinkle Twinkle Little Star'\r\n");
    puts_USART1("♪ Twinkle, twinkle, little star ♪\r\n");

    // Define the melody - "Twinkle Twinkle Little Star"
    musical_note_t melody[] = {
        // "Twinkle, twinkle, little star"
        {NOTE_C4, QUARTER_NOTE},
        {NOTE_C4, QUARTER_NOTE}, // Twin-kle
        {NOTE_G4, QUARTER_NOTE},
        {NOTE_G4, QUARTER_NOTE}, // twin-kle
        {NOTE_A4, QUARTER_NOTE},
        {NOTE_A4, QUARTER_NOTE}, // lit-tle
        {NOTE_G4, HALF_NOTE},    // star

        // "How I wonder what you are"
        {NOTE_F4, QUARTER_NOTE},
        {NOTE_F4, QUARTER_NOTE}, // How I
        {NOTE_E4, QUARTER_NOTE},
        {NOTE_E4, QUARTER_NOTE}, // won-der
        {NOTE_D4, QUARTER_NOTE},
        {NOTE_D4, QUARTER_NOTE}, // what you
        {NOTE_C4, HALF_NOTE},    // are

        // "Up above the world so high"
        {NOTE_G4, QUARTER_NOTE},
        {NOTE_G4, QUARTER_NOTE}, // Up a-
        {NOTE_F4, QUARTER_NOTE},
        {NOTE_F4, QUARTER_NOTE}, // bove the
        {NOTE_E4, QUARTER_NOTE},
        {NOTE_E4, QUARTER_NOTE}, // world so
        {NOTE_D4, HALF_NOTE},    // high

        // "Like a diamond in the sky"
        {NOTE_G4, QUARTER_NOTE},
        {NOTE_G4, QUARTER_NOTE}, // Like a
        {NOTE_F4, QUARTER_NOTE},
        {NOTE_F4, QUARTER_NOTE}, // dia-mond
        {NOTE_E4, QUARTER_NOTE},
        {NOTE_E4, QUARTER_NOTE}, // in the
        {NOTE_D4, HALF_NOTE},    // sky

        // "Twinkle, twinkle, little star" (repeat)
        {NOTE_C4, QUARTER_NOTE},
        {NOTE_C4, QUARTER_NOTE}, // Twin-kle
        {NOTE_G4, QUARTER_NOTE},
        {NOTE_G4, QUARTER_NOTE}, // twin-kle
        {NOTE_A4, QUARTER_NOTE},
        {NOTE_A4, QUARTER_NOTE}, // lit-tle
        {NOTE_G4, HALF_NOTE},    // star

        // "How I wonder what you are" (repeat)
        {NOTE_F4, QUARTER_NOTE},
        {NOTE_F4, QUARTER_NOTE}, // How I
        {NOTE_E4, QUARTER_NOTE},
        {NOTE_E4, QUARTER_NOTE}, // won-der
        {NOTE_D4, QUARTER_NOTE},
        {NOTE_D4, QUARTER_NOTE}, // what you
        {NOTE_C4, WHOLE_NOTE},   // are (final)
    };

    uint8_t melody_length = sizeof(melody) / sizeof(musical_note_t);

    // Play the complete melody
    for (uint8_t i = 0; i < melody_length; i++)
    {
        // Create visual effects
        music_visualization(i);

        // Play the note
        play_note(melody[i].frequency, melody[i].duration);

        // Report progress
        if (i % 4 == 0) // Every 4 notes (one line)
        {
            char buffer[30];
            sprintf(buffer, "Measure %u/8 completed\r\n", (i / 4) + 1);
            puts_USART1(buffer);
        }
    }

    // Turn off all LEDs after song
    PORTB = 0xFF;
    puts_USART1("♪ Song completed! ♪\r\n");
}

// Function to play melody with different tempos
void play_different_tempos()
{
    puts_USART1("Demonstrating different tempos:\r\n");

    // Simple melody for tempo demonstration
    musical_note_t simple_melody[] = {
        {NOTE_C4, QUARTER_NOTE}, {NOTE_E4, QUARTER_NOTE}, {NOTE_G4, QUARTER_NOTE}, {NOTE_C5, QUARTER_NOTE}};

    uint8_t melody_length = 4;

    // Play at different speeds
    float tempo_multipliers[] = {2.0, 1.0, 0.5}; // Fast, normal, slow
    const char *tempo_names[] = {"Fast", "Normal", "Slow"};

    for (uint8_t tempo = 0; tempo < 3; tempo++)
    {
        char buffer[40];
        sprintf(buffer, "Playing at %s tempo\r\n", tempo_names[tempo]);
        puts_USART1(buffer);

        for (uint8_t i = 0; i < melody_length; i++)
        {
            PORTB = ~(1 << i); // Visual feedback

            uint16_t adjusted_duration = simple_melody[i].duration * tempo_multipliers[tempo];
            play_note(simple_melody[i].frequency, adjusted_duration);
        }

        PORTB = 0xFF;    // Turn off LEDs
        _delay_ms(1000); // Pause between tempos
    }
}

int main(void)
{
    // Initialize system components
    init_devices(); // Initialize all peripherals

    // Initialize buzzer system
    Buzzer_init(); // Configure buzzer PWM

    // Initialize UART for musical reports
    Uart1_init();
    puts_USART1("Sound Twinkle Demo Started\r\n");
    puts_USART1("Musical Education: Twinkle Twinkle Little Star\r\n");
    puts_USART1("Demonstrating melody, rhythm, and tempo\r\n");

    // Configure LED output for musical visualization
    DDRB = 0xFF;  // Set PORTB as output for LEDs
    PORTB = 0xFF; // Turn off all LEDs initially

    uint8_t performance_count = 0;

    while (1)
    {
        performance_count++;

        char buffer[50];
        sprintf(buffer, "\r\n=== Performance #%u ===\r\n", performance_count);
        puts_USART1(buffer);

        // Main performance
        play_twinkle_twinkle();

        _delay_ms(2000); // Pause between performances

        // Demonstrate tempo variations every 3rd performance
        if (performance_count % 3 == 0)
        {
            puts_USART1("\r\n=== Tempo Demonstration ===\r\n");
            play_different_tempos();
            _delay_ms(2000);
        }

        // Wait before next performance
        puts_USART1("Waiting 5 seconds before next performance...\r\n");
        _delay_ms(5000);
    }

    return 0;
}