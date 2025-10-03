/*
 * Sound Basic - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Understand PWM for sound generation
 * - Learn buzzer control techniques
 * - Practice frequency calculation and timing
 * - Master audio waveform generation
 *
 * HARDWARE SETUP:
 * - Buzzer connected to OC2 pin (PB7) for PWM sound
 * - LEDs on PORTB for visual feedback
 * - Optional: Speaker amplifier for better audio quality
 */

#include "config.h"

// Musical note frequencies (in Hz)
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_REST 0

// Function to generate tone using PWM
void generate_tone(uint16_t frequency, uint16_t duration_ms)
{
    if (frequency == 0)
    {
        // Rest note - no sound
        Buzzer_off();
        _delay_ms(duration_ms);
        return;
    }

    // Configure Timer2 for PWM sound generation
    // Formula: frequency = F_CPU / (2 * prescaler * (OCR2 + 1))
    uint8_t ocr_value = (F_CPU / (2 * 256 * frequency)) - 1;

    // Set compare value for desired frequency
    OCR2 = ocr_value;

    // Enable PWM output
    Buzzer_on();

    // Play tone for specified duration
    _delay_ms(duration_ms);

    // Turn off sound
    Buzzer_off();
}

// Function to play a simple melody
void play_melody()
{
    // Simple melody: C-D-E-F-G-A-B-C
    uint16_t melody[] = {NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4,
                         NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5};
    uint16_t note_duration = 500; // 500ms per note

    puts_USART1("Playing melody: C-D-E-F-G-A-B-C\r\n");

    for (uint8_t i = 0; i < 8; i++)
    {
        // Visual feedback - light up LEDs
        PORTB = ~(1 << i); // Turn on corresponding LED

        // Play note
        generate_tone(melody[i], note_duration);

        // Brief pause between notes
        _delay_ms(100);

        char buffer[30];
        sprintf(buffer, "Note %u: %u Hz\r\n", i + 1, melody[i]);
        puts_USART1(buffer);
    }

    PORTB = 0xFF; // Turn off all LEDs
}

// Function to demonstrate different sound patterns
void sound_patterns()
{
    puts_USART1("Sound Pattern 1: Ascending chirp\r\n");

    // Ascending frequency sweep
    for (uint16_t freq = 200; freq <= 1000; freq += 50)
    {
        generate_tone(freq, 100);
        _delay_ms(50);
    }

    _delay_ms(500);

    puts_USART1("Sound Pattern 2: Alarm pattern\r\n");

    // Alarm pattern
    for (uint8_t i = 0; i < 5; i++)
    {
        generate_tone(800, 200); // High tone
        _delay_ms(100);
        generate_tone(400, 200); // Low tone
        _delay_ms(100);
    }

    _delay_ms(500);

    puts_USART1("Sound Pattern 3: Beep sequence\r\n");

    // Rapid beep sequence
    for (uint8_t i = 0; i < 10; i++)
    {
        generate_tone(600, 100);
        _delay_ms(100);
    }
}

int main(void)
{
    // Initialize system components
    init_devices(); // Initialize all peripherals

    // Initialize buzzer system
    Buzzer_init(); // Configure buzzer PWM

    // Initialize UART for sound reports
    Uart1_init();
    puts_USART1("Sound Basic Demo Started\r\n");
    puts_USART1("Demonstrating PWM sound generation\r\n");

    // Configure LED output for visual feedback
    DDRB = 0xFF;  // Set PORTB as output for LEDs
    PORTB = 0xFF; // Turn off all LEDs initially

    uint8_t demo_mode = 0;

    while (1)
    {
        switch (demo_mode)
        {
        case 0:
            puts_USART1("\r\n=== Demo 1: Basic Tones ===\r\n");

            // Play individual notes
            puts_USART1("Playing individual notes...\r\n");
            generate_tone(NOTE_C4, 500);
            _delay_ms(200);
            generate_tone(NOTE_E4, 500);
            _delay_ms(200);
            generate_tone(NOTE_G4, 500);
            _delay_ms(200);
            generate_tone(NOTE_C5, 1000);
            _delay_ms(500);
            break;

        case 1:
            puts_USART1("\r\n=== Demo 2: Melody ===\r\n");
            play_melody();
            _delay_ms(1000);
            break;

        case 2:
            puts_USART1("\r\n=== Demo 3: Sound Patterns ===\r\n");
            sound_patterns();
            _delay_ms(1000);
            break;

        case 3:
            puts_USART1("\r\n=== Demo 4: Frequency Test ===\r\n");

            // Test different frequencies
            uint16_t test_frequencies[] = {100, 200, 440, 880, 1760};

            for (uint8_t i = 0; i < 5; i++)
            {
                char buffer[40];
                sprintf(buffer, "Testing %u Hz\r\n", test_frequencies[i]);
                puts_USART1(buffer);

                generate_tone(test_frequencies[i], 800);
                _delay_ms(300);
            }
            _delay_ms(1000);
            break;
        }

        demo_mode = (demo_mode + 1) % 4; // Cycle through demos

        // Pause between demo cycles
        puts_USART1("Waiting 3 seconds before next demo...\r\n");
        _delay_ms(3000);
    }

    return 0;
}