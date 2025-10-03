/*
 * Sound Atari - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Learn retro game sound programming techniques
 * - Understand sound effect synthesis and modulation
 * - Practice creating procedural audio effects
 * - Master vintage gaming audio aesthetics
 *
 * HARDWARE SETUP:
 * - Buzzer connected to OC2 pin (PB7) for PWM sound
 * - LEDs on PORTB for retro gaming light effects
 * - Optional: Arcade-style buttons for sound triggering
 */

#include "config.h"

// Function to generate tone with envelope control
void generate_tone_envelope(uint16_t start_freq, uint16_t end_freq,
                            uint16_t duration_ms, uint8_t attack_time)
{
    uint16_t steps = duration_ms / 10; // 10ms per step
    uint16_t freq_step = (end_freq > start_freq) ? (end_freq - start_freq) / steps : (start_freq - end_freq) / steps;

    uint16_t current_freq = start_freq;

    for (uint16_t i = 0; i < steps; i++)
    {
        // Calculate frequency progression
        if (end_freq > start_freq)
            current_freq = start_freq + (freq_step * i);
        else
            current_freq = start_freq - (freq_step * i);

        // Configure PWM for current frequency
        if (current_freq > 0 && current_freq < 20000)
        {
            uint8_t ocr_value = (F_CPU / (2 * 256 * current_freq)) - 1;
            OCR2 = ocr_value;
            Buzzer_on();
        }
        else
        {
            Buzzer_off();
        }

        _delay_ms(10);
    }

    Buzzer_off();
}

// Classic Atari sound effects
void sound_effect_coin()
{
    puts_USART1("♪ COIN COLLECTED! ♪\r\n");

    // Bright ascending chime
    PORTB = 0x00; // All LEDs on
    generate_tone_envelope(400, 800, 300, 50);
    _delay_ms(100);
    generate_tone_envelope(600, 1200, 200, 30);
    PORTB = 0xFF; // LEDs off
}

void sound_effect_jump()
{
    puts_USART1("♪ JUMP! ♪\r\n");

    // Quick rising bloop
    PORTB = ~(1 << 0); // LED 0 on
    generate_tone_envelope(200, 600, 150, 20);
    PORTB = 0xFF;
}

void sound_effect_laser()
{
    puts_USART1("♪ PEW PEW! ♪\r\n");

    // High to low sweep with noise
    for (uint8_t i = 0; i < 8; i++)
    {
        PORTB = ~(1 << i); // Sequential LED activation
        generate_tone_envelope(1500 - (i * 150), 1500 - ((i + 1) * 150), 50, 10);
        _delay_ms(20);
    }
    PORTB = 0xFF;
}

void sound_effect_explosion()
{
    puts_USART1("♪ BOOM! ♪\r\n");

    // Chaotic noise burst
    for (uint8_t i = 0; i < 20; i++)
    {
        PORTB = (i % 2) ? 0x00 : 0xFF; // Rapid flashing

        // Random-ish frequency generation
        uint16_t noise_freq = 100 + (i * 50) + ((i * 7) % 300);
        generate_tone_envelope(noise_freq, noise_freq / 2, 50, 5);
        _delay_ms(30);
    }
    PORTB = 0xFF;
}

void sound_effect_powerup()
{
    puts_USART1("♪ POWER UP! ♪\r\n");

    // Magical ascending arpeggio
    uint16_t arp_notes[] = {262, 330, 392, 523, 659, 784, 1047}; // C major arpeggio

    for (uint8_t i = 0; i < 7; i++)
    {
        PORTB = ~(1 << i); // Progressive LED lighting
        generate_tone_envelope(arp_notes[i], arp_notes[i], 150, 30);
        _delay_ms(50);
    }

    // Hold final chord
    PORTB = 0x00; // All LEDs on
    generate_tone_envelope(1047, 1047, 300, 100);
    PORTB = 0xFF;
}

void sound_effect_death()
{
    puts_USART1("♪ GAME OVER ♪\r\n");

    // Sad descending sequence
    uint16_t death_notes[] = {523, 440, 392, 330, 262, 196, 147};

    for (uint8_t i = 0; i < 7; i++)
    {
        PORTB = ~(0xFF << i); // Progressive LED dimming
        generate_tone_envelope(death_notes[i], death_notes[i], 200, 50);
        _delay_ms(100);
    }

    // Final dramatic pause
    PORTB = 0xFF;
    _delay_ms(500);
}

void sound_effect_bonus()
{
    puts_USART1("♪ BONUS POINTS! ♪\r\n");

    // Cheerful bouncing melody
    uint16_t bonus_melody[] = {523, 659, 784, 1047, 784, 659, 523, 784};

    for (uint8_t i = 0; i < 8; i++)
    {
        PORTB = ~(1 << (i % 4)); // Bouncing LED pattern
        generate_tone_envelope(bonus_melody[i], bonus_melody[i], 120, 20);
        _delay_ms(80);
    }
    PORTB = 0xFF;
}

void retro_startup_sequence()
{
    puts_USART1("♪ RETRO GAMING SYSTEM BOOT ♪\r\n");

    // Classic system boot sound
    for (uint8_t i = 0; i < 8; i++)
    {
        PORTB = ~(1 << i);
        generate_tone_envelope(100 + (i * 100), 100 + (i * 100), 100, 20);
        _delay_ms(50);
    }

    PORTB = 0x00; // All LEDs on
    generate_tone_envelope(800, 800, 500, 100);
    PORTB = 0xFF;

    puts_USART1("SYSTEM READY!\r\n");
}

int main(void)
{
    // Initialize system components
    init_devices(); // Initialize all peripherals

    // Initialize buzzer system
    Buzzer_init(); // Configure buzzer PWM

    // Initialize UART for retro gaming reports
    Uart1_init();
    puts_USART1("Sound Atari Demo Started\r\n");
    puts_USART1("Retro Gaming Sound Effects Laboratory\r\n");
    puts_USART1("Demonstrating classic 8-bit audio synthesis\r\n");

    // Configure LED output for retro light effects
    DDRB = 0xFF;  // Set PORTB as output for LEDs
    PORTB = 0xFF; // Turn off all LEDs initially

    // Boot sequence
    retro_startup_sequence();
    _delay_ms(2000);

    uint8_t effect_index = 0;
    uint8_t game_cycle = 0;

    while (1)
    {
        game_cycle++;

        char buffer[60];
        sprintf(buffer, "\r\n=== Game Cycle #%u ===\r\n", game_cycle);
        puts_USART1(buffer);

        // Cycle through different sound effects
        switch (effect_index)
        {
        case 0:
            sound_effect_jump();
            _delay_ms(800);
            sound_effect_coin();
            break;

        case 1:
            sound_effect_laser();
            _delay_ms(500);
            sound_effect_laser();
            _delay_ms(500);
            sound_effect_explosion();
            break;

        case 2:
            sound_effect_powerup();
            _delay_ms(1000);
            sound_effect_bonus();
            break;

        case 3:
            // Game over sequence
            sound_effect_death();
            _delay_ms(1000);
            puts_USART1("INSERT COIN TO CONTINUE\r\n");

            // Coin inserted simulation
            _delay_ms(2000);
            sound_effect_coin();
            retro_startup_sequence();
            break;
        }

        effect_index = (effect_index + 1) % 4; // Cycle through demos

        // Interactive pause
        puts_USART1("Waiting 3 seconds before next sequence...\r\n");
        _delay_ms(3000);

        // Special sequence every 5th cycle
        if (game_cycle % 5 == 0)
        {
            puts_USART1("\r\n=== SPECIAL COMBO SEQUENCE ===\r\n");

            // Rapid-fire combo
            for (uint8_t i = 0; i < 5; i++)
            {
                sound_effect_laser();
                _delay_ms(200);
            }
            sound_effect_explosion();
            _delay_ms(500);
            sound_effect_powerup();

            puts_USART1("COMBO BONUS!\r\n");
            _delay_ms(1500);
        }
    }

    return 0;
}