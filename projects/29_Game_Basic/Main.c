/*
 * Game Basic - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Create a simple functional game from scratch
 * - Learn game mechanics and player interaction
 * - Practice real-time input handling and response
 * - Master basic game scoring and progression
 *
 * HARDWARE SETUP:
 * - LEDs on PORTB for game display (8-bit game)
 * - UART for game control and status
 * - Buzzer for sound effects
 * - Optional: Buttons for physical input
 */

#include "config.h"

// Simple LED-based game configuration
#define GAME_WIDTH 8
#define GAME_HEIGHT 1 // Single row of LEDs
#define MAX_ENEMIES 3

// Game structures
typedef struct
{
    uint8_t position; // 0-7 for LED positions
    uint8_t health;
    uint16_t score;
    uint8_t power_level;
} player_t;

typedef struct
{
    uint8_t position;
    uint8_t active;
    uint8_t speed;
} enemy_t;

// Global game variables
player_t player;
enemy_t enemies[MAX_ENEMIES];
uint16_t game_time = 0;
uint8_t game_level = 1;
uint8_t game_active = 1;

// Function to initialize the game
void init_basic_game()
{
    puts_USART1("Initializing Basic LED Game...\r\n");

    // Initialize player
    player.position = 3; // Start in middle
    player.health = 3;
    player.score = 0;
    player.power_level = 1;

    // Initialize enemies
    for (uint8_t i = 0; i < MAX_ENEMIES; i++)
    {
        enemies[i].active = 0;
        enemies[i].position = 0;
        enemies[i].speed = 1;
    }

    game_time = 0;
    game_level = 1;
    game_active = 1;

    puts_USART1("Basic Game Ready!\r\n");
    puts_USART1("Controls: A=Left, D=Right, S=Fire, R=Restart\r\n");
}

// Function to display game state on LEDs
void display_game()
{
    uint8_t led_pattern = 0xFF; // Start with all LEDs off (active low)

    // Show player position
    led_pattern &= ~(1 << player.position); // Turn on player LED

    // Show enemies
    for (uint8_t i = 0; i < MAX_ENEMIES; i++)
    {
        if (enemies[i].active)
        {
            // Blink enemy LEDs
            if ((game_time / 5) % 2)
            {
                led_pattern &= ~(1 << enemies[i].position);
            }
        }
    }

    // Special effects
    if (player.power_level > 1)
    {
        // Power-up effect - rapid blinking
        if ((game_time / 2) % 2)
        {
            led_pattern = 0x00; // All LEDs on
        }
    }

    PORTB = led_pattern;
}

// Function to handle player input
void handle_input()
{
    if (is_USART1_received())
    {
        char input = get_USART1();

        switch (input)
        {
        case 'a':
        case 'A': // Move left
            if (player.position > 0)
            {
                player.position--;
                puts_USART1("Player moved left\r\n");
            }
            break;

        case 'd':
        case 'D': // Move right
            if (player.position < GAME_WIDTH - 1)
            {
                player.position++;
                puts_USART1("Player moved right\r\n");
            }
            break;

        case 's':
        case 'S': // Fire/Attack
            puts_USART1("Player fires!\r\n");

            // Check for enemies at player position
            for (uint8_t i = 0; i < MAX_ENEMIES; i++)
            {
                if (enemies[i].active && enemies[i].position == player.position)
                {
                    enemies[i].active = 0; // Destroy enemy
                    player.score += 10 * game_level;

                    char buffer[30];
                    sprintf(buffer, "Enemy destroyed! Score: %u\r\n", player.score);
                    puts_USART1(buffer);

                    // Sound effect
                    Buzzer_on();
                    _delay_ms(100);
                    Buzzer_off();
                }
            }
            break;

        case 'r':
        case 'R': // Restart
            puts_USART1("Restarting game...\r\n");
            init_basic_game();
            break;

        case 'p':
        case 'P': // Pause
            game_active = !game_active;
            puts_USART1(game_active ? "Game resumed\r\n" : "Game paused\r\n");
            break;
        }
    }
}

// Function to spawn enemies
void spawn_enemies()
{
    // Spawn new enemy every 50 game cycles
    if (game_time % 50 == 0)
    {
        for (uint8_t i = 0; i < MAX_ENEMIES; i++)
        {
            if (!enemies[i].active)
            {
                enemies[i].position = (game_time / 10) % GAME_WIDTH; // Random position
                enemies[i].active = 1;
                enemies[i].speed = game_level;

                puts_USART1("New enemy spawned!\r\n");
                break;
            }
        }
    }
}

// Function to update enemies
void update_enemies()
{
    for (uint8_t i = 0; i < MAX_ENEMIES; i++)
    {
        if (enemies[i].active)
        {
            // Move enemy (simple movement pattern)
            if (game_time % (10 / enemies[i].speed) == 0)
            {
                enemies[i].position = (enemies[i].position + 1) % GAME_WIDTH;
            }

            // Check collision with player
            if (enemies[i].position == player.position)
            {
                enemies[i].active = 0; // Remove enemy
                player.health--;

                char buffer[30];
                sprintf(buffer, "Hit! Health remaining: %u\r\n", player.health);
                puts_USART1(buffer);

                // Damage sound
                for (uint8_t j = 0; j < 3; j++)
                {
                    Buzzer_on();
                    _delay_ms(50);
                    Buzzer_off();
                    _delay_ms(50);
                }

                // Check game over
                if (player.health == 0)
                {
                    puts_USART1("GAME OVER!\r\n");
                    char final_score[40];
                    sprintf(final_score, "Final Score: %u\r\n", player.score);
                    puts_USART1(final_score);
                    puts_USART1("Press R to restart\r\n");
                    game_active = 0;
                }
            }
        }
    }
}

// Function to check level progression
void check_level_up()
{
    // Level up every 100 points
    uint8_t new_level = (player.score / 100) + 1;
    if (new_level > game_level)
    {
        game_level = new_level;
        player.power_level = game_level;

        char buffer[40];
        sprintf(buffer, "LEVEL UP! Now level %u\r\n", game_level);
        puts_USART1(buffer);

        // Level up celebration
        for (uint8_t i = 0; i < 5; i++)
        {
            PORTB = 0x00; // All LEDs on
            Buzzer_on();
            _delay_ms(100);
            PORTB = 0xFF; // All LEDs off
            Buzzer_off();
            _delay_ms(100);
        }
    }
}

// Function to display game status
void display_status()
{
    if (game_time % 100 == 0) // Every 10 seconds
    {
        char status[80];
        sprintf(status, "Time: %u, Score: %u, Level: %u, Health: %u\r\n",
                game_time / 10, player.score, game_level, player.health);
        puts_USART1(status);
    }
}

int main(void)
{
    // Initialize system components
    init_devices();
    Uart1_init();
    Buzzer_init();

    puts_USART1("Basic LED Game Starting...\r\n");
    puts_USART1("Simple arcade-style game using LEDs\r\n");
    puts_USART1("Goal: Avoid and destroy enemies, survive as long as possible\r\n");

    DDRB = 0xFF;  // Set PORTB as output for LEDs
    PORTB = 0xFF; // Turn off all LEDs initially

    // Initialize the game
    init_basic_game();

    puts_USART1("Game started! Good luck!\r\n");

    while (1)
    {
        if (game_active)
        {
            game_time++;

            // Handle player input
            handle_input();

            // Game logic
            spawn_enemies();
            update_enemies();
            check_level_up();

            // Display current game state
            display_game();
            display_status();
        }
        else
        {
            // Game paused or over - just handle input
            handle_input();
        }

        // Game timing - 10 FPS
        _delay_ms(100);
    }

    return 0;
}