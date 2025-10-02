/*
 * Game Obstacle - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Create a fast-paced obstacle avoidance game
 * - Learn real-time collision detection and response
 * - Practice dynamic object generation and movement
 * - Master timing and difficulty progression
 *
 * HARDWARE SETUP:
 * - LEDs on PORTB for game field visualization
 * - UART for game control and status display
 * - Buzzer for collision and scoring sound effects
 * - Optional: Joystick for analog control input
 */

#include "config.h"

// Game configuration
#define FIELD_WIDTH 8  // LED array width
#define FIELD_HEIGHT 8 // Virtual field height
#define MAX_OBSTACLES 4
#define PLAYER_POSITION 7 // Bottom row

// Game structures
typedef struct
{
    uint8_t x; // 0-7 for LED positions
    uint8_t lives;
    uint16_t score;
    uint8_t invulnerable_time;
} player_t;

typedef struct
{
    uint8_t x, y;
    uint8_t active;
    uint8_t speed;
    uint8_t type;
} obstacle_t;

// Global game variables
player_t player;
obstacle_t obstacles[MAX_OBSTACLES];
uint16_t game_time = 0;
uint8_t game_speed = 1;
uint8_t game_active = 1;
uint8_t spawn_rate = 30;

// Function to initialize the obstacle game
void init_obstacle_game()
{
    puts_USART1("Initializing Obstacle Avoidance Game...\r\n");

    // Initialize player
    player.x = FIELD_WIDTH / 2; // Start in middle
    player.lives = 3;
    player.score = 0;
    player.invulnerable_time = 0;

    // Initialize obstacles
    for (uint8_t i = 0; i < MAX_OBSTACLES; i++)
    {
        obstacles[i].active = 0;
        obstacles[i].x = 0;
        obstacles[i].y = 0;
        obstacles[i].speed = 1;
        obstacles[i].type = 1;
    }

    game_time = 0;
    game_speed = 1;
    game_active = 1;
    spawn_rate = 30;

    puts_USART1("Obstacle Game Ready!\r\n");
    puts_USART1("Controls: A=Left, D=Right, P=Pause, R=Restart\r\n");
    puts_USART1("Goal: Avoid falling obstacles!\r\n");
}

// Function to display game field on LEDs
void display_game_field()
{
    uint8_t led_pattern = 0xFF; // Start with all LEDs off

    // Show player position (always on bottom)
    if (player.invulnerable_time == 0 || (player.invulnerable_time / 5) % 2)
    {
        led_pattern &= ~(1 << player.x); // Turn on player LED
    }

    // Show obstacles on bottom row (where collisions happen)
    for (uint8_t i = 0; i < MAX_OBSTACLES; i++)
    {
        if (obstacles[i].active && obstacles[i].y >= PLAYER_POSITION)
        {
            // Obstacle has reached bottom row
            if ((game_time / 3) % 2) // Blink to distinguish from player
            {
                led_pattern &= ~(1 << obstacles[i].x);
            }
        }
    }

    // Special effects based on game state
    if (player.lives <= 1)
    {
        // Danger mode - rapid blinking
        if ((game_time / 2) % 2)
        {
            led_pattern ^= 0xFF; // Invert all LEDs
        }
    }

    PORTB = led_pattern;
}

// Function to handle player input
void handle_player_input()
{
    if (is_USART1_received())
    {
        char input = get_USART1();

        switch (input)
        {
        case 'a':
        case 'A': // Move left
            if (player.x > 0)
            {
                player.x--;
                puts_USART1("Player moved left\r\n");
            }
            break;

        case 'd':
        case 'D': // Move right
            if (player.x < FIELD_WIDTH - 1)
            {
                player.x++;
                puts_USART1("Player moved right\r\n");
            }
            break;

        case 'p':
        case 'P': // Pause
            game_active = !game_active;
            puts_USART1(game_active ? "Game resumed\r\n" : "Game paused\r\n");
            break;

        case 'r':
        case 'R': // Restart
            puts_USART1("Restarting game...\r\n");
            init_obstacle_game();
            break;
        }
    }
}

// Function to spawn new obstacles
void spawn_obstacles()
{
    // Spawn obstacles based on spawn rate
    if (game_time % spawn_rate == 0)
    {
        for (uint8_t i = 0; i < MAX_OBSTACLES; i++)
        {
            if (!obstacles[i].active)
            {
                obstacles[i].x = (game_time / 7) % FIELD_WIDTH; // Pseudo-random position
                obstacles[i].y = 0;                             // Start at top
                obstacles[i].active = 1;
                obstacles[i].speed = game_speed;
                obstacles[i].type = 1 + ((game_time / 20) % 3); // Different types

                char buffer[40];
                sprintf(buffer, "Obstacle spawned at position %u\r\n", obstacles[i].x);
                puts_USART1(buffer);
                break;
            }
        }
    }
}

// Function to update obstacle positions
void update_obstacles()
{
    for (uint8_t i = 0; i < MAX_OBSTACLES; i++)
    {
        if (obstacles[i].active)
        {
            // Move obstacle down
            if (game_time % (10 / obstacles[i].speed) == 0)
            {
                obstacles[i].y++;

                // Remove obstacles that go off screen
                if (obstacles[i].y > FIELD_HEIGHT)
                {
                    obstacles[i].active = 0;
                    player.score += 5; // Points for surviving obstacle

                    char buffer[30];
                    sprintf(buffer, "Obstacle avoided! Score: %u\r\n", player.score);
                    puts_USART1(buffer);
                }
            }
        }
    }
}

// Function to check collisions
void check_collisions()
{
    if (player.invulnerable_time > 0)
    {
        player.invulnerable_time--;
        return; // Player is invulnerable
    }

    for (uint8_t i = 0; i < MAX_OBSTACLES; i++)
    {
        if (obstacles[i].active && obstacles[i].y >= PLAYER_POSITION)
        {
            // Check if obstacle hits player
            if (obstacles[i].x == player.x)
            {
                // Collision detected!
                obstacles[i].active = 0; // Remove obstacle
                player.lives--;
                player.invulnerable_time = 30; // 3 seconds of invulnerability

                char buffer[40];
                sprintf(buffer, "HIT! Lives remaining: %u\r\n", player.lives);
                puts_USART1(buffer);

                // Collision sound effect
                for (uint8_t j = 0; j < 5; j++)
                {
                    Buzzer_on();
                    _delay_ms(50);
                    Buzzer_off();
                    _delay_ms(50);
                }

                // Check game over
                if (player.lives == 0)
                {
                    puts_USART1("\r\n*** GAME OVER! ***\r\n");
                    char final_score[40];
                    sprintf(final_score, "Final Score: %u\r\n", player.score);
                    puts_USART1(final_score);
                    puts_USART1("Press R to restart\r\n");
                    game_active = 0;

                    // Game over LED sequence
                    for (uint8_t k = 0; k < 8; k++)
                    {
                        PORTB = ~(1 << k);
                        _delay_ms(200);
                    }
                    PORTB = 0xFF;
                }

                break; // Only one collision per frame
            }
        }
    }
}

// Function to increase difficulty over time
void update_difficulty()
{
    // Increase speed every 200 points
    uint8_t new_speed = 1 + (player.score / 200);
    if (new_speed > game_speed && new_speed <= 5)
    {
        game_speed = new_speed;

        char buffer[50];
        sprintf(buffer, "SPEED INCREASED! Now level %u\r\n", game_speed);
        puts_USART1(buffer);

        // Speed increase celebration
        for (uint8_t i = 0; i < 3; i++)
        {
            PORTB = 0x00; // All LEDs on
            Buzzer_on();
            _delay_ms(100);
            PORTB = 0xFF; // All LEDs off
            Buzzer_off();
            _delay_ms(100);
        }
    }

    // Increase spawn rate (decrease spawn interval)
    if (spawn_rate > 10)
    {
        spawn_rate = 30 - (player.score / 100);
        if (spawn_rate < 10)
            spawn_rate = 10;
    }
}

// Function to display game status
void display_game_status()
{
    if (game_time % 50 == 0) // Every 5 seconds
    {
        char status[80];
        sprintf(status, "Time: %u, Score: %u, Lives: %u, Speed: %u\r\n",
                game_time / 10, player.score, player.lives, game_speed);
        puts_USART1(status);
    }
}

int main(void)
{
    // Initialize system components
    init_devices();
    Uart1_init();
    Buzzer_init();

    puts_USART1("Obstacle Avoidance Game Starting...\r\n");
    puts_USART1("Fast-paced survival game\r\n");
    puts_USART1("Goal: Avoid falling obstacles for as long as possible\r\n");

    DDRB = 0xFF;  // Set PORTB as output for LEDs
    PORTB = 0xFF; // Turn off all LEDs initially

    // Initialize the game
    init_obstacle_game();

    puts_USART1("Game started! Survive the obstacles!\r\n");

    while (1)
    {
        if (game_active)
        {
            game_time++;

            // Handle player input
            handle_player_input();

            // Game logic
            spawn_obstacles();
            update_obstacles();
            check_collisions();
            update_difficulty();

            // Display game state
            display_game_field();
            display_game_status();
        }
        else
        {
            // Game paused or over - just handle input
            handle_player_input();
        }

        // Game timing - 10 FPS
        _delay_ms(100);
    }

    return 0;
}
acle
        *ATmega128 Educational Framework
            *
                *This project implements an obstacle avoidance
                    *game with moving objects.
                        * /

#include "config.h"

    int main(void)
{
    main_game_obstacle();

    return 0;
}