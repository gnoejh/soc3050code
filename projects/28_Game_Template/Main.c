/*
 * Game Template - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Learn game development structure and architecture
 * - Understand game loops and state management
 * - Practice input handling and display updates
 * - Master modular game programming design
 *
 * HARDWARE SETUP:
 * - GLCD for game display (128x64 pixels)
 * - Buttons for game input (Up, Down, Left, Right, Fire)
 * - LEDs for game status and effects
 * - Buzzer for game audio feedback
 */

#include "config.h"

// Game state enumeration
typedef enum
{
    GAME_MENU,
    GAME_PLAYING,
    GAME_PAUSED,
    GAME_OVER,
    GAME_WIN
} game_state_t;

// Player structure
typedef struct
{
    uint8_t x, y;
    uint8_t health;
    uint16_t score;
    uint8_t lives;
} player_t;

// Game object structure
typedef struct
{
    uint8_t x, y;
    uint8_t width, height;
    uint8_t active;
    uint8_t type;
} game_object_t;

// Game configuration
#define MAX_OBJECTS 10
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define PLAYER_SIZE 4

// Global game variables
game_state_t current_state = GAME_MENU;
player_t player;
game_object_t objects[MAX_OBJECTS];
uint16_t game_timer = 0;
uint8_t input_buffer = 0;

// Function to initialize game
void init_game()
{
    puts_USART1("Initializing Game Template...\r\n");

    // Initialize player
    player.x = SCREEN_WIDTH / 2;
    player.y = SCREEN_HEIGHT - 10;
    player.health = 100;
    player.score = 0;
    player.lives = 3;

    // Initialize game objects
    for (uint8_t i = 0; i < MAX_OBJECTS; i++)
    {
        objects[i].active = 0;
    }

    game_timer = 0;
    current_state = GAME_MENU;

    puts_USART1("Game initialized successfully\r\n");
}

// Function to read input
void read_input()
{
    // Simulate button reading (replace with actual hardware)
    input_buffer = 0;

    // Check for UART input as game controls
    if (is_USART1_received())
    {
        char input = get_USART1();
        switch (input)
        {
        case 'w':
        case 'W':
            input_buffer |= (1 << 0);
            break; // Up
        case 's':
        case 'S':
            input_buffer |= (1 << 1);
            break; // Down
        case 'a':
        case 'A':
            input_buffer |= (1 << 2);
            break; // Left
        case 'd':
        case 'D':
            input_buffer |= (1 << 3);
            break; // Right
        case ' ':
            input_buffer |= (1 << 4);
            break; // Fire/Action
        case 'p':
        case 'P':
            input_buffer |= (1 << 5);
            break; // Pause
        case 'q':
        case 'Q':
            input_buffer |= (1 << 6);
            break; // Quit
        }
    }
}

// Function to update player
void update_player()
{
    // Handle player movement
    if (input_buffer & (1 << 0)) // Up
    {
        if (player.y > 0)
            player.y--;
    }
    if (input_buffer & (1 << 1)) // Down
    {
        if (player.y < SCREEN_HEIGHT - PLAYER_SIZE)
            player.y++;
    }
    if (input_buffer & (1 << 2)) // Left
    {
        if (player.x > 0)
            player.x--;
    }
    if (input_buffer & (1 << 3)) // Right
    {
        if (player.x < SCREEN_WIDTH - PLAYER_SIZE)
            player.x++;
    }

    // Handle action button
    if (input_buffer & (1 << 4)) // Fire/Action
    {
        // Add projectile or perform action
        puts_USART1("Player action!\r\n");
        Buzzer_on();
        _delay_ms(50);
        Buzzer_off();
    }
}

// Function to update game objects
void update_objects()
{
    for (uint8_t i = 0; i < MAX_OBJECTS; i++)
    {
        if (objects[i].active)
        {
            // Move objects (example: falling objects)
            objects[i].y++;

            // Remove objects that go off screen
            if (objects[i].y > SCREEN_HEIGHT)
            {
                objects[i].active = 0;
            }
        }
    }

    // Spawn new objects occasionally
    if (game_timer % 60 == 0) // Every ~1 second
    {
        for (uint8_t i = 0; i < MAX_OBJECTS; i++)
        {
            if (!objects[i].active)
            {
                objects[i].x = game_timer % (SCREEN_WIDTH - 8);
                objects[i].y = 0;
                objects[i].width = 4;
                objects[i].height = 4;
                objects[i].active = 1;
                objects[i].type = 1; // Enemy type
                break;
            }
        }
    }
}

// Function to check collisions
void check_collisions()
{
    for (uint8_t i = 0; i < MAX_OBJECTS; i++)
    {
        if (objects[i].active)
        {
            // Simple bounding box collision
            if (player.x < objects[i].x + objects[i].width &&
                player.x + PLAYER_SIZE > objects[i].x &&
                player.y < objects[i].y + objects[i].height &&
                player.y + PLAYER_SIZE > objects[i].y)
            {
                // Collision detected
                objects[i].active = 0;
                player.health -= 10;
                player.score += 10;

                puts_USART1("Collision! Score: ");
                char buffer[20];
                sprintf(buffer, "%u\r\n", player.score);
                puts_USART1(buffer);

                // Sound effect
                Buzzer_on();
                _delay_ms(100);
                Buzzer_off();

                // LED effect
                PORTB = 0x00;
                _delay_ms(100);
                PORTB = 0xFF;
            }
        }
    }
}

// Function to draw game (simplified for UART output)
void draw_game()
{
    // Clear screen simulation
    puts_USART1("\033[2J\033[H"); // ANSI clear screen

    char buffer[50];
    sprintf(buffer, "Score: %u  Health: %u  Lives: %u\r\n",
            player.score, player.health, player.lives);
    puts_USART1(buffer);

    sprintf(buffer, "Player Position: (%u, %u)\r\n", player.x, player.y);
    puts_USART1(buffer);

    // Show active objects
    uint8_t active_count = 0;
    for (uint8_t i = 0; i < MAX_OBJECTS; i++)
    {
        if (objects[i].active)
            active_count++;
    }
    sprintf(buffer, "Active Objects: %u\r\n", active_count);
    puts_USART1(buffer);

    puts_USART1("Controls: WASD=Move, Space=Action, P=Pause, Q=Quit\r\n");
    puts_USART1("=====================================\r\n");
}

// Function to handle game states
void update_game_state()
{
    switch (current_state)
    {
    case GAME_MENU:
        if (input_buffer & (1 << 4)) // Start game with action button
        {
            current_state = GAME_PLAYING;
            puts_USART1("Game Started!\r\n");
        }
        break;

    case GAME_PLAYING:
        if (input_buffer & (1 << 5)) // Pause
        {
            current_state = GAME_PAUSED;
            puts_USART1("Game Paused\r\n");
        }
        else if (input_buffer & (1 << 6)) // Quit
        {
            current_state = GAME_MENU;
            puts_USART1("Returning to Menu\r\n");
        }
        else if (player.health <= 0)
        {
            current_state = GAME_OVER;
            puts_USART1("Game Over!\r\n");
        }
        break;

    case GAME_PAUSED:
        if (input_buffer & (1 << 5)) // Unpause
        {
            current_state = GAME_PLAYING;
            puts_USART1("Game Resumed\r\n");
        }
        break;

    case GAME_OVER:
        if (input_buffer & (1 << 4)) // Restart
        {
            init_game();
            current_state = GAME_PLAYING;
            puts_USART1("Game Restarted!\r\n");
        }
        break;

    default:
        current_state = GAME_MENU;
        break;
    }
}

int main(void)
{
    // Initialize system components
    init_devices();
    Glcd_init();
    Uart1_init();
    Buzzer_init();

    puts_USART1("Game Template System Starting...\r\n");
    puts_USART1("Educational Game Development Framework\r\n");
    puts_USART1("Features: Game loop, State management, Collision detection\r\n");

    DDRB = 0xFF;
    PORTB = 0xFF;

    // Initialize game
    init_game();

    puts_USART1("Game Template Ready - Press Space to Start\r\n");

    while (1)
    {
        game_timer++;

        // Read player input
        read_input();

        // Update game state
        update_game_state();

        // Game logic (only when playing)
        if (current_state == GAME_PLAYING)
        {
            update_player();
            update_objects();
            check_collisions();
        }

        // Draw game (every 10 cycles to reduce output)
        if (game_timer % 10 == 0)
        {
            draw_game();
        }

        // Game timing
        _delay_ms(50); // 20 FPS game loop
    }

    return 0;
}