/*
 * Game Pong UART - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Implement the classic Pong game with UART control
 * - Learn 2D game physics and collision detection
 * - Practice paddle control and ball movement algorithms
 * - Master game scoring and competitive gameplay
 *
 * HARDWARE SETUP:
 * - LEDs on PORTB for simplified Pong visualization
 * - UART for paddle control and game status
 * - Buzzer for ball collision sound effects
 * - Optional: GLCD for full graphical Pong display
 */

#include "config.h"

// Game configuration
#define FIELD_WIDTH 8  // LED field width
#define FIELD_HEIGHT 8 // Virtual field height
#define PADDLE_SIZE 2
#define WINNING_SCORE 5

// Game structures
typedef struct
{
    int8_t x, y;   // Ball position (signed for collision detection)
    int8_t dx, dy; // Ball velocity
    uint8_t active;
} ball_t;

typedef struct
{
    uint8_t y; // Paddle Y position
    uint8_t score;
} paddle_t;

typedef struct
{
    uint8_t game_active;
    uint8_t game_over;
    uint8_t winner;
    uint16_t rally_count;
} game_state_t;

// Global game variables
ball_t ball;
paddle_t player1; // Left paddle (controlled by player)
paddle_t player2; // Right paddle (AI or second player)
game_state_t game_state;
uint16_t game_time = 0;

// Function to initialize Pong game
void init_pong_game()
{
    puts_USART1("Initializing Pong Game...\r\n");

    // Initialize ball in center
    ball.x = FIELD_WIDTH / 2;
    ball.y = FIELD_HEIGHT / 2;
    ball.dx = 1; // Start moving right
    ball.dy = 1; // Start moving down
    ball.active = 1;

    // Initialize paddles
    player1.y = FIELD_HEIGHT / 2;
    player1.score = 0;
    player2.y = FIELD_HEIGHT / 2;
    player2.score = 0;

    // Initialize game state
    game_state.game_active = 1;
    game_state.game_over = 0;
    game_state.winner = 0;
    game_state.rally_count = 0;

    game_time = 0;

    puts_USART1("Pong Game Ready!\r\n");
    puts_USART1("Controls: W/S=Move paddle, P=Pause, R=Restart\r\n");
    puts_USART1("First to 5 points wins!\r\n");
}

// Function to display game on LEDs
void display_pong_game()
{
    uint8_t led_pattern = 0xFF; // Start with all LEDs off

    // Map game field to LED display
    // For simplicity, we'll show ball position and paddle positions

    // Show ball position (if in LED field range)
    if (ball.x >= 0 && ball.x < FIELD_WIDTH && ball.y >= 0 && ball.y < FIELD_HEIGHT)
    {
        // Map ball position to LED
        uint8_t led_pos = (ball.y * FIELD_WIDTH + ball.x) % 8;
        led_pattern &= ~(1 << led_pos);
    }

    // Show paddle positions on edges
    // Player 1 paddle (left side) - show on LED 0
    if ((game_time / 10) % 2) // Blink to distinguish from ball
    {
        led_pattern &= ~(1 << 0);
    }

    // Player 2 paddle (right side) - show on LED 7
    if ((game_time / 10) % 2)
    {
        led_pattern &= ~(1 << 7);
    }

    // Special effects
    if (game_state.game_over)
    {
        // Game over effect - alternating pattern
        led_pattern = ((game_time / 5) % 2) ? 0x55 : 0xAA;
    }

    PORTB = led_pattern;
}

// Function to handle player input
void handle_pong_input()
{
    if (is_USART1_received())
    {
        char input = get_USART1();

        switch (input)
        {
        case 'w':
        case 'W': // Move paddle up
            if (player1.y > 0)
            {
                player1.y--;
                puts_USART1("Player 1 paddle up\r\n");
            }
            break;

        case 's':
        case 'S': // Move paddle down
            if (player1.y < FIELD_HEIGHT - PADDLE_SIZE)
            {
                player1.y++;
                puts_USART1("Player 1 paddle down\r\n");
            }
            break;

        case 'p':
        case 'P': // Pause
            game_state.game_active = !game_state.game_active;
            puts_USART1(game_state.game_active ? "Game resumed\r\n" : "Game paused\r\n");
            break;

        case 'r':
        case 'R': // Restart
            puts_USART1("Restarting Pong game...\r\n");
            init_pong_game();
            break;
        }
    }
}

// Function to update AI paddle (simple AI)
void update_ai_paddle()
{
    // Simple AI: move towards ball
    if (ball.y < player2.y && player2.y > 0)
    {
        player2.y--;
    }
    else if (ball.y > player2.y + PADDLE_SIZE && player2.y < FIELD_HEIGHT - PADDLE_SIZE)
    {
        player2.y++;
    }
}

// Function to update ball physics
void update_ball_physics()
{
    if (!ball.active)
        return;

    // Move ball
    ball.x += ball.dx;
    ball.y += ball.dy;

    // Check top/bottom wall collisions
    if (ball.y <= 0 || ball.y >= FIELD_HEIGHT - 1)
    {
        ball.dy = -ball.dy; // Reverse Y direction

        // Wall bounce sound
        Buzzer_on();
        _delay_ms(50);
        Buzzer_off();

        puts_USART1("Wall bounce!\r\n");
    }

    // Check paddle collisions
    // Player 1 paddle (left side)
    if (ball.x <= 1 && ball.dx < 0)
    {
        if (ball.y >= player1.y && ball.y <= player1.y + PADDLE_SIZE)
        {
            ball.dx = -ball.dx; // Reverse X direction
            game_state.rally_count++;

            // Paddle hit sound
            Buzzer_on();
            _delay_ms(100);
            Buzzer_off();

            char buffer[30];
            sprintf(buffer, "Player 1 hit! Rally: %u\r\n", game_state.rally_count);
            puts_USART1(buffer);
        }
    }

    // Player 2 paddle (right side)
    if (ball.x >= FIELD_WIDTH - 2 && ball.dx > 0)
    {
        if (ball.y >= player2.y && ball.y <= player2.y + PADDLE_SIZE)
        {
            ball.dx = -ball.dx; // Reverse X direction
            game_state.rally_count++;

            // Paddle hit sound
            Buzzer_on();
            _delay_ms(100);
            Buzzer_off();

            char buffer[30];
            sprintf(buffer, "Player 2 hit! Rally: %u\r\n", game_state.rally_count);
            puts_USART1(buffer);
        }
    }

    // Check scoring (ball goes off sides)
    if (ball.x < 0)
    {
        // Player 2 scores
        player2.score++;
        game_state.rally_count = 0;

        char buffer[40];
        sprintf(buffer, "Player 2 scores! Score: %u-%u\r\n", player1.score, player2.score);
        puts_USART1(buffer);

        // Score sound
        for (uint8_t i = 0; i < 3; i++)
        {
            Buzzer_on();
            _delay_ms(150);
            Buzzer_off();
            _delay_ms(100);
        }

        // Reset ball
        ball.x = FIELD_WIDTH / 2;
        ball.y = FIELD_HEIGHT / 2;
        ball.dx = -1;                       // Serve towards player 1
        ball.dy = (game_time % 2) ? 1 : -1; // Random Y direction
    }
    else if (ball.x >= FIELD_WIDTH)
    {
        // Player 1 scores
        player1.score++;
        game_state.rally_count = 0;

        char buffer[40];
        sprintf(buffer, "Player 1 scores! Score: %u-%u\r\n", player1.score, player2.score);
        puts_USART1(buffer);

        // Score sound
        for (uint8_t i = 0; i < 3; i++)
        {
            Buzzer_on();
            _delay_ms(150);
            Buzzer_off();
            _delay_ms(100);
        }

        // Reset ball
        ball.x = FIELD_WIDTH / 2;
        ball.y = FIELD_HEIGHT / 2;
        ball.dx = 1;                        // Serve towards player 2
        ball.dy = (game_time % 2) ? 1 : -1; // Random Y direction
    }

    // Check for game win
    if (player1.score >= WINNING_SCORE)
    {
        game_state.game_over = 1;
        game_state.winner = 1;
        puts_USART1("\r\n*** PLAYER 1 WINS! ***\r\n");

        // Victory sequence
        for (uint8_t i = 0; i < 5; i++)
        {
            PORTB = 0x0F; // Left side LEDs
            Buzzer_on();
            _delay_ms(200);
            PORTB = 0xFF;
            Buzzer_off();
            _delay_ms(200);
        }
    }
    else if (player2.score >= WINNING_SCORE)
    {
        game_state.game_over = 1;
        game_state.winner = 2;
        puts_USART1("\r\n*** PLAYER 2 WINS! ***\r\n");

        // Victory sequence
        for (uint8_t i = 0; i < 5; i++)
        {
            PORTB = 0xF0; // Right side LEDs
            Buzzer_on();
            _delay_ms(200);
            PORTB = 0xFF;
            Buzzer_off();
            _delay_ms(200);
        }
    }
}

// Function to display game status
void display_game_status()
{
    if (game_time % 100 == 0) // Every 10 seconds
    {
        char status[60];
        sprintf(status, "Score: P1=%u P2=%u, Rally: %u\r\n",
                player1.score, player2.score, game_state.rally_count);
        puts_USART1(status);
    }
}

int main(void)
{
    // Initialize system components
    init_devices();
    Uart1_init();
    Buzzer_init();

    puts_USART1("Pong Game Starting...\r\n");
    puts_USART1("Classic Pong with UART control\r\n");
    puts_USART1("Player vs AI - First to 5 points wins!\r\n");

    DDRB = 0xFF;  // Set PORTB as output for LEDs
    PORTB = 0xFF; // Turn off all LEDs initially

    // Initialize the game
    init_pong_game();

    puts_USART1("Game started! Use W/S to control your paddle!\r\n");

    while (1)
    {
        if (game_state.game_active && !game_state.game_over)
        {
            game_time++;

            // Handle player input
            handle_pong_input();

            // Update AI every few cycles
            if (game_time % 3 == 0)
            {
                update_ai_paddle();
            }

            // Update ball physics every cycle
            update_ball_physics();

            // Display game state
            display_pong_game();
            display_game_status();
        }
        else
        {
            // Game paused or over - just handle input
            handle_pong_input();
        }

        // Game timing - 10 FPS
        _delay_ms(100);
    }

    return 0;
}
UART
        *ATmega128 Educational Framework
            *
                *This project implements Pong game
                    *with UART -
    based control.
            * /

#include "config.h"

        int main(void)
{
    main_game_pong_uart_control();

    return 0;
}