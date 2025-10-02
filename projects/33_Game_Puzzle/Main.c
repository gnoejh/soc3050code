/*
 * Game Puzzle - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Implement a sliding number puzzle game
 * - Learn 2D array manipulation and state management
 * - Practice puzzle-solving algorithms and validation
 * - Master move validation and win condition checking
 *
 * HARDWARE SETUP:
 * - LEDs on PORTB for puzzle state visualization
 * - UART for move input and game status display
 * - Buzzer for move confirmation and victory sounds
 * - Optional: GLCD for full puzzle grid display
 */

#include "config.h"

// Puzzle configuration
#define PUZZLE_SIZE 3
#define TOTAL_TILES (PUZZLE_SIZE * PUZZLE_SIZE - 1) // 8 tiles + 1 empty
#define EMPTY_TILE 0

// Game structures
typedef struct
{
    uint8_t grid[PUZZLE_SIZE][PUZZLE_SIZE];
    uint8_t empty_row, empty_col;
    uint16_t move_count;
    uint8_t game_won;
    uint32_t start_time;
} puzzle_game_t;

// Global game state
puzzle_game_t puzzle;
uint32_t game_timer = 0;

// Solved state for comparison
const uint8_t solved_puzzle[PUZZLE_SIZE][PUZZLE_SIZE] = {
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 0} // 0 represents empty space
};

// Function to initialize puzzle game
void init_puzzle_game()
{
    puts_USART1("Initializing Sliding Puzzle Game...\r\n");

    // Initialize with solved state first
    for (uint8_t row = 0; row < PUZZLE_SIZE; row++)
    {
        for (uint8_t col = 0; col < PUZZLE_SIZE; col++)
        {
            puzzle.grid[row][col] = solved_puzzle[row][col];
        }
    }

    // Set empty tile position
    puzzle.empty_row = PUZZLE_SIZE - 1;
    puzzle.empty_col = PUZZLE_SIZE - 1;

    // Shuffle the puzzle
    shuffle_puzzle();

    puzzle.move_count = 0;
    puzzle.game_won = 0;
    puzzle.start_time = game_timer;

    puts_USART1("Puzzle shuffled and ready!\r\n");
    puts_USART1("Controls: W=Up, S=Down, A=Left, D=Right, R=Restart\r\n");
    puts_USART1("Goal: Arrange numbers 1-8 in order with empty space at bottom-right\r\n");

    display_puzzle();
}

// Function to shuffle puzzle (perform random valid moves)
void shuffle_puzzle()
{
    puts_USART1("Shuffling puzzle...\r\n");

    // Perform 100 random valid moves to shuffle
    for (uint16_t i = 0; i < 100; i++)
    {
        // Choose random direction
        uint8_t direction = (game_timer + i) % 4;

        switch (direction)
        {
        case 0:
            move_tile('w');
            break; // Up
        case 1:
            move_tile('s');
            break; // Down
        case 2:
            move_tile('a');
            break; // Left
        case 3:
            move_tile('d');
            break; // Right
        }

        game_timer++; // Change seed for next random move
    }

    // Reset move counter after shuffling
    puzzle.move_count = 0;
}

// Function to display puzzle state
void display_puzzle()
{
    char buffer[100];

    puts_USART1("\r\n=== SLIDING PUZZLE ===\r\n");

    // Display grid
    for (uint8_t row = 0; row < PUZZLE_SIZE; row++)
    {
        puts_USART1("|");
        for (uint8_t col = 0; col < PUZZLE_SIZE; col++)
        {
            if (puzzle.grid[row][col] == EMPTY_TILE)
            {
                puts_USART1("   |"); // Empty space
            }
            else
            {
                sprintf(buffer, " %u |", puzzle.grid[row][col]);
                puts_USART1(buffer);
            }
        }
        puts_USART1("\r\n+---+---+---+\r\n");
    }

    sprintf(buffer, "Moves: %u\r\n", puzzle.move_count);
    puts_USART1(buffer);

    // Display simplified state on LEDs
    display_puzzle_leds();
}

// Function to display puzzle state on LEDs
void display_puzzle_leds()
{
    uint8_t led_pattern = 0xFF; // All LEDs off initially

    // Map puzzle state to LEDs (simplified)
    // Show which tiles are in correct positions
    uint8_t correct_count = 0;

    for (uint8_t row = 0; row < PUZZLE_SIZE; row++)
    {
        for (uint8_t col = 0; col < PUZZLE_SIZE; col++)
        {
            if (puzzle.grid[row][col] == solved_puzzle[row][col])
            {
                correct_count++;
            }
        }
    }

    // Light up LEDs based on correct tiles
    for (uint8_t i = 0; i < correct_count && i < 8; i++)
    {
        led_pattern &= ~(1 << i);
    }

    // Special pattern if puzzle is solved
    if (puzzle.game_won)
    {
        led_pattern = ((game_timer / 10) % 2) ? 0x00 : 0xFF; // All blinking
    }

    PORTB = led_pattern;
}

// Function to check if move is valid
uint8_t is_move_valid(char direction)
{
    switch (direction)
    {
    case 'w':
    case 'W': // Move tile down into empty space (empty moves up)
        return (puzzle.empty_row > 0);
    case 's':
    case 'S': // Move tile up into empty space (empty moves down)
        return (puzzle.empty_row < PUZZLE_SIZE - 1);
    case 'a':
    case 'A': // Move tile right into empty space (empty moves left)
        return (puzzle.empty_col > 0);
    case 'd':
    case 'D': // Move tile left into empty space (empty moves right)
        return (puzzle.empty_col < PUZZLE_SIZE - 1);
    default:
        return 0;
    }
}

// Function to move tile
void move_tile(char direction)
{
    if (!is_move_valid(direction))
    {
        puts_USART1("Invalid move!\r\n");
        return;
    }

    uint8_t tile_row = puzzle.empty_row;
    uint8_t tile_col = puzzle.empty_col;

    // Calculate which tile to move based on direction
    switch (direction)
    {
    case 'w':
    case 'W': // Move tile from above down
        tile_row = puzzle.empty_row - 1;
        break;
    case 's':
    case 'S': // Move tile from below up
        tile_row = puzzle.empty_row + 1;
        break;
    case 'a':
    case 'A': // Move tile from left right
        tile_col = puzzle.empty_col - 1;
        break;
    case 'd':
    case 'D': // Move tile from right left
        tile_col = puzzle.empty_col + 1;
        break;
    }

    // Perform the swap
    puzzle.grid[puzzle.empty_row][puzzle.empty_col] = puzzle.grid[tile_row][tile_col];
    puzzle.grid[tile_row][tile_col] = EMPTY_TILE;

    // Update empty position
    puzzle.empty_row = tile_row;
    puzzle.empty_col = tile_col;

    puzzle.move_count++;

    // Move sound effect
    Buzzer_on();
    _delay_ms(50);
    Buzzer_off();

    char buffer[40];
    sprintf(buffer, "Moved tile %u (Move #%u)\r\n",
            puzzle.grid[puzzle.empty_row][puzzle.empty_col], puzzle.move_count);
    puts_USART1(buffer);

    // Check for win condition
    check_win_condition();
}

// Function to check if puzzle is solved
void check_win_condition()
{
    // Compare current state with solved state
    for (uint8_t row = 0; row < PUZZLE_SIZE; row++)
    {
        for (uint8_t col = 0; col < PUZZLE_SIZE; col++)
        {
            if (puzzle.grid[row][col] != solved_puzzle[row][col])
            {
                return; // Not solved yet
            }
        }
    }

    // Puzzle is solved!
    puzzle.game_won = 1;

    uint32_t solve_time = game_timer - puzzle.start_time;

    puts_USART1("\r\n*** CONGRATULATIONS! PUZZLE SOLVED! ***\r\n");

    char buffer[60];
    sprintf(buffer, "Solved in %u moves and %lu seconds!\r\n",
            puzzle.move_count, solve_time / 10);
    puts_USART1(buffer);

    // Victory sound sequence
    for (uint8_t i = 0; i < 5; i++)
    {
        Buzzer_on();
        _delay_ms(200);
        Buzzer_off();
        _delay_ms(100);
    }

    // Victory LED sequence
    for (uint8_t i = 0; i < 10; i++)
    {
        PORTB = 0x00; // All LEDs on
        _delay_ms(150);
        PORTB = 0xFF; // All LEDs off
        _delay_ms(150);
    }

    puts_USART1("Press R to play again!\r\n");
}

// Function to handle game input
void handle_puzzle_input()
{
    if (is_USART1_received())
    {
        char input = get_USART1();

        // Echo input
        put_USART1(input);
        puts_USART1("\r\n");

        if (input == 'r' || input == 'R')
        {
            // Restart game
            puts_USART1("Restarting puzzle...\r\n");
            init_puzzle_game();
        }
        else if (!puzzle.game_won)
        {
            // Process move
            move_tile(input);
            display_puzzle();
        }
        else
        {
            puts_USART1("Puzzle solved! Press R to restart.\r\n");
        }
    }
}

int main(void)
{
    // Initialize system components
    init_devices();
    Uart1_init();
    Buzzer_init();

    puts_USART1("Sliding Puzzle Game Starting...\r\n");
    puts_USART1("Classic 3x3 number sliding puzzle\r\n");
    puts_USART1("Arrange numbers 1-8 in order!\r\n");

    DDRB = 0xFF;  // Set PORTB as output for LEDs
    PORTB = 0xFF; // Turn off all LEDs initially

    // Initialize the game
    init_puzzle_game();

    while (1)
    {
        game_timer++;

        // Handle player input
        handle_puzzle_input();

        // Update LED display
        if (game_timer % 10 == 0)
        {
            display_puzzle_leds();
        }

        // Game timing
        _delay_ms(100);
    }

    return 0;
}
le
        *ATmega128 Educational Framework
            *
                *This project implements a sliding puzzle
                    *game with user interaction.
                        * /

#include "config.h"

    int main(void)
{
    main_game_puzzle();

    return 0;
}