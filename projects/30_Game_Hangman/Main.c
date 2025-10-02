/*
 * Game Hangman - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Implement a classic word-guessing game
 * - Learn string manipulation and character arrays
 * - Practice game logic and user input validation
 * - Master text-based game display and feedback
 *
 * HARDWARE SETUP:
 * - UART for game input/output
 * - LEDs for game progress visualization
 * - Buzzer for correct/incorrect guess feedback
 * - Optional: GLCD for graphical hangman display
 */

#include "config.h"

// Game configuration
#define MAX_WORD_LENGTH 12
#define MAX_WRONG_GUESSES 6
#define TOTAL_WORDS 10

// Game state structure
typedef struct
{
    char word[MAX_WORD_LENGTH + 1];
    char guessed_word[MAX_WORD_LENGTH + 1];
    char guessed_letters[26];
    uint8_t wrong_guesses;
    uint8_t game_won;
    uint8_t game_over;
    uint16_t score;
} hangman_game_t;

// Word list for the game
const char *word_list[TOTAL_WORDS] = {
    "ATMEGA", "PROGRAM", "CIRCUIT", "VOLTAGE",
    "CURRENT", "SENSOR", "BUZZER", "DISPLAY",
    "MEMORY", "INTERRUPT"};

// Global game state
hangman_game_t game;
uint8_t current_word_index = 0;

// Function to initialize a new game
void init_hangman_game()
{
    puts_USART1("\r\n=== NEW HANGMAN GAME ===\r\n");

    // Select word
    strcpy(game.word, word_list[current_word_index]);
    current_word_index = (current_word_index + 1) % TOTAL_WORDS;

    // Initialize guessed word with underscores
    uint8_t word_len = strlen(game.word);
    for (uint8_t i = 0; i < word_len; i++)
    {
        game.guessed_word[i] = '_';
    }
    game.guessed_word[word_len] = '\0';

    // Clear guessed letters
    for (uint8_t i = 0; i < 26; i++)
    {
        game.guessed_letters[i] = 0;
    }

    game.wrong_guesses = 0;
    game.game_won = 0;
    game.game_over = 0;

    puts_USART1("Word selected! Start guessing letters.\r\n");
    display_game_state();
}

// Function to display hangman figure
void display_hangman_figure()
{
    puts_USART1("\r\n  +---+\r\n");
    puts_USART1("  |   |\r\n");

    // Head
    if (game.wrong_guesses >= 1)
        puts_USART1("  |   O\r\n");
    else
        puts_USART1("  |    \r\n");

    // Body and arms
    if (game.wrong_guesses >= 4)
        puts_USART1("  |  /|\\\r\n"); // Both arms
    else if (game.wrong_guesses >= 3)
        puts_USART1("  |  /| \r\n"); // Left arm
    else if (game.wrong_guesses >= 2)
        puts_USART1("  |   | \r\n"); // Body
    else
        puts_USART1("  |     \r\n");

    // Legs
    if (game.wrong_guesses >= 6)
        puts_USART1("  |  / \\\r\n"); // Both legs
    else if (game.wrong_guesses >= 5)
        puts_USART1("  |  /  \r\n"); // Left leg
    else
        puts_USART1("  |     \r\n");

    puts_USART1("  |     \r\n");
    puts_USART1("==========\r\n\r\n");

    // Update LED display based on wrong guesses
    uint8_t led_pattern = 0xFF;
    for (uint8_t i = 0; i < game.wrong_guesses && i < 8; i++)
    {
        led_pattern &= ~(1 << i); // Turn on LEDs for wrong guesses
    }
    PORTB = led_pattern;
}

// Function to display current game state
void display_game_state()
{
    char buffer[100];

    puts_USART1("\r\n--- HANGMAN GAME ---\r\n");

    // Show guessed word
    sprintf(buffer, "Word: %s\r\n", game.guessed_word);
    puts_USART1(buffer);

    // Show wrong guesses remaining
    sprintf(buffer, "Wrong guesses: %u/%u\r\n", game.wrong_guesses, MAX_WRONG_GUESSES);
    puts_USART1(buffer);

    // Show guessed letters
    puts_USART1("Guessed letters: ");
    for (uint8_t i = 0; i < 26; i++)
    {
        if (game.guessed_letters[i])
        {
            char letter = 'A' + i;
            sprintf(buffer, "%c ", letter);
            puts_USART1(buffer);
        }
    }
    puts_USART1("\r\n");

    // Visual hangman display
    display_hangman_figure();

    if (!game.game_over)
    {
        puts_USART1("Enter a letter (A-Z): ");
    }
}

// Function to process a guess
void process_guess(char letter)
{
    // Convert to uppercase
    if (letter >= 'a' && letter <= 'z')
    {
        letter = letter - 'a' + 'A';
    }

    // Validate input
    if (letter < 'A' || letter > 'Z')
    {
        puts_USART1("Invalid input! Please enter a letter A-Z.\r\n");
        return;
    }

    // Check if already guessed
    if (game.guessed_letters[letter - 'A'])
    {
        puts_USART1("You already guessed that letter!\r\n");
        return;
    }

    // Mark letter as guessed
    game.guessed_letters[letter - 'A'] = 1;

    // Check if letter is in word
    uint8_t found = 0;
    for (uint8_t i = 0; i < strlen(game.word); i++)
    {
        if (game.word[i] == letter)
        {
            game.guessed_word[i] = letter;
            found = 1;
        }
    }

    if (found)
    {
        char buffer[50];
        sprintf(buffer, "Correct! '%c' is in the word.\r\n", letter);
        puts_USART1(buffer);

        // Sound effect for correct guess
        Buzzer_on();
        _delay_ms(200);
        Buzzer_off();

        // Check if word is complete
        if (strcmp(game.word, game.guessed_word) == 0)
        {
            game.game_won = 1;
            game.game_over = 1;
            game.score += (MAX_WRONG_GUESSES - game.wrong_guesses) * 10;

            puts_USART1("\r\n*** CONGRATULATIONS! YOU WON! ***\r\n");
            char score_buffer[30];
            sprintf(score_buffer, "Score: %u points\r\n", game.score);
            puts_USART1(score_buffer);

            // Victory sound
            for (uint8_t i = 0; i < 3; i++)
            {
                Buzzer_on();
                _delay_ms(150);
                Buzzer_off();
                _delay_ms(100);
            }

            // Victory LED pattern
            for (uint8_t i = 0; i < 5; i++)
            {
                PORTB = 0x00; // All LEDs on
                _delay_ms(200);
                PORTB = 0xFF; // All LEDs off
                _delay_ms(200);
            }
        }
    }
    else
    {
        char buffer[50];
        sprintf(buffer, "Wrong! '%c' is not in the word.\r\n", letter);
        puts_USART1(buffer);

        game.wrong_guesses++;

        // Sound effect for wrong guess
        for (uint8_t i = 0; i < 2; i++)
        {
            Buzzer_on();
            _delay_ms(100);
            Buzzer_off();
            _delay_ms(100);
        }

        // Check if game is over
        if (game.wrong_guesses >= MAX_WRONG_GUESSES)
        {
            game.game_over = 1;

            puts_USART1("\r\n*** GAME OVER! ***\r\n");
            char reveal_buffer[50];
            sprintf(reveal_buffer, "The word was: %s\r\n", game.word);
            puts_USART1(reveal_buffer);

            // Game over LED pattern
            for (uint8_t i = 0; i < 3; i++)
            {
                PORTB = 0x55; // Alternating pattern
                _delay_ms(300);
                PORTB = 0xAA;
                _delay_ms(300);
            }
            PORTB = 0xFF; // All LEDs off
        }
    }
}

int main(void)
{
    // Initialize system components
    init_devices();
    Uart1_init();
    Buzzer_init();

    puts_USART1("Hangman Game Starting...\r\n");
    puts_USART1("Educational Word Guessing Game\r\n");
    puts_USART1("Guess the word letter by letter!\r\n");
    puts_USART1("Commands: Letters A-Z to guess, 'N' for new game\r\n");

    DDRB = 0xFF;  // Set PORTB as output for LEDs
    PORTB = 0xFF; // Turn off all LEDs initially

    // Initialize first game
    init_hangman_game();

    while (1)
    {
        if (is_USART1_received())
        {
            char input = get_USART1();

            // Echo the input
            put_USART1(input);
            puts_USART1("\r\n");

            if (input == 'N' || input == 'n')
            {
                // Start new game
                init_hangman_game();
            }
            else if (!game.game_over)
            {
                // Process guess
                process_guess(input);
                display_game_state();
            }
            else
            {
                puts_USART1("Game over! Press 'N' for new game.\r\n");
            }
        }

        _delay_ms(50);
    }

    return 0;
}