/*
 * Game Word Puzzle - Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Implement a word search and manipulation game
 * - Learn string processing and pattern matching
 * - Practice character arrays and text algorithms
 * - Master word validation and scoring systems
 *
 * HARDWARE SETUP:
 * - UART for word input and game interaction
 * - LEDs for word length and progress indication
 * - Buzzer for correct/incorrect word feedback
 * - Optional: GLCD for word grid display
 */

#include "config.h"

// Game configuration
#define MAX_WORD_LENGTH 12
#define MAX_WORDS 20
#define LETTER_SET_SIZE 7

// Game structures
typedef struct
{
    char letters[LETTER_SET_SIZE + 1]; // Available letters
    char found_words[MAX_WORDS][MAX_WORD_LENGTH + 1];
    uint8_t word_count;
    uint16_t score;
    uint8_t game_active;
} word_puzzle_t;

// Dictionary of valid words (simplified)
const char *valid_words[] = {
    "CAT", "DOG", "BIRD", "FISH", "TREE", "LEAF", "STAR",
    "CODE", "GAME", "PLAY", "WORK", "TIME", "DATA", "FILE",
    "ATMEGA", "LED", "UART", "PORT", "TIMER", "ADC",
    "MICRO", "CHIP", "BOARD", "WIRE", "VOLT", "AMP"};

#define DICTIONARY_SIZE (sizeof(valid_words) / sizeof(valid_words[0]))

// Global game state
word_puzzle_t word_game;
char current_input[MAX_WORD_LENGTH + 1];
uint8_t input_length = 0;

// Function to generate random letter set
void generate_letter_set()
{
    // Generate a useful set of letters
    const char *letter_sets[] = {
        "CATBIRD",  // Can make: CAT, BIRD, ART, etc.
        "GAMEBOY",  // Can make: GAME, BOY, etc.
        "ATMEGA",   // Can make: ATMEGA, TEAM, etc.
        "CODEFIX",  // Can make: CODE, FIX, etc.
        "PLAYTIME", // Can make: PLAY, TIME, etc.
    };

    uint8_t set_index = (word_game.word_count + word_game.score) % 5;
    strcpy(word_game.letters, letter_sets[set_index]);

    char buffer[40];
    sprintf(buffer, "Letter set: %s\r\n", word_game.letters);
    puts_USART1(buffer);
}

// Function to initialize word puzzle game
void init_word_puzzle()
{
    puts_USART1("Initializing Word Puzzle Game...\r\n");

    // Clear found words
    word_game.word_count = 0;
    for (uint8_t i = 0; i < MAX_WORDS; i++)
    {
        word_game.found_words[i][0] = '\0';
    }

    word_game.score = 0;
    word_game.game_active = 1;

    input_length = 0;
    current_input[0] = '\0';

    // Generate initial letter set
    generate_letter_set();

    puts_USART1("Word Puzzle Ready!\r\n");
    puts_USART1("Goal: Form words using the given letters\r\n");
    puts_USART1("Commands: Type word + ENTER, 'hint' for help, 'new' for new letters\r\n");

    display_game_status();
}

// Function to check if word uses only available letters
uint8_t can_form_word(const char *word)
{
    char letters_copy[LETTER_SET_SIZE + 1];
    strcpy(letters_copy, word_game.letters);

    for (uint8_t i = 0; i < strlen(word); i++)
    {
        char letter = word[i];
        uint8_t found = 0;

        // Find and remove letter from available set
        for (uint8_t j = 0; j < strlen(letters_copy); j++)
        {
            if (letters_copy[j] == letter)
            {
                // Remove this letter (shift array)
                for (uint8_t k = j; k < strlen(letters_copy); k++)
                {
                    letters_copy[k] = letters_copy[k + 1];
                }
                found = 1;
                break;
            }
        }

        if (!found)
            return 0; // Letter not available
    }

    return 1; // All letters available
}

// Function to check if word is valid
uint8_t is_valid_word(const char *word)
{
    // Check against dictionary
    for (uint16_t i = 0; i < DICTIONARY_SIZE; i++)
    {
        if (strcmp(word, valid_words[i]) == 0)
        {
            return 1;
        }
    }
    return 0;
}

// Function to check if word already found
uint8_t is_word_already_found(const char *word)
{
    for (uint8_t i = 0; i < word_game.word_count; i++)
    {
        if (strcmp(word_game.found_words[i], word) == 0)
        {
            return 1;
        }
    }
    return 0;
}

// Function to add word to found words
void add_found_word(const char *word)
{
    if (word_game.word_count < MAX_WORDS)
    {
        strcpy(word_game.found_words[word_game.word_count], word);
        word_game.word_count++;

        // Calculate score based on word length
        uint8_t word_len = strlen(word);
        uint16_t points = word_len * word_len; // Longer words worth more
        word_game.score += points;

        char buffer[50];
        sprintf(buffer, "Word added! +%u points (Total: %u)\r\n", points, word_game.score);
        puts_USART1(buffer);

        // Success sound
        Buzzer_on();
        _delay_ms(200);
        Buzzer_off();

        // Update LED display
        PORTB = ~((1 << word_game.word_count) - 1); // Light up LEDs for found words
    }
}

// Function to process word input
void process_word(const char *word)
{
    // Convert to uppercase
    char upper_word[MAX_WORD_LENGTH + 1];
    for (uint8_t i = 0; i < strlen(word) && i < MAX_WORD_LENGTH; i++)
    {
        upper_word[i] = (word[i] >= 'a' && word[i] <= 'z') ? (word[i] - 'a' + 'A') : word[i];
    }
    upper_word[strlen(word)] = '\0';

    char buffer[60];
    sprintf(buffer, "Checking word: %s\r\n", upper_word);
    puts_USART1(buffer);

    // Validate word
    if (strlen(upper_word) < 2)
    {
        puts_USART1("Word too short! Minimum 2 letters.\r\n");
        return;
    }

    if (!can_form_word(upper_word))
    {
        puts_USART1("Cannot form word with available letters!\r\n");

        // Error sound
        for (uint8_t i = 0; i < 2; i++)
        {
            Buzzer_on();
            _delay_ms(100);
            Buzzer_off();
            _delay_ms(100);
        }
        return;
    }

    if (!is_valid_word(upper_word))
    {
        puts_USART1("Not a valid word in dictionary!\r\n");

        // Error sound
        for (uint8_t i = 0; i < 2; i++)
        {
            Buzzer_on();
            _delay_ms(100);
            Buzzer_off();
            _delay_ms(100);
        }
        return;
    }

    if (is_word_already_found(upper_word))
    {
        puts_USART1("Word already found!\r\n");
        return;
    }

    // Valid new word!
    puts_USART1("*** CORRECT! ***\r\n");
    add_found_word(upper_word);
}

// Function to display game status
void display_game_status()
{
    char buffer[80];

    puts_USART1("\r\n=== WORD PUZZLE STATUS ===\r\n");

    sprintf(buffer, "Available letters: %s\r\n", word_game.letters);
    puts_USART1(buffer);

    sprintf(buffer, "Words found: %u\r\n", word_game.word_count);
    puts_USART1(buffer);

    sprintf(buffer, "Current score: %u\r\n", word_game.score);
    puts_USART1(buffer);

    if (word_game.word_count > 0)
    {
        puts_USART1("Found words: ");
        for (uint8_t i = 0; i < word_game.word_count; i++)
        {
            puts_USART1(word_game.found_words[i]);
            if (i < word_game.word_count - 1)
                puts_USART1(", ");
        }
        puts_USART1("\r\n");
    }

    puts_USART1("\r\nEnter a word: ");
}

// Function to provide hints
void show_hint()
{
    puts_USART1("\r\n=== HINTS ===\r\n");
    puts_USART1("Try these word lengths:\r\n");

    // Show possible word lengths from dictionary
    uint8_t possible_lengths[10] = {0};

    for (uint16_t i = 0; i < DICTIONARY_SIZE; i++)
    {
        if (can_form_word(valid_words[i]) && !is_word_already_found(valid_words[i]))
        {
            uint8_t len = strlen(valid_words[i]);
            if (len <= 9)
                possible_lengths[len] = 1;
        }
    }

    for (uint8_t i = 2; i <= 9; i++)
    {
        if (possible_lengths[i])
        {
            char buffer[30];
            sprintf(buffer, "- %u letter words\r\n", i);
            puts_USART1(buffer);
        }
    }
}

// Function to handle game input
void handle_word_input()
{
    if (is_USART1_received())
    {
        char received = get_USART1();

        if (received == '\r' || received == '\n')
        {
            if (input_length > 0)
            {
                current_input[input_length] = '\0';

                // Check for special commands
                if (strcmp(current_input, "hint") == 0)
                {
                    show_hint();
                }
                else if (strcmp(current_input, "new") == 0)
                {
                    puts_USART1("Generating new letter set...\r\n");
                    generate_letter_set();
                    display_game_status();
                }
                else if (strcmp(current_input, "status") == 0)
                {
                    display_game_status();
                }
                else
                {
                    // Process as word
                    process_word(current_input);
                }

                input_length = 0;
                current_input[0] = '\0';
            }
        }
        else if (received == '\b' || received == 127) // Backspace
        {
            if (input_length > 0)
            {
                input_length--;
                current_input[input_length] = '\0';
                puts_USART1("\b \b"); // Erase character
            }
        }
        else if (input_length < MAX_WORD_LENGTH &&
                 ((received >= 'A' && received <= 'Z') ||
                  (received >= 'a' && received <= 'z')))
        {
            current_input[input_length] = received;
            input_length++;
            put_USART1(received); // Echo character
        }
    }
}

int main(void)
{
    // Initialize system components
    init_devices();
    Uart1_init();
    Buzzer_init();

    puts_USART1("Word Puzzle Game Starting...\r\n");
    puts_USART1("Educational word formation game\r\n");
    puts_USART1("Form words using the given letters!\r\n");

    DDRB = 0xFF;  // Set PORTB as output for LEDs
    PORTB = 0xFF; // Turn off all LEDs initially

    // Initialize the game
    init_word_puzzle();

    while (1)
    {
        if (word_game.game_active)
        {
            // Handle word input
            handle_word_input();
        }

        _delay_ms(50);
    }

    return 0;
}
Puzzle
        *ATmega128 Educational Framework
            *
                *This project implements a word puzzle
                    *game with letter manipulation.
                        * /

#include "config.h"

    int main(void)
{
    main_game_word_puzzle();

    return 0;
}