/*
 * Game Word Puzzle V2 - Enhanced Educational Example
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Advanced word puzzle with multiple game modes
 * - Learn complex string algorithms and data structures
 * - Practice advanced game state management
 * - Master progressive difficulty and achievements
 *
 * HARDWARE SETUP:
 * - UART for enhanced word input and game interface
 * - LEDs for advanced progress and mode indication
 * - Buzzer for rich audio feedback and effects
 * - EEPROM for high score and achievement persistence
 */

#include "config.h"

// Enhanced game configuration
#define MAX_WORD_LENGTH 15
#define MAX_WORDS 30
#define LETTER_SET_SIZE 9
#define NUM_DIFFICULTY_LEVELS 5
#define MAX_ACHIEVEMENTS 8

// Game modes
typedef enum
{
    MODE_CLASSIC,
    MODE_TIMED,
    MODE_TARGET,
    MODE_CHALLENGE
} game_mode_t;

// Achievement types
typedef enum
{
    ACHIEVEMENT_FIRST_WORD,
    ACHIEVEMENT_LONG_WORD,
    ACHIEVEMENT_MANY_WORDS,
    ACHIEVEMENT_HIGH_SCORE,
    ACHIEVEMENT_SPEED_DEMON,
    ACHIEVEMENT_WORD_MASTER,
    ACHIEVEMENT_PERFECTIONIST,
    ACHIEVEMENT_PERSISTENT
} achievement_t;

// Enhanced game structures
typedef struct
{
    char letters[LETTER_SET_SIZE + 1];
    char found_words[MAX_WORDS][MAX_WORD_LENGTH + 1];
    uint8_t word_count;
    uint16_t score;
    uint8_t level;
    game_mode_t mode;
    uint16_t time_remaining;
    uint8_t target_words;
    uint8_t achievements[MAX_ACHIEVEMENTS];
    uint16_t high_score;
    uint8_t game_active;
} word_puzzle_v2_t;

// Extended dictionary with categorized words
const char *tech_words[] = {
    "ATMEGA", "MICRO", "CHIP", "CODE", "DATA", "UART", "TIMER", "PORT",
    "VOLTAGE", "CURRENT", "CIRCUIT", "SENSOR", "DISPLAY", "MEMORY"};

const char *common_words[] = {
    "TIME", "GAME", "PLAY", "WORK", "TEAM", "MAKE", "TAKE", "GIVE",
    "COME", "GOOD", "GREAT", "POWER", "LIGHT", "WATER", "WORLD"};

const char *nature_words[] = {
    "TREE", "LEAF", "BIRD", "FISH", "STAR", "MOON", "EARTH", "WIND",
    "FIRE", "OCEAN", "RIVER", "MOUNTAIN", "FOREST", "FLOWER"};

#define TECH_WORDS_COUNT (sizeof(tech_words) / sizeof(tech_words[0]))
#define COMMON_WORDS_COUNT (sizeof(common_words) / sizeof(common_words[0]))
#define NATURE_WORDS_COUNT (sizeof(nature_words) / sizeof(nature_words[0]))

// Global game state
word_puzzle_v2_t word_game_v2;
char current_input[MAX_WORD_LENGTH + 1];
uint8_t input_length = 0;
uint16_t game_timer = 0;

// Advanced letter sets for different levels
const char *advanced_letter_sets[] = {
    "TEAMWORK",   // Level 1: 8 letters
    "EDUCATION",  // Level 2: 9 letters
    "MICROCHIP",  // Level 3: 9 letters
    "ALGORITHM",  // Level 4: 9 letters
    "PROGRAMMING" // Level 5: 11 letters (truncated to 9)
};

// Function to save high score to EEPROM
void save_high_score()
{
    EEPROM_write(0x10, (word_game_v2.high_score >> 8) & 0xFF);
    _delay_ms(5);
    EEPROM_write(0x11, word_game_v2.high_score & 0xFF);
    _delay_ms(5);
}

// Function to load high score from EEPROM
void load_high_score()
{
    uint8_t high_byte = EEPROM_read(0x10);
    uint8_t low_byte = EEPROM_read(0x11);
    word_game_v2.high_score = (high_byte << 8) | low_byte;

    // Validate reasonable high score
    if (word_game_v2.high_score > 9999)
    {
        word_game_v2.high_score = 0;
        save_high_score();
    }
}

// Function to check and award achievements
void check_achievements()
{
    uint8_t new_achievement = 0;

    // First word achievement
    if (word_game_v2.word_count == 1 && !word_game_v2.achievements[ACHIEVEMENT_FIRST_WORD])
    {
        word_game_v2.achievements[ACHIEVEMENT_FIRST_WORD] = 1;
        puts_USART1("\r\n*** ACHIEVEMENT: First Word! ***\r\n");
        new_achievement = 1;
    }

    // Long word achievement (6+ letters)
    if (!word_game_v2.achievements[ACHIEVEMENT_LONG_WORD])
    {
        for (uint8_t i = 0; i < word_game_v2.word_count; i++)
        {
            if (strlen(word_game_v2.found_words[i]) >= 6)
            {
                word_game_v2.achievements[ACHIEVEMENT_LONG_WORD] = 1;
                puts_USART1("\r\n*** ACHIEVEMENT: Long Word Master! ***\r\n");
                new_achievement = 1;
                break;
            }
        }
    }

    // Many words achievement (10+ words)
    if (word_game_v2.word_count >= 10 && !word_game_v2.achievements[ACHIEVEMENT_MANY_WORDS])
    {
        word_game_v2.achievements[ACHIEVEMENT_MANY_WORDS] = 1;
        puts_USART1("\r\n*** ACHIEVEMENT: Word Collector! ***\r\n");
        new_achievement = 1;
    }

    // High score achievement
    if (word_game_v2.score >= 500 && !word_game_v2.achievements[ACHIEVEMENT_HIGH_SCORE])
    {
        word_game_v2.achievements[ACHIEVEMENT_HIGH_SCORE] = 1;
        puts_USART1("\r\n*** ACHIEVEMENT: High Scorer! ***\r\n");
        new_achievement = 1;
    }

    if (new_achievement)
    {
        // Achievement sound
        for (uint8_t i = 0; i < 4; i++)
        {
            Buzzer_on();
            _delay_ms(100);
            Buzzer_off();
            _delay_ms(50);
        }
    }
}

// Function to generate advanced letter set
void generate_advanced_letter_set()
{
    uint8_t set_index = (word_game_v2.level - 1) % NUM_DIFFICULTY_LEVELS;

    // Copy up to LETTER_SET_SIZE characters
    strncpy(word_game_v2.letters, advanced_letter_sets[set_index], LETTER_SET_SIZE);
    word_game_v2.letters[LETTER_SET_SIZE] = '\0';

    char buffer[50];
    sprintf(buffer, "Level %u letters: %s\r\n", word_game_v2.level, word_game_v2.letters);
    puts_USART1(buffer);
}

// Function to check if word uses only available letters
uint8_t can_form_word_v2(const char *word)
{
    char letters_copy[LETTER_SET_SIZE + 1];
    strcpy(letters_copy, word_game_v2.letters);

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

// Function to check if word is valid (enhanced dictionary)
uint8_t is_valid_word_v2(const char *word)
{
    // Check tech words
    for (uint8_t i = 0; i < TECH_WORDS_COUNT; i++)
    {
        if (strcmp(word, tech_words[i]) == 0)
            return 1;
    }

    // Check common words
    for (uint8_t i = 0; i < COMMON_WORDS_COUNT; i++)
    {
        if (strcmp(word, common_words[i]) == 0)
            return 1;
    }

    // Check nature words
    for (uint8_t i = 0; i < NATURE_WORDS_COUNT; i++)
    {
        if (strcmp(word, nature_words[i]) == 0)
            return 1;
    }

    return 0;
}

// Function to check if word already found
uint8_t is_word_already_found_v2(const char *word)
{
    for (uint8_t i = 0; i < word_game_v2.word_count; i++)
    {
        if (strcmp(word_game_v2.found_words[i], word) == 0)
        {
            return 1;
        }
    }
    return 0;
}

// Function to calculate enhanced score
uint16_t calculate_word_score(const char *word)
{
    uint8_t length = strlen(word);
    uint16_t base_score = length * length;

    // Bonus for different word types
    for (uint8_t i = 0; i < TECH_WORDS_COUNT; i++)
    {
        if (strcmp(word, tech_words[i]) == 0)
        {
            return base_score * 2; // Tech word bonus
        }
    }

    // Bonus for very long words
    if (length >= 7)
        base_score += 50;
    if (length >= 9)
        base_score += 100;

    return base_score;
}

// Function to initialize enhanced word puzzle
void init_word_puzzle_v2()
{
    puts_USART1("Initializing Word Puzzle V2...\r\n");

    // Load saved data
    load_high_score();

    // Clear found words
    word_game_v2.word_count = 0;
    for (uint8_t i = 0; i < MAX_WORDS; i++)
    {
        word_game_v2.found_words[i][0] = '\0';
    }

    word_game_v2.score = 0;
    word_game_v2.level = 1;
    word_game_v2.mode = MODE_CLASSIC;
    word_game_v2.time_remaining = 300; // 5 minutes for timed mode
    word_game_v2.target_words = 5;
    word_game_v2.game_active = 1;

    // Clear achievements for this session
    for (uint8_t i = 0; i < MAX_ACHIEVEMENTS; i++)
    {
        word_game_v2.achievements[i] = 0;
    }

    input_length = 0;
    current_input[0] = '\0';
    game_timer = 0;

    // Generate initial letter set
    generate_advanced_letter_set();

    puts_USART1("Word Puzzle V2 Ready!\r\n");
    puts_USART1("Enhanced features: Multiple modes, achievements, high scores\r\n");
    puts_USART1("Commands: word+ENTER, 'mode', 'stats', 'help'\r\n");

    display_enhanced_status();
}

// Function to display enhanced game status
void display_enhanced_status()
{
    char buffer[100];

    puts_USART1("\r\n=== WORD PUZZLE V2 STATUS ===\r\n");

    sprintf(buffer, "Mode: %s | Level: %u\r\n",
            (word_game_v2.mode == MODE_CLASSIC) ? "Classic" : (word_game_v2.mode == MODE_TIMED) ? "Timed"
                                                          : (word_game_v2.mode == MODE_TARGET)  ? "Target"
                                                                                                : "Challenge",
            word_game_v2.level);
    puts_USART1(buffer);

    sprintf(buffer, "Letters: %s\r\n", word_game_v2.letters);
    puts_USART1(buffer);

    sprintf(buffer, "Score: %u | High Score: %u\r\n", word_game_v2.score, word_game_v2.high_score);
    puts_USART1(buffer);

    sprintf(buffer, "Words found: %u\r\n", word_game_v2.word_count);
    puts_USART1(buffer);

    if (word_game_v2.mode == MODE_TIMED)
    {
        sprintf(buffer, "Time remaining: %u seconds\r\n", word_game_v2.time_remaining);
        puts_USART1(buffer);
    }

    if (word_game_v2.mode == MODE_TARGET)
    {
        sprintf(buffer, "Target: %u words\r\n", word_game_v2.target_words);
        puts_USART1(buffer);
    }

    puts_USART1("\r\nEnter word: ");
}

// Function to process word (enhanced)
void process_word_v2(const char *word)
{
    // Enhanced word processing with better feedback
    char upper_word[MAX_WORD_LENGTH + 1];
    for (uint8_t i = 0; i < strlen(word) && i < MAX_WORD_LENGTH; i++)
    {
        upper_word[i] = (word[i] >= 'a' && word[i] <= 'z') ? (word[i] - 'a' + 'A') : word[i];
    }
    upper_word[strlen(word)] = '\0';

    // Enhanced validation and scoring
    if (strlen(upper_word) >= 2 && can_form_word_v2(upper_word) &&
        is_valid_word_v2(upper_word) && !is_word_already_found_v2(upper_word))
    {
        uint16_t points = calculate_word_score(upper_word);
        word_game_v2.score += points;

        strcpy(word_game_v2.found_words[word_game_v2.word_count], upper_word);
        word_game_v2.word_count++;

        char buffer[80];
        sprintf(buffer, "*** EXCELLENT! *** %s (+%u pts) Total: %u\r\n",
                upper_word, points, word_game_v2.score);
        puts_USART1(buffer);

        // Check for new high score
        if (word_game_v2.score > word_game_v2.high_score)
        {
            word_game_v2.high_score = word_game_v2.score;
            save_high_score();
            puts_USART1("*** NEW HIGH SCORE! ***\r\n");
        }

        check_achievements();

        // Enhanced success sound
        Buzzer_on();
        _delay_ms(250);
        Buzzer_off();
    }
    else
    {
        puts_USART1("Invalid word. Try again!\r\n");
        // Error feedback
        for (uint8_t i = 0; i < 2; i++)
        {
            Buzzer_on();
            _delay_ms(80);
            Buzzer_off();
            _delay_ms(80);
        }
    }
}

// Function to handle enhanced game input
void handle_enhanced_input()
{
    if (is_USART1_received())
    {
        char received = get_USART1();

        if (received == '\r' || received == '\n')
        {
            if (input_length > 0)
            {
                current_input[input_length] = '\0';

                // Enhanced command processing
                if (strcmp(current_input, "mode") == 0)
                {
                    puts_USART1("\r\nGame Modes:\r\n");
                    puts_USART1("1. Classic - No time limit\r\n");
                    puts_USART1("2. Timed - Beat the clock\r\n");
                    puts_USART1("3. Target - Find specific number of words\r\n");
                    puts_USART1("4. Challenge - Advanced difficulty\r\n");
                }
                else if (strcmp(current_input, "stats") == 0)
                {
                    puts_USART1("\r\n=== STATISTICS ===\r\n");
                    char stats[60];
                    sprintf(stats, "Session score: %u\r\n", word_game_v2.score);
                    puts_USART1(stats);
                    sprintf(stats, "All-time high: %u\r\n", word_game_v2.high_score);
                    puts_USART1(stats);
                    sprintf(stats, "Current level: %u\r\n", word_game_v2.level);
                    puts_USART1(stats);
                }
                else if (strcmp(current_input, "help") == 0)
                {
                    puts_USART1("\r\n=== HELP ===\r\n");
                    puts_USART1("Form words using available letters\r\n");
                    puts_USART1("Longer words = more points\r\n");
                    puts_USART1("Tech words give bonus points\r\n");
                    puts_USART1("Unlock achievements for rewards!\r\n");
                }
                else
                {
                    // Process as word (enhanced)
                    process_word_v2(current_input);
                }

                input_length = 0;
                current_input[0] = '\0';
            }
        }
        else if (received == '\b' || received == 127)
        {
            if (input_length > 0)
            {
                input_length--;
                current_input[input_length] = '\0';
                puts_USART1("\b \b");
            }
        }
        else if (input_length < MAX_WORD_LENGTH &&
                 ((received >= 'A' && received <= 'Z') ||
                  (received >= 'a' && received <= 'z')))
        {
            current_input[input_length] = received;
            input_length++;
            put_USART1(received);
        }
    }
}

int main(void)
{
    // Initialize system components
    init_devices();
    Uart1_init();
    Buzzer_init();
    EEPROM_init();

    puts_USART1("Word Puzzle V2 Starting...\r\n");
    puts_USART1("Enhanced word game with achievements and modes\r\n");
    puts_USART1("Challenge yourself to reach new high scores!\r\n");

    DDRB = 0xFF;
    PORTB = 0xFF;

    // Initialize the enhanced game
    init_word_puzzle_v2();

    while (1)
    {
        if (word_game_v2.game_active)
        {
            game_timer++;

            // Handle enhanced input
            handle_enhanced_input();

            // Update timed mode
            if (word_game_v2.mode == MODE_TIMED && game_timer % 10 == 0)
            {
                if (word_game_v2.time_remaining > 0)
                {
                    word_game_v2.time_remaining--;
                }
                else
                {
                    puts_USART1("\r\n*** TIME'S UP! ***\r\n");
                    word_game_v2.game_active = 0;
                }
            }

            // Update LED progress
            uint8_t progress = (word_game_v2.word_count * 8) / 10; // Scale to 8 LEDs
            PORTB = ~((1 << progress) - 1);
        }

        _delay_ms(100);
    }

    return 0;
}