/*
 * =============================================================================
 * KEYPAD MATRIX INPUT - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master 4x4 matrix keypad scanning and input processing
 * DURATION: 90 minutes
 * DIFFICULTY: Intermediate-Advanced
 *
 * STUDENTS WILL:
 * - Implement keypad scanning algorithms
 * - Handle debouncing techniques
 * - Build password entry systems
 * - Create calculator applications
 * - Design menu navigation interfaces
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - 4x4 Matrix Keypad
 *   Layout: 1 2 3 A
 *           4 5 6 B
 *           7 8 9 C
 *           * 0 # D
 *
 * KEYPAD PINOUT:
 * - Rows (Output): PD0-PD3
 * - Columns (Input with pullups): PD4-PD7
 *
 * =============================================================================
 */

#include "config.h"
#include <util/delay.h>

// Keypad configuration
#define KEYPAD_PORT PORTD
#define KEYPAD_PIN PIND
#define KEYPAD_DDR DDRD

#define ROW_MASK 0x0F
#define COL_MASK 0xF0

// Key layout
const char KEYMAP[4][4] PROGMEM = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};

// Global variables
uint16_t lab_score = 0;
volatile uint8_t key_pressed = 0;

/*
 * =============================================================================
 * KEYPAD DRIVER FUNCTIONS
 * =============================================================================
 */

void Keypad_Init(void)
{
    // Rows as outputs (low)
    KEYPAD_DDR = ROW_MASK;
    // Columns as inputs with pullups
    KEYPAD_PORT = COL_MASK;
    KEYPAD_DDR &= ~COL_MASK;
}

char Keypad_Scan(void)
{
    for (uint8_t row = 0; row < 4; row++)
    {
        // Set one row low at a time
        KEYPAD_PORT = COL_MASK | ~(1 << row);
        _delay_us(10); // Stabilization delay

        // Check columns
        uint8_t col_state = KEYPAD_PIN & COL_MASK;

        for (uint8_t col = 0; col < 4; col++)
        {
            if (!(col_state & (1 << (col + 4))))
            {
                // Key pressed at [row][col]
                return pgm_read_byte(&KEYMAP[row][col]);
            }
        }
    }

    return 0; // No key pressed
}

char Keypad_Get_Key(void)
{
    char key = Keypad_Scan();

    if (key)
    {
        // Wait for key release
        while (Keypad_Scan())
            ;
        _delay_ms(50); // Debounce
    }

    return key;
}

char Keypad_Wait_Key(void)
{
    char key;
    while (!(key = Keypad_Get_Key()))
        ;
    return key;
}

/*
 * =============================================================================
 * LAB EXERCISE 1: KEYPAD SCANNING (15 minutes)
 * =============================================================================
 * OBJECTIVE: Understand matrix scanning principles
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex1_basic_scan(void)
{
    /*
     * CHALLENGE: Scan keypad and display pressed keys
     * TASK: Show real-time key detection
     * LEARNING: Row-column scanning, matrix keyboard theory
     */

    puts_USART1("\r\n=== Lab 1.1: Basic Key Scanning ===\r\n");
    puts_USART1("Press keys on the keypad. Press '#' to exit.\r\n\r\n");

    uint16_t key_count = 0;

    while (1)
    {
        char key = Keypad_Get_Key();

        if (key)
        {
            key_count++;

            char buffer[60];
            sprintf(buffer, "Key #%u: '%c' (0x%02X)\r\n", key_count, key, key);
            puts_USART1(buffer);

            // Exit condition
            if (key == '#')
            {
                puts_USART1("\r\nExiting scan test.\r\n");
                break;
            }
        }
    }

    char summary[50];
    sprintf(summary, "Total keys pressed: %u\r\n", key_count);
    puts_USART1(summary);

    lab_score += 75;
}

void lab_ex1_scan_speed_test(void)
{
    /*
     * CHALLENGE: Measure scan rate
     * TASK: Test keypad response time
     * LEARNING: Timing analysis, bounce characteristics
     */

    puts_USART1("\r\n=== Lab 1.2: Scan Speed Test ===\r\n");
    puts_USART1("Press any key rapidly 10 times\r\n\r\n");

    uint32_t start_time = 0; // Would use timer in real implementation
    uint8_t count = 0;

    char last_key = 0;

    while (count < 10)
    {
        char key = Keypad_Scan();

        if (key && key != last_key)
        {
            count++;

            char msg[40];
            sprintf(msg, "Press %u/10: '%c'\r\n", count, key);
            puts_USART1(msg);

            // Wait for release
            while (Keypad_Scan())
                ;
            _delay_ms(50);
        }

        last_key = key;
    }

    puts_USART1("\r\n10 keypresses detected!\r\n");

    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: DEBOUNCING TECHNIQUES (20 minutes)
 * =============================================================================
 * OBJECTIVE: Implement reliable key detection
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

char Keypad_Debounce_Software(void)
{
    /*
     * Software debouncing with multiple reads
     */
    char key1 = Keypad_Scan();
    _delay_ms(10);
    char key2 = Keypad_Scan();
    _delay_ms(10);
    char key3 = Keypad_Scan();

    if (key1 == key2 && key2 == key3 && key1 != 0)
    {
        // Wait for release
        while (Keypad_Scan())
            ;
        _delay_ms(20);
        return key1;
    }

    return 0;
}

void lab_ex2_debounce_comparison(void)
{
    /*
     * CHALLENGE: Compare debouncing methods
     * TASK: Test different debounce algorithms
     * LEARNING: Contact bounce, signal conditioning
     */

    puts_USART1("\r\n=== Lab 2.1: Debounce Methods ===\r\n");
    puts_USART1("Testing debouncing techniques\r\n\r\n");

    puts_USART1("Method 1: Simple delay debounce\r\n");
    puts_USART1("Press 5 keys...\r\n");

    for (uint8_t i = 0; i < 5; i++)
    {
        char key = Keypad_Get_Key();
        if (key)
        {
            char msg[30];
            sprintf(msg, "  Key %u: '%c'\r\n", i + 1, key);
            puts_USART1(msg);
        }
    }

    puts_USART1("\r\nMethod 2: Triple-read verification\r\n");
    puts_USART1("Press 5 keys...\r\n");

    for (uint8_t i = 0; i < 5; i++)
    {
        char key;
        while (!(key = Keypad_Debounce_Software()))
            ;

        char msg[30];
        sprintf(msg, "  Key %u: '%c'\r\n", i + 1, key);
        puts_USART1(msg);
    }

    puts_USART1("\r\nDebounce comparison complete!\r\n");

    lab_score += 125;
}

void lab_ex2_bounce_analyzer(void)
{
    /*
     * CHALLENGE: Visualize contact bounce
     * TASK: Show raw scan data during key press
     * LEARNING: Physical switch characteristics
     */

    puts_USART1("\r\n=== Lab 2.2: Bounce Analysis ===\r\n");
    puts_USART1("Press a key to see bounce pattern\r\n\r\n");

    puts_USART1("Waiting for keypress...\r\n");

    // Wait for first keypress
    while (!Keypad_Scan())
        ;

    puts_USART1("Key detected! Analyzing bounce...\r\n\r\n");

    // Fast scan for 100ms to capture bounce
    uint8_t samples = 0;
    char last_state = 0;
    uint8_t transitions = 0;

    for (uint16_t i = 0; i < 100; i++) // 100ms worth of 1ms samples
    {
        char current = Keypad_Scan();

        if (current != last_state)
        {
            transitions++;

            char msg[50];
            sprintf(msg, "%3ums: %s -> %s\r\n",
                    i,
                    last_state ? "PRESSED" : "RELEASED",
                    current ? "PRESSED" : "RELEASED");
            puts_USART1(msg);
        }

        if (current)
            samples++;
        last_state = current;

        _delay_ms(1);
    }

    puts_USART1("\r\n=== Bounce Statistics ===\r\n");

    char buffer[60];
    sprintf(buffer, "Transitions detected: %u\r\n", transitions);
    puts_USART1(buffer);
    sprintf(buffer, "Pressed samples: %u/100\r\n", samples);
    puts_USART1(buffer);

    if (transitions > 5)
    {
        puts_USART1("Result: Significant bounce detected\r\n");
    }
    else
    {
        puts_USART1("Result: Minimal bounce\r\n");
    }

    lab_score += 150;
}

/*
 * =============================================================================
 * LAB EXERCISE 3: PASSWORD SYSTEM (25 minutes)
 * =============================================================================
 * OBJECTIVE: Build secure input application
 * DIFFICULTY: ★★★★☆ (Advanced)
 */

void lab_ex3_password_entry(void)
{
    /*
     * CHALLENGE: Implement password entry system
     * TASK: Create 4-digit PIN with verification
     * LEARNING: String handling, security concepts
     */

    puts_USART1("\r\n=== Lab 3.1: Password Entry System ===\r\n");
    puts_USART1("Create a 4-digit PIN\r\n\r\n");

    char password[5] = {0};
    char entered[5] = {0};

    // Set password
    puts_USART1("Enter new PIN (4 digits):\r\n");

    for (uint8_t i = 0; i < 4; i++)
    {
        char key;
        do
        {
            key = Keypad_Wait_Key();
        } while (key < '0' || key > '9');

        password[i] = key;
        puts_USART1("*"); // Masked input
    }
    password[4] = 0;

    puts_USART1("\r\n\r\nPIN set successfully!\r\n");
    puts_USART1("Try to unlock (3 attempts)\r\n\r\n");

    uint8_t attempts = 0;

    while (attempts < 3)
    {
        puts_USART1("Enter PIN: ");

        for (uint8_t i = 0; i < 4; i++)
        {
            char key;
            do
            {
                key = Keypad_Wait_Key();
            } while (key < '0' || key > '9');

            entered[i] = key;
            puts_USART1("*");
        }
        entered[4] = 0;

        puts_USART1("\r\n");

        if (strcmp(password, entered) == 0)
        {
            puts_USART1("\r\n*** ACCESS GRANTED ***\r\n");
            puts_USART1("Password correct!\r\n");
            break;
        }
        else
        {
            attempts++;
            char msg[60];
            sprintf(msg, "Wrong PIN! Attempts remaining: %u\r\n\r\n", 3 - attempts);
            puts_USART1(msg);

            if (attempts >= 3)
            {
                puts_USART1("*** ACCESS DENIED ***\r\n");
                puts_USART1("Too many failed attempts!\r\n");
            }
        }
    }

    lab_score += 150;
}

void lab_ex3_pattern_lock(void)
{
    /*
     * CHALLENGE: Pattern-based unlock system
     * TASK: Enter sequence of keys in correct order
     * LEARNING: Sequence matching, state machines
     */

    puts_USART1("\r\n=== Lab 3.2: Pattern Lock ===\r\n");
    puts_USART1("Learn this pattern: 1-4-7-*-#\r\n");
    puts_USART1("Press 'A' when ready to try\r\n\r\n");

    while (Keypad_Wait_Key() != 'A')
        ;

    const char pattern[] = "147*#";
    char input[6] = {0};
    uint8_t attempts = 0;

    while (attempts < 3)
    {
        puts_USART1("Enter pattern (5 keys): ");

        for (uint8_t i = 0; i < 5; i++)
        {
            input[i] = Keypad_Wait_Key();
            putch_USART1(input[i]);
            putch_USART1(' ');
        }
        input[5] = 0;

        puts_USART1("\r\n");

        if (strcmp(pattern, input) == 0)
        {
            puts_USART1("\r\n*** PATTERN CORRECT! ***\r\n");
            puts_USART1("System unlocked!\r\n");
            break;
        }
        else
        {
            attempts++;

            // Show which keys were wrong
            puts_USART1("Pattern: ");
            for (uint8_t i = 0; i < 5; i++)
            {
                if (input[i] == pattern[i])
                {
                    puts_USART1("✓ ");
                }
                else
                {
                    puts_USART1("✗ ");
                }
            }

            char msg[50];
            sprintf(msg, "\r\nAttempts left: %u\r\n\r\n", 3 - attempts);
            puts_USART1(msg);

            if (attempts >= 3)
            {
                puts_USART1("*** LOCKED OUT ***\r\n");
            }
        }
    }

    lab_score += 175;
}

/*
 * =============================================================================
 * LAB EXERCISE 4: CALCULATOR APPLICATION (25 minutes)
 * =============================================================================
 * OBJECTIVE: Build functional calculator
 * DIFFICULTY: ★★★★★ (Expert)
 */

void lab_ex4_simple_calculator(void)
{
    /*
     * CHALLENGE: Create basic calculator
     * TASK: Implement +, -, *, / operations
     * LEARNING: Parser design, arithmetic operations
     */

    puts_USART1("\r\n=== Lab 4.1: Simple Calculator ===\r\n");
    puts_USART1("Format: NUM OP NUM #\r\n");
    puts_USART1("Example: 12 + 34 #\r\n");
    puts_USART1("Operators: + - * /\r\n");
    puts_USART1("Press '*' to exit\r\n\r\n");

    while (1)
    {
        puts_USART1("Enter calculation: ");

        // Get first number
        int16_t num1 = 0;
        char key;

        while (1)
        {
            key = Keypad_Wait_Key();
            putch_USART1(key);
            putch_USART1(' ');

            if (key >= '0' && key <= '9')
            {
                num1 = num1 * 10 + (key - '0');
            }
            else
            {
                break; // Operator or command
            }
        }

        if (key == '*')
        {
            puts_USART1("\r\nExiting calculator.\r\n");
            break;
        }

        char op = key;

        // Get second number
        int16_t num2 = 0;

        while (1)
        {
            key = Keypad_Wait_Key();
            putch_USART1(key);
            putch_USART1(' ');

            if (key >= '0' && key <= '9')
            {
                num2 = num2 * 10 + (key - '0');
            }
            else if (key == '#')
            {
                break; // End of expression
            }
        }

        // Calculate result
        int16_t result = 0;
        uint8_t valid = 1;

        switch (op)
        {
        case '+':
            result = num1 + num2;
            break;
        case '-':
            result = num1 - num2;
            break;
        case 'A':
            result = num1 * num2;
            break; // 'A' for multiply
        case 'B':  // 'B' for divide
            if (num2 != 0)
                result = num1 / num2;
            else
            {
                puts_USART1("\r\nError: Division by zero!\r\n\r\n");
                valid = 0;
            }
            break;
        default:
            puts_USART1("\r\nError: Invalid operator!\r\n\r\n");
            valid = 0;
        }

        if (valid)
        {
            char expr[60];
            sprintf(expr, "\r\n= %d\r\n\r\n", result);
            puts_USART1(expr);
        }
    }

    lab_score += 200;
}

/*
 * =============================================================================
 * LAB MENU SYSTEM
 * =============================================================================
 */

void print_lab_menu(void)
{
    puts_USART1("\r\n");
    puts_USART1("========================================\r\n");
    puts_USART1("  KEYPAD MATRIX INPUT - LAB\r\n");
    puts_USART1("========================================\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 1: Keypad Scanning\r\n");
    puts_USART1("  1. Basic Key Scanning\r\n");
    puts_USART1("  2. Scan Speed Test\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 2: Debouncing Techniques\r\n");
    puts_USART1("  3. Debounce Method Comparison\r\n");
    puts_USART1("  4. Contact Bounce Analyzer\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 3: Password System\r\n");
    puts_USART1("  5. PIN Entry System\r\n");
    puts_USART1("  6. Pattern Lock\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 4: Calculator\r\n");
    puts_USART1("  7. Simple Calculator\r\n");
    puts_USART1("\r\n");
    puts_USART1("  0. Run All Exercises\r\n");
    puts_USART1("  X. Exit Lab\r\n");
    puts_USART1("\r\n");
    char score_str[40];
    sprintf(score_str, "Current Score: %u points\r\n\r\n", lab_score);
    puts_USART1(score_str);
    puts_USART1("Select exercise (1-7, 0, X): ");
}

int main(void)
{
    // Initialize system
    init_devices();
    Uart1_init();
    Keypad_Init();

    _delay_ms(100);

    puts_USART1("\r\n\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("*  ATmega128 KEYPAD MATRIX INPUT LAB           *\r\n");
    puts_USART1("*  4x4 Matrix Keypad Exercises                 *\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("\r\n");
    puts_USART1("Welcome to the Keypad Input Lab!\r\n");
    puts_USART1("Master matrix scanning and input processing.\r\n");

    while (1)
    {
        print_lab_menu();

        char choice = getch_USART1();
        putch_USART1(choice);
        putch_USART1('\r');
        putch_USART1('\n');

        switch (choice)
        {
        case '1':
            lab_ex1_basic_scan();
            break;
        case '2':
            lab_ex1_scan_speed_test();
            break;
        case '3':
            lab_ex2_debounce_comparison();
            break;
        case '4':
            lab_ex2_bounce_analyzer();
            break;
        case '5':
            lab_ex3_password_entry();
            break;
        case '6':
            lab_ex3_pattern_lock();
            break;
        case '7':
            lab_ex4_simple_calculator();
            break;

        case '0':
            puts_USART1("\r\n*** RUNNING ALL EXERCISES ***\r\n");
            lab_ex1_basic_scan();
            lab_ex1_scan_speed_test();
            lab_ex2_debounce_comparison();
            lab_ex2_bounce_analyzer();
            lab_ex3_password_entry();
            lab_ex3_pattern_lock();
            lab_ex4_simple_calculator();

            char final_buffer[80];
            sprintf(final_buffer, "\r\n*** ALL EXERCISES COMPLETE! ***\r\nFinal Score: %u points\r\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\r\nExiting lab. Great work!\r\n");
            while (1)
                ;

        default:
            puts_USART1("Invalid choice. Please try again.\r\n");
        }

        puts_USART1("\r\nPress any key to continue...\r\n");
        getch_USART1();
    }

    return 0;
}
