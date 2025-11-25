/*
 * =============================================================================
 * KEYPAD CALCULATOR APPLICATION - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: Keypad_Calculator_App
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of interactive calculator system using matrix keypad.
 * Students learn state machine programming and practical user interface design.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master state machine design for user interfaces
 * 2. Learn multi-digit number entry and validation
 * 3. Practice arithmetic operations and display formatting
 * 4. Implement error handling and user feedback
 * 5. Build practical embedded applications
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - 4x4 matrix keypad with calculator layout
 * - Rows (PA0-PA3) configured as outputs
 * - Columns (PA4-PA7) configured as inputs with pull-ups
 * - LCD display for calculator interface
 * - LEDs for operation status
 * - Serial connection for debugging (9600 baud)
 *
 * CALCULATOR KEYPAD LAYOUT:
 *   [1] [2] [3] [+]  (A = Add)
 *   [4] [5] [6] [-]  (B = Subtract)
 *   [7] [8] [9] [×]  (C = Multiply)
 *   [C] [0] [=] [÷]  (* = Clear, # = Equals, D = Divide)
 *
 * FEATURES:
 * - Basic arithmetic operations (+, -, ×, ÷)
 * - Multi-digit number entry with validation
 * - Error handling (division by zero, overflow)
 * - Clear and reset functions
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Basic Calculator Functions
 * - Demo 2: Advanced Operations
 * - Demo 3: Error Handling
 * - Demo 4: User Interface Enhancement
 *
 * =============================================================================
 */

#include "config.h"

// LCD Control Pins
#define LCD_DDR DDRG
#define LCD_PORT PORTG
#define LCD_RS 0
#define LCD_E 1
#define LCD_D4 2

// LCD Commands
#define LCD_CLEAR 0x01
#define LCD_ENTRY_MODE 0x06
#define LCD_DISPLAY_ON 0x0C
#define LCD_FUNCTION_SET 0x28
#define LCD_DDRAM_ADDR 0x80

// Basic LCD functions
void lcd_enable_pulse(void)
{
    LCD_PORT |= (1 << LCD_E);
    _delay_us(1);
    LCD_PORT &= ~(1 << LCD_E);
    _delay_us(50);
}

void lcd_write_nibble(uint8_t nibble)
{
    LCD_PORT = (LCD_PORT & 0xC3) | ((nibble & 0x0F) << LCD_D4);
    lcd_enable_pulse();
}

void lcd_write_byte(uint8_t data, uint8_t rs)
{
    if (rs)
        LCD_PORT |= (1 << LCD_RS);
    else
        LCD_PORT &= ~(1 << LCD_RS);
    lcd_write_nibble(data >> 4);
    lcd_write_nibble(data & 0x0F);
    _delay_us(50);
}

void lcd_command(uint8_t cmd)
{
    lcd_write_byte(cmd, 0);
    if (cmd == LCD_CLEAR)
        _delay_ms(2);
}

void lcd_data(uint8_t data)
{
    lcd_write_byte(data, 1);
}

void lcd_init(void)
{
    LCD_DDR |= (1 << LCD_RS) | (1 << LCD_E) | (1 << LCD_D4) | (1 << 3) | (1 << 4) | (1 << 5);
    _delay_ms(50);
    LCD_PORT &= ~(1 << LCD_RS);
    lcd_write_nibble(0x03);
    _delay_ms(5);
    lcd_write_nibble(0x03);
    _delay_us(150);
    lcd_write_nibble(0x03);
    _delay_us(150);
    lcd_write_nibble(0x02);
    _delay_us(150);
    lcd_command(LCD_FUNCTION_SET);
    lcd_command(LCD_DISPLAY_ON);
    lcd_command(LCD_CLEAR);
    lcd_command(LCD_ENTRY_MODE);
}

void lcd_clear(void)
{
    lcd_command(LCD_CLEAR);
}

void lcd_goto(uint8_t row, uint8_t col)
{
    uint8_t address = (row == 0) ? 0x00 + col : 0x40 + col;
    lcd_command(LCD_DDRAM_ADDR | address);
}

void lcd_puts(const char *str)
{
    while (*str)
        lcd_data(*str++);
}

void lcd_puts_at(uint8_t row, uint8_t col, const char *str)
{
    lcd_goto(row, col);
    lcd_puts(str);
}

// Keypad definitions
#define KEYPAD_DDR DDRA
#define KEYPAD_PORT PORTA
#define KEYPAD_PIN PINA
#define ROW_MASK 0x0F
#define COL_MASK 0xF0

const char keypad_map[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};

void keypad_init(void)
{
    KEYPAD_DDR = ROW_MASK;
    KEYPAD_PORT = 0xFF;
}

char keypad_scan(void)
{
    for (uint8_t row = 0; row < 4; row++)
    {
        KEYPAD_PORT |= ROW_MASK;
        KEYPAD_PORT &= ~(1 << row);
        _delay_us(5);

        uint8_t cols = KEYPAD_PIN & COL_MASK;

        for (uint8_t col = 0; col < 4; col++)
        {
            if (!(cols & (1 << (col + 4))))
            {
                return keypad_map[row][col];
            }
        }
    }
    return '\0';
}

char keypad_getkey(void)
{
    char key;
    do
    {
        key = keypad_scan();
    } while (key == '\0');

    _delay_ms(20);
    while (keypad_scan() != '\0')
    {
        _delay_ms(10);
    }
    _delay_ms(20);

    return key;
}

/* ========================================================================
 * DEMO 1: Basic Calculator
 * ======================================================================== */
void demo1_calculator(void)
{
    puts_USART1("\r\n=== DEMO 1: Basic Calculator ===\r\n");
    puts_USART1("A=+, B=-, C=*, D=/, #==, *=Clear\r\n\r\n");

    int32_t operand1 = 0;
    int32_t operand2 = 0;
    int32_t result = 0;
    char operation = '\0';
    uint8_t entering_second = 0;
    uint8_t show_result = 0;

    lcd_clear();
    lcd_puts_at(0, 0, "Calculator");
    lcd_puts_at(1, 0, "0");

    while (1)
    {
        char key = keypad_getkey();

        char buf[40];
        sprintf(buf, "Key: %c\r\n", key);
        puts_USART1(buf);

        if (key >= '0' && key <= '9')
        {
            // Digit entered
            if (show_result)
            {
                // Start new calculation
                operand1 = 0;
                operand2 = 0;
                operation = '\0';
                entering_second = 0;
                show_result = 0;
            }

            if (!entering_second)
            {
                operand1 = operand1 * 10 + (key - '0');
                if (operand1 > 9999)
                    operand1 = 9999; // Limit

                lcd_goto(1, 0);
                sprintf(buf, "%ld       ", operand1);
                lcd_puts(buf);
            }
            else
            {
                operand2 = operand2 * 10 + (key - '0');
                if (operand2 > 9999)
                    operand2 = 9999;

                lcd_goto(1, 0);
                sprintf(buf, "%ld%c%ld    ", operand1, operation, operand2);
                lcd_puts(buf);
            }

            PORTC = (PORTC << 1) | 0x01;
            if (PORTC > 0x0F)
                PORTC = 0x01;
        }
        else if (key == 'A' || key == 'B' || key == 'C' || key == 'D')
        {
            // Operation
            if (show_result)
            {
                operand1 = result;
                show_result = 0;
            }

            // Map to operation symbols
            if (key == 'A')
                operation = '+';
            else if (key == 'B')
                operation = '-';
            else if (key == 'C')
                operation = '*';
            else if (key == 'D')
                operation = '/';

            entering_second = 1;
            operand2 = 0;

            lcd_goto(1, 0);
            sprintf(buf, "%ld%c       ", operand1, operation);
            lcd_puts(buf);

            PORTC = 0x03;
        }
        else if (key == '#')
        {
            // Equals
            if (operation != '\0' && entering_second)
            {
                // Calculate result
                switch (operation)
                {
                case '+':
                    result = operand1 + operand2;
                    break;
                case '-':
                    result = operand1 - operand2;
                    break;
                case '*':
                    result = operand1 * operand2;
                    break;
                case '/':
                    if (operand2 == 0)
                    {
                        lcd_clear();
                        lcd_puts_at(0, 0, "Error:");
                        lcd_puts_at(1, 0, "Divide by zero!");

                        puts_USART1("ERROR: Division by zero!\r\n");

                        PORTC = 0xFF;
                        _delay_ms(2000);

                        lcd_clear();
                        lcd_puts_at(0, 0, "Calculator");
                        operand1 = 0;
                        operand2 = 0;
                        operation = '\0';
                        entering_second = 0;
                        result = 0;
                        lcd_puts_at(1, 0, "0");
                        continue;
                    }
                    result = operand1 / operand2;
                    break;
                }

                // Display result
                lcd_goto(0, 0);
                sprintf(buf, "%ld%c%ld=      ", operand1, operation, operand2);
                lcd_puts(buf);

                lcd_goto(1, 0);
                sprintf(buf, "%ld       ", result);
                lcd_puts(buf);

                sprintf(buf, "Calculation: %ld %c %ld = %ld\r\n",
                        operand1, operation, operand2, result);
                puts_USART1(buf);

                show_result = 1;
                PORTC = 0xFF;
                _delay_ms(500);
                PORTC = 0x00;
            }
        }
        else if (key == '*')
        {
            // Clear
            operand1 = 0;
            operand2 = 0;
            operation = '\0';
            entering_second = 0;
            show_result = 0;
            result = 0;

            lcd_clear();
            lcd_puts_at(0, 0, "Calculator");
            lcd_puts_at(1, 0, "0");

            puts_USART1("Cleared\r\n");
            PORTC = 0x00;
        }

        _delay_ms(100);
    }
}

/* ========================================================================
 * DEMO 2: Menu System
 * ======================================================================== */
void demo2_menu_system(void)
{
    puts_USART1("\r\n=== DEMO 2: Interactive Menu ===\r\n");
    puts_USART1("Use keypad to navigate menu\r\n\r\n");

    const char *menu_items[] = {
        "1.Settings",
        "2.Display",
        "3.Sensors",
        "4.About",
        "5.Exit"};
    uint8_t menu_size = 5;
    uint8_t current_page = 0;

    while (1)
    {
        // Display two menu items at a time
        lcd_clear();
        lcd_goto(0, 0);
        if (current_page < menu_size)
        {
            lcd_puts(menu_items[current_page]);
        }
        lcd_goto(1, 0);
        if (current_page + 1 < menu_size)
        {
            lcd_puts(menu_items[current_page + 1]);
        }

        PORTC = 1 << (current_page % 8);

        char key = keypad_getkey();

        if (key == '2' || key == '8')
        {
            // Scroll down
            if (current_page + 2 < menu_size)
            {
                current_page += 2;
            }
        }
        else if (key == '4' || key == '6')
        {
            // Scroll up
            if (current_page >= 2)
            {
                current_page -= 2;
            }
            else
            {
                current_page = 0;
            }
        }
        else if (key >= '1' && key <= '5')
        {
            // Select item
            uint8_t selection = key - '1';

            lcd_clear();
            lcd_puts_at(0, 0, "Selected:");
            lcd_puts_at(1, 0, menu_items[selection] + 2); // Skip number

            char buf[50];
            sprintf(buf, "Menu selection: %s\r\n", menu_items[selection]);
            puts_USART1(buf);

            if (selection == 4)
            {
                _delay_ms(1000);
                puts_USART1("Exiting menu...\r\n");
                return;
            }

            PORTC = 0xFF;
            _delay_ms(2000);
            PORTC = 0x00;
        }
    }
}

/* ========================================================================
 * DEMO 3: Number Guessing Game
 * ======================================================================== */
void demo3_guessing_game(void)
{
    puts_USART1("\r\n=== DEMO 3: Number Guessing Game ===\r\n");
    puts_USART1("Guess number between 0-99\r\n");
    puts_USART1("# to submit, * to clear\r\n\r\n");

    // Simple pseudo-random number (based on timer, not secure)
    uint8_t secret = 42; // For demo, use fixed number

    lcd_clear();
    lcd_puts_at(0, 0, "Guess 0-99:");

    puts_USART1("Secret number set! Start guessing...\r\n");

    uint8_t attempts = 0;

    while (1)
    {
        char guess_str[3] = {0};
        uint8_t digit_count = 0;

        lcd_goto(1, 0);
        lcd_puts("__        ");

        while (1)
        {
            char key = keypad_getkey();

            if (key >= '0' && key <= '9' && digit_count < 2)
            {
                guess_str[digit_count++] = key;

                lcd_goto(1, 0);
                lcd_puts(guess_str);
                lcd_puts("        ");
            }
            else if (key == '*')
            {
                // Clear
                digit_count = 0;
                guess_str[0] = 0;
                guess_str[1] = 0;

                lcd_goto(1, 0);
                lcd_puts("__        ");
            }
            else if (key == '#' && digit_count > 0)
            {
                // Submit guess
                uint8_t guess = 0;
                if (digit_count == 1)
                {
                    guess = guess_str[0] - '0';
                }
                else
                {
                    guess = (guess_str[0] - '0') * 10 + (guess_str[1] - '0');
                }

                attempts++;

                char buf[50];
                sprintf(buf, "Guess #%u: %u\r\n", attempts, guess);
                puts_USART1(buf);

                if (guess == secret)
                {
                    // Correct!
                    lcd_clear();
                    lcd_puts_at(0, 0, "Correct!");
                    sprintf(buf, "In %u tries", attempts);
                    lcd_puts_at(1, 0, buf);

                    sprintf(buf, "*** CORRECT! The number was %u ***\r\n", secret);
                    puts_USART1(buf);
                    sprintf(buf, "You guessed it in %u attempts!\r\n", attempts);
                    puts_USART1(buf);

                    // Victory animation
                    for (uint8_t i = 0; i < 5; i++)
                    {
                        PORTC = 0xFF;
                        _delay_ms(200);
                        PORTC = 0x00;
                        _delay_ms(200);
                    }

                    _delay_ms(2000);
                    return;
                }
                else if (guess < secret)
                {
                    lcd_clear();
                    lcd_puts_at(0, 0, "Too Low!");
                    sprintf(buf, "Try:%u", attempts);
                    lcd_puts_at(1, 0, buf);

                    puts_USART1("  -> Too low! Guess higher.\r\n");

                    PORTC = 0x0F;
                }
                else
                {
                    lcd_clear();
                    lcd_puts_at(0, 0, "Too High!");
                    sprintf(buf, "Try:%u", attempts);
                    lcd_puts_at(1, 0, buf);

                    puts_USART1("  -> Too high! Guess lower.\r\n");

                    PORTC = 0xF0;
                }

                _delay_ms(1500);

                lcd_clear();
                lcd_puts_at(0, 0, "Guess 0-99:");
                break;
            }
        }
    }
}

/* ========================================================================
 * DEMO 4: Stopwatch
 * ======================================================================== */
void demo4_stopwatch(void)
{
    puts_USART1("\r\n=== DEMO 4: Stopwatch ===\r\n");
    puts_USART1("1=Start/Stop, 2=Reset, *=Exit\r\n\r\n");

    uint16_t milliseconds = 0;
    uint8_t running = 0;

    lcd_clear();
    lcd_puts_at(0, 0, "Stopwatch:");
    lcd_puts_at(1, 0, "00:00.0");

    while (1)
    {
        // Check for key press (non-blocking)
        char key = keypad_scan();
        static char last_key = '\0';

        if (key != '\0' && key != last_key)
        {
            last_key = key;

            if (key == '1')
            {
                // Start/Stop
                running = !running;

                char buf[30];
                sprintf(buf, "%s\r\n", running ? "Started" : "Stopped");
                puts_USART1(buf);

                PORTC = running ? 0xFF : 0x00;
            }
            else if (key == '2')
            {
                // Reset
                milliseconds = 0;
                running = 0;

                lcd_puts_at(1, 0, "00:00.0");
                puts_USART1("Reset\r\n");

                PORTC = 0x00;
            }
            else if (key == '*')
            {
                puts_USART1("Exiting stopwatch...\r\n");
                return;
            }
        }
        else if (key == '\0')
        {
            last_key = '\0';
        }

        if (running)
        {
            milliseconds++;

            uint16_t total_seconds = milliseconds / 100;
            uint8_t minutes = total_seconds / 60;
            uint8_t seconds = total_seconds % 60;
            uint8_t tenths = (milliseconds % 100) / 10;

            char buf[20];
            sprintf(buf, "%02u:%02u.%u", minutes, seconds, tenths);
            lcd_puts_at(1, 0, buf);

            // Blink LED
            if (milliseconds % 50 == 0)
            {
                PORTC ^= 0x01;
            }
        }

        _delay_ms(10);
    }
}

/* ========================================================================
 * Main Menu System
 * ======================================================================== */
void display_main_menu(void)
{
    puts_USART1("\r\n\r\n");
    puts_USART1("╔════════════════════════════════════════╗\r\n");
    puts_USART1("║  Keypad Calculator - ATmega128        ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Basic Calculator\r\n");
    puts_USART1("  [2] Menu System\r\n");
    puts_USART1("  [3] Guessing Game\r\n");
    puts_USART1("  [4] Stopwatch\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Initialize peripherals
    Uart1_init();
    lcd_init();
    keypad_init();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** Keypad Calculator System ***\r\n");
    puts_USART1("Interactive Applications\r\n");

    // Welcome screen
    lcd_clear();
    lcd_puts_at(0, 0, "Calculator");
    lcd_puts_at(1, 0, "  Ready!");

    PORTC = 0x0F;
    _delay_ms(2000);
    PORTC = 0x00;

    while (1)
    {
        display_main_menu();

        char choice = getch_USART1();
        putch_USART1(choice);
        puts_USART1("\r\n");

        switch (choice)
        {
        case '1':
            demo1_calculator();
            break;
        case '2':
            demo2_menu_system();
            break;
        case '3':
            demo3_guessing_game();
            break;
        case '4':
            demo4_stopwatch();
            break;
        default:
            puts_USART1("Invalid selection!\r\n");
            lcd_clear();
            lcd_puts_at(0, 0, "Invalid!");
            _delay_ms(1000);
            break;
        }

        _delay_ms(500);
    }

    return 0;
}
