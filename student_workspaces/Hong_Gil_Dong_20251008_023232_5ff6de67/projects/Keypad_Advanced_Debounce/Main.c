/*
 * =============================================================================
 * KEYPAD ADVANCED DEBOUNCING - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: Keypad_Advanced_Debounce
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of advanced keypad programming with robust debouncing.
 * Students learn software debouncing algorithms and input validation techniques.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master software debouncing algorithms and state machines
 * 2. Learn robust input validation and error handling
 * 3. Practice key repeat and long-press detection
 * 4. Implement secure PIN/password entry systems
 * 5. Understand real-time input processing
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - 4x4 matrix keypad with standard layout
 * - Rows (PA0-PA3) configured as outputs
 * - Columns (PA4-PA7) configured as inputs with pull-ups
 * - LCD display for user interface
 * - LEDs for status indication
 * - Serial connection for debugging (9600 baud)
 *
 * DEBOUNCING TECHNIQUES:
 * - Time-based delay debouncing
 * - State machine debouncing
 * - Counter-based validation
 * - Hysteresis filtering
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Basic Debouncing Implementation
 * - Demo 2: State Machine Approach
 * - Demo 3: Advanced Key Detection
 * - Demo 4: Secure PIN Entry System
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

// Debouncing parameters
#define DEBOUNCE_TIME_MS 20
#define LONG_PRESS_TIME_MS 1000
#define REPEAT_DELAY_MS 500
#define REPEAT_RATE_MS 100

const char keypad_map[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};

// Key state tracking
typedef struct
{
    char current_key;
    char last_key;
    uint8_t is_pressed;
    uint16_t press_duration;
    uint8_t debounce_count;
} keypad_state_t;

keypad_state_t key_state = {'\0', '\0', 0, 0, 0};

/*
 * Initialize keypad
 */
void keypad_init(void)
{
    KEYPAD_DDR = ROW_MASK;
    KEYPAD_PORT = 0xFF;
}

/*
 * Raw scan (no debouncing)
 */
char keypad_scan_raw(void)
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

/*
 * Debounced key reading with state machine
 */
char keypad_get_debounced(void)
{
    char raw_key = keypad_scan_raw();

    if (raw_key != '\0')
    {
        if (raw_key == key_state.current_key)
        {
            // Same key, increment debounce counter
            if (key_state.debounce_count < 255)
            {
                key_state.debounce_count++;
            }

            // Key is stable for debounce time
            if (key_state.debounce_count >= 3 && !key_state.is_pressed)
            {
                key_state.is_pressed = 1;
                key_state.press_duration = 0;
                return key_state.current_key; // Valid key press
            }
        }
        else
        {
            // Different key detected
            key_state.current_key = raw_key;
            key_state.debounce_count = 1;
        }
    }
    else
    {
        // No key pressed
        if (key_state.debounce_count > 0)
        {
            key_state.debounce_count--;
        }

        if (key_state.debounce_count == 0)
        {
            key_state.is_pressed = 0;
            key_state.current_key = '\0';
        }
    }

    return '\0'; // No valid key press
}

/*
 * Wait for key with full debouncing
 */
char keypad_getkey_debounced(void)
{
    char key;

    // Wait for key press
    do
    {
        key = keypad_get_debounced();
        _delay_ms(10);
    } while (key == '\0');

    // Wait for key release
    while (keypad_scan_raw() != '\0')
    {
        _delay_ms(10);
    }

    // Reset state
    key_state.current_key = '\0';
    key_state.is_pressed = 0;
    key_state.debounce_count = 0;

    return key;
}

/* ========================================================================
 * DEMO 1: Debouncing Comparison
 * ======================================================================== */
void demo1_debouncing_test(void)
{
    puts_USART1("\r\n=== DEMO 1: Debouncing Comparison ===\r\n");
    puts_USART1("Testing raw vs debounced key reading\r\n");
    puts_USART1("Press keys rapidly, then press 'D' to exit\r\n\r\n");

    lcd_clear();
    lcd_puts_at(0, 0, "Debounce Test:");

    uint16_t raw_count = 0;
    uint16_t debounced_count = 0;
    char last_raw = '\0';

    while (1)
    {
        // Raw scan
        char raw = keypad_scan_raw();
        if (raw != '\0' && raw != last_raw)
        {
            raw_count++;
            last_raw = raw;

            char buf[30];
            sprintf(buf, "Raw: %c (#%u)\r\n", raw, raw_count);
            puts_USART1(buf);
        }
        else if (raw == '\0')
        {
            last_raw = '\0';
        }

        // Debounced scan
        char debounced = keypad_get_debounced();
        if (debounced != '\0')
        {
            debounced_count++;

            char buf[30];
            sprintf(buf, "  Debounced: %c (#%u)\r\n", debounced, debounced_count);
            puts_USART1(buf);

            lcd_goto(1, 0);
            sprintf(buf, "R:%u D:%u    ", raw_count, debounced_count);
            lcd_puts(buf);

            if (debounced == 'D')
            {
                _delay_ms(500);
                char summary[80];
                sprintf(summary, "\r\nResults: Raw=%u, Debounced=%u, Bounces=%u\r\n",
                        raw_count, debounced_count, raw_count - debounced_count);
                puts_USART1(summary);
                return;
            }
        }

        _delay_ms(10);
    }
}

/* ========================================================================
 * DEMO 2: Long Press Detection
 * ======================================================================== */
void demo2_long_press(void)
{
    puts_USART1("\r\n=== DEMO 2: Long Press Detection ===\r\n");
    puts_USART1("Short press = normal, Long press = special\r\n");
    puts_USART1("Press 'D' to exit\r\n\r\n");

    lcd_clear();
    lcd_puts_at(0, 0, "Long Press Test:");

    while (1)
    {
        char key = keypad_scan_raw();

        if (key != '\0' && key_state.current_key == '\0')
        {
            // New key press
            key_state.current_key = key;
            key_state.press_duration = 0;

            lcd_goto(1, 0);
            char buf[20];
            sprintf(buf, "Press: %c...   ", key);
            lcd_puts(buf);
        }
        else if (key != '\0' && key == key_state.current_key)
        {
            // Key still pressed
            key_state.press_duration++;

            if (key_state.press_duration == 100)
            { // ~1 second
                lcd_goto(1, 0);
                char buf[20];
                sprintf(buf, "LONG: %c!!!   ", key);
                lcd_puts(buf);

                sprintf(buf, "Long press detected: %c\r\n", key);
                puts_USART1(buf);

                PORTC = 0xFF;
            }
        }
        else if (key == '\0' && key_state.current_key != '\0')
        {
            // Key released
            char pressed_key = key_state.current_key;
            uint16_t duration = key_state.press_duration;

            char buf[40];
            if (duration < 100)
            {
                sprintf(buf, "Short press: %c (%ums)\r\n", pressed_key, duration * 10);
                lcd_goto(1, 0);
                sprintf(buf, "Short: %c      ", pressed_key);
                lcd_puts(buf);
            }
            else
            {
                sprintf(buf, "Long press: %c (%ums)\r\n", pressed_key, duration * 10);
            }
            puts_USART1(buf);

            if (pressed_key == 'D')
            {
                _delay_ms(500);
                puts_USART1("\r\nExiting demo...\r\n");
                key_state.current_key = '\0';
                return;
            }

            key_state.current_key = '\0';
            PORTC = 0x00;
        }

        _delay_ms(10);
    }
}

/* ========================================================================
 * DEMO 3: PIN Entry System
 * ======================================================================== */
void demo3_pin_entry(void)
{
    puts_USART1("\r\n=== DEMO 3: PIN Entry System ===\r\n");
    puts_USART1("Enter 4-digit PIN (default: 1234)\r\n");
    puts_USART1("Press # to submit, * to clear\r\n\r\n");

    const char correct_pin[] = "1234";
    char entered_pin[5] = {0};
    uint8_t pin_index = 0;
    uint8_t attempts = 0;
    const uint8_t max_attempts = 3;

    lcd_clear();
    lcd_puts_at(0, 0, "Enter PIN:");

    while (attempts < max_attempts)
    {
        lcd_goto(1, 0);
        lcd_puts("PIN: ");

        // Display entered digits as asterisks
        for (uint8_t i = 0; i < pin_index; i++)
        {
            lcd_data('*');
        }
        for (uint8_t i = pin_index; i < 4; i++)
        {
            lcd_data('_');
        }
        lcd_puts("   ");

        // Get key
        char key = keypad_getkey_debounced();

        char buf[30];
        sprintf(buf, "Key: %c\r\n", key);
        puts_USART1(buf);

        if (key >= '0' && key <= '9' && pin_index < 4)
        {
            // Add digit
            entered_pin[pin_index++] = key;
            PORTC = pin_index * 64;
        }
        else if (key == '*')
        {
            // Clear
            pin_index = 0;
            for (uint8_t i = 0; i < 5; i++)
                entered_pin[i] = 0;
            PORTC = 0x00;
            puts_USART1("PIN cleared\r\n");
        }
        else if (key == '#' && pin_index == 4)
        {
            // Submit
            entered_pin[4] = '\0';

            sprintf(buf, "Submitted PIN: %s\r\n", entered_pin);
            puts_USART1(buf);

            // Check PIN
            uint8_t correct = 1;
            for (uint8_t i = 0; i < 4; i++)
            {
                if (entered_pin[i] != correct_pin[i])
                {
                    correct = 0;
                    break;
                }
            }

            if (correct)
            {
                lcd_clear();
                lcd_puts_at(0, 0, "ACCESS GRANTED!");
                lcd_puts_at(1, 0, "  Welcome!");

                puts_USART1("\r\n*** ACCESS GRANTED ***\r\n");

                // Success animation
                for (uint8_t i = 0; i < 5; i++)
                {
                    PORTC = 0xFF;
                    _delay_ms(100);
                    PORTC = 0x00;
                    _delay_ms(100);
                }

                _delay_ms(2000);
                return;
            }
            else
            {
                attempts++;

                lcd_clear();
                lcd_puts_at(0, 0, "ACCESS DENIED!");

                char msg[20];
                sprintf(msg, "Attempts: %u/%u", attempts, max_attempts);
                lcd_puts_at(1, 0, msg);

                sprintf(buf, "Wrong PIN! Attempts: %u/%u\r\n", attempts, max_attempts);
                puts_USART1(buf);

                PORTC = 0xFF;
                _delay_ms(1500);
                PORTC = 0x00;

                if (attempts < max_attempts)
                {
                    lcd_clear();
                    lcd_puts_at(0, 0, "Enter PIN:");
                    pin_index = 0;
                    for (uint8_t i = 0; i < 5; i++)
                        entered_pin[i] = 0;
                }
            }
        }

        _delay_ms(100);
    }

    // Locked out
    lcd_clear();
    lcd_puts_at(0, 0, " SYSTEM LOCKED");
    lcd_puts_at(1, 0, "  Try Again!");

    puts_USART1("\r\n*** SYSTEM LOCKED - Too many attempts ***\r\n");

    for (uint8_t i = 0; i < 10; i++)
    {
        PORTC = 0xFF;
        _delay_ms(200);
        PORTC = 0x00;
        _delay_ms(200);
    }

    _delay_ms(2000);
}

/* ========================================================================
 * DEMO 4: Input Validation and Filtering
 * ======================================================================== */
void demo4_input_validation(void)
{
    puts_USART1("\r\n=== DEMO 4: Input Validation ===\r\n");
    puts_USART1("Enter phone number (digits only, 10 chars)\r\n");
    puts_USART1("# to submit, * to backspace, D to exit\r\n\r\n");

    char phone[11] = {0};
    uint8_t index = 0;

    lcd_clear();
    lcd_puts_at(0, 0, "Phone Number:");

    while (1)
    {
        lcd_goto(1, 0);
        lcd_puts(phone);
        for (uint8_t i = index; i < 10; i++)
        {
            lcd_data('_');
        }
        lcd_puts(" ");

        char key = keypad_getkey_debounced();

        if (key == 'D')
        {
            puts_USART1("\r\nExiting...\r\n");
            return;
        }
        else if (key >= '0' && key <= '9' && index < 10)
        {
            // Valid digit
            phone[index++] = key;

            char buf[30];
            sprintf(buf, "Added digit: %c\r\n", key);
            puts_USART1(buf);

            PORTC = (index * 255) / 10;
        }
        else if (key == '*' && index > 0)
        {
            // Backspace
            index--;
            phone[index] = '\0';

            puts_USART1("Backspace\r\n");
            PORTC = (index * 255) / 10;
        }
        else if (key == '#')
        {
            if (index == 10)
            {
                // Valid submission
                lcd_clear();
                lcd_puts_at(0, 0, "Number Saved:");
                lcd_puts_at(1, 0, phone);

                char buf[50];
                sprintf(buf, "\r\nPhone number saved: %s\r\n", phone);
                puts_USART1(buf);

                PORTC = 0xFF;
                _delay_ms(2000);
                PORTC = 0x00;

                // Reset
                index = 0;
                for (uint8_t i = 0; i < 11; i++)
                    phone[i] = 0;

                lcd_clear();
                lcd_puts_at(0, 0, "Phone Number:");
            }
            else
            {
                // Invalid length
                lcd_clear();
                lcd_puts_at(0, 0, "Error: Need 10");
                lcd_puts_at(1, 0, "digits!");

                puts_USART1("ERROR: Phone must be 10 digits\r\n");

                PORTC = 0xFF;
                _delay_ms(1500);
                PORTC = 0x00;

                lcd_clear();
                lcd_puts_at(0, 0, "Phone Number:");
            }
        }
        else
        {
            // Invalid key
            lcd_clear();
            lcd_puts_at(0, 0, "Invalid key!");
            lcd_puts_at(1, 0, "Digits only");

            char buf[40];
            sprintf(buf, "Invalid key: %c (use digits only)\r\n", key);
            puts_USART1(buf);

            _delay_ms(1000);

            lcd_clear();
            lcd_puts_at(0, 0, "Phone Number:");
        }
    }
}

/* ========================================================================
 * Main Menu System
 * ======================================================================== */
void display_main_menu(void)
{
    puts_USART1("\r\n\r\n");
    puts_USART1("╔════════════════════════════════════════╗\r\n");
    puts_USART1("║  Keypad Advanced - ATmega128          ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Debouncing Test\r\n");
    puts_USART1("  [2] Long Press Detection\r\n");
    puts_USART1("  [3] PIN Entry System\r\n");
    puts_USART1("  [4] Input Validation\r\n");
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
    puts_USART1("\r\n\r\n*** Keypad Advanced Features ***\r\n");
    puts_USART1("Debouncing & Validation\r\n");

    // Welcome screen
    lcd_clear();
    lcd_puts_at(0, 0, "Keypad Advanced");
    lcd_puts_at(1, 0, "  Ready!");

    PORTC = 0x01;
    _delay_ms(2000);

    while (1)
    {
        display_main_menu();

        char choice = getch_USART1();
        putch_USART1(choice);
        puts_USART1("\r\n");

        switch (choice)
        {
        case '1':
            demo1_debouncing_test();
            break;
        case '2':
            demo2_long_press();
            break;
        case '3':
            demo3_pin_entry();
            break;
        case '4':
            demo4_input_validation();
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
