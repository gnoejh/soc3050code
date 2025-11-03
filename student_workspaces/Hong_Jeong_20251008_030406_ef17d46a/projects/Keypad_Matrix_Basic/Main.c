/*
 * =============================================================================
 * MATRIX KEYPAD INTERFACE - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: Keypad_Matrix_Basic
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of matrix keypad scanning and input processing.
 * Students learn efficient input methods and debouncing techniques.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master matrix scanning algorithms and timing
 * 2. Learn row/column scanning techniques
 * 3. Practice key debouncing and state management
 * 4. Implement multiple key detection systems
 * 5. Compare polling vs interrupt-based input methods
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - 4x4 matrix keypad with standard layout
 * - Row and column connections to GPIO ports
 * - LCD display for key feedback
 * - Serial connection for debugging (9600 baud)
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Basic Matrix Scanning
 * - Demo 2: Key Debouncing Implementation
 * - Demo 3: Multiple Key Detection
 * - Demo 4: Key Combination Processing
 * - Demo 5: Advanced Input Validation
 *
 * =============================================================================
 */
*[*][0][#][D] *
    *Connections : *Rows(outputs with pull - up) : *-Row 0 → PA0
    * -Row 1 → PA1
    * -Row 2 → PA2
    * -Row 3 → PA3
    *
    *Columns(inputs with pull - up) : *-Col 0 → PA4
    * -Col 1 → PA5
    * -Col 2 → PA6
    * -Col 3 → PA7
    *
    *SCANNING PRINCIPLE : *1. Set one row LOW,
    others HIGH
        * 2. Read all columns
        * 3. If column is LOW,
    key at(row, col) is pressed
 * 4. Repeat for all rows
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

// Keypad port definitions
#define KEYPAD_DDR DDRA
#define KEYPAD_PORT PORTA
#define KEYPAD_PIN PINA

#define ROW_MASK 0x0F // PA0-PA3 (rows)
#define COL_MASK 0xF0 // PA4-PA7 (columns)

// Keypad layout
const char keypad_map[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};

// Key state tracking
char last_key = '\0';
uint8_t key_pressed = 0;

/*
 * Initialize keypad
 */
void keypad_init(void)
{
    // Rows as outputs (PA0-PA3), Columns as inputs (PA4-PA7)
    KEYPAD_DDR = ROW_MASK;

    // Enable pull-ups on columns, set rows HIGH initially
    KEYPAD_PORT = 0xFF;
}

/*
 * Scan keypad and return pressed key (0 if none)
 */
char keypad_scan(void)
{
    for (uint8_t row = 0; row < 4; row++)
    {
        // Set all rows HIGH
        KEYPAD_PORT |= ROW_MASK;

        // Set current row LOW
        KEYPAD_PORT &= ~(1 << row);

        // Small delay for signal to stabilize
        _delay_us(5);

        // Read columns
        uint8_t cols = KEYPAD_PIN & COL_MASK;

        // Check each column
        for (uint8_t col = 0; col < 4; col++)
        {
            if (!(cols & (1 << (col + 4))))
            {
                // Key pressed at (row, col)
                return keypad_map[row][col];
            }
        }
    }

    // No key pressed
    return '\0';
}

/*
 * Get key with debouncing (blocking)
 */
char keypad_getkey(void)
{
    char key;

    // Wait for key press
    do
    {
        key = keypad_scan();
    } while (key == '\0');

    // Debounce delay
    _delay_ms(20);

    // Wait for key release
    while (keypad_scan() != '\0')
    {
        _delay_ms(10);
    }

    // Debounce delay
    _delay_ms(20);

    return key;
}

/* ========================================================================
 * DEMO 1: Basic Key Detection
 * ======================================================================== */
void demo1_basic_detection(void)
{
    puts_USART1("\r\n=== DEMO 1: Basic Key Detection ===\r\n");
    puts_USART1("Press keys on the keypad\r\n");
    puts_USART1("Press 'D' to exit\r\n\r\n");

    lcd_clear();
    lcd_puts_at(0, 0, "Press any key:");

    uint16_t key_count = 0;

    while (1)
    {
        char key = keypad_scan();

        if (key != '\0' && key != last_key)
        {
            // New key pressed
            key_count++;

            // Display on LCD
            char buf[20];
            sprintf(buf, "Key: %c (#%u)  ", key, key_count);
            lcd_puts_at(1, 0, buf);

            // Send to UART
            sprintf(buf, "Key pressed: '%c' (Count: %u)\r\n", key, key_count);
            puts_USART1(buf);

            // LED pattern based on key
            if (key >= '0' && key <= '9')
            {
                PORTC = key - '0';
            }
            else
            {
                PORTC = 0xFF;
            }

            last_key = key;

            // Exit on 'D'
            if (key == 'D')
            {
                _delay_ms(500);
                puts_USART1("\r\nExiting demo...\r\n");
                return;
            }
        }
        else if (key == '\0')
        {
            last_key = '\0';
            PORTC = 0x00;
        }

        _delay_ms(10);
    }
}

/* ========================================================================
 * DEMO 2: Keypad Test Pattern
 * ======================================================================== */
void demo2_test_pattern(void)
{
    puts_USART1("\r\n=== DEMO 2: Keypad Test Pattern ===\r\n");
    puts_USART1("Press all keys in order: 1-9, 0, *, #, A-D\r\n\r\n");

    const char test_sequence[] = "123456789*0#ABCD";
    uint8_t seq_index = 0;
    uint8_t total_keys = 16;

    lcd_clear();
    lcd_puts_at(0, 0, "Test: Press");

    char buf[20];
    sprintf(buf, "Key: %c (%u/%u)", test_sequence[seq_index], seq_index + 1, total_keys);
    lcd_puts_at(1, 0, buf);

    while (seq_index < total_keys)
    {
        char key = keypad_getkey();

        sprintf(buf, "Pressed: '%c', Expected: '%c'\r\n", key, test_sequence[seq_index]);
        puts_USART1(buf);

        if (key == test_sequence[seq_index])
        {
            // Correct key!
            puts_USART1("  ✓ Correct!\r\n");
            seq_index++;

            // Progress LEDs
            PORTC = (seq_index * 255) / total_keys;

            if (seq_index < total_keys)
            {
                sprintf(buf, "Key: %c (%u/%u)", test_sequence[seq_index], seq_index + 1, total_keys);
                lcd_puts_at(1, 0, buf);
            }
        }
        else
        {
            // Wrong key
            puts_USART1("  ✗ Wrong key!\r\n");
            lcd_clear();
            lcd_puts_at(0, 0, "Wrong! Try:");
            sprintf(buf, "%c", test_sequence[seq_index]);
            lcd_puts_at(1, 0, buf);

            // Flash LEDs
            for (uint8_t i = 0; i < 3; i++)
            {
                PORTC = 0xFF;
                _delay_ms(100);
                PORTC = 0x00;
                _delay_ms(100);
            }

            lcd_clear();
            lcd_puts_at(0, 0, "Test: Press");
            sprintf(buf, "Key: %c (%u/%u)", test_sequence[seq_index], seq_index + 1, total_keys);
            lcd_puts_at(1, 0, buf);
        }
    }

    // All keys tested!
    lcd_clear();
    lcd_puts_at(0, 0, "Test Complete!");
    lcd_puts_at(1, 0, "All keys OK");

    puts_USART1("\r\n✓ Keypad test PASSED!\r\n");
    PORTC = 0xFF;
    _delay_ms(2000);

    puts_USART1("Press any key to continue...");
    keypad_getkey();
}

/* ========================================================================
 * DEMO 3: Multi-Key Detection
 * ======================================================================== */
void demo3_multi_key(void)
{
    puts_USART1("\r\n=== DEMO 3: Multi-Key Detection ===\r\n");
    puts_USART1("Detecting simultaneous key presses\r\n");
    puts_USART1("Press 'D' to exit\r\n\r\n");

    lcd_clear();
    lcd_puts_at(0, 0, "Multi-Key Test:");

    while (1)
    {
        uint8_t key_count = 0;
        char keys_pressed[16];

        // Scan for all pressed keys
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
                    keys_pressed[key_count++] = keypad_map[row][col];
                }
            }
        }

        if (key_count > 0)
        {
            // Display pressed keys
            lcd_goto(1, 0);
            char buf[20];

            if (key_count == 1)
            {
                sprintf(buf, "Keys: %c       ", keys_pressed[0]);
                lcd_puts(buf);

                // Check for exit
                if (keys_pressed[0] == 'D')
                {
                    _delay_ms(500);
                    puts_USART1("\r\nExiting demo...\r\n");
                    return;
                }
            }
            else
            {
                sprintf(buf, "Keys:%u [", key_count);
                lcd_puts(buf);

                puts_USART1("\rMultiple keys: ");
                for (uint8_t i = 0; i < key_count && i < 5; i++)
                {
                    lcd_data(keys_pressed[i]);
                    putch_USART1(keys_pressed[i]);
                    putch_USART1(' ');
                }
                lcd_puts("]   ");
                puts_USART1("   ");
            }

            PORTC = key_count * 32;
        }
        else
        {
            lcd_puts_at(1, 0, "No keys         ");
            PORTC = 0x00;
        }

        _delay_ms(50);
    }
}

/* ========================================================================
 * DEMO 4: Keypad Frequency Counter
 * ======================================================================== */
void demo4_frequency_test(void)
{
    puts_USART1("\r\n=== DEMO 4: Scan Frequency Test ===\r\n");
    puts_USART1("Measuring keypad scan rate\r\n");
    puts_USART1("Press any key, hold, then release\r\n\r\n");

    lcd_clear();
    lcd_puts_at(0, 0, "Scan Rate Test:");

    puts_USART1("Starting scan rate measurement...\r\n");

    uint32_t scan_count = 0;
    uint16_t key_detect_count = 0;

    // Run for 5 seconds
    for (uint8_t sec = 0; sec < 5; sec++)
    {
        uint32_t start_count = scan_count;
        uint16_t start_detect = key_detect_count;

        // Scan for 1 second
        uint16_t iterations = 0;
        while (iterations < 100)
        { // ~1 second at 10ms per iteration
            char key = keypad_scan();
            scan_count++;

            if (key != '\0')
            {
                key_detect_count++;
                lcd_goto(1, 0);
                char buf[20];
                sprintf(buf, "Key: %c Cnt:%u ", key, key_detect_count);
                lcd_puts(buf);
                PORTC = 0xFF;
            }
            else
            {
                PORTC = 0x01;
            }

            _delay_ms(10);
            iterations++;
        }

        // Report for this second
        char buf[60];
        sprintf(buf, "Sec %u: Scans=%lu, Detections=%u\r\n",
                sec + 1, scan_count - start_count, key_detect_count - start_detect);
        puts_USART1(buf);
    }

    // Final report
    char buf[80];
    sprintf(buf, "\r\nTotal scans: %lu (~%lu scans/sec)\r\n", scan_count, scan_count / 5);
    puts_USART1(buf);
    sprintf(buf, "Total detections: %u\r\n", key_detect_count);
    puts_USART1(buf);

    lcd_clear();
    lcd_puts_at(0, 0, "Test Complete!");
    sprintf(buf, "Rate:%lu/s", scan_count / 5);
    lcd_puts_at(1, 0, buf);

    _delay_ms(2000);

    puts_USART1("Press any key to continue...");
    keypad_getkey();
}

/* ========================================================================
 * Main Menu System
 * ======================================================================== */
void display_main_menu(void)
{
    puts_USART1("\r\n\r\n");
    puts_USART1("╔════════════════════════════════════════╗\r\n");
    puts_USART1("║  Keypad Matrix Scanning - ATmega128  ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Basic Key Detection\r\n");
    puts_USART1("  [2] Keypad Test Pattern\r\n");
    puts_USART1("  [3] Multi-Key Detection\r\n");
    puts_USART1("  [4] Scan Frequency Test\r\n");
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
    puts_USART1("\r\n\r\n*** 4x4 Matrix Keypad Interface ***\r\n");
    puts_USART1("Row/Column scanning technique\r\n");

    // Welcome screen
    lcd_clear();
    lcd_puts_at(0, 0, " Keypad Ready");
    lcd_puts_at(1, 0, " 4x4 Matrix");

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
            demo1_basic_detection();
            break;
        case '2':
            demo2_test_pattern();
            break;
        case '3':
            demo3_multi_key();
            break;
        case '4':
            demo4_frequency_test();
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
