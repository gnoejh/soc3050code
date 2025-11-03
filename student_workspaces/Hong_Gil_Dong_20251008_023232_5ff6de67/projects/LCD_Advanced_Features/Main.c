/*
 * =============================================================================
 * LCD ADVANCED FEATURES - EDUCATIONAL DEMONSTRATION
 * =============================================================================
 *
 * PROJECT: LCD_Advanced_Features
 * COURSE: SOC 3050 - Embedded Systems and Applications
 * YEAR: 2025
 * AUTHOR: Professor Hong Jeong
 *
 * PURPOSE:
 * Educational demonstration of advanced LCD programming techniques and animations.
 * Students learn sophisticated display control and interactive interface design.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master advanced LCD control techniques
 * 2. Learn scrolling text and animation programming
 * 3. Practice custom character creation and graphics
 * 4. Implement interactive menu systems
 * 5. Design professional user interfaces
 *
 * HARDWARE REQUIREMENTS:
 * - ATmega128 microcontroller @ 16MHz
 * - 16x2 Character LCD (HD44780 compatible)
 * - LCD connections: RS (PG0), E (PG1), D4-D7 (PG2-PG5)
 * - Contrast adjustment potentiometer
 * - Push buttons for menu navigation
 * - Serial connection for debugging (9600 baud)
 *
 * ADVANCED FEATURES:
 * - Horizontal scrolling marquee text
 * - Vertical scrolling displays
 * - Animated progress bars and meters
 * - Custom character animations
 * - Multi-level menu navigation
 *
 * LEARNING PROGRESSION:
 * - Demo 1: Scrolling Text Effects
 * - Demo 2: Progress Bar Animations
 * - Demo 3: Custom Graphics
 * - Demo 4: Interactive Menu Systems
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
#define LCD_D5 3
#define LCD_D6 4
#define LCD_D7 5

// LCD Commands
#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_ENTRY_MODE 0x06
#define LCD_DISPLAY_ON 0x0C
#define LCD_FUNCTION_SET 0x28
#define LCD_CGRAM_ADDR 0x40
#define LCD_DDRAM_ADDR 0x80

// LCD dimensions
#define LCD_ROWS 2
#define LCD_COLS 16

/*
 * Basic LCD functions (same as basic project)
 */
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
    if (cmd == LCD_CLEAR || cmd == LCD_HOME)
        _delay_ms(2);
}

void lcd_data(uint8_t data)
{
    lcd_write_byte(data, 1);
}

void lcd_init(void)
{
    LCD_DDR |= (1 << LCD_RS) | (1 << LCD_E) |
               (1 << LCD_D4) | (1 << LCD_D5) |
               (1 << LCD_D6) | (1 << LCD_D7);

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
    uint8_t address;
    switch (row)
    {
    case 0:
        address = 0x00 + col;
        break;
    case 1:
        address = 0x40 + col;
        break;
    default:
        address = 0x00;
        break;
    }
    lcd_command(LCD_DDRAM_ADDR | address);
}

void lcd_puts(const char *str)
{
    while (*str)
    {
        lcd_data(*str++);
    }
}

void lcd_puts_at(uint8_t row, uint8_t col, const char *str)
{
    lcd_goto(row, col);
    lcd_puts(str);
}

void lcd_create_char(uint8_t location, const uint8_t *pattern)
{
    lcd_command(LCD_CGRAM_ADDR | (location << 3));
    for (uint8_t i = 0; i < 8; i++)
    {
        lcd_data(pattern[i]);
    }
}

/*
 * Clear a specific row
 */
void lcd_clear_row(uint8_t row)
{
    lcd_goto(row, 0);
    for (uint8_t i = 0; i < LCD_COLS; i++)
    {
        lcd_data(' ');
    }
}

/* ========================================================================
 * DEMO 1: Horizontal Scrolling Marquee
 * ======================================================================== */
void demo1_scrolling_text(void)
{
    puts_USART1("\r\n=== DEMO 1: Scrolling Text ===\r\n");
    puts_USART1("Press any key to stop\r\n");

    const char *message = "    ATmega128 Microcontroller - Advanced LCD Features - Scrolling Demo    ";
    uint8_t msg_len = 0;
    while (message[msg_len])
        msg_len++;

    lcd_clear();
    lcd_puts_at(0, 0, "Scrolling Demo:");

    uint8_t offset = 0;

    while (1)
    {
        // Display 16 characters starting from offset
        lcd_goto(1, 0);
        for (uint8_t i = 0; i < LCD_COLS; i++)
        {
            lcd_data(message[(offset + i) % msg_len]);
        }

        offset++;
        if (offset >= msg_len)
            offset = 0;

        // Update UART
        char buf[20];
        sprintf(buf, "\rOffset: %u  ", offset);
        puts_USART1(buf);

        // LED animation
        PORTC = 1 << (offset % 8);

        _delay_ms(200);

        if (UCSR1A & (1 << RXC1))
        {
            getch_USART1();
            puts_USART1("\r\n\r\nScrolling stopped.\r\n");
            return;
        }
    }
}

/* ========================================================================
 * DEMO 2: Progress Bars and Indicators
 * ======================================================================== */
void demo2_progress_bars(void)
{
    puts_USART1("\r\n=== DEMO 2: Progress Bars ===\r\n");

    // Define custom characters for progress bar
    const uint8_t bar_empty[8] = {
        0b11111, 0b10001, 0b10001, 0b10001,
        0b10001, 0b10001, 0b10001, 0b11111};
    const uint8_t bar_full[8] = {
        0b11111, 0b11111, 0b11111, 0b11111,
        0b11111, 0b11111, 0b11111, 0b11111};
    const uint8_t bar_1[8] = {
        0b11111, 0b11001, 0b11001, 0b11001,
        0b11001, 0b11001, 0b11001, 0b11111};
    const uint8_t bar_2[8] = {
        0b11111, 0b11101, 0b11101, 0b11101,
        0b11101, 0b11101, 0b11101, 0b11111};
    const uint8_t bar_3[8] = {
        0b11111, 0b11111, 0b11101, 0b11101,
        0b11101, 0b11101, 0b11111, 0b11111};

    lcd_create_char(0, bar_empty);
    lcd_create_char(1, bar_1);
    lcd_create_char(2, bar_2);
    lcd_create_char(3, bar_3);
    lcd_create_char(4, bar_full);

    // Demo: Loading simulation
    lcd_clear();
    lcd_puts_at(0, 0, "Loading...");

    for (uint8_t percent = 0; percent <= 100; percent += 5)
    {
        // Calculate bar representation
        uint16_t total_segments = LCD_COLS * 4; // 4 segments per character
        uint16_t filled = (total_segments * percent) / 100;

        lcd_goto(1, 0);
        for (uint8_t i = 0; i < LCD_COLS; i++)
        {
            uint16_t seg_pos = i * 4;
            if (seg_pos + 4 <= filled)
            {
                lcd_data(4); // Full
            }
            else if (seg_pos >= filled)
            {
                lcd_data(0); // Empty
            }
            else
            {
                uint8_t partial = filled - seg_pos;
                lcd_data(partial); // Partial (1-3)
            }
        }

        // Display percentage
        char buf[20];
        sprintf(buf, "\rProgress: %u%%    ", percent);
        puts_USART1(buf);

        PORTC = (percent * 255) / 100;
        _delay_ms(100);
    }

    lcd_clear();
    lcd_puts_at(0, 0, "Complete!");
    lcd_puts_at(1, 0, "    100%");

    puts_USART1("\r\n\r\nProgress complete!\r\n");
    _delay_ms(2000);

    puts_USART1("Press any key to continue...");
    getch_USART1();
}

/* ========================================================================
 * DEMO 3: Animated Graphics
 * ======================================================================== */
void demo3_animations(void)
{
    puts_USART1("\r\n=== DEMO 3: Animated Graphics ===\r\n");

    // Spinner animation characters
    const uint8_t spinner1[8] = {
        0b00000, 0b00000, 0b00100, 0b01110,
        0b11111, 0b01110, 0b00100, 0b00000};
    const uint8_t spinner2[8] = {
        0b00000, 0b00100, 0b01100, 0b11100,
        0b11100, 0b01100, 0b00100, 0b00000};
    const uint8_t spinner3[8] = {
        0b00000, 0b01000, 0b01000, 0b11000,
        0b11000, 0b01000, 0b01000, 0b00000};
    const uint8_t spinner4[8] = {
        0b01000, 0b01000, 0b01100, 0b01110,
        0b01110, 0b01100, 0b01000, 0b01000};

    lcd_create_char(0, spinner1);
    lcd_create_char(1, spinner2);
    lcd_create_char(2, spinner3);
    lcd_create_char(3, spinner4);

    lcd_clear();
    lcd_puts_at(0, 0, "Processing...");

    puts_USART1("Displaying spinner animation\r\n");

    for (uint8_t cycle = 0; cycle < 20; cycle++)
    {
        for (uint8_t frame = 0; frame < 4; frame++)
        {
            lcd_goto(1, 7);
            lcd_data(frame);

            PORTC = 1 << frame;
            _delay_ms(100);
        }
    }

    // Bouncing ball animation
    lcd_clear();
    lcd_puts_at(0, 0, "Bouncing:");

    const uint8_t ball[8] = {
        0b00000, 0b00000, 0b01110,
        0b11111, 0b11111, 0b01110,
        0b00000, 0b00000};
    lcd_create_char(0, ball);

    puts_USART1("Displaying bouncing ball\r\n");

    for (uint8_t bounce = 0; bounce < 3; bounce++)
    {
        // Move right
        for (uint8_t pos = 0; pos < LCD_COLS; pos++)
        {
            lcd_clear_row(1);
            lcd_goto(1, pos);
            lcd_data(0);
            _delay_ms(80);
        }
        // Move left
        for (int8_t pos = LCD_COLS - 1; pos >= 0; pos--)
        {
            lcd_clear_row(1);
            lcd_goto(1, pos);
            lcd_data(0);
            _delay_ms(80);
        }
    }

    puts_USART1("\r\nAnimation complete!\r\n");
    puts_USART1("Press any key to continue...");
    getch_USART1();
}

/* ========================================================================
 * DEMO 4: Interactive Menu System
 * ======================================================================== */
void demo4_menu_system(void)
{
    puts_USART1("\r\n=== DEMO 4: Menu System ===\r\n");
    puts_USART1("Commands: w=up, s=down, ENTER=select, q=quit\r\n");

    // Arrow characters
    const uint8_t arrow_right[8] = {
        0b00000, 0b01000, 0b01100, 0b01110,
        0b01100, 0b01000, 0b00000, 0b00000};
    lcd_create_char(0, arrow_right);

    const char *menu_items[] = {
        "Settings",
        "Display",
        "Sensors",
        "About"};
    uint8_t menu_size = 4;
    uint8_t selected = 0;

    while (1)
    {
        // Display menu
        lcd_clear();
        lcd_puts_at(0, 0, "MENU:");

        // Show current and next item
        lcd_goto(1, 0);
        lcd_data(0); // Arrow
        lcd_data(' ');
        lcd_puts(menu_items[selected]);

        // UART feedback
        char buf[40];
        sprintf(buf, "\rSelected: %s      ", menu_items[selected]);
        puts_USART1(buf);

        // LED indicator
        PORTC = 1 << selected;

        // Wait for input
        char cmd = getch_USART1();
        putch_USART1(cmd);

        switch (cmd)
        {
        case 'w':
        case 'W':
            if (selected > 0)
                selected--;
            else
                selected = menu_size - 1;
            break;

        case 's':
        case 'S':
            if (selected < menu_size - 1)
                selected++;
            else
                selected = 0;
            break;

        case '\r':
        case '\n':
            // Item selected
            lcd_clear();
            lcd_puts_at(0, 0, "Selected:");
            lcd_puts_at(1, 0, menu_items[selected]);

            sprintf(buf, "\r\n\r\nOpening: %s\r\n", menu_items[selected]);
            puts_USART1(buf);

            PORTC = 0xFF;
            _delay_ms(1500);
            break;

        case 'q':
        case 'Q':
            puts_USART1("\r\n\r\nExiting menu...\r\n");
            return;
        }

        _delay_ms(100);
    }
}

/* ========================================================================
 * Main Menu System
 * ======================================================================== */
void display_main_menu(void)
{
    puts_USART1("\r\n\r\n");
    puts_USART1("╔════════════════════════════════════════╗\r\n");
    puts_USART1("║  LCD Advanced Features - ATmega128    ║\r\n");
    puts_USART1("╚════════════════════════════════════════╝\r\n");
    puts_USART1("\r\n");
    puts_USART1("Select Demo:\r\n");
    puts_USART1("  [1] Scrolling Text\r\n");
    puts_USART1("  [2] Progress Bars\r\n");
    puts_USART1("  [3] Animated Graphics\r\n");
    puts_USART1("  [4] Interactive Menu\r\n");
    puts_USART1("\r\n");
    puts_USART1("Enter selection (1-4): ");
}

int main(void)
{
    // Initialize peripherals
    Uart1_init();
    lcd_init();

    // Configure status LEDs
    DDRC = 0xFF;
    PORTC = 0x00;

    // Send startup message
    _delay_ms(500);
    puts_USART1("\r\n\r\n*** LCD Advanced Features ***\r\n");
    puts_USART1("Scrolling, Animation, Menus\r\n");

    // Welcome animation
    lcd_clear();
    const char *msg = "  LCD Advanced  ";
    for (uint8_t i = 0; i < 16; i++)
    {
        lcd_goto(0, i);
        lcd_data(msg[i]);
        PORTC = 1 << (i % 8);
        _delay_ms(50);
    }

    lcd_puts_at(1, 0, "  Features!");
    _delay_ms(1500);

    while (1)
    {
        display_main_menu();

        char choice = getch_USART1();
        putch_USART1(choice);
        puts_USART1("\r\n");

        switch (choice)
        {
        case '1':
            demo1_scrolling_text();
            break;
        case '2':
            demo2_progress_bars();
            break;
        case '3':
            demo3_animations();
            break;
        case '4':
            demo4_menu_system();
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
