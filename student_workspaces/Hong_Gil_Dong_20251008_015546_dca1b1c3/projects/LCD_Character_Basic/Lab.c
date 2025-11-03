/*
 * =============================================================================
 * LCD CHARACTER DISPLAY - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master LCD character display control (HD44780 compatible)
 * DURATION: 90 minutes
 * DIFFICULTY: Intermediate
 *
 * STUDENTS WILL:
 * - Control 16x2 or 20x4 character LCD
 * - Create custom characters
 * - Design animated displays
 * - Build real-time sensor dashboards
 * - Implement user interfaces
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - HD44780 LCD (16x2 or 20x4)
 * - ADC sensors (optional for dashboard)
 * - Buttons for navigation
 *
 * LCD PINOUT (4-bit mode):
 * - RS  -> PC0
 * - RW  -> GND (write only)
 * - E   -> PC1
 * - D4-D7 -> PC4-PC7
 *
 * =============================================================================
 */

#include "config.h"
#include <util/delay.h>

// LCD configuration
#define LCD_RS PC0
#define LCD_E PC1
#define LCD_DATA PORTC
#define LCD_DDR DDRC

#define LCD_ROWS 2
#define LCD_COLS 16

// Global variables
uint16_t lab_score = 0;
uint32_t milliseconds = 0;

/*
 * =============================================================================
 * LCD DRIVER FUNCTIONS (Basic 4-bit mode)
 * =============================================================================
 */

void LCD_Pulse_Enable(void)
{
    LCD_DATA |= (1 << LCD_E);
    _delay_us(1);
    LCD_DATA &= ~(1 << LCD_E);
    _delay_us(50);
}

void LCD_Send_Nibble(uint8_t nibble)
{
    LCD_DATA = (LCD_DATA & 0x0F) | (nibble & 0xF0);
    LCD_Pulse_Enable();
}

void LCD_Send_Byte(uint8_t byte, uint8_t rs)
{
    if (rs)
        LCD_DATA |= (1 << LCD_RS);
    else
        LCD_DATA &= ~(1 << LCD_RS);

    LCD_Send_Nibble(byte);
    LCD_Send_Nibble(byte << 4);
    _delay_us(50);
}

void LCD_Command(uint8_t cmd)
{
    LCD_Send_Byte(cmd, 0);
    if (cmd <= 3)
        _delay_ms(2); // Clear/Home commands need more time
}

void LCD_Data(uint8_t data)
{
    LCD_Send_Byte(data, 1);
}

void LCD_Init(void)
{
    LCD_DDR = 0xFF;
    _delay_ms(50);

    // Initialize 4-bit mode
    LCD_Send_Nibble(0x30);
    _delay_ms(5);
    LCD_Send_Nibble(0x30);
    _delay_us(150);
    LCD_Send_Nibble(0x30);
    _delay_us(150);
    LCD_Send_Nibble(0x20);
    _delay_us(150);

    LCD_Command(0x28); // 4-bit, 2 lines, 5x8 font
    LCD_Command(0x0C); // Display ON, cursor OFF
    LCD_Command(0x06); // Entry mode: increment, no shift
    LCD_Command(0x01); // Clear display
    _delay_ms(2);
}

void LCD_Clear(void)
{
    LCD_Command(0x01);
    _delay_ms(2);
}

void LCD_Goto(uint8_t row, uint8_t col)
{
    uint8_t address = (row == 0) ? 0x00 : 0x40;
    address += col;
    LCD_Command(0x80 | address);
}

void LCD_Print(const char *str)
{
    while (*str)
    {
        LCD_Data(*str++);
    }
}

void LCD_Print_P(const char *str)
{
    char c;
    while ((c = pgm_read_byte(str++)))
    {
        LCD_Data(c);
    }
}

void LCD_Create_Char(uint8_t location, const uint8_t *pattern)
{
    LCD_Command(0x40 + (location * 8));
    for (uint8_t i = 0; i < 8; i++)
    {
        LCD_Data(pattern[i]);
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 1: BASIC LCD CONTROL (15 minutes)
 * =============================================================================
 * OBJECTIVE: Master LCD positioning and text display
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex1_hello_world(void)
{
    /*
     * CHALLENGE: Display text at specific positions
     * TASK: Show multiple lines and positioning
     * LEARNING: LCD coordinates, text placement
     */

    puts_USART1("\r\n=== Lab 1.1: Hello World ===\r\n");

    LCD_Clear();

    // Row 0: Left-aligned
    LCD_Goto(0, 0);
    LCD_Print("Hello, ATmega128");

    _delay_ms(2000);

    // Row 1: Centered text
    LCD_Goto(1, 0);
    LCD_Print("  Embedded Lab  ");

    _delay_ms(3000);

    // Changing text
    LCD_Clear();
    LCD_Goto(0, 0);
    LCD_Print("Position Test:");

    for (uint8_t col = 0; col < 16; col++)
    {
        LCD_Goto(1, col);
        LCD_Data('*');
        _delay_ms(200);
    }

    _delay_ms(1000);

    puts_USART1("Hello World complete!\r\n");
    lab_score += 75;
}

void lab_ex1_scrolling_text(void)
{
    /*
     * CHALLENGE: Create smooth scrolling text
     * TASK: Scroll message across display
     * LEARNING: String manipulation, timing
     */

    puts_USART1("\r\n=== Lab 1.2: Scrolling Text ===\r\n");

    const char message[] = "*** Welcome to ATmega128 LCD Lab! Learn embedded systems with hands-on exercises ***";
    uint8_t msg_len = strlen(message);

    LCD_Clear();
    LCD_Goto(0, 0);
    LCD_Print("Scrolling Demo:");

    for (uint8_t offset = 0; offset < msg_len - 16; offset++)
    {
        LCD_Goto(1, 0);

        for (uint8_t i = 0; i < 16; i++)
        {
            LCD_Data(message[offset + i]);
        }

        _delay_ms(300);
    }

    _delay_ms(1000);

    puts_USART1("Scrolling text complete!\r\n");
    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: CUSTOM CHARACTERS (20 minutes)
 * =============================================================================
 * OBJECTIVE: Design and display custom 5x8 characters
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

// Custom character patterns (5x8 pixels)
const uint8_t CHAR_HEART[] PROGMEM = {
    0b00000,
    0b01010,
    0b11111,
    0b11111,
    0b01110,
    0b00100,
    0b00000,
    0b00000};

const uint8_t CHAR_BELL[] PROGMEM = {
    0b00100,
    0b01110,
    0b01110,
    0b01110,
    0b11111,
    0b00000,
    0b00100,
    0b00000};

const uint8_t CHAR_SPEAKER[] PROGMEM = {
    0b00001,
    0b00011,
    0b01111,
    0b01111,
    0b01111,
    0b00011,
    0b00001,
    0b00000};

const uint8_t CHAR_UP_ARROW[] PROGMEM = {
    0b00100,
    0b01110,
    0b11111,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00000};

const uint8_t CHAR_DOWN_ARROW[] PROGMEM = {
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b11111,
    0b01110,
    0b00100,
    0b00000};

const uint8_t CHAR_BATTERY_FULL[] PROGMEM = {
    0b01110,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b00000};

const uint8_t CHAR_BATTERY_HALF[] PROGMEM = {
    0b01110,
    0b11111,
    0b11111,
    0b11111,
    0b10001,
    0b10001,
    0b11111,
    0b00000};

const uint8_t CHAR_BATTERY_EMPTY[] PROGMEM = {
    0b01110,
    0b11111,
    0b10001,
    0b10001,
    0b10001,
    0b10001,
    0b11111,
    0b00000};

void lab_ex2_custom_characters(void)
{
    /*
     * CHALLENGE: Create and display custom icons
     * TASK: Load 8 custom characters and display them
     * LEARNING: CGRAM programming, icon design
     */

    puts_USART1("\r\n=== Lab 2.1: Custom Characters ===\r\n");

    // Load custom characters into CGRAM
    uint8_t pattern[8];

    // Character 0: Heart
    for (uint8_t i = 0; i < 8; i++)
        pattern[i] = pgm_read_byte(&CHAR_HEART[i]);
    LCD_Create_Char(0, pattern);

    // Character 1: Bell
    for (uint8_t i = 0; i < 8; i++)
        pattern[i] = pgm_read_byte(&CHAR_BELL[i]);
    LCD_Create_Char(1, pattern);

    // Character 2: Speaker
    for (uint8_t i = 0; i < 8; i++)
        pattern[i] = pgm_read_byte(&CHAR_SPEAKER[i]);
    LCD_Create_Char(2, pattern);

    // Character 3: Up Arrow
    for (uint8_t i = 0; i < 8; i++)
        pattern[i] = pgm_read_byte(&CHAR_UP_ARROW[i]);
    LCD_Create_Char(3, pattern);

    // Character 4: Down Arrow
    for (uint8_t i = 0; i < 8; i++)
        pattern[i] = pgm_read_byte(&CHAR_DOWN_ARROW[i]);
    LCD_Create_Char(4, pattern);

    // Character 5-7: Battery states
    for (uint8_t i = 0; i < 8; i++)
        pattern[i] = pgm_read_byte(&CHAR_BATTERY_FULL[i]);
    LCD_Create_Char(5, pattern);

    for (uint8_t i = 0; i < 8; i++)
        pattern[i] = pgm_read_byte(&CHAR_BATTERY_HALF[i]);
    LCD_Create_Char(6, pattern);

    for (uint8_t i = 0; i < 8; i++)
        pattern[i] = pgm_read_byte(&CHAR_BATTERY_EMPTY[i]);
    LCD_Create_Char(7, pattern);

    // Display custom characters
    LCD_Clear();
    LCD_Goto(0, 0);
    LCD_Print("Custom Icons:");

    LCD_Goto(1, 0);
    LCD_Data(0); // Heart
    LCD_Data(' ');
    LCD_Data(1); // Bell
    LCD_Data(' ');
    LCD_Data(2); // Speaker
    LCD_Data(' ');
    LCD_Data(3); // Up
    LCD_Data(' ');
    LCD_Data(4); // Down
    LCD_Data(' ');
    LCD_Data(5); // Battery full
    LCD_Data(' ');
    LCD_Data(6); // Battery half
    LCD_Data(' ');
    LCD_Data(7); // Battery empty

    _delay_ms(5000);

    puts_USART1("Custom characters created!\r\n");
    lab_score += 125;
}

void lab_ex2_animated_loading(void)
{
    /*
     * CHALLENGE: Create animated loading bar
     * TASK: Show progress with custom block characters
     * LEARNING: Animation timing, visual feedback
     */

    puts_USART1("\r\n=== Lab 2.2: Animated Loading Bar ===\r\n");

    // Custom loading characters (partial blocks)
    const uint8_t load_chars[4][8] = {
        {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00}, // 1/4
        {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00}, // 2/4
        {0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x00}, // 3/4
        {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x00}  // 4/4 (full)
    };

    for (uint8_t i = 0; i < 4; i++)
    {
        LCD_Create_Char(i, load_chars[i]);
    }

    LCD_Clear();
    LCD_Goto(0, 0);
    LCD_Print("Loading...");

    LCD_Goto(1, 0);
    LCD_Data('[');
    LCD_Goto(1, 15);
    LCD_Data(']');

    // Animate loading bar
    for (uint8_t pos = 0; pos < 14; pos++)
    {
        LCD_Goto(1, 1 + pos);

        // Show gradual fill
        for (uint8_t phase = 0; phase < 4; phase++)
        {
            LCD_Goto(1, 1 + pos);
            LCD_Data(phase);
            _delay_ms(50);
        }

        uint8_t percent = ((pos + 1) * 100) / 14;
        LCD_Goto(0, 11);

        char pct[6];
        sprintf(pct, "%3u%%", percent);
        LCD_Print(pct);
    }

    _delay_ms(1000);

    LCD_Clear();
    LCD_Goto(0, 0);
    LCD_Print("   Complete!   ");
    LCD_Goto(1, 0);
    LCD_Data(0); // Heart
    LCD_Print("  Loading Done ");
    LCD_Data(0); // Heart

    _delay_ms(2000);

    puts_USART1("Loading animation complete!\r\n");
    lab_score += 150;
}

/*
 * =============================================================================
 * LAB EXERCISE 3: REAL-TIME DISPLAY (25 minutes)
 * =============================================================================
 * OBJECTIVE: Create live updating displays
 * DIFFICULTY: ★★★★☆ (Advanced)
 */

void lab_ex3_digital_clock(void)
{
    /*
     * CHALLENGE: Display running clock
     * TASK: Show time in HH:MM:SS format
     * LEARNING: Time tracking, display formatting
     */

    puts_USART1("\r\n=== Lab 3.1: Digital Clock ===\r\n");
    puts_USART1("Clock running for 1 minute. Press 'Q' to exit.\r\n");

    LCD_Clear();
    LCD_Goto(0, 0);
    LCD_Print("  Digital Clock ");

    uint8_t seconds = 0, minutes = 0, hours = 12;

    for (uint16_t ticks = 0; ticks < 600; ticks++) // 60 seconds
    {
        LCD_Goto(1, 4);

        char time_str[9];
        sprintf(time_str, "%02u:%02u:%02u", hours, minutes, seconds);
        LCD_Print(time_str);

        _delay_ms(100);

        if (ticks % 10 == 9) // Every second
        {
            seconds++;
            if (seconds >= 60)
            {
                seconds = 0;
                minutes++;
                if (minutes >= 60)
                {
                    minutes = 0;
                    hours++;
                    if (hours >= 24)
                        hours = 0;
                }
            }
        }

        // Check for exit
        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    puts_USART1("Clock test complete!\r\n");
    lab_score += 100;
}

void lab_ex3_sensor_dashboard(void)
{
    /*
     * CHALLENGE: Multi-sensor dashboard
     * TASK: Display 3 sensors with labels and values
     * LEARNING: Data formatting, real-time updates
     */

    puts_USART1("\r\n=== Lab 3.2: Sensor Dashboard ===\r\n");
    puts_USART1("Displaying ADC sensors. Press 'Q' to exit.\r\n");

    Adc_init();

    // Load arrow icons
    uint8_t pattern[8];
    for (uint8_t i = 0; i < 8; i++)
        pattern[i] = pgm_read_byte(&CHAR_UP_ARROW[i]);
    LCD_Create_Char(0, pattern);

    for (uint8_t i = 0; i < 8; i++)
        pattern[i] = pgm_read_byte(&CHAR_DOWN_ARROW[i]);
    LCD_Create_Char(1, pattern);

    LCD_Clear();
    LCD_Goto(0, 0);
    LCD_Print("Temp:    Light:");

    uint16_t last_temp = 0, last_light = 0;

    for (uint16_t i = 0; i < 300; i++) // 30 seconds
    {
        // Read sensors (ADC0=temp, ADC2=light)
        uint16_t temp = Read_Adc_Data(0);
        uint16_t light = Read_Adc_Data(2);

        // Display temperature
        LCD_Goto(0, 5);
        char val[4];
        sprintf(val, "%4u", temp);
        LCD_Print(val);

        // Show trend
        LCD_Goto(0, 9);
        if (temp > last_temp + 5)
            LCD_Data(0); // Up arrow
        else if (temp < last_temp - 5)
            LCD_Data(1); // Down arrow
        else
            LCD_Data(' ');

        // Display light
        LCD_Goto(1, 6);
        sprintf(val, "%4u", light);
        LCD_Print(val);

        // Show trend
        LCD_Goto(1, 10);
        if (light > last_light + 5)
            LCD_Data(0);
        else if (light < last_light - 5)
            LCD_Data(1);
        else
            LCD_Data(' ');

        // Bar graphs
        LCD_Goto(0, 11);
        uint8_t temp_bars = temp / 128; // 0-7
        for (uint8_t j = 0; j < 5; j++)
        {
            LCD_Data(j < temp_bars ? 0xFF : ' ');
        }

        LCD_Goto(1, 11);
        uint8_t light_bars = light / 128;
        for (uint8_t j = 0; j < 5; j++)
        {
            LCD_Data(j < light_bars ? 0xFF : ' ');
        }

        last_temp = temp;
        last_light = light;

        _delay_ms(100);

        if (UCSR1A & (1 << RXC1))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    puts_USART1("Dashboard test complete!\r\n");
    lab_score += 175;
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
    puts_USART1("  LCD CHARACTER DISPLAY - LAB\r\n");
    puts_USART1("========================================\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 1: Basic LCD Control\r\n");
    puts_USART1("  1. Hello World & Positioning\r\n");
    puts_USART1("  2. Scrolling Text\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 2: Custom Characters\r\n");
    puts_USART1("  3. Custom Icon Library\r\n");
    puts_USART1("  4. Animated Loading Bar\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 3: Real-Time Display\r\n");
    puts_USART1("  5. Digital Clock\r\n");
    puts_USART1("  6. Sensor Dashboard\r\n");
    puts_USART1("\r\n");
    puts_USART1("  0. Run All Exercises\r\n");
    puts_USART1("  X. Exit Lab\r\n");
    puts_USART1("\r\n");
    char score_str[40];
    sprintf(score_str, "Current Score: %u points\r\n\r\n", lab_score);
    puts_USART1(score_str);
    puts_USART1("Select exercise (1-6, 0, X): ");
}

int main(void)
{
    // Initialize system
    init_devices();
    Uart1_init();
    LCD_Init();

    _delay_ms(100);

    puts_USART1("\r\n\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("*  ATmega128 LCD CHARACTER DISPLAY LAB         *\r\n");
    puts_USART1("*  HD44780 16x2 LCD Exercises                  *\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("\r\n");
    puts_USART1("Welcome to the LCD Display Lab!\r\n");
    puts_USART1("Master character LCD control and user interfaces.\r\n");

    // Splash screen on LCD
    LCD_Clear();
    LCD_Goto(0, 0);
    LCD_Print("  ATmega128 Lab ");
    LCD_Goto(1, 0);
    LCD_Print(" LCD Exercises  ");
    _delay_ms(2000);

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
            lab_ex1_hello_world();
            break;
        case '2':
            lab_ex1_scrolling_text();
            break;
        case '3':
            lab_ex2_custom_characters();
            break;
        case '4':
            lab_ex2_animated_loading();
            break;
        case '5':
            lab_ex3_digital_clock();
            break;
        case '6':
            lab_ex3_sensor_dashboard();
            break;

        case '0':
            puts_USART1("\r\n*** RUNNING ALL EXERCISES ***\r\n");
            lab_ex1_hello_world();
            lab_ex1_scrolling_text();
            lab_ex2_custom_characters();
            lab_ex2_animated_loading();
            lab_ex3_digital_clock();
            lab_ex3_sensor_dashboard();

            char final_buffer[80];
            sprintf(final_buffer, "\r\n*** ALL EXERCISES COMPLETE! ***\r\nFinal Score: %u points\r\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            LCD_Clear();
            LCD_Goto(0, 0);
            LCD_Print("  Goodbye!  ");
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
