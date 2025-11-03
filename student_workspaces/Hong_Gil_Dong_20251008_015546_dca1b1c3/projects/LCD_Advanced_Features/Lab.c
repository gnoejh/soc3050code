/*
 * =============================================================================
 * LCD ADVANCED FEATURES - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master advanced LCD programming techniques and applications
 * DURATION: 75 minutes
 * DIFFICULTY: Intermediate-Advanced
 *
 * STUDENTS WILL:
 * - Create custom characters and graphical elements
 * - Implement advanced text effects and animations
 * - Build interactive user interfaces with menus
 * - Design scrolling displays and data visualization
 * - Implement cursor control and user input systems
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - 20x4 Character LCD display (HD44780 compatible)
 * - LCD connections: RS, EN, D4-D7 data lines
 * - Potentiometer for contrast adjustment
 * - Multiple input buttons for navigation
 * - Optional: Buzzer for user feedback
 * - Optional: External memory for pattern storage
 *
 * LCD ADVANCED FEATURES:
 * - Custom Character Generation (CGRAM)
 * - Multiple display modes and effects
 * - Cursor positioning and control
 * - User interface design patterns
 * - Real-time data display techniques
 *
 * LAB STRUCTURE:
 * - Exercise 1: Custom characters and graphics (20 min)
 * - Exercise 2: Advanced text effects and animations (20 min)
 * - Exercise 3: Interactive menu systems (20 min)
 * - Exercise 4: Data visualization dashboard (15 min)
 *
 * =============================================================================
 */

#include "config.h"

// LCD Advanced Features Configuration
#define LCD_ROWS 4
#define LCD_COLS 20
#define MENU_ITEMS 8
#define SCROLL_SPEED 3 // Characters per second

// Custom character indices (0-7 available on HD44780)
#define CHAR_HEART 0
#define CHAR_ARROW_UP 1
#define CHAR_ARROW_DOWN 2
#define CHAR_BATTERY 3
#define CHAR_DEGREE 4
#define CHAR_GRAPH_BAR 5
#define CHAR_BELL 6
#define CHAR_LOCK 7

// Animation configuration
#define ANIM_FRAMES 8
#define ANIM_DELAY 200 // ms between frames

// Menu system configuration
#define MAX_MENU_DEPTH 3
#define MENU_TIMEOUT 30 // seconds

// Lab session variables
uint16_t lab_score = 0;
uint8_t current_menu_item = 0;
uint8_t menu_depth = 0;
uint32_t animation_counter = 0;
uint16_t characters_created = 0;
uint16_t effects_demonstrated = 0;

/*
 * =============================================================================
 * CUSTOM CHARACTER DEFINITIONS
 * =============================================================================
 */

// Custom character patterns (8x5 pixel matrix)
uint8_t custom_chars[][8] = {
    // CHAR_HEART (0)
    {
        0b00000,
        0b01010,
        0b11111,
        0b11111,
        0b01110,
        0b00100,
        0b00000,
        0b00000},
    // CHAR_ARROW_UP (1)
    {
        0b00100,
        0b01110,
        0b11111,
        0b00100,
        0b00100,
        0b00100,
        0b00100,
        0b00000},
    // CHAR_ARROW_DOWN (2)
    {
        0b00100,
        0b00100,
        0b00100,
        0b00100,
        0b11111,
        0b01110,
        0b00100,
        0b00000},
    // CHAR_BATTERY (3)
    {
        0b01110,
        0b11011,
        0b10001,
        0b10001,
        0b10001,
        0b10001,
        0b11111,
        0b00000},
    // CHAR_DEGREE (4)
    {
        0b01100,
        0b10010,
        0b10010,
        0b01100,
        0b00000,
        0b00000,
        0b00000,
        0b00000},
    // CHAR_GRAPH_BAR (5)
    {
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b00000},
    // CHAR_BELL (6)
    {
        0b00100,
        0b01110,
        0b01110,
        0b01110,
        0b11111,
        0b00000,
        0b00100,
        0b00000},
    // CHAR_LOCK (7)
    {
        0b01110,
        0b10001,
        0b10001,
        0b11111,
        0b11011,
        0b11011,
        0b11111,
        0b00000}};

/*
 * =============================================================================
 * ADVANCED LCD FUNCTIONS
 * =============================================================================
 */

void lcd_create_custom_char(uint8_t char_code, uint8_t *pattern)
{
    if (char_code > 7)
        return; // Only 8 custom characters allowed

    // Set CGRAM address
    lcd_command(0x40 + (char_code * 8));

    // Write pattern data
    for (uint8_t i = 0; i < 8; i++)
    {
        lcd_data(pattern[i]);
        _delay_us(50);
    }

    // Return to DDRAM
    lcd_command(0x80);
    characters_created++;
}

void lcd_load_all_custom_chars(void)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        lcd_create_custom_char(i, custom_chars[i]);
    }
}

void lcd_print_custom_char(uint8_t row, uint8_t col, uint8_t char_code)
{
    lcd_gotoxy(col, row);
    lcd_data(char_code);
}

void lcd_centered_text(uint8_t row, char *text)
{
    uint8_t len = strlen(text);
    uint8_t start_col = 0;

    if (len < LCD_COLS)
    {
        start_col = (LCD_COLS - len) / 2;
    }

    lcd_string(row, start_col, text);

    // Clear remaining characters in row
    for (uint8_t i = start_col + len; i < LCD_COLS; i++)
    {
        lcd_gotoxy(i, row);
        lcd_data(' ');
    }
}

void lcd_reverse_text(uint8_t row, uint8_t start_col, char *text)
{
    // Create inverted display effect using custom characters
    uint8_t len = strlen(text);

    for (uint8_t i = 0; i < len && (start_col + i) < LCD_COLS; i++)
    {
        lcd_gotoxy(start_col + i, row);

        // For demonstration, use full block character as "inverse"
        if (text[i] == ' ')
        {
            lcd_data(0xFF); // Full block for space
        }
        else
        {
            lcd_data(text[i] | 0x80); // Attempt inverse (display dependent)
        }
    }
}

void lcd_progress_bar(uint8_t row, uint8_t col, uint8_t width, uint8_t percent)
{
    uint8_t filled_chars = (percent * width) / 100;

    lcd_gotoxy(col, row);

    for (uint8_t i = 0; i < width; i++)
    {
        if (i < filled_chars)
        {
            lcd_data(CHAR_GRAPH_BAR); // Filled bar
        }
        else
        {
            lcd_data(' '); // Empty space
        }
    }
}

void lcd_animated_cursor(uint8_t row, uint8_t col)
{
    static uint8_t cursor_state = 0;
    const uint8_t cursor_chars[] = {'|', '/', '-', '\\'};

    lcd_gotoxy(col, row);
    lcd_data(cursor_chars[cursor_state]);

    cursor_state = (cursor_state + 1) % 4;
}

void lcd_scroll_text(uint8_t row, char *text, uint8_t speed)
{
    uint8_t text_len = strlen(text);

    if (text_len <= LCD_COLS)
    {
        lcd_string(row, 0, text);
        return;
    }

    for (uint8_t offset = 0; offset <= text_len - LCD_COLS + 5; offset++)
    {
        lcd_gotoxy(0, row);

        // Display visible portion
        for (uint8_t i = 0; i < LCD_COLS; i++)
        {
            if (offset + i < text_len)
            {
                lcd_data(text[offset + i]);
            }
            else
            {
                lcd_data(' ');
            }
        }

        _delay_ms(1000 / speed);
    }
}

/*
 * =============================================================================
 * LAB EXERCISE 1: CUSTOM CHARACTERS AND GRAPHICS (20 minutes)
 * =============================================================================
 * OBJECTIVE: Create and use custom LCD characters
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex1_custom_characters(void)
{
    /*
     * CHALLENGE: Design and implement custom LCD characters
     * TASK: Create graphical icons and symbols for user interface
     * LEARNING: CGRAM programming, character design, graphic interfaces
     */

    puts_USART1("\\r\\n=== Lab 1: Custom Characters ===\\r\\n");
    puts_USART1("Creating custom LCD characters and icons\\r\\n");

    lcd_clear();
    lcd_centered_text(0, "CUSTOM CHARACTERS");
    lcd_centered_text(1, "Icon Design Lab");

    _delay_ms(2000);

    // Load all custom characters
    puts_USART1("Loading custom character set...\\r\\n");
    lcd_load_all_custom_chars();

    // Demonstrate each custom character
    lcd_clear();
    lcd_string(0, 0, "Custom Icons:");

    // Row 1: Basic icons
    lcd_string(1, 0, "Heart:");
    lcd_print_custom_char(1, 7, CHAR_HEART);

    lcd_string(1, 10, "Bell:");
    lcd_print_custom_char(1, 16, CHAR_BELL);

    // Row 2: Arrow icons
    lcd_string(2, 0, "Arrows:");
    lcd_print_custom_char(2, 8, CHAR_ARROW_UP);
    lcd_print_custom_char(2, 10, CHAR_ARROW_DOWN);

    lcd_string(2, 13, "Degree:");
    lcd_print_custom_char(2, 19, CHAR_DEGREE);

    // Row 3: Utility icons
    lcd_string(3, 0, "Battery:");
    lcd_print_custom_char(3, 9, CHAR_BATTERY);

    lcd_string(3, 12, "Lock:");
    lcd_print_custom_char(3, 18, CHAR_LOCK);

    puts_USART1("Basic icon set displayed\\r\\n");
    _delay_ms(3000);

    // Interactive character editor
    puts_USART1("\\r\\nInteractive character editor\\r\\n");
    puts_USART1("Design your own 8x5 character!\\r\\n");

    lcd_clear();
    lcd_centered_text(0, "CHARACTER EDITOR");
    lcd_string(1, 0, "Design 8x5 pattern:");

    uint8_t custom_pattern[8] = {0};
    uint8_t edit_row = 0, edit_col = 0;

    // Simple pattern editor (demonstration)
    char pattern_display[30];
    sprintf(pattern_display, "Row %d: 0b00000", edit_row);
    lcd_string(2, 0, pattern_display);

    puts_USART1("Creating demo pattern...\\r\\n");

    // Create a simple demo pattern (smiley face)
    custom_pattern[0] = 0b00000; // Top
    custom_pattern[1] = 0b01010; // Eyes
    custom_pattern[2] = 0b00000; //
    custom_pattern[3] = 0b10001; // Mouth corners
    custom_pattern[4] = 0b01110; // Smile
    custom_pattern[5] = 0b00000; //
    custom_pattern[6] = 0b00000; //
    custom_pattern[7] = 0b00000; // Bottom

    // Load and display custom character
    lcd_create_custom_char(0, custom_pattern);

    lcd_string(3, 0, "Your character:");
    lcd_print_custom_char(3, 16, 0);

    puts_USART1("Custom smiley face created!\\r\\n");

    char pattern_msg[40];
    sprintf(pattern_msg, "Characters created: %d\\r\\n", characters_created);
    puts_USART1(pattern_msg);

    lab_score += 150;
    _delay_ms(3000);
}

void lab_ex1_graphical_elements(void)
{
    /*
     * CHALLENGE: Create complex graphical displays using custom characters
     * TASK: Build graphical user interface elements
     * LEARNING: Combining characters, interface design, visual feedback
     */

    puts_USART1("\\r\\n=== Lab 1.2: Graphical Elements ===\\r\\n");
    puts_USART1("Building complex graphical displays\\r\\n");

    lcd_clear();
    lcd_centered_text(0, "GRAPHICAL ELEMENTS");

    // Battery level indicator
    puts_USART1("Creating battery level indicator...\\r\\n");

    for (uint8_t battery_level = 0; battery_level <= 100; battery_level += 20)
    {
        lcd_string(1, 0, "Battery Level:");

        // Display battery icon
        lcd_print_custom_char(1, 15, CHAR_BATTERY);

        char battery_text[10];
        sprintf(battery_text, "%d%%", battery_level);
        lcd_string(1, 17, battery_text);

        // Progress bar
        lcd_string(2, 0, "Progress:");
        lcd_progress_bar(2, 10, 8, battery_level);

        char progress_msg[30];
        sprintf(progress_msg, "Battery at %d%%\\r\\n", battery_level);
        puts_USART1(progress_msg);

        _delay_ms(1000);
    }

    // Temperature display with degree symbol
    puts_USART1("Creating temperature display...\\r\\n");

    for (int16_t temp = -10; temp <= 40; temp += 10)
    {
        lcd_string(3, 0, "Temperature:");

        char temp_text[10];
        sprintf(temp_text, "%d", temp);
        lcd_string(3, 13, temp_text);

        lcd_print_custom_char(3, 13 + strlen(temp_text), CHAR_DEGREE);
        lcd_string(3, 14 + strlen(temp_text), "C");

        char temp_msg[30];
        sprintf(temp_msg, "Temperature: %d°C\\r\\n", temp);
        puts_USART1(temp_msg);

        _delay_ms(800);
    }

    // Heart animation
    puts_USART1("Creating heart animation...\\r\\n");

    lcd_clear();
    lcd_centered_text(1, "Heart Animation");

    for (uint8_t i = 0; i < 10; i++)
    {
        // Animate heart beating
        uint8_t heart_col = 10 + (i % 2);

        lcd_gotoxy(heart_col - 1, 2);
        lcd_data(' ');
        lcd_gotoxy(heart_col + 1, 2);
        lcd_data(' ');

        lcd_print_custom_char(2, heart_col, CHAR_HEART);

        _delay_ms(500);

        // Clear for beat effect
        lcd_gotoxy(heart_col, 2);
        lcd_data(' ');
        _delay_ms(200);
    }

    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: ADVANCED TEXT EFFECTS (20 minutes)
 * =============================================================================
 * OBJECTIVE: Implement sophisticated text animations and effects
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

void lab_ex2_text_animations(void)
{
    /*
     * CHALLENGE: Create engaging text animations and effects
     * TASK: Implement scrolling, fading, and dynamic text displays
     * LEARNING: Animation timing, visual effects, user engagement
     */

    puts_USART1("\\r\\n=== Lab 2: Text Animations ===\\r\\n");
    puts_USART1("Creating advanced text effects\\r\\n");

    lcd_clear();
    lcd_centered_text(0, "TEXT ANIMATIONS");

    // Scrolling marquee text
    puts_USART1("Demo 1: Scrolling marquee\\r\\n");

    char long_message[] = "Welcome to the ATmega128 LCD Advanced Features Lab! This is a demonstration of scrolling text that exceeds the display width.";

    lcd_string(1, 0, "Scrolling Text:");
    lcd_scroll_text(2, long_message, SCROLL_SPEED);

    _delay_ms(1000);

    // Typewriter effect
    puts_USART1("Demo 2: Typewriter effect\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "Typewriter Effect:");

    char typewriter_text[] = "Hello, World!";
    uint8_t type_row = 2;
    uint8_t type_col = 3;

    for (uint8_t i = 0; i < strlen(typewriter_text); i++)
    {
        lcd_gotoxy(type_col + i, type_row);
        lcd_data(typewriter_text[i]);

        // Optional: Add cursor after current character
        if (i < strlen(typewriter_text) - 1)
        {
            lcd_gotoxy(type_col + i + 1, type_row);
            lcd_data('_');
            _delay_ms(300);
            lcd_gotoxy(type_col + i + 1, type_row);
            lcd_data(' ');
        }

        _delay_ms(200);
    }

    _delay_ms(2000);

    // Text reveal effect
    puts_USART1("Demo 3: Text reveal effect\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "Text Reveal Effect:");

    char reveal_text[] = "*** REVEALED! ***";
    uint8_t reveal_row = 2;
    uint8_t reveal_start = 1;

    // Initially show asterisks
    for (uint8_t i = 0; i < strlen(reveal_text); i++)
    {
        lcd_gotoxy(reveal_start + i, reveal_row);
        lcd_data('*');
    }

    _delay_ms(1000);

    // Reveal text character by character from center outward
    uint8_t center = strlen(reveal_text) / 2;
    for (uint8_t offset = 0; offset <= center; offset++)
    {
        // Reveal from center outward
        if (center + offset < strlen(reveal_text))
        {
            lcd_gotoxy(reveal_start + center + offset, reveal_row);
            lcd_data(reveal_text[center + offset]);
        }

        if (center >= offset && offset > 0)
        {
            lcd_gotoxy(reveal_start + center - offset, reveal_row);
            lcd_data(reveal_text[center - offset]);
        }

        _delay_ms(300);
    }

    _delay_ms(2000);
    effects_demonstrated++;
}

void lab_ex2_dynamic_content(void)
{
    /*
     * CHALLENGE: Create dynamic content displays with real-time updates
     * TASK: Implement live data displays with smooth updates
     * LEARNING: Real-time display updates, data formatting, smooth transitions
     */

    puts_USART1("\\r\\n=== Lab 2.2: Dynamic Content ===\\r\\n");
    puts_USART1("Creating real-time dynamic displays\\r\\n");

    lcd_clear();
    lcd_centered_text(0, "DYNAMIC CONTENT");

    // Real-time clock simulation
    puts_USART1("Demo 1: Digital clock simulation\\r\\n");

    uint8_t hours = 12, minutes = 30, seconds = 0;

    for (uint8_t i = 0; i < 15; i++) // Run for 15 "seconds"
    {
        char time_str[20];
        sprintf(time_str, "Time: %02d:%02d:%02d", hours, minutes, seconds);

        lcd_centered_text(1, time_str);

        // Add animated seconds indicator
        lcd_print_custom_char(2, 10, (seconds % 2) ? CHAR_HEART : ' ');

        seconds++;
        if (seconds >= 60)
        {
            seconds = 0;
            minutes++;
            if (minutes >= 60)
            {
                minutes = 0;
                hours++;
                if (hours > 12)
                    hours = 1;
            }
        }

        _delay_ms(500); // Simulate 0.5 second intervals
    }

    // Live sensor data simulation
    puts_USART1("Demo 2: Live sensor dashboard\\r\\n");

    lcd_clear();
    lcd_centered_text(0, "SENSOR DASHBOARD");

    for (uint8_t reading = 0; reading < 20; reading++)
    {
        // Simulate sensor readings
        uint16_t temperature = 200 + (reading * 5) + (rand() % 20);
        uint16_t humidity = 450 + (reading * 2) + (rand() % 30);
        uint16_t pressure = 1000 + (rand() % 50);

        // Format and display
        char temp_str[20];
        sprintf(temp_str, "Temp: %d.%d", temperature / 10, temperature % 10);
        lcd_string(1, 0, temp_str);
        lcd_print_custom_char(1, strlen(temp_str), CHAR_DEGREE);
        lcd_string(1, strlen(temp_str) + 1, "C");

        char humid_str[20];
        sprintf(humid_str, "Humid: %d.%d%%", humidity / 10, humidity % 10);
        lcd_string(2, 0, humid_str);

        char press_str[20];
        sprintf(press_str, "Press: %d hPa", pressure);
        lcd_string(3, 0, press_str);

        // Add animated indicator
        lcd_animated_cursor(1, 19);

        char sensor_msg[60];
        sprintf(sensor_msg, "Sensors: T=%d.%d°C, H=%d.%d%%, P=%dhPa\\r\\n",
                temperature / 10, temperature % 10, humidity / 10, humidity % 10, pressure);
        puts_USART1(sensor_msg);

        _delay_ms(800);
    }

    // Text sliding effect
    puts_USART1("Demo 3: Text sliding transitions\\r\\n");

    char messages[][20] = {
        "Message 1",
        "Another Message",
        "Third Display",
        "Final Message"};

    for (uint8_t msg = 0; msg < 4; msg++)
    {
        lcd_clear();
        lcd_string(0, 0, "Sliding Text Demo:");

        // Slide in from right
        for (int8_t pos = LCD_COLS; pos >= 2; pos--)
        {
            // Clear previous position
            if (pos < LCD_COLS - 1)
            {
                lcd_gotoxy(pos + 1, 2);
                for (uint8_t j = 0; j < strlen(messages[msg]); j++)
                {
                    lcd_data(' ');
                }
            }

            // Draw at new position
            if (pos >= 0)
            {
                lcd_string(2, pos, messages[msg]);
            }

            _delay_ms(100);
        }

        _delay_ms(1500); // Hold message
    }

    effects_demonstrated += 3;
    lab_score += 200;
}

/*
 * =============================================================================
 * LAB EXERCISE 3: INTERACTIVE MENU SYSTEMS (20 minutes)
 * =============================================================================
 * OBJECTIVE: Build sophisticated menu-driven user interfaces
 * DIFFICULTY: ★★★★☆ (Medium-Hard)
 */

typedef struct
{
    char title[20];
    uint8_t item_count;
    char items[8][16];
} menu_t;

menu_t main_menu = {
    "MAIN MENU",
    6,
    {"Settings",
     "Sensors",
     "Display",
     "System Info",
     "Diagnostics",
     "Exit"}};

menu_t settings_menu = {
    "SETTINGS",
    4,
    {"Brightness",
     "Contrast",
     "Language",
     "Back"}};

menu_t sensor_menu = {
    "SENSORS",
    5,
    {"Temperature",
     "Humidity",
     "Pressure",
     "Calibration",
     "Back"}};

void lab_ex3_menu_system(void)
{
    /*
     * CHALLENGE: Create a hierarchical menu system with navigation
     * TASK: Implement multi-level menus with proper navigation
     * LEARNING: User interface design, navigation logic, state management
     */

    puts_USART1("\\r\\n=== Lab 3: Interactive Menu System ===\\r\\n");
    puts_USART1("Building hierarchical menu navigation\\r\\n");

    lcd_clear();
    lcd_centered_text(0, "MENU SYSTEM DEMO");
    lcd_centered_text(1, "Use buttons to navigate");

    _delay_ms(3000);

    menu_t *current_menu = &main_menu;
    uint8_t selected_item = 0;
    uint8_t menu_active = 1;
    uint8_t menu_timeout = 0;

    while (menu_active && menu_timeout < MENU_TIMEOUT)
    {
        // Display current menu
        lcd_clear();
        lcd_centered_text(0, current_menu->title);

        // Display menu items with selection indicator
        uint8_t display_start = 0;
        uint8_t display_count = (current_menu->item_count > 3) ? 3 : current_menu->item_count;

        // Adjust display window if selection is beyond visible area
        if (selected_item >= 3)
        {
            display_start = selected_item - 2;
            if (display_start + display_count > current_menu->item_count)
            {
                display_start = current_menu->item_count - display_count;
            }
        }

        for (uint8_t i = 0; i < display_count; i++)
        {
            uint8_t item_index = display_start + i;
            uint8_t row = i + 1;

            // Selection indicator
            if (item_index == selected_item)
            {
                lcd_print_custom_char(row, 0, CHAR_ARROW_UP);
                lcd_reverse_text(row, 2, current_menu->items[item_index]);
            }
            else
            {
                lcd_string(row, 2, current_menu->items[item_index]);
            }
        }

        // Show scroll indicators if needed
        if (display_start > 0)
        {
            lcd_print_custom_char(1, 19, CHAR_ARROW_UP);
        }
        if (display_start + display_count < current_menu->item_count)
        {
            lcd_print_custom_char(3, 19, CHAR_ARROW_DOWN);
        }

        char menu_msg[50];
        sprintf(menu_msg, "Menu: %s, Item: %s\\r\\n",
                current_menu->title, current_menu->items[selected_item]);
        puts_USART1(menu_msg);

        // Simulate button navigation (for demonstration)
        _delay_ms(1000);

        // Auto-navigate for demo (normally would be button controlled)
        uint8_t button_action = (menu_timeout % 4);

        switch (button_action)
        {
        case 0: // Down button
            if (selected_item < current_menu->item_count - 1)
            {
                selected_item++;
                puts_USART1("Action: DOWN\\r\\n");
            }
            break;

        case 1: // Up button
            if (selected_item > 0)
            {
                selected_item--;
                puts_USART1("Action: UP\\r\\n");
            }
            break;

        case 2: // Select button
            puts_USART1("Action: SELECT\\r\\n");

            // Handle menu selection
            if (current_menu == &main_menu)
            {
                switch (selected_item)
                {
                case 0: // Settings
                    current_menu = &settings_menu;
                    selected_item = 0;
                    break;
                case 1: // Sensors
                    current_menu = &sensor_menu;
                    selected_item = 0;
                    break;
                case 2: // Display
                    lcd_clear();
                    lcd_centered_text(1, "DISPLAY SETTINGS");
                    lcd_centered_text(2, "Feature not yet");
                    lcd_centered_text(3, "implemented");
                    _delay_ms(2000);
                    break;
                case 5: // Exit
                    menu_active = 0;
                    break;
                }
            }
            else
            {
                // Submenu selections - generally go back to main
                if (strcmp(current_menu->items[selected_item], "Back") == 0)
                {
                    current_menu = &main_menu;
                    selected_item = 0;
                }
                else
                {
                    // Show selected function
                    lcd_clear();
                    lcd_centered_text(1, "FUNCTION:");
                    lcd_centered_text(2, current_menu->items[selected_item]);
                    lcd_centered_text(3, "Press any key");
                    _delay_ms(2000);
                }
            }
            break;

        case 3: // Back button
            puts_USART1("Action: BACK\\r\\n");
            if (current_menu != &main_menu)
            {
                current_menu = &main_menu;
                selected_item = 0;
            }
            else
            {
                menu_active = 0; // Exit from main menu
            }
            break;
        }

        menu_timeout++;
    }

    lcd_clear();
    lcd_centered_text(1, "MENU DEMO");
    lcd_centered_text(2, "COMPLETE");

    puts_USART1("Menu system demonstration complete\\r\\n");
    lab_score += 200;
    _delay_ms(2000);
}

void lab_ex3_user_input_forms(void)
{
    /*
     * CHALLENGE: Create data entry forms with validation
     * TASK: Implement input fields, validation, and confirmation
     * LEARNING: Form design, input validation, user feedback
     */

    puts_USART1("\\r\\n=== Lab 3.2: User Input Forms ===\\r\\n");
    puts_USART1("Creating data entry forms\\r\\n");

    lcd_clear();
    lcd_centered_text(0, "DATA ENTRY FORM");

    // Simulated form with various field types
    typedef struct
    {
        char name[16];
        uint8_t age;
        uint16_t sensor_threshold;
        uint8_t enable_alerts;
    } user_config_t;

    user_config_t config = {"User123", 25, 500, 1};

    // Display form fields
    lcd_string(1, 0, "Name:");
    lcd_string(1, 6, config.name);

    char age_str[10];
    sprintf(age_str, "%d", config.age);
    lcd_string(2, 0, "Age:");
    lcd_string(2, 6, age_str);

    char threshold_str[10];
    sprintf(threshold_str, "%d", config.sensor_threshold);
    lcd_string(3, 0, "Threshold:");
    lcd_string(3, 11, threshold_str);

    puts_USART1("Form fields displayed\\r\\n");

    // Simulate form navigation and editing
    uint8_t current_field = 0;
    uint8_t editing = 0;

    for (uint8_t demo_step = 0; demo_step < 12; demo_step++)
    {
        // Clear field indicators
        for (uint8_t row = 1; row <= 3; row++)
        {
            lcd_gotoxy(0, row);
            lcd_data(' ');
        }

        // Show current field indicator
        if (!editing)
        {
            lcd_print_custom_char(current_field + 1, 0, CHAR_ARROW_UP);
        }
        else
        {
            lcd_animated_cursor(current_field + 1, 0);
        }

        char form_msg[50];
        sprintf(form_msg, "Form field %d, editing: %s\\r\\n",
                current_field, editing ? "YES" : "NO");
        puts_USART1(form_msg);

        _delay_ms(800);

        // Simulate user interactions
        uint8_t action = demo_step % 4;

        switch (action)
        {
        case 0: // Move to next field
            if (!editing)
            {
                current_field = (current_field + 1) % 3;
                puts_USART1("  Action: Next field\\r\\n");
            }
            break;

        case 1: // Start/stop editing
            editing = !editing;
            puts_USART1(editing ? "  Action: Start edit\\r\\n" : "  Action: Stop edit\\r\\n");
            break;

        case 2: // Modify value (if editing)
            if (editing)
            {
                switch (current_field)
                {
                case 0: // Name
                    strcpy(config.name, "NewUser");
                    lcd_string(1, 6, "NewUser     ");
                    puts_USART1("  Modified name\\r\\n");
                    break;
                case 1: // Age
                    config.age = 30;
                    lcd_string(2, 6, "30 ");
                    puts_USART1("  Modified age\\r\\n");
                    break;
                case 2: // Threshold
                    config.sensor_threshold = 750;
                    lcd_string(3, 11, "750 ");
                    puts_USART1("  Modified threshold\\r\\n");
                    break;
                }
            }
            break;

        case 3: // Validation check
            if (config.age > 100 || config.age < 1)
            {
                lcd_string(3, 15, "ERR");
                puts_USART1("  Validation: Age error\\r\\n");
            }
            else if (config.sensor_threshold > 1000)
            {
                lcd_string(3, 15, "HI ");
                puts_USART1("  Validation: Threshold high\\r\\n");
            }
            else
            {
                lcd_string(3, 15, "OK ");
                puts_USART1("  Validation: OK\\r\\n");
            }
            break;
        }
    }

    // Form submission confirmation
    lcd_clear();
    lcd_centered_text(0, "CONFIRM SETTINGS");

    char confirm_name[20];
    sprintf(confirm_name, "Name: %s", config.name);
    lcd_string(1, 0, confirm_name);

    char confirm_age[20];
    sprintf(confirm_age, "Age: %d", config.age);
    lcd_string(2, 0, confirm_age);

    char confirm_threshold[20];
    sprintf(confirm_threshold, "Threshold: %d", config.sensor_threshold);
    lcd_string(3, 0, confirm_threshold);

    puts_USART1("Form completed and confirmed\\r\\n");

    _delay_ms(3000);
    lab_score += 150;
}

/*
 * =============================================================================
 * LAB EXERCISE 4: DATA VISUALIZATION DASHBOARD (15 minutes)
 * =============================================================================
 * OBJECTIVE: Create a comprehensive data visualization dashboard
 * DIFFICULTY: ★★★★★ (Hard)
 */

void lab_ex4_dashboard(void)
{
    /*
     * CHALLENGE: Build a complete data visualization dashboard
     * TASK: Implement graphs, charts, and real-time data display
     * LEARNING: Data visualization, dashboard design, information presentation
     */

    puts_USART1("\\r\\n=== Lab 4: Data Visualization Dashboard ===\\r\\n");
    puts_USART1("Creating comprehensive data dashboard\\r\\n");

    lcd_clear();
    lcd_centered_text(0, "DATA DASHBOARD");

    // Multi-sensor dashboard with various visualization types
    uint16_t sensor_history[4][10]; // 4 sensors, 10 historical values
    uint8_t history_index = 0;

    // Initialize with some sample data
    for (uint8_t s = 0; s < 4; s++)
    {
        for (uint8_t h = 0; h < 10; h++)
        {
            sensor_history[s][h] = 200 + (s * 100) + (h * 10) + (rand() % 50);
        }
    }

    puts_USART1("Dashboard initialized with sample data\\r\\n");

    // Dashboard view 1: Numeric display with status
    for (uint8_t cycle = 0; cycle < 15; cycle++)
    {
        lcd_clear();
        lcd_string(0, 0, "Sensors");

        // Current time indicator
        char time_indicator[10];
        sprintf(time_indicator, "T+%d", cycle);
        lcd_string(0, 15, time_indicator);

        // Update sensor readings
        for (uint8_t s = 0; s < 4; s++)
        {
            sensor_history[s][history_index] = sensor_history[s][(history_index + 9) % 10] +
                                               ((rand() % 21) - 10); // Random walk

            if (sensor_history[s][history_index] < 100)
                sensor_history[s][history_index] = 100;
            if (sensor_history[s][history_index] > 900)
                sensor_history[s][history_index] = 900;
        }

        // Display current values
        char sensor_line[21];
        sprintf(sensor_line, "T:%d H:%d P:%d L:%d",
                sensor_history[0][history_index],
                sensor_history[1][history_index],
                sensor_history[2][history_index],
                sensor_history[3][history_index]);
        lcd_string(1, 0, sensor_line);

        // Trend indicators
        lcd_string(2, 0, "Trend:");
        for (uint8_t s = 0; s < 4; s++)
        {
            uint8_t current = sensor_history[s][history_index];
            uint8_t previous = sensor_history[s][(history_index + 9) % 10];

            if (current > previous + 10)
            {
                lcd_print_custom_char(2, 7 + (s * 3), CHAR_ARROW_UP);
            }
            else if (current < previous - 10)
            {
                lcd_print_custom_char(2, 7 + (s * 3), CHAR_ARROW_DOWN);
            }
            else
            {
                lcd_gotoxy(7 + (s * 3), 2);
                lcd_data('-');
            }
        }

        // Bar graph representation
        lcd_string(3, 0, "Graph:");
        for (uint8_t s = 0; s < 4; s++)
        {
            uint8_t bar_height = (sensor_history[s][history_index] * 5) / 1000; // Scale to 0-5

            if (bar_height >= 3)
            {
                lcd_print_custom_char(3, 7 + (s * 3), CHAR_GRAPH_BAR);
            }
            else if (bar_height >= 1)
            {
                lcd_gotoxy(7 + (s * 3), 3);
                lcd_data('|');
            }
            else
            {
                lcd_gotoxy(7 + (s * 3), 3);
                lcd_data('_');
            }
        }

        char dashboard_msg[80];
        sprintf(dashboard_msg, "Dashboard cycle %d: T=%d, H=%d, P=%d, L=%d\\r\\n",
                cycle, sensor_history[0][history_index], sensor_history[1][history_index],
                sensor_history[2][history_index], sensor_history[3][history_index]);
        puts_USART1(dashboard_msg);

        history_index = (history_index + 1) % 10;
        _delay_ms(1000);
    }

    // Dashboard view 2: Alert and status system
    puts_USART1("Switching to alert dashboard...\\r\\n");

    uint8_t alert_count = 0;
    uint8_t system_status = 1; // 1 = OK, 0 = Error

    for (uint8_t alert_cycle = 0; alert_cycle < 8; alert_cycle++)
    {
        lcd_clear();
        lcd_centered_text(0, "SYSTEM STATUS");

        // Check for alert conditions
        alert_count = 0;
        for (uint8_t s = 0; s < 4; s++)
        {
            if (sensor_history[s][(history_index + 9) % 10] > 700 ||
                sensor_history[s][(history_index + 9) % 10] < 200)
            {
                alert_count++;
            }
        }

        // System status display
        if (alert_count == 0)
        {
            lcd_string(1, 0, "Status: ");
            lcd_string(1, 8, "ALL OK");
            lcd_print_custom_char(1, 15, CHAR_HEART);
            system_status = 1;
        }
        else
        {
            lcd_string(1, 0, "Status: ");
            char alert_text[15];
            sprintf(alert_text, "%d ALERTS", alert_count);
            lcd_string(1, 8, alert_text);
            lcd_print_custom_char(1, 15, CHAR_BELL);
            system_status = 0;
        }

        // Battery and system health
        uint8_t battery_level = 90 - (alert_cycle * 5); // Simulated discharge
        char battery_text[20];
        sprintf(battery_text, "Battery: %d%%", battery_level);
        lcd_string(2, 0, battery_text);
        lcd_print_custom_char(2, 13, CHAR_BATTERY);

        if (battery_level < 20)
        {
            lcd_string(2, 15, "LOW");
        }

        // Security status
        lcd_string(3, 0, "Security: ");
        if (system_status && alert_count == 0)
        {
            lcd_string(3, 10, "SECURE");
            lcd_print_custom_char(3, 17, CHAR_LOCK);
        }
        else
        {
            lcd_string(3, 10, "CHECK ");
        }

        char status_msg[60];
        sprintf(status_msg, "System status: %s, Alerts: %d, Battery: %d%%\\r\\n",
                system_status ? "OK" : "ERROR", alert_count, battery_level);
        puts_USART1(status_msg);

        _delay_ms(1500);
    }

    // Final dashboard summary
    lcd_clear();
    lcd_centered_text(0, "DASHBOARD SUMMARY");

    char summary_line1[20];
    sprintf(summary_line1, "Data Points: %d", cycle * 4);
    lcd_string(1, 0, summary_line1);

    char summary_line2[20];
    sprintf(summary_line2, "Effects: %d", effects_demonstrated);
    lcd_string(2, 0, summary_line2);

    char summary_line3[20];
    sprintf(summary_line3, "Characters: %d", characters_created);
    lcd_string(3, 0, summary_line3);

    puts_USART1("Data visualization dashboard complete!\\r\\n");

    lab_score += 250;
    _delay_ms(3000);
}

/*
 * =============================================================================
 * LAB MAIN PROGRAM - EXERCISE SELECTION
 * =============================================================================
 */

void show_lab_menu(void)
{
    puts_USART1("\\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("   LCD ADVANCED FEATURES - LAB EXERCISES     \\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("1. Custom Characters & Graphical Elements   \\r\\n");
    puts_USART1("2. Advanced Text Effects & Animations       \\r\\n");
    puts_USART1("3. Interactive Menu Systems                 \\r\\n");
    puts_USART1("4. Data Visualization Dashboard             \\r\\n");
    puts_USART1("                                              \\r\\n");
    puts_USART1("0. Run All Exercises                         \\r\\n");
    puts_USART1("X. Exit Lab                                   \\r\\n");
    puts_USART1("==============================================\\r\\n");
    char score_msg[50];
    sprintf(score_msg, "Current Score: %d points\\r\\n", lab_score);
    puts_USART1(score_msg);
    char stats_msg[60];
    sprintf(stats_msg, "Stats: %d chars, %d effects, %d animations\\r\\n",
            characters_created, effects_demonstrated, animation_counter);
    puts_USART1(stats_msg);
    puts_USART1("Select exercise (1-4, 0, X): ");
}

int main(void)
{
    init_devices();

    puts_USART1("\\r\\n*** LCD ADVANCED FEATURES LAB SESSION ***\\r\\n");
    puts_USART1("Welcome to advanced LCD programming!\\r\\n");
    puts_USART1("This lab covers custom characters, animations, and interfaces\\r\\n");
    puts_USART1("Ensure LCD is properly connected and contrast is adjusted\\r\\n");

    lcd_clear();
    lcd_string(1, 0, "LCD ADVANCED LAB");
    lcd_string(2, 0, "Features & Effects");
    lcd_string(4, 0, "Use Serial Menu");

    while (1)
    {
        show_lab_menu();
        char choice = getch_USART1();
        putch_USART1(choice);
        putch_USART1('\\r');
        putch_USART1('\\n');

        switch (choice)
        {
        case '1':
            lab_ex1_custom_characters();
            lab_ex1_graphical_elements();
            break;

        case '2':
            lab_ex2_text_animations();
            lab_ex2_dynamic_content();
            break;

        case '3':
            lab_ex3_menu_system();
            lab_ex3_user_input_forms();
            break;

        case '4':
            lab_ex4_dashboard();
            break;

        case '0':
            puts_USART1("\\r\\n*** RUNNING ALL EXERCISES ***\\r\\n");
            lab_ex1_custom_characters();
            lab_ex1_graphical_elements();
            lab_ex2_text_animations();
            lab_ex2_dynamic_content();
            lab_ex3_menu_system();
            lab_ex3_user_input_forms();
            lab_ex4_dashboard();

            char final_buffer[80];
            sprintf(final_buffer, "\\r\\n*** ALL EXERCISES COMPLETE! ***\\r\\nFinal Score: %d points\\r\\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\\r\\nExiting lab. Great work on LCD programming!\\r\\n");
            puts_USART1("Remember: LCD interfaces are powerful tools for user interaction!\\r\\n");
            lcd_clear();
            lcd_string(2, 0, "LAB COMPLETE!");
            char exit_score[30];
            sprintf(exit_score, "Score: %d pts", lab_score);
            lcd_string(3, 0, exit_score);
            while (1)
                ;

        default:
            puts_USART1("Invalid choice. Please try again.\\r\\n");
        }

        puts_USART1("\\r\\nPress any key to continue...\\r\\n");
        getch_USART1();
    }

    return 0;
}