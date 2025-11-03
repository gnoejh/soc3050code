/*
 * =============================================================================
 * JOYSTICK CONTROL - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master analog joystick control and ADC interfacing
 * DURATION: 60 minutes
 * DIFFICULTY: Beginner to Intermediate
 *
 * STUDENTS WILL:
 * - Read joystick analog values using ADC
 * - Implement cursor control systems
 * - Create joystick-controlled games
 * - Build analog input processing algorithms
 * - Debug analog input issues
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - Analog joystick (X on ADC0, Y on ADC1)
 * - 128x64 GLCD display
 * - 4 buttons for additional controls
 * - 8 LEDs for feedback
 *
 * LAB STRUCTURE:
 * - Exercise 1: Basic joystick reading and calibration (15 min)
 * - Exercise 2: Cursor control and movement (15 min)
 * - Exercise 3: Joystick-controlled LED patterns (15 min)
 * - Exercise 4: Mini-game implementation (15 min)
 *
 * =============================================================================
 */

#include "config.h"

// Lab configuration - Joystick ADC channels
#define JOYSTICK_X_CHANNEL 0 // ADC0
#define JOYSTICK_Y_CHANNEL 1 // ADC1

// Joystick calibration values (adjust based on hardware)
#define JOYSTICK_CENTER_X 512
#define JOYSTICK_CENTER_Y 512
#define JOYSTICK_DEADZONE 50

// Global variables for lab exercises
uint16_t lab_score = 0;
uint8_t cursor_x = 64, cursor_y = 32;

/*
 * =============================================================================
 * LAB EXERCISE 1: JOYSTICK CALIBRATION AND READING (15 minutes)
 * =============================================================================
 * OBJECTIVE: Learn to read and interpret joystick values
 * DIFFICULTY: ★☆☆☆☆ (Basic)
 */

void lab_ex1_joystick_calibration(void)
{
    /*
     * CHALLENGE: Display real-time joystick values
     * TASK: Read X/Y values and show them on display
     * LEARNING: ADC reading and value interpretation
     */

    puts_USART1("\\r\\n=== Lab 1: Joystick Calibration ===\\r\\n");
    puts_USART1("Move joystick and observe values\\r\\n");
    puts_USART1("Press button to continue...\\r\\n\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "JOYSTICK CALIBRATION");
    lcd_string(1, 0, "Move stick around");

    uint16_t samples = 0;
    while (samples < 100 && !button_pressed(0)) // 10 seconds of sampling
    {
        uint16_t x_val = Read_Adc_Data(JOYSTICK_X_CHANNEL);
        uint16_t y_val = Read_Adc_Data(JOYSTICK_Y_CHANNEL);

        // Display values on LCD
        char buffer[20];
        sprintf(buffer, "X: %4d Y: %4d", x_val, y_val);
        lcd_string(3, 0, buffer);

        // Send to serial for debugging
        char serial_buffer[50];
        sprintf(serial_buffer, "Sample %d: X=%d, Y=%d\\r\\n", samples, x_val, y_val);
        puts_USART1(serial_buffer);

        _delay_ms(100);
        samples++;
    }

    puts_USART1("Calibration complete!\\r\\n");
    lab_score += 50;
}

void lab_ex1_deadzone_testing(void)
{
    /*
     * CHALLENGE: Implement and test deadzone functionality
     * TASK: Create a deadzone around center position
     * LEARNING: Noise filtering and input processing
     */

    puts_USART1("\\r\\n=== Lab 1.2: Deadzone Testing ===\\r\\n");
    puts_USART1("Testing deadzone implementation\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "DEADZONE TEST");
    lcd_string(1, 0, "Center = No movement");

    for (int test = 0; test < 50 && !button_pressed(0); test++)
    {
        uint16_t x_val = Read_Adc_Data(JOYSTICK_X_CHANNEL);
        uint16_t y_val = Read_Adc_Data(JOYSTICK_Y_CHANNEL);

        // Apply deadzone
        int16_t x_offset = x_val - JOYSTICK_CENTER_X;
        int16_t y_offset = y_val - JOYSTICK_CENTER_Y;

        bool in_deadzone = (abs(x_offset) < JOYSTICK_DEADZONE && abs(y_offset) < JOYSTICK_DEADZONE);

        lcd_string(3, 0, in_deadzone ? "STATUS: DEADZONE   " : "STATUS: ACTIVE     ");

        char debug[30];
        sprintf(debug, "X:%+4d Y:%+4d", x_offset, y_offset);
        lcd_string(4, 0, debug);

        _delay_ms(200);
    }

    lab_score += 50;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: CURSOR CONTROL SYSTEM (15 minutes)
 * =============================================================================
 * OBJECTIVE: Implement precise cursor movement
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex2_cursor_control(void)
{
    /*
     * CHALLENGE: Create smooth cursor movement
     * TASK: Control a cursor on screen with joystick
     * LEARNING: Coordinate mapping and boundary checking
     */

    puts_USART1("\\r\\n=== Lab 2: Cursor Control ===\\r\\n");
    puts_USART1("Use joystick to move cursor around screen\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "CURSOR CONTROL");
    lcd_string(1, 0, "Use joystick to move");

    cursor_x = 64; // Center of screen
    cursor_y = 32;

    for (int moves = 0; moves < 200 && !button_pressed(0); moves++)
    {
        uint16_t x_val = Read_Adc_Data(JOYSTICK_X_CHANNEL);
        uint16_t y_val = Read_Adc_Data(JOYSTICK_Y_CHANNEL);

        // Calculate movement with deadzone
        int16_t x_offset = x_val - JOYSTICK_CENTER_X;
        int16_t y_offset = y_val - JOYSTICK_CENTER_Y;

        int8_t dx = 0, dy = 0;

        if (abs(x_offset) > JOYSTICK_DEADZONE)
        {
            dx = (x_offset > 0) ? 1 : -1;
            if (abs(x_offset) > 200)
                dx *= 2; // Faster movement for larger deflection
        }

        if (abs(y_offset) > JOYSTICK_DEADZONE)
        {
            dy = (y_offset > 0) ? 1 : -1;
            if (abs(y_offset) > 200)
                dy *= 2;
        }

        // Update cursor position with boundary checking
        int16_t new_x = cursor_x + dx;
        int16_t new_y = cursor_y + dy;

        if (new_x >= 0 && new_x < 128)
            cursor_x = new_x;
        if (new_y >= 16 && new_y < 64)
            cursor_y = new_y; // Leave room for text

        // Clear previous cursor and draw new one
        GLCD_Rectangle(0, 16, 127, 63); // Clear drawing area
        GLCD_Set_Dot(cursor_x, cursor_y);
        GLCD_Circle(cursor_x, cursor_y, 3); // Draw cursor

        // Show position
        char pos_buffer[20];
        sprintf(pos_buffer, "X:%3d Y:%3d", cursor_x, cursor_y);
        lcd_string(6, 0, pos_buffer);

        _delay_ms(50);
    }

    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 3: LED PATTERN CONTROL (15 minutes)
 * =============================================================================
 * OBJECTIVE: Control LED patterns with joystick
 * DIFFICULTY: ★★☆☆☆ (Medium)
 */

void lab_ex3_led_joystick_control(void)
{
    /*
     * CHALLENGE: Control LED patterns with joystick direction
     * TASK: Different joystick positions create different LED patterns
     * LEARNING: Multi-output control and pattern generation
     */

    puts_USART1("\\r\\n=== Lab 3: LED Joystick Control ===\\r\\n");
    puts_USART1("Joystick direction controls LED patterns\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "LED CONTROL");
    lcd_string(1, 0, "Move stick for patterns");

    // Configure LEDs
    DDRB = 0xFF;  // All PORTB as output
    PORTB = 0x00; // Start with LEDs off

    for (int cycle = 0; cycle < 100 && !button_pressed(0); cycle++)
    {
        uint16_t x_val = Read_Adc_Data(JOYSTICK_X_CHANNEL);
        uint16_t y_val = Read_Adc_Data(JOYSTICK_Y_CHANNEL);

        // Determine joystick direction
        int16_t x_offset = x_val - JOYSTICK_CENTER_X;
        int16_t y_offset = y_val - JOYSTICK_CENTER_Y;

        uint8_t led_pattern = 0x00;
        const char *direction = "CENTER";

        if (abs(x_offset) > JOYSTICK_DEADZONE || abs(y_offset) > JOYSTICK_DEADZONE)
        {
            if (abs(x_offset) > abs(y_offset))
            {
                if (x_offset > 0) // Right
                {
                    led_pattern = 0x0F; // Right half LEDs
                    direction = "RIGHT ";
                }
                else // Left
                {
                    led_pattern = 0xF0; // Left half LEDs
                    direction = "LEFT  ";
                }
            }
            else
            {
                if (y_offset > 0) // Down
                {
                    led_pattern = 0xAA; // Alternating pattern
                    direction = "DOWN  ";
                }
                else // Up
                {
                    led_pattern = 0x55; // Opposite alternating
                    direction = "UP    ";
                }
            }
        }

        PORTB = led_pattern;
        lcd_string(3, 0, direction);

        char pattern_str[20];
        sprintf(pattern_str, "Pattern: 0x%02X", led_pattern);
        lcd_string(4, 0, pattern_str);

        _delay_ms(100);
    }

    PORTB = 0x00; // Turn off LEDs
    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 4: JOYSTICK MINI-GAME (15 minutes)
 * =============================================================================
 * OBJECTIVE: Build a simple joystick-controlled game
 * DIFFICULTY: ★★★☆☆ (Medium-Hard)
 */

void lab_ex4_catch_the_dot(void)
{
    /*
     * CHALLENGE: Create a "catch the dot" game
     * TASK: Use joystick to move player and catch randomly placed targets
     * LEARNING: Game logic, collision detection, scoring
     */

    puts_USART1("\\r\\n=== Lab 4: Catch The Dot Game ===\\r\\n");
    puts_USART1("Move to catch the targets!\\r\\n");

    lcd_clear();
    lcd_string(0, 0, "CATCH THE DOT!");
    lcd_string(1, 0, "Score: 0");

    uint8_t player_x = 64, player_y = 40;
    uint8_t target_x = 30, target_y = 50;
    uint16_t game_score = 0;
    uint8_t targets_caught = 0;

    srand(42); // Simple seed for random numbers

    while (targets_caught < 5 && !button_pressed(0))
    {
        // Read joystick for player movement
        uint16_t x_val = Read_Adc_Data(JOYSTICK_X_CHANNEL);
        uint16_t y_val = Read_Adc_Data(JOYSTICK_Y_CHANNEL);

        int16_t x_offset = x_val - JOYSTICK_CENTER_X;
        int16_t y_offset = y_val - JOYSTICK_CENTER_Y;

        // Move player
        if (abs(x_offset) > JOYSTICK_DEADZONE)
        {
            int8_t dx = (x_offset > 0) ? 1 : -1;
            int16_t new_x = player_x + dx;
            if (new_x >= 5 && new_x < 123)
                player_x = new_x;
        }

        if (abs(y_offset) > JOYSTICK_DEADZONE)
        {
            int8_t dy = (y_offset > 0) ? 1 : -1;
            int16_t new_y = player_y + dy;
            if (new_y >= 20 && new_y < 60)
                player_y = new_y;
        }

        // Check collision (within 5 pixels)
        uint8_t dx = abs(player_x - target_x);
        uint8_t dy = abs(player_y - target_y);

        if (dx < 5 && dy < 5)
        {
            // Target caught!
            targets_caught++;
            game_score += 100;

            // Generate new target position
            target_x = 10 + (rand() % 108);
            target_y = 25 + (rand() % 35);

            // Update score display
            char score_str[20];
            sprintf(score_str, "Score: %d", game_score);
            lcd_string(1, 0, score_str);

            puts_USART1("Target caught!\\r\\n");
        }

        // Clear game area and redraw
        GLCD_Rectangle(0, 16, 127, 63); // Clear game area

        // Draw player (square)
        GLCD_Rectangle(player_x - 2, player_y - 2, player_x + 2, player_y + 2);

        // Draw target (circle)
        GLCD_Circle(target_x, target_y, 3);

        _delay_ms(50);
    }

    lcd_clear();
    lcd_string(2, 0, "GAME COMPLETE!");
    char final_score[20];
    sprintf(final_score, "Final Score: %d", game_score);
    lcd_string(3, 0, final_score);

    lab_score += game_score;
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
    puts_USART1("     JOYSTICK CONTROL - LAB EXERCISES        \\r\\n");
    puts_USART1("==============================================\\r\\n");
    puts_USART1("1. Joystick Calibration & Deadzone          \\r\\n");
    puts_USART1("2. Cursor Control System                     \\r\\n");
    puts_USART1("3. LED Pattern Control                       \\r\\n");
    puts_USART1("4. Catch The Dot Game                        \\r\\n");
    puts_USART1("                                              \\r\\n");
    puts_USART1("0. Run All Exercises                         \\r\\n");
    puts_USART1("X. Exit Lab                                   \\r\\n");
    puts_USART1("==============================================\\r\\n");
    char score_msg[50];
    sprintf(score_msg, "Current Score: %d points\\r\\n", lab_score);
    puts_USART1(score_msg);
    puts_USART1("Select exercise (1-4, 0, X): ");
}

int main(void)
{
    init_devices();

    puts_USART1("\\r\\n*** JOYSTICK CONTROL LAB SESSION ***\\r\\n");
    puts_USART1("Welcome to hands-on joystick programming!\\r\\n");

    lcd_clear();
    lcd_string(1, 0, "JOYSTICK LAB");
    lcd_string(2, 0, "Select Exercise");
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
            lab_ex1_joystick_calibration();
            lab_ex1_deadzone_testing();
            break;

        case '2':
            lab_ex2_cursor_control();
            break;

        case '3':
            lab_ex3_led_joystick_control();
            break;

        case '4':
            lab_ex4_catch_the_dot();
            break;

        case '0':
            puts_USART1("\\r\\n*** RUNNING ALL EXERCISES ***\\r\\n");
            lab_ex1_joystick_calibration();
            lab_ex1_deadzone_testing();
            lab_ex2_cursor_control();
            lab_ex3_led_joystick_control();
            lab_ex4_catch_the_dot();

            char final_buffer[80];
            sprintf(final_buffer, "\\r\\n*** ALL EXERCISES COMPLETE! ***\\r\\nFinal Score: %d points\\r\\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\\r\\nExiting lab. Great work!\\r\\n");
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