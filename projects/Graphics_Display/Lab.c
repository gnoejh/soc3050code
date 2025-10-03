/*
 * =============================================================================
 * GRAPHICS PROGRAMMING - HANDS-ON LAB EXERCISES
 * =============================================================================
 * ATmega128 Educational Framework - Lab Session
 *
 * OBJECTIVE: Master GLCD graphics through creative projects
 * DURATION: 90 minutes
 * DIFFICULTY: Intermediate to Advanced
 *
 * STUDENTS WILL:
 * - Create interactive drawing applications
 * - Build data visualization tools
 * - Design user interfaces
 * - Implement animations
 * - Develop mini-games
 *
 * HARDWARE REQUIRED:
 * - ATmega128 board
 * - 128x64 GLCD display
 * - Joystick (ADC channels 0-1)
 * - 4 buttons for controls
 *
 * =============================================================================
 */

#include "config.h"

// Lab configuration
#define JOYSTICK_X_CHANNEL 0
#define JOYSTICK_Y_CHANNEL 1

// Global variables
uint16_t lab_score = 0;
uint8_t cursor_x = 64, cursor_y = 32;

/*
 * =============================================================================
 * LAB EXERCISE 1: INTERACTIVE DRAWING (20 minutes)
 * =============================================================================
 * OBJECTIVE: Create interactive drawing application
 * DIFFICULTY: ★★☆☆☆ (Easy-Medium)
 */

void lab_ex1_etch_a_sketch(void)
{
    /*
     * CHALLENGE: Build an Etch-A-Sketch style drawing tool
     * TASK: Control cursor with joystick, draw with button
     * LEARNING: Interactive graphics and user input
     */

    puts_USART1("\r\n=== Lab 1.1: Etch-A-Sketch ===\r\n");
    puts_USART1("Use joystick to draw!\r\n");
    puts_USART1("Button 0: Draw | Button 1: Erase | Button 2: Clear | Button 3: Exit\r\n\r\n");

    GLCD_Clear();
    GLCD_String(0, 0, "ETCH-A-SKETCH");
    GLCD_String(0, 1, "Joy:Move Btn:Draw");

    _delay_ms(2000);
    GLCD_Clear();

    cursor_x = 64;
    cursor_y = 32;
    uint8_t drawing = 0;

    while (1)
    {
        // Read joystick
        uint16_t joy_x = Read_Adc_Data(JOYSTICK_X_CHANNEL);
        uint16_t joy_y = Read_Adc_Data(JOYSTICK_Y_CHANNEL);

        // Map to movement (-2 to +2)
        int8_t dx = 0, dy = 0;
        if (joy_x < 300)
            dx = -2;
        else if (joy_x > 700)
            dx = 2;
        else if (joy_x < 450)
            dx = -1;
        else if (joy_x > 550)
            dx = 1;

        if (joy_y < 300)
            dy = -2;
        else if (joy_y > 700)
            dy = 2;
        else if (joy_y < 450)
            dy = -1;
        else if (joy_y > 550)
            dy = 1;

        // Update cursor position
        int16_t new_x = cursor_x + dx;
        int16_t new_y = cursor_y + dy;

        if (new_x >= 0 && new_x < 128)
            cursor_x = new_x;
        if (new_y >= 0 && new_y < 64)
            cursor_y = new_y;

        // Check buttons (PD4-7)
        uint8_t buttons = ~PIND & 0xF0;

        if (buttons & (1 << 4)) // Draw mode
        {
            drawing = 1;
        }
        else if (buttons & (1 << 5)) // Erase mode
        {
            drawing = 2;
        }
        else if (buttons & (1 << 6)) // Clear screen
        {
            GLCD_Clear();
            drawing = 0;
        }
        else if (buttons & (1 << 7)) // Exit
        {
            break;
        }
        else
        {
            drawing = 0;
        }

        // Draw or erase
        if (drawing == 1)
        {
            GLCD_Set_Dot(cursor_x, cursor_y);
        }
        else if (drawing == 2)
        {
            GLCD_Clr_Dot(cursor_x, cursor_y);
        }

        // Show cursor (blinking)
        static uint8_t blink = 0;
        if (++blink > 5)
        {
            blink = 0;
            // Draw cursor as small cross
            GLCD_Line(cursor_x - 2, cursor_y, cursor_x + 2, cursor_y);
            GLCD_Line(cursor_x, cursor_y - 2, cursor_x, cursor_y + 2);
            _delay_ms(50);
            GLCD_Line(cursor_x - 2, cursor_y, cursor_x + 2, cursor_y); // Erase by redrawing (XOR)
            GLCD_Line(cursor_x, cursor_y - 2, cursor_x, cursor_y + 2);
        }

        _delay_ms(50);
    }

    puts_USART1("Etch-A-Sketch complete!\r\n");
    lab_score += 100;
}

void lab_ex1_pattern_generator(void)
{
    /*
     * CHALLENGE: Generate geometric patterns
     * TASK: Create various mathematical art patterns
     * LEARNING: Parametric equations and graphics
     */

    puts_USART1("\r\n=== Lab 1.2: Pattern Generator ===\r\n");
    puts_USART1("Generating mathematical patterns...\r\n\r\n");

    // Pattern 1: Spiral
    puts_USART1("Pattern 1: Spiral\r\n");
    GLCD_Clear();
    GLCD_String(0, 0, "Spiral Pattern");

    for (uint16_t angle = 0; angle < 720; angle += 5)
    {
        float rad = angle * 3.14159 / 180.0;
        float r = angle / 20.0;
        int16_t x = 64 + r * cos(rad);
        int16_t y = 32 + r * sin(rad);

        if (x >= 0 && x < 128 && y >= 0 && y < 64)
        {
            GLCD_Set_Dot(x, y);
        }
    }

    _delay_ms(3000);

    // Pattern 2: Lissajous curve
    puts_USART1("Pattern 2: Lissajous Curve\r\n");
    GLCD_Clear();
    GLCD_String(0, 0, "Lissajous 3:2");

    for (uint16_t t = 0; t < 360; t += 2)
    {
        float rad = t * 3.14159 / 180.0;
        int16_t x = 64 + 50 * sin(3 * rad);
        int16_t y = 32 + 25 * sin(2 * rad);

        GLCD_Set_Dot(x, y);
    }

    _delay_ms(3000);

    // Pattern 3: Star polygon
    puts_USART1("Pattern 3: Star Polygon\r\n");
    GLCD_Clear();
    GLCD_String(0, 0, "Star Polygon");

#define STAR_POINTS 5
    for (uint8_t i = 0; i < STAR_POINTS; i++)
    {
        uint8_t next = (i + 2) % STAR_POINTS;

        float angle1 = i * 2 * 3.14159 / STAR_POINTS - 3.14159 / 2;
        float angle2 = next * 2 * 3.14159 / STAR_POINTS - 3.14159 / 2;

        int16_t x1 = 64 + 40 * cos(angle1);
        int16_t y1 = 32 + 25 * sin(angle1);
        int16_t x2 = 64 + 40 * cos(angle2);
        int16_t y2 = 32 + 25 * sin(angle2);

        GLCD_Line(x1, y1, x2, y2);
    }

    _delay_ms(3000);

    puts_USART1("Pattern generation complete!\r\n");
    lab_score += 75;
}

/*
 * =============================================================================
 * LAB EXERCISE 2: DATA VISUALIZATION (20 minutes)
 * =============================================================================
 * OBJECTIVE: Create real-time data visualization
 * DIFFICULTY: ★★★☆☆ (Medium)
 */

void lab_ex2_live_graph(void)
{
    /*
     * CHALLENGE: Plot sensor data in real-time
     * TASK: Create scrolling graph of ADC readings
     * LEARNING: Data plotting and scaling
     */

    puts_USART1("\r\n=== Lab 2.1: Live Data Graph ===\r\n");
    puts_USART1("Real-time scrolling graph of potentiometer\r\n\r\n");

    GLCD_Clear();
    GLCD_String(0, 0, "Live ADC Graph");

    // Draw axes
    GLCD_Line(10, 10, 10, 60);  // Y-axis
    GLCD_Line(10, 60, 125, 60); // X-axis

    // Y-axis labels
    GLCD_String(0, 1, "1023");
    GLCD_String(0, 5, "512");
    GLCD_String(0, 7, "0");

    uint8_t history[115]; // 115 pixels wide
    uint8_t write_pos = 0;

    // Initialize history
    for (uint8_t i = 0; i < 115; i++)
    {
        history[i] = 60; // Baseline
    }

    for (uint16_t sample = 0; sample < 500; sample++)
    {
        // Read ADC
        uint16_t adc = Read_Adc_Data(0);

        // Scale to display (10-60 pixels = ADC 0-1023)
        uint8_t y = 60 - ((uint32_t)adc * 50 / 1023);

        // Store in history
        history[write_pos] = y;
        write_pos = (write_pos + 1) % 115;

        // Clear graph area
        GLCD_Rectangle(11, 11, 125, 59, 0); // Clear plot area

        // Plot all points
        for (uint8_t i = 0; i < 114; i++)
        {
            uint8_t idx1 = (write_pos + i) % 115;
            uint8_t idx2 = (write_pos + i + 1) % 115;

            GLCD_Line(11 + i, history[idx1], 11 + i + 1, history[idx2]);
        }

        // Display current value
        char buffer[20];
        sprintf(buffer, "%4u", adc);
        GLCD_Rectangle(90, 0, 127, 7, 0); // Clear value area
        GLCD_String(90, 0, buffer);

        _delay_ms(50);

        // Exit check
        if (sample % 10 == 0 && (UCSR1A & (1 << RXC1)))
        {
            char c = UDR1;
            if (c == 'Q' || c == 'q')
                break;
        }
    }

    puts_USART1("Graphing complete!\r\n");
    lab_score += 125;
}

void lab_ex2_bar_chart_race(void)
{
    /*
     * CHALLENGE: Animated bar chart of multiple sensors
     * TASK: Display 3 sensors as racing bars
     * LEARNING: Multi-data visualization and animation
     */

    puts_USART1("\r\n=== Lab 2.2: Bar Chart Race ===\r\n");
    puts_USART1("Showing 3 sensor channels as racing bars\r\n\r\n");

    GLCD_Clear();
    GLCD_String(0, 0, "Sensor Race!");

    // Channel labels
    GLCD_String(0, 2, "CH0:");
    GLCD_String(0, 4, "CH1:");
    GLCD_String(0, 6, "CH2:");

    for (uint16_t i = 0; i < 200; i++)
    {
        // Read 3 channels
        uint16_t ch0 = Read_Adc_Data(0);
        uint16_t ch1 = Read_Adc_Data(1);
        uint16_t ch2 = Read_Adc_Data(2);

        // Scale to bar widths (0-90 pixels)
        uint8_t bar0 = (uint32_t)ch0 * 90 / 1023;
        uint8_t bar1 = (uint32_t)ch1 * 90 / 1023;
        uint8_t bar2 = (uint32_t)ch2 * 90 / 1023;

        // Clear and redraw bars
        GLCD_Rectangle(30, 16, 125, 23, 0);
        GLCD_Bar_Horizontal(30, 16, bar0, 6, 90);

        GLCD_Rectangle(30, 32, 125, 39, 0);
        GLCD_Bar_Horizontal(30, 32, bar1, 6, 90);

        GLCD_Rectangle(30, 48, 125, 55, 0);
        GLCD_Bar_Horizontal(30, 48, bar2, 6, 90);

        // Display values
        char buffer[10];
        sprintf(buffer, "%4u", ch0);
        GLCD_String(100, 2, buffer);
        sprintf(buffer, "%4u", ch1);
        GLCD_String(100, 4, buffer);
        sprintf(buffer, "%4u", ch2);
        GLCD_String(100, 6, buffer);

        _delay_ms(100);
    }

    puts_USART1("Bar chart complete!\r\n");
    lab_score += 100;
}

/*
 * =============================================================================
 * LAB EXERCISE 3: USER INTERFACE DESIGN (20 minutes)
 * =============================================================================
 * OBJECTIVE: Build professional UI elements
 * DIFFICULTY: ★★★★☆ (Advanced)
 */

void lab_ex3_menu_system(void)
{
    /*
     * CHALLENGE: Create interactive menu with selection
     * TASK: Navigate menu with joystick
     * LEARNING: UI design and navigation logic
     */

    puts_USART1("\r\n=== Lab 3.1: Menu System ===\r\n");
    puts_USART1("Navigate menu with joystick\r\n\r\n");

    const char *menu_items[] = {
        "1. Sensor Test",
        "2. Calibration",
        "3. Data Logger",
        "4. Settings",
        "5. About"};

    uint8_t selected = 0;
    uint8_t num_items = 5;

    while (1)
    {
        GLCD_Clear();
        GLCD_String(30, 0, "MAIN MENU");
        GLCD_Line(0, 9, 127, 9);

        // Display menu items
        for (uint8_t i = 0; i < num_items; i++)
        {
            if (i == selected)
            {
                // Highlight selected item
                GLCD_Rectangle_Fill(0, 12 + i * 10, 127, 20 + i * 10);
                // Invert text (would need inverse char function)
                GLCD_String(5, i + 2, menu_items[i]);
            }
            else
            {
                GLCD_String(5, i + 2, menu_items[i]);
            }
        }

        // Navigation arrows
        GLCD_String(110, 2, "^");
        GLCD_String(110, 6, "v");

        _delay_ms(200);

        // Read joystick
        uint16_t joy_y = Read_Adc_Data(JOYSTICK_Y_CHANNEL);

        if (joy_y < 300) // Up
        {
            if (selected > 0)
                selected--;
            _delay_ms(300); // Debounce
        }
        else if (joy_y > 700) // Down
        {
            if (selected < num_items - 1)
                selected++;
            _delay_ms(300);
        }

        // Button to select (PD4)
        if (!(PIND & (1 << 4)))
        {
            char msg[50];
            sprintf(msg, "Selected: %s\r\n", menu_items[selected]);
            puts_USART1(msg);

            GLCD_Clear();
            GLCD_String(20, 3, "SELECTED:");
            GLCD_String(10, 4, menu_items[selected]);
            _delay_ms(2000);

            if (selected == 4) // About = Exit
                break;
        }
    }

    puts_USART1("Menu system complete!\r\n");
    lab_score += 150;
}

void lab_ex3_dashboard_design(void)
{
    /*
     * CHALLENGE: Design professional dashboard
     * TASK: Create multi-widget dashboard layout
     * LEARNING: Layout design and information hierarchy
     */

    puts_USART1("\r\n=== Lab 3.2: Dashboard Design ===\r\n");
    puts_USART1("Professional sensor dashboard\r\n\r\n");

    for (uint16_t refresh = 0; refresh < 100; refresh++)
    {
        GLCD_Clear();

        // Title bar
        GLCD_Rectangle_Fill(0, 0, 127, 9);
        // Would need inverse text here
        GLCD_String(10, 0, "SENSOR DASHBOARD");

        // Left panel - Temperature
        GLCD_Rectangle(2, 12, 62, 40);
        GLCD_String(5, 2, "TEMP");

        uint16_t temp_adc = Read_Adc_Data(1);
        char buffer[20];
        sprintf(buffer, "%4u", temp_adc);
        GLCD_String_Large(10, 3, buffer);

        // Right panel - Light
        GLCD_Rectangle(66, 12, 125, 40);
        GLCD_String(70, 2, "LIGHT");

        uint16_t light_adc = Read_Adc_Data(2);
        sprintf(buffer, "%4u", light_adc);
        GLCD_String_Large(74, 3, buffer);

        // Bottom panel - Progress bar
        GLCD_String(5, 6, "Status:");
        uint16_t pot = Read_Adc_Data(0);
        uint8_t progress = (uint32_t)pot * 100 / 1023;
        GLCD_Progress_Bar(5, 50, 115, 10, progress);

        sprintf(buffer, "%3u%%", progress);
        GLCD_String(102, 7, buffer);

        // Status icons
        if (temp_adc > 512)
        {
            GLCD_Icon_8x8(2, 50, ICON_TEMP);
        }

        if (light_adc > 512)
        {
            GLCD_Icon_8x8(118, 12, ICON_STAR);
        }

        _delay_ms(200);
    }

    puts_USART1("Dashboard design complete!\r\n");
    lab_score += 175;
}

/*
 * =============================================================================
 * LAB EXERCISE 4: MINI-GAMES (30 minutes)
 * =============================================================================
 * OBJECTIVE: Create interactive games
 * DIFFICULTY: ★★★★★ (Expert)
 */

void lab_ex4_pong_game(void)
{
    /*
     * CHALLENGE: Implement single-player Pong
     * TASK: Joystick controls paddle, bounce ball
     * LEARNING: Game logic and physics
     */

    puts_USART1("\r\n=== Lab 4.1: Pong Game ===\r\n");
    puts_USART1("Classic Pong game!\r\n\r\n");

    // Game variables
    int8_t ball_x = 64, ball_y = 32;
    int8_t ball_dx = 2, ball_dy = 1;
    uint8_t paddle_y = 28;
    uint16_t score = 0;
    uint8_t lives = 3;

    while (lives > 0)
    {
        GLCD_Clear();

        // Score display
        char buffer[30];
        sprintf(buffer, "Score:%u Lives:%u", score, lives);
        GLCD_String(0, 0, buffer);

        // Update ball position
        ball_x += ball_dx;
        ball_y += ball_dy;

        // Ball collision with top/bottom
        if (ball_y <= 10 || ball_y >= 63)
        {
            ball_dy = -ball_dy;
        }

        // Ball collision with right wall
        if (ball_x >= 127)
        {
            ball_dx = -ball_dx;
            score += 10;
        }

        // Ball collision with paddle
        if (ball_x <= 5 && ball_y >= paddle_y && ball_y <= paddle_y + 16)
        {
            ball_dx = -ball_dx;
            ball_x = 6;
            score += 5;
        }

        // Ball missed
        if (ball_x < 0)
        {
            lives--;
            ball_x = 64;
            ball_y = 32;
            ball_dx = 2;
            ball_dy = 1;
            _delay_ms(1000);
        }

        // Read joystick for paddle
        uint16_t joy_y = Read_Adc_Data(JOYSTICK_Y_CHANNEL);
        paddle_y = 10 + ((uint32_t)(1023 - joy_y) * 38 / 1023);

        // Draw paddle (left side)
        GLCD_Rectangle_Fill(2, paddle_y, 4, paddle_y + 16);

        // Draw ball
        GLCD_Circle_Fill(ball_x, ball_y, 2);

        _delay_ms(50);
    }

    GLCD_Clear();
    GLCD_String(30, 2, "GAME OVER!");
    char final_score[30];
    sprintf(final_score, "Score: %u", score);
    GLCD_String(35, 4, final_score);

    puts_USART1("Pong game complete!\r\n");
    sprintf(final_score, "Final score: %u\r\n", score);
    puts_USART1(final_score);

    _delay_ms(3000);
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
    puts_USART1("  GRAPHICS PROGRAMMING - LAB EXERCISES\r\n");
    puts_USART1("========================================\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 1: Interactive Drawing\r\n");
    puts_USART1("  1. Etch-A-Sketch\r\n");
    puts_USART1("  2. Pattern Generator\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 2: Data Visualization\r\n");
    puts_USART1("  3. Live Data Graph\r\n");
    puts_USART1("  4. Bar Chart Race\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 3: UI Design\r\n");
    puts_USART1("  5. Menu System\r\n");
    puts_USART1("  6. Dashboard Design\r\n");
    puts_USART1("\r\n");
    puts_USART1("EXERCISE 4: Mini-Games\r\n");
    puts_USART1("  7. Pong Game\r\n");
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
    Adc_init();
    GLCD_Init();

    // Configure buttons
    DDRD &= ~0xF0; // PD4-7 as inputs
    PORTD |= 0xF0; // Enable pull-ups

    _delay_ms(100);

    puts_USART1("\r\n\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("*  ATmega128 GRAPHICS PROGRAMMING LAB          *\r\n");
    puts_USART1("*  Interactive GLCD Exercises                  *\r\n");
    puts_USART1("*************************************************\r\n");
    puts_USART1("\r\n");
    puts_USART1("Welcome to the Graphics Programming Lab!\r\n");
    puts_USART1("Create amazing visuals and interactive apps.\r\n");

    GLCD_Clear();
    GLCD_String(10, 2, "GRAPHICS LAB");
    GLCD_String(5, 4, "See Serial Monitor");
    GLCD_String(15, 6, "for Menu");

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
            lab_ex1_etch_a_sketch();
            break;
        case '2':
            lab_ex1_pattern_generator();
            break;
        case '3':
            lab_ex2_live_graph();
            break;
        case '4':
            lab_ex2_bar_chart_race();
            break;
        case '5':
            lab_ex3_menu_system();
            break;
        case '6':
            lab_ex3_dashboard_design();
            break;
        case '7':
            lab_ex4_pong_game();
            break;

        case '0':
            puts_USART1("\r\n*** RUNNING ALL EXERCISES ***\r\n");
            lab_ex1_etch_a_sketch();
            lab_ex1_pattern_generator();
            lab_ex2_live_graph();
            lab_ex2_bar_chart_race();
            lab_ex3_menu_system();
            lab_ex3_dashboard_design();
            lab_ex4_pong_game();

            char final_buffer[80];
            sprintf(final_buffer, "\r\n*** ALL EXERCISES COMPLETE! ***\r\nFinal Score: %u points\r\n", lab_score);
            puts_USART1(final_buffer);
            break;

        case 'X':
        case 'x':
            puts_USART1("\r\nExiting lab. Great work!\r\n");
            GLCD_Clear();
            GLCD_String(30, 3, "LAB COMPLETE!");
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
