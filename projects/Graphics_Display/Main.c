/*
 * Graphics Display - Comprehensive Educational Demo
 * ATmega128 Educational Framework
 *
 * LEARNING OBJECTIVES:
 * - Understand GLCD (Graphic LCD) operation
 * - Learn bitmap graphics programming
 * - Practice text and shape drawing
 * - Master data visualization techniques
 * - Explore UI element creation
 *
 * HARDWARE SETUP:
 * - Connect GLCD to appropriate pins (see config.h)
 * - Ensure proper power supply and contrast adjustment
 * - Optional: buttons for demo navigation
 *
 * NEW CAPABILITIES DEMONSTRATED:
 * 1. Filled shapes (rectangles, circles)
 * 2. Data visualization (bar graphs, progress bars)
 * 3. Large text rendering (2x size)
 * 4. Bitmap and icon display
 * 5. Triangle and polygon drawing
 * 6. Formatted data display
 */

#include "config.h"

// Sample icons (8x8 pixels)
const unsigned char icon_battery[8] = {
    0x3C, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x3C};

const unsigned char icon_temp[8] = {
    0x04, 0x0A, 0x0A, 0x0A, 0x0A, 0x1F, 0x1F, 0x0E};

const unsigned char icon_signal[8] = {
    0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF};

// Sample bitmap (16x16 logo)
const unsigned char logo_16x16[32] = {
    0xFF, 0x81, 0xBD, 0xA5, 0xA5, 0xBD, 0x81, 0xFF,
    0xFF, 0x81, 0x81, 0xBD, 0xBD, 0x81, 0x81, 0xFF,
    0xFF, 0x81, 0xBD, 0xA5, 0xA5, 0xBD, 0x81, 0xFF,
    0xFF, 0x81, 0x81, 0xBD, 0xBD, 0x81, 0x81, 0xFF};

void demo_shapes(void);
void demo_filled_shapes(void);
void demo_data_visualization(void);
void demo_text_formatting(void);
void demo_icons_bitmaps(void);
void demo_sensor_display(void);

int main(void)
{
    // Initialize system components
    init_devices();
    lcd_init(); // Initialize GLCD

    uint8_t demo_mode = 0;

    while (1)
    {
        ScreenBuffer_clear();
        lcd_clear();

        switch (demo_mode)
        {
        case 0:
            demo_shapes();
            break;
        case 1:
            demo_filled_shapes();
            break;
        case 2:
            demo_data_visualization();
            break;
        case 3:
            demo_text_formatting();
            break;
        case 4:
            demo_icons_bitmaps();
            break;
        case 5:
            demo_sensor_display();
            break;
        }

        _delay_ms(3000);
        demo_mode = (demo_mode + 1) % 6;
    }

    return 0;
}

/*
 * Demo 1: Basic Shapes
 * Demonstrates line, rectangle, circle, and triangle drawing
 */
void demo_shapes(void)
{
    lcd_string(0, 0, "Basic Shapes");

    // Draw lines
    GLCD_Line(5, 10, 40, 20);
    GLCD_Line(5, 20, 40, 10);

    // Draw rectangle
    GLCD_Rectangle(10, 30, 20, 60);

    // Draw circle
    GLCD_Circle(15, 90, 10);

    // Draw triangle
    GLCD_Triangle(30, 30, 25, 45, 35, 45);

    lcd_string(0, 7, "Lines/Rect/Circle");
}

/*
 * Demo 2: Filled Shapes
 * Demonstrates filled rectangles and circles
 */
void demo_filled_shapes(void)
{
    lcd_string(0, 0, "Filled Shapes");

    // Filled rectangles
    GLCD_Rectangle_Fill(5, 10, 15, 30);
    GLCD_Rectangle_Fill(5, 40, 15, 60);
    GLCD_Rectangle_Fill(5, 70, 15, 90);

    // Filled circles of different sizes
    GLCD_Circle_Fill(30, 20, 8);
    GLCD_Circle_Fill(30, 50, 6);
    GLCD_Circle_Fill(30, 80, 4);

    lcd_string(0, 7, "Solid Fills");
}

/*
 * Demo 3: Data Visualization
 * Demonstrates bar graphs and progress bars for sensor data
 */
void demo_data_visualization(void)
{
    lcd_string(0, 0, "Data Charts");

    // Horizontal bars showing different values
    lcd_string(0, 1, "CPU:");
    GLCD_Bar_Horizontal(8, 36, 40, 5, 75);

    lcd_string(0, 2, "MEM:");
    GLCD_Bar_Horizontal(16, 36, 40, 5, 50);

    lcd_string(0, 3, "DSK:");
    GLCD_Bar_Horizontal(24, 36, 40, 5, 90);

    // Vertical bars
    GLCD_Bar_Vertical(35, 55, 6, 25, 60);
    GLCD_Bar_Vertical(35, 68, 6, 25, 80);
    GLCD_Bar_Vertical(35, 81, 6, 25, 40);

    // Progress bar
    GLCD_Progress_Bar(40, 10, 60, 6, 65);
    lcd_string(5, 7, "Progress: 65%");
}

/*
 * Demo 4: Text Formatting
 * Demonstrates different text sizes and formatted output
 */
void demo_text_formatting(void)
{
    // Regular text
    lcd_string(0, 0, "Regular Text");
    lcd_string(0, 1, "5x7 Font");

    // Large text (2x size)
    GLCD_String_Large(20, 40, "BIG");

    // Formatted value display
    GLCD_Display_Value(0, 5, "Temp:", 25, 2);
    GLCD_Display_Value(0, 6, "Val:", 123, 3);

    lcd_string(0, 7, "Text Styles");
}

/*
 * Demo 5: Icons and Bitmaps
 * Demonstrates custom graphics display
 */
void demo_icons_bitmaps(void)
{
    lcd_string(0, 0, "Icons & Bitmaps");

    // Display various 8x8 icons
    GLCD_Icon_8x8(10, 10, icon_battery);
    GLCD_Icon_8x8(10, 25, icon_temp);
    GLCD_Icon_8x8(10, 40, icon_signal);

    // Display 16x16 logo
    GLCD_Bitmap(20, 60, 16, 16, logo_16x16);

    lcd_string(0, 6, "Battery Temp Sig");
    lcd_string(0, 7, "Custom Graphics");
}

/*
 * Demo 6: Simulated Sensor Display
 * Demonstrates complete sensor dashboard with all features
 */
void demo_sensor_display(void)
{
    // Title
    lcd_string(0, 0, "Sensor Dashboard");

    // Temperature display with icon
    GLCD_Icon_8x8(5, 10, icon_temp);
    GLCD_Display_Value(1, 2, "T:", 28, 2);
    lcd_char('C');

    // Battery level with icon and bar
    GLCD_Icon_8x8(5, 40, icon_battery);
    GLCD_Bar_Horizontal(13, 46, 30, 5, 85);

    // Signal strength with icon and bars
    GLCD_Icon_8x8(5, 85, icon_signal);
    GLCD_Bar_Vertical(13, 120, 3, 10, 90);
    GLCD_Bar_Vertical(17, 120, 3, 10, 70);
    GLCD_Bar_Vertical(21, 120, 3, 10, 50);

    // Multiple sensor readings
    GLCD_Display_Value(3, 0, "ADC0:", 512, 3);
    GLCD_Display_Value(4, 0, "ADC1:", 768, 3);

    // Status indicators (filled circles)
    GLCD_Circle_Fill(50, 10, 3); // Active sensor
    GLCD_Circle(50, 25, 3);      // Inactive sensor

    lcd_string(0, 7, "All Features!");
}
