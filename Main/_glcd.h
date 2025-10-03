/*
 * =============================================================================
 * EDUCATIONAL ATmega128 GRAPHICS LCD LIBRARY - HEADER FILE
 * =============================================================================
 *
 * COURSE: SOC 3050 - Embedded Systems and IoT
 * AUTHOR: Professor Kim
 *
 * PURPOSE:
 * Educational header for ATmega128 Graphics LCD (GLCD) operations using
 * KS0108 controller. This library provides comprehensive graphics functionality
 * with educational documentation for 128x64 pixel monochrome displays.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Understand graphics LCD architecture and pixel addressing
 * 2. Learn parallel interface communication protocols
 * 3. Implement graphics primitives and text rendering
 * 4. Explore bitmap operations and screen buffer management
 * 5. Practice coordinate systems and mathematical graphics
 *
 * HARDWARE SPECIFICATIONS:
 * - Display: 128x64 pixels monochrome LCD
 * - Controller: KS0108 (dual controller for left/right half)
 * - Interface: 8-bit parallel data bus + control signals
 * - Memory: Internal display RAM for pixel data storage
 * - Font: 5x7 pixel ASCII character set included
 *
 * LEARNING PROGRESSION:
 * Assembly → C → Python → IoT
 * Direct ports → Structured graphics → Object-oriented UI → Web interfaces
 *
 * =============================================================================
 */

#ifndef _GLCD_H_
#define _GLCD_H_

#include <avr/io.h>

/*
 * =============================================================================
 * GRAPHICS LCD HARDWARE CONSTANTS AND DEFINITIONS
 * =============================================================================
 */

/* Display specifications */
#define GLCD_WIDTH 128           // Display width in pixels
#define GLCD_HEIGHT 64           // Display height in pixels
#define GLCD_PAGES 8             // Number of pages (height/8)
#define GLCD_CONTROLLER_WIDTH 64 // Width per controller (left/right)

/* Character dimensions */
#define CHAR_WIDTH 5       // Character width in pixels
#define CHAR_HEIGHT 7      // Character height in pixels
#define CHAR_SPACING 1     // Space between characters
#define CHARS_PER_LINE 20  // Characters per line (128/6)
#define LINES_PER_SCREEN 8 // Lines per screen (64/8)

/* Drawing colors/modes */
#define GLCD_PIXEL_ON 1  // Set pixel (black)
#define GLCD_PIXEL_OFF 0 // Clear pixel (white)
#define GLCD_PIXEL_XOR 2 // XOR pixel (invert)

/*
 * =============================================================================
 * LOW-LEVEL HARDWARE INTERFACE FUNCTIONS
 * =============================================================================
 */

/*
 * HARDWARE FUNCTION: Send Command to Left Controller
 *
 * PURPOSE: Send control commands to left half of display (0-63 pixels)
 * PARAMETERS: cmd - Command byte for KS0108 controller
 * EDUCATIONAL VALUE: Parallel interface communication, controller addressing
 */
void cmndl(unsigned char cmd);

/*
 * HARDWARE FUNCTION: Send Command to Right Controller
 *
 * PURPOSE: Send control commands to right half of display (64-127 pixels)
 * PARAMETERS: cmd - Command byte for KS0108 controller
 * EDUCATIONAL VALUE: Dual controller management, display partitioning
 */
void cmndr(unsigned char cmd);

/*
 * HARDWARE FUNCTION: Send Command to Both Controllers
 *
 * PURPOSE: Send same command to both display halves simultaneously
 * PARAMETERS: cmd - Command byte for both controllers
 * EDUCATIONAL VALUE: Broadcast operations, synchronization
 */
void cmnda(unsigned char cmd);

/*
 * HARDWARE FUNCTION: Send Data to Left Controller
 *
 * PURPOSE: Write pixel data to left half of display
 * PARAMETERS: dat - Data byte (8 vertical pixels)
 * EDUCATIONAL VALUE: Pixel data format, vertical byte organization
 */
void datal(unsigned char dat);

/*
 * HARDWARE FUNCTION: Send Data to Right Controller
 *
 * PURPOSE: Write pixel data to right half of display
 * PARAMETERS: dat - Data byte (8 vertical pixels)
 * EDUCATIONAL VALUE: Controller coordination, memory mapping
 */
void datar(unsigned char dat);

/*
 * HARDWARE FUNCTION: Send Data to Both Controllers
 *
 * PURPOSE: Write same data to both display halves
 * PARAMETERS: dat - Data byte for both controllers
 * EDUCATIONAL VALUE: Parallel operations, pattern generation
 */
void dataa(unsigned char dat);

/*
 * =============================================================================
 * BASIC DISPLAY CONTROL FUNCTIONS
 * =============================================================================
 */

/*
 * SYSTEM FUNCTION: Initialize Graphics LCD
 *
 * PURPOSE: Set up GLCD hardware and prepare for operation
 * EDUCATIONAL VALUE: Hardware initialization sequences, controller setup
 */
void lcd_init(void);

/*
 * SYSTEM FUNCTION: Clear Entire Display
 *
 * PURPOSE: Set all pixels to off state (white/clear)
 * EDUCATIONAL VALUE: Memory clearing, display buffer management
 */
void lcd_clear(void);

/*
 * POSITIONING FUNCTION: Set Character Position
 *
 * PURPOSE: Position cursor for text output at character coordinates
 * PARAMETERS:
 *   x - Character column (0-19)
 *   y - Character row (0-7)
 * EDUCATIONAL VALUE: Character-based coordinate systems
 */
void lcd_xy(unsigned char x, unsigned char y);

/*
 * =============================================================================
 * TEXT RENDERING FUNCTIONS
 * =============================================================================
 */

/*
 * TEXT FUNCTION: Display Single Character
 *
 * PURPOSE: Render ASCII character at current cursor position
 * PARAMETERS: character - ASCII character code (32-126)
 * EDUCATIONAL VALUE: Font rendering, character-to-bitmap conversion
 */
void lcd_char(unsigned char character);

/*
 * TEXT FUNCTION: Display Text String
 *
 * PURPOSE: Render null-terminated string at specified position
 * PARAMETERS:
 *   x - Starting character column (0-19)
 *   y - Character row (0-7)
 *   string - Null-terminated text string
 * EDUCATIONAL VALUE: String processing, automatic positioning
 */
void lcd_string(unsigned char x, unsigned char y, char *string);

/*
 * =============================================================================
 * GRAPHICS PRIMITIVES - PIXEL LEVEL OPERATIONS
 * =============================================================================
 */

/*
 * GRAPHICS FUNCTION: Set Pixel Address
 *
 * PURPOSE: Position for pixel-level drawing operations
 * PARAMETERS:
 *   x - Horizontal pixel position (0-127)
 *   y - Vertical pixel position (0-63)
 * EDUCATIONAL VALUE: Pixel addressing, coordinate transformation
 */
void GLCD_Axis_xy(unsigned char x, unsigned char y);

/*
 * GRAPHICS FUNCTION: Draw Single Pixel
 *
 * PURPOSE: Set or clear individual pixel at specified coordinates
 * PARAMETERS:
 *   x - Horizontal pixel position (0-127)
 *   y - Vertical pixel position (0-63)
 * EDUCATIONAL VALUE: Pixel manipulation, bitmap concepts
 */
void GLCD_Dot(unsigned char x, unsigned char y);

/*
 * BUFFER FUNCTION: Clear Screen Buffer
 *
 * PURPOSE: Clear internal display buffer for off-screen drawing
 * EDUCATIONAL VALUE: Double buffering, memory management
 */
void ScreenBuffer_clear(void);

/*
 * =============================================================================
 * GRAPHICS PRIMITIVES - GEOMETRIC SHAPES
 * =============================================================================
 */

/*
 * GRAPHICS FUNCTION: Draw Line
 *
 * PURPOSE: Draw line between two points using Bresenham algorithm
 * PARAMETERS:
 *   x1, y1 - Starting point coordinates (0-127, 0-63)
 *   x2, y2 - Ending point coordinates (0-127, 0-63)
 * EDUCATIONAL VALUE: Line drawing algorithms, vector graphics
 */
void GLCD_Line(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2);

/*
 * GRAPHICS FUNCTION: Draw Rectangle
 *
 * PURPOSE: Draw rectangle outline between two corner points
 * PARAMETERS:
 *   x1, y1 - Top-left corner coordinates (0-127, 0-63)
 *   x2, y2 - Bottom-right corner coordinates (0-127, 0-63)
 * EDUCATIONAL VALUE: Geometric primitives, boundary operations
 */
void GLCD_Rectangle(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2);

/*
 * GRAPHICS FUNCTION: Draw Circle
 *
 * PURPOSE: Draw circle outline using midpoint circle algorithm
 * PARAMETERS:
 *   x1, y1 - Center point coordinates (0-127, 0-63)
 *   r - Radius in pixels
 * EDUCATIONAL VALUE: Circle algorithms, trigonometric approximations
 */
void GLCD_Circle(unsigned char x1, unsigned char y1, unsigned char r);

/*
 * GRAPHICS FUNCTION: Draw Filled Rectangle
 *
 * PURPOSE: Draw solid filled rectangle between two corner points
 * PARAMETERS:
 *   x1, y1 - Top-left corner coordinates (0-127, 0-63)
 *   x2, y2 - Bottom-right corner coordinates (0-127, 0-63)
 * EDUCATIONAL VALUE: Area filling algorithms, pixel scanning
 */
void GLCD_Rectangle_Fill(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2);

/*
 * GRAPHICS FUNCTION: Draw Filled Circle
 *
 * PURPOSE: Draw solid filled circle using scan-line fill algorithm
 * PARAMETERS:
 *   x1, y1 - Center point coordinates (0-127, 0-63)
 *   r - Radius in pixels
 * EDUCATIONAL VALUE: Scan-line filling, circle equations
 */
void GLCD_Circle_Fill(unsigned char x1, unsigned char y1, unsigned char r);

/*
 * GRAPHICS FUNCTION: Draw Triangle
 *
 * PURPOSE: Draw triangle outline connecting three points
 * PARAMETERS:
 *   x1, y1 - First vertex coordinates (0-127, 0-63)
 *   x2, y2 - Second vertex coordinates (0-127, 0-63)
 *   x3, y3 - Third vertex coordinates (0-127, 0-63)
 * EDUCATIONAL VALUE: Polygon rendering, vector graphics
 */
void GLCD_Triangle(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2,
                   unsigned char x3, unsigned char y3);

/*
 * =============================================================================
 * DATA VISUALIZATION FUNCTIONS
 * =============================================================================
 */

/*
 * VISUALIZATION FUNCTION: Draw Horizontal Bar Graph
 *
 * PURPOSE: Display horizontal bar representing value magnitude
 * PARAMETERS:
 *   x, y - Starting position (0-127, 0-63)
 *   width - Maximum bar width in pixels
 *   height - Bar height in pixels
 *   value - Current value (0-100 percentage)
 * EDUCATIONAL VALUE: Data visualization, proportional representation
 */
void GLCD_Bar_Horizontal(unsigned char x, unsigned char y, unsigned char width,
                         unsigned char height, unsigned char value);

/*
 * VISUALIZATION FUNCTION: Draw Vertical Bar Graph
 *
 * PURPOSE: Display vertical bar representing value magnitude
 * PARAMETERS:
 *   x, y - Starting position (bottom-left, 0-127, 0-63)
 *   width - Bar width in pixels
 *   height - Maximum bar height in pixels
 *   value - Current value (0-100 percentage)
 * EDUCATIONAL VALUE: Data visualization, column charts
 */
void GLCD_Bar_Vertical(unsigned char x, unsigned char y, unsigned char width,
                       unsigned char height, unsigned char value);

/*
 * VISUALIZATION FUNCTION: Draw Progress Bar
 *
 * PURPOSE: Display progress indicator with border and fill
 * PARAMETERS:
 *   x, y - Top-left position (0-127, 0-63)
 *   width - Total width in pixels
 *   height - Total height in pixels
 *   value - Progress value (0-100 percentage)
 * EDUCATIONAL VALUE: User interface elements, status indicators
 */
void GLCD_Progress_Bar(unsigned char x, unsigned char y, unsigned char width,
                       unsigned char height, unsigned char value);

/*
 * =============================================================================
 * TEXT ENHANCEMENT FUNCTIONS
 * =============================================================================
 */

/*
 * TEXT FUNCTION: Display Large Character (2x size)
 *
 * PURPOSE: Render character at double size for better visibility
 * PARAMETERS:
 *   x, y - Character position (0-9, 0-3)
 *   character - ASCII character code (32-126)
 * EDUCATIONAL VALUE: Font scaling, pixel replication
 */
void GLCD_Char_Large(unsigned char x, unsigned char y, unsigned char character);

/*
 * TEXT FUNCTION: Display Large String (2x size)
 *
 * PURPOSE: Render string at double size for headings and important text
 * PARAMETERS:
 *   x, y - Starting position (0-9, 0-3)
 *   string - Null-terminated text string
 * EDUCATIONAL VALUE: Scaled text rendering, display hierarchy
 */
void GLCD_String_Large(unsigned char x, unsigned char y, char *string);

/*
 * TEXT FUNCTION: Display Formatted Integer
 *
 * PURPOSE: Display integer with label for sensor readings
 * PARAMETERS:
 *   x, y - Text position (0-19, 0-7)
 *   label - Descriptive text label
 *   value - Integer value to display
 *   digits - Number of digits (2-4)
 * EDUCATIONAL VALUE: Formatted output, data labeling
 */
void GLCD_Display_Value(unsigned char x, unsigned char y, char *label,
                        unsigned int value, unsigned char digits);

/*
 * =============================================================================
 * BITMAP AND ICON FUNCTIONS
 * =============================================================================
 */

/*
 * GRAPHICS FUNCTION: Draw Bitmap Image
 *
 * PURPOSE: Display custom bitmap at specified position
 * PARAMETERS:
 *   x, y - Top-left position (0-127, 0-63)
 *   width - Bitmap width in pixels
 *   height - Bitmap height in pixels
 *   bitmap - Pointer to bitmap data array
 * EDUCATIONAL VALUE: Image rendering, memory-to-display mapping
 */
void GLCD_Bitmap(unsigned char x, unsigned char y, unsigned char width,
                 unsigned char height, const unsigned char *bitmap);

/*
 * GRAPHICS FUNCTION: Draw Small Icon (8x8)
 *
 * PURPOSE: Display 8x8 pixel icon for UI elements
 * PARAMETERS:
 *   x, y - Icon position (0-127, 0-63)
 *   icon - Pointer to 8-byte icon data
 * EDUCATIONAL VALUE: Icon systems, compact graphics
 */
void GLCD_Icon_8x8(unsigned char x, unsigned char y, const unsigned char *icon);

/*
 * =============================================================================
 * NUMERIC DISPLAY FUNCTIONS
 * =============================================================================
 */

/*
 * NUMERIC FUNCTION: Display 1-Digit Decimal
 *
 * PURPOSE: Display single digit with optional leading zero suppression
 * PARAMETERS:
 *   number - Value to display (0-9)
 *   flag - Display format flag
 * RETURNS: Display control flag
 * EDUCATIONAL VALUE: Number formatting, conditional display
 */
unsigned char GLCD_1DigitDecimal(unsigned char number, unsigned char flag);

/*
 * NUMERIC FUNCTION: Display 2-Digit Decimal
 *
 * PURPOSE: Display two-digit decimal number (00-99)
 * PARAMETERS: number - Value to display (0-99)
 * EDUCATIONAL VALUE: Multi-digit formatting, decimal conversion
 */
void GLCD_2DigitDecimal(unsigned char number);

/*
 * NUMERIC FUNCTION: Display 3-Digit Decimal
 *
 * PURPOSE: Display three-digit decimal number (000-999)
 * PARAMETERS: number - Value to display (0-999)
 * EDUCATIONAL VALUE: Number processing, digit extraction
 */
void GLCD_3DigitDecimal(unsigned int number);

/*
 * NUMERIC FUNCTION: Display 4-Digit Decimal
 *
 * PURPOSE: Display four-digit decimal number (0000-9999)
 * PARAMETERS: number - Value to display (0-9999)
 * EDUCATIONAL VALUE: Large number handling, formatting algorithms
 */
void GLCD_4DigitDecimal(unsigned int number);

/*
 * =============================================================================
 * EDUCATIONAL USAGE EXAMPLES AND LEARNING OBJECTIVES
 * =============================================================================
 *
 * BASIC DISPLAY EXAMPLE:
 *   lcd_init();                              // Initialize display
 *   lcd_clear();                             // Clear screen
 *   lcd_string(0, 0, "Hello World!");        // Display text
 *
 * GRAPHICS EXAMPLE:
 *   GLCD_Line(0, 0, 127, 63);               // Draw diagonal line
 *   GLCD_Rectangle(10, 10, 50, 30);         // Draw rectangle
 *   GLCD_Circle(64, 32, 20);                // Draw circle
 *   GLCD_Rectangle_Fill(15, 15, 45, 25);    // Draw filled rectangle
 *   GLCD_Circle_Fill(64, 32, 15);           // Draw filled circle
 *   GLCD_Triangle(30, 10, 20, 30, 40, 30);  // Draw triangle
 *
 * DATA VISUALIZATION EXAMPLE:
 *   GLCD_Bar_Horizontal(10, 20, 60, 8, 75);      // 75% horizontal bar
 *   GLCD_Bar_Vertical(10, 50, 8, 30, 50);        // 50% vertical bar
 *   GLCD_Progress_Bar(10, 10, 80, 10, 65);       // 65% progress bar
 *
 * TEXT FORMATTING EXAMPLE:
 *   lcd_string(0, 0, "Normal Text");             // Regular 5x7 text
 *   GLCD_String_Large(0, 2, "BIG");              // Large 2x text
 *   GLCD_Display_Value(0, 4, "Temp:", 25, 2);    // Labeled value
 *
 * ICON AND BITMAP EXAMPLE:
 *   const unsigned char battery_icon[8] = {0x3C,0x24,0x24,0x24,0x24,0x24,0x24,0x3C};
 *   GLCD_Icon_8x8(10, 10, battery_icon);         // Display 8x8 icon
 *   GLCD_Bitmap(20, 30, 16, 16, my_logo);        // Display bitmap
 *
 * SENSOR DASHBOARD EXAMPLE:
 *   lcd_string(0, 0, "Dashboard");
 *   GLCD_Display_Value(0, 2, "ADC:", adc_val, 3);
 *   GLCD_Bar_Horizontal(8, 40, 60, 6, battery_pct);
 *   GLCD_Icon_8x8(5, 10, temp_icon);
 *
 * COMMON ICON DEFINITIONS (8x8 pixels):
 *   Battery:    {0x3C,0x24,0x24,0x24,0x24,0x24,0x24,0x3C}
 *   Temp:       {0x04,0x0A,0x0A,0x0A,0x0A,0x1F,0x1F,0x0E}
 *   Signal:     {0x01,0x03,0x07,0x0F,0x1F,0x3F,0x7F,0xFF}
 *   WiFi:       {0x00,0x0E,0x11,0x04,0x0A,0x00,0x04,0x00}
 *   Heart:      {0x00,0x66,0x99,0x81,0x42,0x24,0x18,0x00}
 *   Star:       {0x08,0x08,0x2A,0x1C,0x1C,0x2A,0x08,0x08}
 *   Check:      {0x00,0x01,0x02,0x04,0x48,0x50,0x20,0x00}
 *   X:          {0x00,0x41,0x22,0x14,0x14,0x22,0x41,0x00}
 *   Arrow Up:   {0x08,0x1C,0x2A,0x49,0x08,0x08,0x08,0x00}
 *   Arrow Down: {0x00,0x08,0x08,0x08,0x49,0x2A,0x1C,0x08}
 *
 * LEARNING OBJECTIVES ACHIEVED:
 * 1. ✓ Graphics LCD hardware interface and communication
 * 2. ✓ Pixel addressing and coordinate systems
 * 3. ✓ Text rendering and font management (normal and scaled)
 * 4. ✓ Graphics primitives and geometric algorithms
 * 5. ✓ Buffer management and display optimization
 * 6. ✓ Numeric formatting and data presentation
 * 7. ✓ Real-time graphics and user interface design
 * 8. ✓ Integration with sensor data and system feedback
 * 9. ✓ Data visualization with charts and graphs
 * 10. ✓ Custom icon and bitmap display
 * 11. ✓ Professional UI element creation
 * 12. ✓ Complete sensor dashboard development
 *
 * PRACTICAL APPLICATIONS:
 * - Sensor data visualization (temperature, humidity, light)
 * - System status displays (battery, signal, CPU usage)
 * - User interfaces (menus, selections, confirmations)
 * - Real-time monitoring (ADC values, motor speed, position)
 * - IoT device displays (connection status, data sync)
 * - Educational demonstrations (waveforms, patterns, animations)
 * - Embedded system debugging (variable monitoring, state display)
 * - Industrial displays (process control, alarm indicators)
 *
 * =============================================================================
 */

#endif /* _GLCD_H_ */
