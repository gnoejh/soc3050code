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
 *
 * NUMERIC DISPLAY EXAMPLE:
 *   lcd_xy(0, 2);                           // Position cursor
 *   GLCD_3DigitDecimal(temperature);        // Display sensor value
 *
 * LEARNING OBJECTIVES ACHIEVED:
 * 1. ✓ Graphics LCD hardware interface and communication
 * 2. ✓ Pixel addressing and coordinate systems
 * 3. ✓ Text rendering and font management
 * 4. ✓ Graphics primitives and geometric algorithms
 * 5. ✓ Buffer management and display optimization
 * 6. ✓ Numeric formatting and data presentation
 * 7. ✓ Real-time graphics and user interface design
 * 8. ✓ Integration with sensor data and system feedback
 *
 * =============================================================================
 */

#endif /* _GLCD_H_ */
