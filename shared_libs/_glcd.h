/*
 * =============================================================================
 * COMPLETE KS0108 GRAPHICS LCD LIBRARY - HEADER FILE
 * =============================================================================
 *
 * AUTHOR: AI Assistant
 * PURPOSE: Comprehensive KS0108 GLCD library built from detailed specifications
 *
 * HARDWARE SPECIFICATIONS:
 * - Display: 128x64 pixels monochrome LCD
 * - Controller: Dual KS0108 controllers (left/right 64x64 each)
 * - Interface: 8-bit parallel data bus + 5 control signals
 * - Memory: 8 pages × 64 columns per controller (page = 8 vertical pixels)
 *
 * PIN CONNECTIONS (ATmega128):
 * - D0-D7:  PORTA (8-bit data bus)
 * - RS:     PE4 (Register Select: 0=Command, 1=Data)
 * - R/W:    GND (Read/Write: tied to ground for write-only)
 * - E:      PE5 (Enable: falling edge triggers operation)
 * - CS1:    PE7 (Chip Select 1: left controller, columns 0-63)
 * - CS2:    PE6 (Chip Select 2: right controller, columns 64-127)
 *
 * KS0108 COMMAND SET:
 * - 0x3E: Display OFF
 * - 0x3F: Display ON
 * - 0x40-0x7F: Set Y Address (column 0-63)
 * - 0xB8-0xBF: Set X Address (page 0-7)
 * - 0xC0-0xFF: Set Z Address (start line 0-63)
 *
 * TIMING REQUIREMENTS:
 * - Enable pulse width: minimum 450ns
 * - Setup time: minimum 140ns
 * - Hold time: minimum 10ns
 * - Command execution: varies (2μs typical)
 *
 * =============================================================================
 */

#ifndef KS0108_COMPLETE_H
#define KS0108_COMPLETE_H

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

/*
 * =============================================================================
 * HARDWARE CONSTANTS AND CONFIGURATION
 * =============================================================================
 */

/* Display specifications */
#define KS0108_WIDTH 128           // Total display width in pixels
#define KS0108_HEIGHT 64           // Total display height in pixels
#define KS0108_PAGES 8             // Number of pages (height/8)
#define KS0108_CONTROLLER_WIDTH 64 // Width per controller
#define KS0108_CONTROLLERS 2       // Number of controllers (left/right)

/* Hardware pin definitions (ATmega128) */
#define KS0108_DATA_PORT PORTA // 8-bit data bus
#define KS0108_DATA_DDR DDRA
#define KS0108_DATA_PIN PINA

#define KS0108_CONTROL_PORT PORTE // Control signals
#define KS0108_CONTROL_DDR DDRE
#define KS0108_CONTROL_PIN PINE

#define KS0108_RS_BIT 4  // PE4 - Register Select
#define KS0108_E_BIT 5   // PE5 - Enable
#define KS0108_CS2_BIT 6 // PE6 - Chip Select 2 (right)
#define KS0108_CS1_BIT 7 // PE7 - Chip Select 1 (left)

/* KS0108 Commands */
#define KS0108_CMD_DISPLAY_OFF 0x3E    // Turn display off
#define KS0108_CMD_DISPLAY_ON 0x3F     // Turn display on
#define KS0108_CMD_SET_Y_ADDRESS 0x40  // Set Y address (OR with column 0-63)
#define KS0108_CMD_SET_PAGE 0xB8       // Set page address (OR with page 0-7)
#define KS0108_CMD_SET_START_LINE 0xC0 // Set start line (OR with line 0-63)

/* Controller selection */
#define KS0108_LEFT_CONTROLLER 1  // Left controller (CS1)
#define KS0108_RIGHT_CONTROLLER 2 // Right controller (CS2)
#define KS0108_BOTH_CONTROLLERS 3 // Both controllers

/* Timing constants (microseconds) */
#define KS0108_DELAY_SETUP 1    // Setup time before enable
#define KS0108_DELAY_ENABLE 1   // Enable pulse width
#define KS0108_DELAY_HOLD 2     // Hold time after enable
#define KS0108_DELAY_COMMAND 10 // Command execution time

/* Drawing modes */
#define KS0108_PIXEL_OFF 0 // Clear pixel (white)
#define KS0108_PIXEL_ON 1  // Set pixel (black)
#define KS0108_PIXEL_XOR 2 // XOR pixel (invert)

/* Character dimensions */
#define KS0108_CHAR_WIDTH 5       // Character width in pixels
#define KS0108_CHAR_HEIGHT 7      // Character height in pixels
#define KS0108_CHAR_SPACING 1     // Space between characters
#define KS0108_CHARS_PER_LINE 21  // Characters per line (128/6)
#define KS0108_LINES_PER_SCREEN 8 // Text lines per screen

/*
 * =============================================================================
 * DATA TYPES AND STRUCTURES
 * =============================================================================
 */

/* Point structure for coordinates */
typedef struct
{
    uint8_t x;
    uint8_t y;
} ks0108_point_t;

/* Rectangle structure */
typedef struct
{
    uint8_t x;
    uint8_t y;
    uint8_t width;
    uint8_t height;
} ks0108_rect_t;

/* Display state structure */
typedef struct
{
    uint8_t current_page[KS0108_CONTROLLERS];   // Current page for each controller
    uint8_t current_column[KS0108_CONTROLLERS]; // Current column for each controller
    uint8_t display_on;                         // Display on/off state
    uint8_t start_line[KS0108_CONTROLLERS];     // Start line for each controller
} ks0108_state_t;

/*
 * =============================================================================
 * LOW-LEVEL HARDWARE FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Initialize KS0108 GLCD hardware and controllers
 *
 * Sets up port directions, initializes both controllers, and clears display.
 * Call this function once at startup before using any other functions.
 */
void ks0108_init(void);

/*
 * Compatibility function for legacy code
 * Provides backward compatibility with old lcd_init() calls
 */
void lcd_init(void);

/**
 * @brief Send command to specified controller(s)
 *
 * @param cmd Command byte to send
 * @param controller Controller selection (LEFT, RIGHT, or BOTH)
 */
void ks0108_command(uint8_t cmd, uint8_t controller);

/**
 * @brief Send data byte to specified controller(s)
 *
 * @param data Data byte to send
 * @param controller Controller selection (LEFT, RIGHT, or BOTH)
 */
void ks0108_data(uint8_t data, uint8_t controller);

/**
 * @brief Read data byte from specified controller
 *
 * @param controller Controller to read from (LEFT or RIGHT only)
 * @return Data byte read from controller
 */
uint8_t ks0108_read(uint8_t controller);

/**
 * @brief Set page address for specified controller(s)
 *
 * @param page Page number (0-7)
 * @param controller Controller selection (LEFT, RIGHT, or BOTH)
 */
void ks0108_set_page(uint8_t page, uint8_t controller);

/**
 * @brief Set column address for specified controller(s)
 *
 * @param column Column number (0-63 for each controller)
 * @param controller Controller selection (LEFT, RIGHT, or BOTH)
 */
void ks0108_set_column(uint8_t column, uint8_t controller);

/**
 * @brief Set cursor position (page, column) for drawing operations
 *
 * @param page Page number (0-7)
 * @param column Global column number (0-127)
 */
void ks0108_goto_xy(uint8_t page, uint8_t column);

/*
 * =============================================================================
 * DISPLAY CONTROL FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Turn display on or off
 *
 * @param on 1 to turn display on, 0 to turn off
 */
void ks0108_display_on_off(uint8_t on);

/**
 * @brief Clear entire display (set all pixels to off/white)
 */
void ks0108_clear_screen(void);

/**
 * @brief Fill entire display (set all pixels to on/black)
 */
void ks0108_fill_screen(void);

/**
 * @brief Set start line for display scrolling
 *
 * @param line Start line (0-63)
 * @param controller Controller selection (LEFT, RIGHT, or BOTH)
 */
void ks0108_set_start_line(uint8_t line, uint8_t controller);

/*
 * =============================================================================
 * PIXEL AND GRAPHICS FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Set, clear, or invert a single pixel
 *
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @param mode Pixel mode (ON, OFF, or XOR)
 */
void ks0108_set_pixel(uint8_t x, uint8_t y, uint8_t mode);

/**
 * @brief Get pixel state at specified coordinates
 *
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @return 1 if pixel is on, 0 if off
 */
uint8_t ks0108_get_pixel(uint8_t x, uint8_t y);

/**
 * @brief Draw horizontal line
 *
 * @param x Starting X coordinate
 * @param y Y coordinate
 * @param length Line length in pixels
 * @param mode Pixel mode (ON, OFF, or XOR)
 */
void ks0108_draw_hline(uint8_t x, uint8_t y, uint8_t length, uint8_t mode);

/**
 * @brief Draw vertical line
 *
 * @param x X coordinate
 * @param y Starting Y coordinate
 * @param length Line length in pixels
 * @param mode Pixel mode (ON, OFF, or XOR)
 */
void ks0108_draw_vline(uint8_t x, uint8_t y, uint8_t length, uint8_t mode);

/**
 * @brief Draw line between two points using Bresenham's algorithm
 *
 * @param x1 Starting X coordinate
 * @param y1 Starting Y coordinate
 * @param x2 Ending X coordinate
 * @param y2 Ending Y coordinate
 * @param mode Pixel mode (ON, OFF, or XOR)
 */
void ks0108_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode);

/**
 * @brief Draw rectangle outline
 *
 * @param x Top-left X coordinate
 * @param y Top-left Y coordinate
 * @param width Rectangle width
 * @param height Rectangle height
 * @param mode Pixel mode (ON, OFF, or XOR)
 */
void ks0108_draw_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t mode);

/**
 * @brief Draw filled rectangle
 *
 * @param x Top-left X coordinate
 * @param y Top-left Y coordinate
 * @param width Rectangle width
 * @param height Rectangle height
 * @param mode Pixel mode (ON, OFF, or XOR)
 */
void ks0108_fill_rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t mode);

/**
 * @brief Draw circle outline using midpoint algorithm
 *
 * @param cx Center X coordinate
 * @param cy Center Y coordinate
 * @param radius Circle radius
 * @param mode Pixel mode (ON, OFF, or XOR)
 */
void ks0108_draw_circle(uint8_t cx, uint8_t cy, uint8_t radius, uint8_t mode);

/**
 * @brief Draw filled circle
 *
 * @param cx Center X coordinate
 * @param cy Center Y coordinate
 * @param radius Circle radius
 * @param mode Pixel mode (ON, OFF, or XOR)
 */
void ks0108_fill_circle(uint8_t cx, uint8_t cy, uint8_t radius, uint8_t mode);

/*
 * =============================================================================
 * TEXT AND CHARACTER FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Set text cursor position
 *
 * @param line Text line (0-7)
 * @param column Character column (0-20)
 */
void ks0108_set_cursor(uint8_t line, uint8_t column);

/**
 * @brief Print single character at current cursor position
 *
 * @param c Character to print
 */
void ks0108_putchar(char c);

/**
 * @brief Print string at current cursor position
 *
 * @param str Null-terminated string to print
 */
void ks0108_puts(const char *str);

/**
 * @brief Print string at specified position
 *
 * @param line Text line (0-7)
 * @param column Character column (0-20)
 * @param str Null-terminated string to print
 */
void ks0108_puts_at(uint8_t line, uint8_t column, const char *str);

/**
 * @brief Print formatted string (basic printf functionality)
 *
 * @param format Format string with placeholders
 * @param ... Variable arguments
 */
void ks0108_printf(const char *format, ...);

/*
 * =============================================================================
 * UTILITY AND DEBUG FUNCTIONS
 * =============================================================================
 */

/**
 * @brief Get current display state
 *
 * @return Pointer to display state structure
 */
ks0108_state_t *ks0108_get_state(void);

/**
 * @brief Test display with various patterns
 *
 * Cycles through test patterns to verify display functionality
 */
void ks0108_test_display(void);

/**
 * @brief Display information about the library
 */
void ks0108_show_info(void);

#endif /* KS0108_COMPLETE_H */