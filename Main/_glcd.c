/*
 * =============================================================================
 * EDUCATIONAL ATmega128 GRAPHICS LCD LIBRARY - IMPLEMENTATION
 * =============================================================================
 *
 * COURSE: SOC 3050 - Embedded Systems and IoT
 * AUTHOR: Professor Kim
 *
 * PURPOSE:
 * Comprehensive Graphics LCD library for ATmega128 using KS0108 controller.
 * This implementation provides educational framework for learning graphics
 * programming, display interfaces, and visual feedback systems.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master parallel interface protocols and timing
 * 2. Understand pixel addressing and memory organization
 * 3. Implement graphics algorithms and geometric primitives
 * 4. Learn text rendering and font management
 * 5. Explore user interface design principles
 *
 * HARDWARE INTERFACE:
 * - KS0108 Graphics LCD Controller (128x64 pixels)
 * - Dual controller architecture (left/right halves)
 * - 8-bit parallel data bus with control signals
 * - Memory-mapped display with page organization
 *
 * LEARNING PROGRESSION:
 * Assembly → C → Python → IoT
 * Port operations → Graphics functions → GUI frameworks → Web interfaces
 *
 * =============================================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "config.h"
#include "_main.h"
#include "_glcd.h"

// Only compile GLCD functions if not using self-contained assembly example
#ifndef ASSEMBLY_BLINK_BASIC

/*
 * =============================================================================
 * EDUCATIONAL CONSTANTS AND TYPE DEFINITIONS
 * =============================================================================
 */

/* Legacy type definitions for compatibility */
typedef unsigned char byte; // 8-bit unsigned integer
typedef unsigned int word;	// 16-bit unsigned integer

/* Display timing constants (microseconds) */
#define GLCD_DELAY_BEFORE 0 // Setup time before operation
#define GLCD_DELAY_AFTER 10 // Hold time after operation
#define GLCD_DELAY_MIDDLE 0 // Intermediate timing

/* Legacy timing constants for compatibility */
#define D_BEFORE GLCD_DELAY_BEFORE
#define D_AFTER GLCD_DELAY_AFTER
#define D_MIDDLE GLCD_DELAY_MIDDLE

/* KS0108 Controller Commands */
#define GLCD_CMD_DISPLAY_ON 0x3F  // Turn display on
#define GLCD_CMD_DISPLAY_OFF 0x3E // Turn display off
#define GLCD_CMD_SET_ADDRESS 0x40 // Set Y address (0-63)
#define GLCD_CMD_SET_PAGE 0xB8	  // Set X page (0-7)
#define GLCD_CMD_START_LINE 0xC0  // Set start line (0-63)

/* Legacy command constants for compatibility */
#define DISPON GLCD_CMD_DISPLAY_ON
#define DISPOFF GLCD_CMD_DISPLAY_OFF

/*
 * =============================================================================
 * GLOBAL VARIABLES FOR GRAPHICS STATE MANAGEMENT
 * =============================================================================
 */

/* Current cursor position for character operations */
unsigned char xchar = 0; // Character X position (0-19)
unsigned char ychar = 0; // Character Y position (0-7)

/* Display state variables */
#if defined(GRAPHICS_EXAMPLES) || defined(ADVANCED_EXAMPLES_ACTIVE)
static unsigned char glcd_initialized = 0; // Initialization flag
static unsigned char current_page = 0;	   // Current display page
static unsigned char current_column = 0;   // Current column position
static word d = 0;						   // General purpose delay variable
#endif

/*
 * =============================================================================
 * EDUCATIONAL FONT DATA - 5x7 ASCII CHARACTER SET
 * =============================================================================
 *
 * PURPOSE: Bitmap font data for text rendering
 * FORMAT: Each character is 5 bytes wide, drawn vertically (top to bottom)
 * RANGE: ASCII 32-126 (printable characters)
 * SIZE: 95 characters × 5 bytes = 475 bytes
 *
 * EDUCATIONAL NOTES:
 * - Each byte represents 8 vertical pixels
 * - Font is stored column by column (not row by row)
 * - Bit 0 = top pixel, Bit 7 = bottom pixel
 * - Characters are 5 pixels wide with 1 pixel spacing
 */

byte font[95][5] = {								 /* 5x7 ASCII character font draw a byte downwards */
					{0x00, 0x00, 0x00, 0x00, 0x00},	 /* 0x20 space */
					{0x00, 0x00, 0x4f, 0x00, 0x00},	 /* 0x21 ! */
					{0x00, 0x07, 0x00, 0x07, 0x00},	 /* 0x22 " */
					{0x14, 0x7f, 0x14, 0x7f, 0x14},	 /* 0x23 # */
					{0x24, 0x2a, 0x7f, 0x2a, 0x12},	 /* 0x24 $ */
					{0x23, 0x13, 0x08, 0x64, 0x62},	 /* 0x25 % */
					{0x36, 0x49, 0x55, 0x22, 0x50},	 /* 0x26 & */
					{0x00, 0x05, 0x03, 0x00, 0x00},	 /* 0x27 ' */
					{0x00, 0x1c, 0x22, 0x41, 0x00},	 /* 0x28 ( */
					{0x00, 0x41, 0x22, 0x1c, 0x00},	 /* 0x29 ) */
					{0x14, 0x08, 0x3e, 0x08, 0x14},	 /* 0x2a * */
					{0x08, 0x08, 0x3e, 0x08, 0x08},	 /* 0x2b + */
					{0x00, 0x50, 0x30, 0x00, 0x00},	 /* 0x2c , */
					{0x08, 0x08, 0x08, 0x08, 0x08},	 /* 0x2d - */
					{0x00, 0x60, 0x60, 0x00, 0x00},	 /* 0x2e . */
					{0x20, 0x10, 0x08, 0x04, 0x02},	 /* 0x2f / */
					{0x3e, 0x51, 0x49, 0x45, 0x3e},	 /* 0x30 0 */
					{0x00, 0x42, 0x7f, 0x40, 0x00},	 /* 0x31 1 */
					{0x42, 0x61, 0x51, 0x49, 0x46},	 /* 0x32 2 */
					{0x21, 0x41, 0x45, 0x4b, 0x31},	 /* 0x33 3 */
					{0x18, 0x14, 0x12, 0x7f, 0x10},	 /* 0x34 4 */
					{0x27, 0x45, 0x45, 0x45, 0x39},	 /* 0x35 5 */
					{0x3c, 0x4a, 0x49, 0x49, 0x30},	 /* 0x36 6 */
					{0x01, 0x71, 0x09, 0x05, 0x03},	 /* 0x37 7 */
					{0x36, 0x49, 0x49, 0x49, 0x36},	 /* 0x38 8 */
					{0x06, 0x49, 0x49, 0x29, 0x1e},	 /* 0x39 9 */
					{0x00, 0x36, 0x36, 0x00, 0x00},	 /* 0x3a : */
					{0x00, 0x56, 0x36, 0x00, 0x00},	 /* 0x3b ; */
					{0x08, 0x14, 0x22, 0x41, 0x00},	 /* 0x3c < */
					{0x14, 0x14, 0x14, 0x14, 0x14},	 /* 0x3d = */
					{0x00, 0x41, 0x22, 0x14, 0x08},	 /* 0x3e > */
					{0x02, 0x01, 0x51, 0x09, 0x06},	 /* 0x3f ? */
					{0x32, 0x49, 0x79, 0x41, 0x3e},	 /* 0x40 @ */
					{0x7e, 0x11, 0x11, 0x11, 0x7e},	 /* 0x41 A */
					{0x7f, 0x49, 0x49, 0x49, 0x36},	 /* 0x42 B */
					{0x3e, 0x41, 0x41, 0x41, 0x22},	 /* 0x43 C */
					{0x7f, 0x41, 0x41, 0x22, 0x1c},	 /* 0x44 D */
					{0x7f, 0x49, 0x49, 0x49, 0x41},	 /* 0x45 E */
					{0x7f, 0x09, 0x09, 0x09, 0x01},	 /* 0x46 F */
					{0x3e, 0x41, 0x49, 0x49, 0x7a},	 /* 0x47 G */
					{0x7f, 0x08, 0x08, 0x08, 0x7f},	 /* 0x48 H */
					{0x00, 0x41, 0x7f, 0x41, 0x00},	 /* 0x49 I */
					{0x20, 0x40, 0x41, 0x3f, 0x01},	 /* 0x4a J */
					{0x7f, 0x08, 0x14, 0x22, 0x41},	 /* 0x4b K */
					{0x7f, 0x40, 0x40, 0x40, 0x40},	 /* 0x4c L */
					{0x7f, 0x02, 0x0c, 0x02, 0x7f},	 /* 0x4d M */
					{0x7f, 0x04, 0x08, 0x10, 0x7f},	 /* 0x4e N */
					{0x3e, 0x41, 0x41, 0x41, 0x3e},	 /* 0x4f O */
					{0x7f, 0x09, 0x09, 0x09, 0x06},	 /* 0x50 P */
					{0x3e, 0x41, 0x51, 0x21, 0x5e},	 /* 0x51 Q */
					{0x7f, 0x09, 0x19, 0x29, 0x46},	 /* 0x52 R */
					{0x26, 0x49, 0x49, 0x49, 0x32},	 /* 0x53 S */
					{0x01, 0x01, 0x7f, 0x01, 0x01},	 /* 0x54 T */
					{0x3f, 0x40, 0x40, 0x40, 0x3f},	 /* 0x55 U */
					{0x1f, 0x20, 0x40, 0x20, 0x1f},	 /* 0x56 V */
					{0x3f, 0x40, 0x38, 0x40, 0x3f},	 /* 0x57 W */
					{0x63, 0x14, 0x08, 0x14, 0x63},	 /* 0x58 X */
					{0x07, 0x08, 0x70, 0x08, 0x07},	 /* 0x59 Y */
					{0x61, 0x51, 0x49, 0x45, 0x43},	 /* 0x5a Z */
					{0x00, 0x7f, 0x41, 0x41, 0x00},	 /* 0x5b [ */
					{0x02, 0x04, 0x08, 0x10, 0x20},	 /* 0x5c \ */
					{0x00, 0x41, 0x41, 0x7f, 0x00},	 /* 0x5d ] */
					{0x04, 0x02, 0x01, 0x02, 0x04},	 /* 0x5e ^ */
					{0x40, 0x40, 0x40, 0x40, 0x40},	 /* 0x5f _ */
					{0x00, 0x01, 0x02, 0x04, 0x00},	 /* 0x60 ` */
					{0x20, 0x54, 0x54, 0x54, 0x78},	 /* 0x61 a */
					{0x7f, 0x48, 0x44, 0x44, 0x38},	 /* 0x62 b */
					{0x38, 0x44, 0x44, 0x44, 0x20},	 /* 0x63 c */
					{0x38, 0x44, 0x44, 0x48, 0x7f},	 /* 0x64 d */
					{0x38, 0x54, 0x54, 0x54, 0x18},	 /* 0x65 e */
					{0x08, 0x7e, 0x09, 0x01, 0x02},	 /* 0x66 f */
					{0x0c, 0x52, 0x52, 0x52, 0x3e},	 /* 0x67 g */
					{0x7f, 0x08, 0x04, 0x04, 0x78},	 /* 0x68 h */
					{0x00, 0x04, 0x7d, 0x00, 0x00},	 /* 0x69 i */
					{0x20, 0x40, 0x44, 0x3d, 0x00},	 /* 0x6a j */
					{0x7f, 0x10, 0x28, 0x44, 0x00},	 /* 0x6b k */
					{0x00, 0x41, 0x7f, 0x40, 0x00},	 /* 0x6c l */
					{0x7c, 0x04, 0x18, 0x04, 0x7c},	 /* 0x6d m */
					{0x7c, 0x08, 0x04, 0x04, 0x78},	 /* 0x6e n */
					{0x38, 0x44, 0x44, 0x44, 0x38},	 /* 0x6f o */
					{0x7c, 0x14, 0x14, 0x14, 0x08},	 /* 0x70 p */
					{0x08, 0x14, 0x14, 0x18, 0x7c},	 /* 0x71 q */
					{0x7c, 0x08, 0x04, 0x04, 0x08},	 /* 0x72 r */
					{0x48, 0x54, 0x54, 0x54, 0x20},	 /* 0x73 s */
					{0x04, 0x3f, 0x44, 0x40, 0x20},	 /* 0x74 t */
					{0x3c, 0x40, 0x40, 0x20, 0x7c},	 /* 0x75 u */
					{0x1c, 0x20, 0x40, 0x20, 0x1c},	 /* 0x76 v */
					{0x3c, 0x40, 0x30, 0x40, 0x3c},	 /* 0x77 w */
					{0x44, 0x28, 0x10, 0x28, 0x44},	 /* 0x78 x */
					{0x0c, 0x50, 0x50, 0x50, 0x3c},	 /* 0x79 y */
					{0x44, 0x64, 0x54, 0x4c, 0x44},	 /* 0x7a z */
					{0x00, 0x08, 0x36, 0x41, 0x00},	 /* 0x7b { */
					{0x00, 0x00, 0x77, 0x00, 0x00},	 /* 0x7c | */
					{0x00, 0x41, 0x36, 0x08, 0x00},	 /* 0x7d } */
					{0x08, 0x04, 0x08, 0x10, 0x08}}; /* 0x7e ~ */

/*
 * =============================================================================
 * HARDWARE INTERFACE DOCUMENTATION
 * =============================================================================
 *
 * KS0108 GRAPHICS LCD CONTROLLER PIN CONFIGURATION:
 *
 * Control Pins (connected to ATmega128 PORTE):
 * - RS  (PE4): Register Select [L=Command, H=Data]        → PIN 14
 * - RW  (GND): Read/Write [L=Write fixed]                 → GND
 * - E   (PE5): Enable [L=Off, H=On]                       → PIN 1
 * - CS1 (PE7): Chip Select 1 (Left controller) [L=Off, H=On]  → PIN 16
 * - CS2 (PE6): Chip Select 2 (Right controller) [L=Off, H=On] → PIN 17
 *
 * Data Bus (connected to ATmega128 PORTA):
 * - D0-D7: 8-bit parallel data bus                        → PORTA
 *
 * TIMING REQUIREMENTS:
 * - Enable pulse width: minimum 450ns
 * - Setup time: minimum 140ns
 * - Hold time: minimum 10ns
 * - Command execution: varies by operation
 *
 * DUAL CONTROLLER ARCHITECTURE:
 * - Left controller (CS1): manages columns 0-63
 * - Right controller (CS2): manages columns 64-127
 * - Independent addressing and operation
 * - Synchronized for seamless display
 */

/*
 * EDUCATIONAL FUNCTION: Send Command to Left Controller
 *
 * PURPOSE: Send control commands to left half of display (columns 0-63)
 * PARAMETERS: cmd - KS0108 command byte
 *
 * HARDWARE INTERFACE:
 * - PORTA: 8-bit data bus (command byte)
 * - PORTE.4 (RS): Register Select (0=Command, 1=Data)
 * - PORTE.5 (E): Enable signal (falling edge triggers operation)
 * - PORTE.6 (CS2): Chip Select 2 (right controller)
 * - PORTE.7 (CS1): Chip Select 1 (left controller)
 */
void cmndl(unsigned char cmd) // left 128x64
{
	PORTA = cmd;		   // Place command on data bus
	ClrBit(PORTE, PORTE4); // RS = 0 (Command mode)
	ClrBit(PORTE, PORTE6); // CS2 = 0 (Disable right controller)
	SetBit(PORTE, PORTE7); // CS1 = 1 (Enable left controller)

	_delay_us(D_MIDDLE);   // Setup time
	SetBit(PORTE, PORTE5); // E = 1 (Enable high)
	_delay_us(D_BEFORE);   // Enable pulse width
	ClrBit(PORTE, PORTE5); // E = 0 (Enable low - execute command)
	_delay_us(D_AFTER);	   // Hold time

	/*
	 * EDUCATIONAL NOTE:
	 * The KS0108 requires specific timing:
	 * - Enable pulse width: minimum 450ns
	 * - Setup time: minimum 140ns
	 * - Hold time: minimum 10ns
	 * - Command execution time: varies by command
	 */
} /*
   * EDUCATIONAL FUNCTION: Send Command to Right Controller
   *
   * PURPOSE: Send control commands to right half of display (columns 64-127)
   * PARAMETERS: cmd - KS0108 command byte
   *
   * DUAL CONTROLLER ARCHITECTURE:
   * The 128x64 GLCD uses two KS0108 controllers:
   * - Left controller: manages columns 0-63
   * - Right controller: manages columns 64-127
   * - Each controller has independent addressing
   * - Enables parallel processing for better performance
   */
void cmndr(unsigned char cmd)
{
	PORTA = cmd;		   // Place command on data bus
	ClrBit(PORTE, PORTE4); // RS = 0 (Command mode)
	SetBit(PORTE, PORTE6); // CS2 = 1 (Enable right controller)
	ClrBit(PORTE, PORTE7); // CS1 = 0 (Disable left controller)

	_delay_us(D_MIDDLE);   // Setup time
	SetBit(PORTE, PORTE5); // E = 1 (Enable high)
	_delay_us(D_BEFORE);   // Enable pulse width
	ClrBit(PORTE, PORTE5); // E = 0 (Enable low - execute command)
	_delay_us(D_AFTER);	   // Hold time
}

/*
 * EDUCATIONAL FUNCTION: Send Command to Both Controllers
 *
 * PURPOSE: Send same command to both display halves simultaneously
 * PARAMETERS: cmd - KS0108 command byte
 *
 * BROADCAST OPERATION:
 * This function demonstrates broadcast communication:
 * - Single command affects entire display
 * - Reduces communication overhead
 * - Ensures synchronization between controllers
 * - Used for global operations (clear, display on/off)
 */
void cmnda(unsigned char cmd)
{
	PORTA = cmd;		   // Place command on data bus
	ClrBit(PORTE, PORTE4); // RS = 0 (Command mode)
	SetBit(PORTE, PORTE6); // CS2 = 1 (Enable right controller)
	SetBit(PORTE, PORTE7); // CS1 = 1 (Enable left controller)

	_delay_us(D_MIDDLE);   // Setup time
	SetBit(PORTE, PORTE5); // E = 1 (Enable high)
	_delay_us(D_BEFORE);   // Enable pulse width
	ClrBit(PORTE, PORTE5); // E = 0 (Enable low - execute command)
	_delay_us(D_AFTER);	   // Hold time

	/*
	 * EDUCATIONAL NOTE:
	 * Broadcast operations are useful for:
	 * - Display initialization
	 * - Global clear operations
	 * - Display on/off control
	 * - Start line synchronization
	 */
}

/* 1 character output  */
void datal(byte dat) // left 128x64
{
	PORTA = dat;		   // DATA pin PORTA
	SetBit(PORTE, PORTE4); // PORTE.4 = 1;  // RS
	ClrBit(PORTE, PORTE6);
	SetBit(PORTE, PORTE7); // PORTE.6 = 0;  PORTE.7 = 1;
	_delay_us(D_MIDDLE);
	SetBit(PORTE, PORTE5); // PORTE.5 = 1;		//  E
	_delay_us(D_BEFORE);
	ClrBit(PORTE, PORTE5); // PORTE.5 = 0;		// E
	_delay_us(D_AFTER);
}

void datar(byte dat) // right 128x64
{
	PORTA = dat;		   // DATA pin PORTA
	SetBit(PORTE, PORTE4); // PORTE.4 = 1;  // RS
	SetBit(PORTE, PORTE6);
	ClrBit(PORTE, PORTE7); // PORTE.6 = 1;  PORTE.7 = 0;
	_delay_us(D_MIDDLE);
	SetBit(PORTE, PORTE5); // PORTE.5 = 1;		//  E
	_delay_us(D_BEFORE);
	ClrBit(PORTE, PORTE5); // PORTE.5 = 0;		// E
	_delay_us(D_AFTER);
}

void dataa(byte dat) // both 128x64
{
	PORTA = dat;		   // DATA pin PORTA
	SetBit(PORTE, PORTE4); // PORTE.4 = 1;  // RS
	SetBit(PORTE, PORTE6);
	SetBit(PORTE, PORTE7); // PORTE.6 = 1;  PORTE.7 = 1;
	_delay_us(D_MIDDLE);
	SetBit(PORTE, PORTE5); // PORTE.5 = 1;		//  E
	_delay_us(D_BEFORE);
	ClrBit(PORTE, PORTE5); // PORTE.5 = 0;		// E
	_delay_us(D_AFTER);
}

/* GLCD Clear */
void lcd_clear(void) /* clear LCD screen */
{
	byte i, j, x, y;
	x = 0xB8; /* X start address Page 0*/
	y = 0x40; /* Y start address Column 0*/
	for (i = 0; i <= 7; i++)
	{
		cmnda(x);
		cmnda(y);
		for (j = 0; j <= 63; j++)
			dataa(0x00); /* clear CS1 and CS2 */
		x++;
	}
}

/* GLCD Initialize */
void lcd_init(void)
{
	cmnda(DISPON); // Display ON
	cmnda(0xc0);   // z = 0 first line
	cmnda(0xb8);   // x = 0 first page
	cmnda(0x40);   // y = 0 first column
}

/* character position */
void lcd_xy(byte x, byte y)
{
	xchar = x;						 /* x = 0~7 */
	ychar = y;						 /* y = 0~9 */
	cmnda(0xB8 + xchar);			 /* X address */
	if (ychar <= 9)					 /* if y <= 9, CS1 Y address 8x10 characters for a pannel 4 offset */
		cmndl(0x40 + ychar * 6 + 4); /* 5x7 -> 6x8 actually bottom for cursor right for empty vertical */
	else							 /* if y >= 10, CS2 Y address */
		cmndr(0x40 + (ychar - 10) * 6);
}

/* character output */
void lcd_char(byte character)
{
	byte i;
	for (i = 0; i <= 4; i++)
	{
		if (ychar <= 9) /* if y <= 9, CS1 */
			datal(font[character - 0x20][i]);
		else /* if y >= 10, CS2 */
			datar(font[character - 0x20][i]);
	}
	if (ychar <= 9)
		datal(0x00); /* last byte 0x00 making 6x8 pixel per a character */
	else
		datar(0x00); /* last byte 0x00 making 6x8 pixel per a character*/
}

/* character sequence output */
void lcd_string(byte x, byte y, char *string)
{
	xchar = x;
	ychar = y;
	lcd_xy(x, y);
	while (*string != '\0') /* null */
	{
		if (ychar == 10) /* change from CS1 to CS2 */
			cmndr(0x40);
		lcd_char(*string); /* display a character */
		string++;		   /* next character */
		ychar++;		   /* next line */
	}
}

void GLCD_Axis_xy(unsigned char x, unsigned char y) // draw a byte downwards
{
	cmnda(0xB8 + x); // X address page
	if (y <= 63)
	{
		cmndl(0x40 + y); // CS1 Y address column
	}
	else
	{
		cmndr(0x40 + y - 64); // CS2 Y address
	}
}

unsigned char ScreenBuffer[8][128]; // screen buffer
// draw a dot on GLCD
void GLCD_Dot(unsigned char xx, unsigned char y)
{
	unsigned char x, i;

	// check resolution (128.64)
	if ((xx > 63) || (y > 127))
		return;
	x = xx / 8; // calculate x address
	i = xx % 8; // pixel number in th x address downwards
	if (i == 0)
	{
		i = 0x01;
	} // top pixel
	else if (i == 1)
	{
		i = 0x02;
	}
	else if (i == 2)
	{
		i = 0x04;
	}
	else if (i == 3)
	{
		i = 0x08;
	}
	else if (i == 4)
	{
		i = 0x10;
	}
	else if (i == 5)
	{
		i = 0x20;
	}
	else if (i == 6)
	{
		i = 0x40;
	}
	else
	{
		i = 0x80;
	} // bottom pixel

	ScreenBuffer[x][y] |= i; // OR old data with new data
	GLCD_Axis_xy(x, y);		 // draw dot on GLCD screen
	if (y <= 63)
	{
		datal(ScreenBuffer[x][y]);
	}
	else
	{
		datar(ScreenBuffer[x][y]);
	}
}

void ScreenBuffer_clear(void)
{
	unsigned char i, j;

	for (i = 0; i < 8; i++)
	{
		for (j = 0; j < 128; j++)
		{
			ScreenBuffer[i][j] = 0x00;
		}
	}
}

void GLCD_Line(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2)
{
	int x, y;
	if (y1 != y2) // if y1 != y2, y is variable
	{
		if (y1 < y2) // x is function
		{
			for (y = y1; y <= y2; y++)
			{
				x = x1 + (y - y1) * (x2 - x1) / (y2 - y1);
				GLCD_Dot(x, y);
			}
		}
		else
		{
			for (y = y1; y >= y2; y--)
			{
				x = x1 + (y - y1) * (x2 - x1) / (y2 - y1);
				GLCD_Dot(x, y);
			}
		}
	}
	else if (x1 != x2) // if x1 != x2, x is variable
	{
		if (x1 < x2) // y is function
		{
			for (x = x1; x <= x2; x++)
			{
				y = y1 + (x - x1) * (y2 - y1) / (x2 - x1);
				GLCD_Dot(x, y);
			}
		}
		else
		{
			for (x = x1; x >= x2; x--)
			{
				y = y1 + (x - x1) * (y2 - y1) / (x2 - x1);
				GLCD_Dot(x, y);
			}
		}
	}
	else // if x1 == x2 and y1 == y2,
	{
		GLCD_Dot(x1, y1); // it is a dot
	}
}

// draw a rectangle
void GLCD_Rectangle(unsigned char x1, unsigned char y1, unsigned char x2, unsigned char y2)
{
	GLCD_Line(x1, y1, x1, y2); // horizontal line
	GLCD_Line(x2, y1, x2, y2);
	GLCD_Line(x1, y1, x2, y1); // vertical line
	GLCD_Line(x1, y2, x2, y2);
}

// draw a circle
void GLCD_Circle(unsigned char x1, unsigned char y1, unsigned char r)
{
	int x, y;
	float s;
	for (y = y1 - r * 3 / 4; y <= y1 + r * 3 / 4; y++) // draw with y variable
	{
		s = sqrt(r * r - (y - y1) * (y - y1)) + 0.5;
		x = x1 + (unsigned char)s;
		GLCD_Dot(x, y);
		x = x1 - (unsigned char)s;
		GLCD_Dot(x, y);
	}
	for (x = x1 - r * 3 / 4; x <= x1 + r * 3 / 4; x++) // draw with x variable
	{
		s = sqrt(r * r - (x - x1) * (x - x1)) + 0.5;
		y = y1 + (unsigned char)s;
		GLCD_Dot(x, y);
		y = y1 - (unsigned char)s;
		GLCD_Dot(x, y);
	}
}

// display 1-digit decimal number
unsigned char GLCD_1DigitDecimal(unsigned char number, unsigned char flag)
{
	number %= 10; // 10^0

	if ((number == 0) && (flag == 0))
	{
		lcd_char(' ');
		return 0;
	}

	lcd_char(number + '0');
	return 1;
}

// display 2-digit decimal number
void GLCD_2DigitDecimal(unsigned char number)
{
	unsigned int i;
	unsigned char flag;

	flag = 0;
	number = number % 100;
	i = number / 10;
	flag = GLCD_1DigitDecimal(i, flag); // 10^1
	i = number % 10;
	lcd_char(i + '0'); // 10^0
}

// display 3-digit decimal number
void GLCD_3DigitDecimal(unsigned int number)
{
	unsigned int i;
	unsigned char flag;
	flag = 0;
	number = number % 1000;
	i = number / 100;
	flag = GLCD_1DigitDecimal(i, flag); // 10^2
	number = number % 100;
	i = number / 10;
	flag = GLCD_1DigitDecimal(i, flag); // 10^1
	i = number % 10;
	lcd_char(i + '0'); // 10^0
}

// display 4-digit decimal number
void GLCD_4DigitDecimal(unsigned int number)
{
	unsigned int i;
	unsigned char flag;
	flag = 0;
	number = number % 10000;
	i = number / 1000;
	flag = GLCD_1DigitDecimal(i, flag); // 10^3
	number = number % 1000;
	i = number / 100;
	flag = GLCD_1DigitDecimal(i, flag); // 10^2
	number = number % 100;
	i = number / 10;
	flag = GLCD_1DigitDecimal(i, flag); // 10^1
	i = number % 10;
	lcd_char(i + '0'); // 10^0
}

/*-------------------------------------------------------------------------*/

#endif // !ASSEMBLY_BLINK_BASIC
