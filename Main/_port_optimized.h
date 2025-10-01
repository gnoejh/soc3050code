/*
 * _port.h - ATmega128 Port Configuration Library Header
 * Educational Version for Assembly→C→Python Learning Progression
 */

#ifndef _PORT_H_
#define _PORT_H_

// Core initialization function
void Port_init(void);

// LED Control Functions (Educational Bridge Functions)
void LED_All_On(void);                         // Turn all LEDs ON
void LED_All_Off(void);                        // Turn all LEDs OFF
void LED_Set_Pattern(unsigned char pattern);   // Set LED pattern (0=ON, 1=OFF inverted internally)
void LED_Set_Bit(unsigned char bit_number);    // Turn specific LED ON
void LED_Clear_Bit(unsigned char bit_number);  // Turn specific LED OFF
void LED_Toggle_Bit(unsigned char bit_number); // Toggle specific LED

// Button Reading Functions (Educational Input Functions)
unsigned char Button_Read_All(void);                     // Read all button states
unsigned char Button_Read_Bit(unsigned char bit_number); // Read specific button

/*
 * EDUCATIONAL CONSTANTS for bit manipulation learning
 * These help students understand bit positions and patterns
 */
#define LED_0 0 // LED bit positions
#define LED_1 1
#define LED_2 2
#define LED_3 3
#define LED_4 4
#define LED_5 5
#define LED_6 6
#define LED_7 7

#define BUTTON_0 0 // Button bit positions
#define BUTTON_1 1
#define BUTTON_2 2
#define BUTTON_3 3
#define BUTTON_4 4
#define BUTTON_5 5
#define BUTTON_6 6
#define BUTTON_7 7

// Pattern examples for educational use
#define LED_PATTERN_ALTERNATE_1 0xAA // 10101010
#define LED_PATTERN_ALTERNATE_2 0x55 // 01010101
#define LED_PATTERN_CHASE_LEFT 0x01  // 00000001
#define LED_PATTERN_CHASE_RIGHT 0x80 // 10000000
#define LED_PATTERN_CENTER_OUT 0x18  // 00011000
#define LED_PATTERN_ALL_ON 0xFF      // 11111111
#define LED_PATTERN_ALL_OFF 0x00     // 00000000

#endif // _PORT_H_