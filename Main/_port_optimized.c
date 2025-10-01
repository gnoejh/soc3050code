/*
 * _port.c - ATmega128 Port Configuration Library
 * Educational Version for Assembly→C→Python Learning Progression
 *
 * LEARNING OBJECTIVES:
 * 1. Understand DDR (Data Direction Register) vs PORT register concepts
 * 2. Learn bit manipulation patterns for hardware control
 * 3. Bridge assembly register access to C abstraction
 *
 * ASSEMBLY EQUIVALENT CONCEPTS:
 * - DDRA = 0xFF    ≡  LDI R16, 0xFF; OUT DDRA, R16
 * - PORTA = 0x00   ≡  LDI R16, 0x00; OUT PORTA, R16
 * - Bit setting    ≡  SBI PORTB, 3  (set bit 3)
 * - Bit clearing   ≡  CBI PORTB, 3  (clear bit 3)
 */

#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h>

#include "_main.h"
#include "_port.h"

/*
 * Port_init() - Initialize ATmega128 ports for development board
 *
 * EDUCATIONAL NOTES:
 * - DDR = 0xFF means all pins are OUTPUT (can drive LEDs, motors)
 * - DDR = 0x00 means all pins are INPUT (can read buttons, sensors)
 * - PORT = 0xFF on input pins enables internal pull-up resistors
 * - PORT = 0x00 on output pins sets LOW voltage (0V)
 *
 * HARDWARE MAPPING (Development Board):
 * - PORTA: Data bus for LCD/GLCD (8-bit parallel)
 * - PORTB: LED array (8 individual LEDs)
 * - PORTD: Button inputs (switches with pull-ups)
 * - PORTE: Control signals (Enable, Register Select, Chip Select)
 * - PORTG: Buzzer control (PWM output pin)
 */
void Port_init(void)
{
    // PORTA: 8-bit data bus for graphics LCD
    // Assembly equivalent: LDI R16, 0x00; OUT PORTA, R16; LDI R16, 0xFF; OUT DDRA, R16
    PORTA = 0x00; // Initialize data lines to LOW
    DDRA = 0xFF;  // Set as OUTPUT for driving LCD data bus

    // PORTB: LED array for visual feedback
    // Assembly equivalent: LDI R16, 0xFF; OUT PORTB, R16; OUT DDRB, R16
    PORTB = 0xFF; // Turn OFF all LEDs (active LOW configuration)
    DDRB = 0xFF;  // Set as OUTPUT for driving LEDs

    // PORTD: Button input array
    // Assembly equivalent: LDI R16, 0x00; OUT PORTD, R16; OUT DDRD, R16
    PORTD = 0x00; // Disable internal pull-ups (external pull-ups used)
    DDRD = 0x00;  // Set as INPUT for reading button states

    // PORTE: Control signals for LCD/GLCD interface
    // Assembly equivalent: LDI R16, 0x00; OUT PORTE, R16; LDI R16, 0xFF; OUT DDRE, R16
    PORTE = 0x00; // Initialize control signals to LOW
    DDRE = 0xFF;  // Set as OUTPUT for:
                  // PE4: R/S (Register Select)
                  // PE5: E (Enable)
                  // PE6: CS2 (Chip Select 2)
                  // PE7: CS1 (Chip Select 1)

    // PORTG: Buzzer and additional control
    // Assembly equivalent: LDI R16, 0x00; OUT PORTG, R16; LDI R16, 0xFF; OUT DDRG, R16
    PORTG = 0x00; // Initialize buzzer to OFF
    DDRG = 0xFF;  // Set as OUTPUT for:
                  // PG4: Buzzer control (PWM capable)
}

/*
 * Educational Functions for Assembly→C Learning Progression
 * These functions demonstrate register manipulation patterns
 */

// LED Control Functions - Bridge between assembly and C
void LED_All_On(void)
{
    PORTB = 0x00; // Active LOW: 0 = LED ON
}

void LED_All_Off(void)
{
    PORTB = 0xFF; // Active LOW: 1 = LED OFF
}

void LED_Set_Pattern(unsigned char pattern)
{
    PORTB = ~pattern; // Invert because LEDs are active LOW
}

// Individual LED control with educational bit manipulation
void LED_Set_Bit(unsigned char bit_number)
{
    if (bit_number < 8)
    {
        PORTB &= ~(1 << bit_number); // Clear bit = LED ON (active LOW)
    }
}

void LED_Clear_Bit(unsigned char bit_number)
{
    if (bit_number < 8)
    {
        PORTB |= (1 << bit_number); // Set bit = LED OFF (active LOW)
    }
}

void LED_Toggle_Bit(unsigned char bit_number)
{
    if (bit_number < 8)
    {
        PORTB ^= (1 << bit_number); // XOR bit = Toggle LED
    }
}

// Button Reading Functions
unsigned char Button_Read_All(void)
{
    return PIND; // Read entire port D
}

unsigned char Button_Read_Bit(unsigned char bit_number)
{
    if (bit_number < 8)
    {
        return (PIND & (1 << bit_number)) ? 1 : 0;
    }
    return 0;
}

/*
 * EDUCATIONAL PROGRESSION NOTES:
 *
 * 1. ASSEMBLY LEVEL (Direct Register Access):
 *    Students first learn: LDI R16, 0xFF; OUT DDRB, R16
 *
 * 2. C ABSTRACTION LEVEL (This Library):
 *    Students then learn: LED_All_On();
 *    Understanding it calls: PORTB = 0x00;
 *
 * 3. PYTHON LEVEL (High-Level Interface):
 *    Students finally learn: atmega.led.all_on()
 *    Understanding the full hardware→software→network stack
 */