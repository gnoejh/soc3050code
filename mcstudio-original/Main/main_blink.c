/*
 * blink.c
 *
 * Created: 2024-09-22
 * Author : hjeong
 */ 
#include "config.h"
#include <avr/io.h>
//#define F_CPU 16000000UL
#include "util/delay.h"

#ifdef BLINK_PORT

void main_blink_port(void)
{
	DDRB = 0xFF; //make port B as output port
	while(1)
	{
		PORTB = 0xAA; //make all pins HIGH
		_delay_ms(1000); //wait 1 sec
		PORTB = 0x55; //make all pins LOW
		_delay_ms(1000); //wait 1 sec
	}
}
#endif

#ifdef BLINK_PIN

void main_blink_pin(void)
{
    // Set PORTB as output for LEDs
    DDRB = 0xFF; // 0xFF means all bits of PORTB are set as output (all LEDs)
    // Set PD0 as input for the button and enable the internal pull-up resistor
    DDRD &= ~(1 << PD7); // Clear bit 0 of DDRD to make PD0 an input
    PORTD |= (1 << PD7); // Set bit 0 of PORTD to enable the pull-up resistor on PD0, Duplicated (Circuit PU)

    uint8_t direction = 0; // 0 for clockwise, 1 for counterclockwise
    uint8_t led_state = 0x01; // Start with the first LED on (binary: 00000001)
    uint8_t last_button_state = 1; // Track the last state of the button (1 = not pressed, due to pull-up)

    while (1)
    {
        // Read the current state of the button
        uint8_t current_button_state = PIND & (1 << PD7);

        // Check for a falling edge: button is pressed (current state = 0) and was not pressed before (last state = 1)
        if (current_button_state == 0 && last_button_state != 0)
        {
            // Toggle the direction whenever the button is pressed
            direction = !direction;
        }

        // Store the current button state as the last state for the next loop iteration
        last_button_state = current_button_state;

        // Update LED states based on the direction (active LOW)
        if (direction == 0)
        {
            PORTB = ~led_state; // Output the inverted LED state to PORTB (turn on a specific LED)
            led_state <<= 1;   // Shift left to move to the next LED (clockwise)
            if (led_state == 0x00)
            {
                led_state = 0x01; // Loop back to the first LED if we reach beyond the last one
            }
        }
        else
        {
            PORTB = ~led_state; // Output the inverted LED state to PORTB (turn on a specific LED)
            led_state >>= 1;   // Shift right to move to the previous LED (counterclockwise)
            if (led_state == 0x00)
            {
                led_state = 0x80; // Loop back to the last LED if we reach beyond the first one
            }
        }

        // Wait for 1 second before changing to the next LED
        _delay_ms(500);
    }
}
#endif
