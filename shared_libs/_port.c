/*
 * _port.c - ATmega128 Educational Port Control Library
 * Part of Assembly → C → Python Learning Progression
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Understand DDR (Data Direction Register) concepts
 * 2. Learn PORT register manipulation for outputs
 * 3. Master PIN register reading for inputs
 * 4. Practice bit manipulation techniques
 *
 * ATmega128 PORT OVERVIEW:
 * - PORTA: General purpose I/O and ADC inputs
 * - PORTB: General purpose I/O and SPI
 * - PORTC: General purpose I/O (limited pins)
 * - PORTD: General purpose I/O and UART
 * - PORTE: General purpose I/O and external interrupts
 * - PORTF: ADC inputs and JTAG
 * - PORTG: General purpose I/O and timers
 *
 * REGISTER EXPLANATION:
 * - DDRx: Data Direction Register (0=input, 1=output)
 * - PORTx: Output data register / pull-up enable for inputs
 * - PINx: Input data register (read-only)
 */

#include <avr/io.h>
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <util/delay.h>

/*
 * EDUCATIONAL FUNCTION: Complete Port Initialization
 *
 * PURPOSE: Configure all ATmega128 ports for educational board layout
 * LEARNING: Shows systematic approach to port configuration
 *
 * EDUCATIONAL BOARD CONFIGURATION:
 * - PORTA: General data output (8 LEDs or similar)
 * - PORTB: LED array (8 LEDs, active LOW)
 * - PORTC: Mixed I/O (upper 4 bits input, lower 4 bits output)
 * - PORTD: Button inputs with pull-ups
 * - PORTE: LCD control signals
 * - PORTF: ADC sensor inputs
 * - PORTG: Buzzer and additional outputs
 */
void Port_init(void)
{
	/*
	 * PORTA CONFIGURATION: General Data Output
	 * Educational use: 8-bit data display, pattern generation
	 * DDR=0xFF (all outputs), PORT=0x00 (start with all LOW)
	 */
	DDRA = 0xFF;  // Configure PORTA as output
	PORTA = 0x00; // Initialize all pins LOW

	/*
	 * PORTB CONFIGURATION: LED Array (Active LOW)
	 * Educational use: Visual feedback, binary counting, patterns
	 * DDR=0xFF (all outputs), PORT=0xFF (LEDs OFF - active LOW)
	 *
	 * LEARNING NOTE: Many educational boards use active LOW LEDs
	 * - Writing 0 turns LED ON (current flows through LED)
	 * - Writing 1 turns LED OFF (no current flow)
	 */
	DDRB = 0xFF;  // Configure PORTB as output
	PORTB = 0xFF; // Initialize all LEDs OFF (active LOW)

	/*
	 * PORTC CONFIGURATION: Mixed I/O
	 * Educational use: Partial input/output demonstration
	 * Upper 4 bits (PC7-PC4): Inputs with pull-ups
	 * Lower 4 bits (PC3-PC0): Outputs
	 */
	DDRC = 0x0F;  // PC7-PC4 inputs, PC3-PC0 outputs
	PORTC = 0xF0; // Enable pull-ups on inputs, outputs start LOW

	/*
	 * PORTD CONFIGURATION: Button Inputs
	 * Educational use: User input, interrupt sources
	 * DDR=0x00 (all inputs), PORT=0xFF (enable pull-ups)
	 *
	 * LEARNING NOTE: Pull-up resistors prevent floating inputs
	 * - Internal pull-ups provide stable HIGH when button not pressed
	 * - Button press pulls pin LOW (active LOW buttons)
	 */
	DDRD = 0x00;  // Configure PORTD as input
	PORTD = 0xFF; // Enable internal pull-up resistors

	/*
	 * PORTE CONFIGURATION: LCD Control and Special Functions
	 * Educational use: LCD interface, external interrupts
	 * PE0, PE1: External interrupt inputs
	 * PE2, PE3: General I/O
	 * PE4-PE7: LCD control signals
	 */
	DDRE = 0xF0;  // PE7-PE4 outputs (LCD), PE3-PE0 inputs
	PORTE = 0x0F; // Enable pull-ups on inputs, LCD signals start LOW

	/*
	 * PORTF CONFIGURATION: ADC Sensor Inputs
	 * Educational use: Analog sensor reading
	 * DDR=0x00 (all inputs), PORT=0x00 (no pull-ups for analog)
	 *
	 * LEARNING NOTE: ADC inputs should not have pull-ups enabled
	 * - Pull-ups interfere with accurate analog measurements
	 * - External sensors provide their own signal levels
	 */
	DDRF = 0x00;  // Configure PORTF as input (ADC)
	PORTF = 0x00; // No pull-ups for analog inputs

	/*
	 * PORTG CONFIGURATION: Audio and Additional Outputs
	 * Educational use: Buzzer control, PWM outputs
	 * PG4: Buzzer output
	 * Others: General purpose outputs
	 */
	DDRG = 0xFF;  // Configure PORTG as output
	PORTG = 0x00; // Initialize all outputs LOW
}

/*
 * EDUCATIONAL FUNCTION: LED Control Functions
 *
 * PURPOSE: Provide easy-to-use LED control for learning
 * LEARNING: Shows function abstraction over direct register access
 */

/* Turn on specific LED (0-7) */
void led_on(unsigned char led_number)
{
	if (led_number < 8)
	{
		PORTB &= ~(1 << led_number); // Clear bit (LED ON - active LOW)
	}
}

/* Turn off specific LED (0-7) */
void led_off(unsigned char led_number)
{
	if (led_number < 8)
	{
		PORTB |= (1 << led_number); // Set bit (LED OFF - active LOW)
	}
}

/* Toggle specific LED (0-7) */
void led_toggle(unsigned char led_number)
{
	if (led_number < 8)
	{
		PORTB ^= (1 << led_number); // XOR bit (toggle state)
	}
}

/* Set all LEDs to specific pattern */
void led_pattern(unsigned char pattern)
{
	PORTB = ~pattern; // Invert because LEDs are active LOW
}

/* Turn all LEDs off */
void led_all_off(void)
{
	PORTB = 0xFF; // All bits HIGH = all LEDs OFF (active LOW)
}

/* Turn all LEDs on */
void led_all_on(void)
{
	PORTB = 0x00; // All bits LOW = all LEDs ON (active LOW)
}

/*
 * EDUCATIONAL FUNCTION: Button Reading Functions
 *
 * PURPOSE: Provide easy-to-use button input for learning
 * LEARNING: Shows input debouncing and state management
 */

/* Read specific button state (0-7) */
unsigned char button_pressed(unsigned char button_number)
{
	if (button_number < 8)
	{
		/* Return 1 if button pressed (pin LOW due to active LOW buttons) */
		return !(PIND & (1 << button_number));
	}
	return 0;
}

/* Read all button states as 8-bit value */
unsigned char read_buttons(void)
{
	return ~PIND; // Invert because buttons are active LOW
}

/* Wait for any button press (simple debouncing) */
void wait_for_button_press(void)
{
	/* Wait for button release first */
	while (PIND != 0xFF)
	{
		_delay_ms(10);
	}

	/* Wait for button press */
	while (PIND == 0xFF)
	{
		_delay_ms(10);
	}

	/* Simple debounce delay */
	_delay_ms(50);
}

/*
 * EDUCATIONAL FUNCTION: Port Pattern Demonstrations
 *
 * PURPOSE: Show various output patterns for learning
 * LEARNING: Demonstrates algorithmic pattern generation
 */

/* Binary counting pattern on LEDs */
void led_binary_count(unsigned char max_count)
{
	for (unsigned char i = 0; i < max_count; i++)
	{
		led_pattern(i);
		_delay_ms(500);
	}
}

/* Running light pattern */
void led_running_light(unsigned char cycles)
{
	for (unsigned char cycle = 0; cycle < cycles; cycle++)
	{
		for (unsigned char i = 0; i < 8; i++)
		{
			led_all_off();
			led_on(i);
			_delay_ms(200);
		}
	}
}

/* Knight Rider pattern (bouncing light) */
void led_knight_rider(unsigned char cycles)
{
	for (unsigned char cycle = 0; cycle < cycles; cycle++)
	{
		/* Left to right */
		for (unsigned char i = 0; i < 8; i++)
		{
			led_all_off();
			led_on(i);
			_delay_ms(150);
		}

		/* Right to left */
		for (unsigned char i = 7; i > 0; i--)
		{
			led_all_off();
			led_on(i - 1);
			_delay_ms(150);
		}
	}
}

/*
 * EDUCATIONAL FUNCTION: Port Diagnostic Functions
 *
 * PURPOSE: Help students verify port configuration
 * LEARNING: Shows how to read and verify register states
 */

/* Test all LEDs individually */
void test_all_leds(void)
{
	/* Test each LED individually */
	for (unsigned char i = 0; i < 8; i++)
	{
		led_all_off();
		led_on(i);
		_delay_ms(300);
	}
	led_all_off();
}

/* Test all buttons and show result on LEDs */
void test_all_buttons(void)
{
	while (1)
	{
		unsigned char button_state = read_buttons();
		led_pattern(button_state); // Show button state on LEDs

		/* Exit test when all buttons pressed */
		if (button_state == 0xFF)
		{
			break;
		}

		_delay_ms(50);
	}
	led_all_off();
}

/*
 * =============================================================================
 * ENHANCED PORT FUNCTIONS - Professional Quality Additions
 * =============================================================================
 */

/* Debounce configuration */
static unsigned char debounce_delay_ms = 20; // Default 20ms debounce

/*
 * Variable delay function for debouncing
 * Uses busy-wait loop since _delay_ms() requires compile-time constant
 */
static void delay_ms_variable(unsigned char ms)
{
	while (ms--)
	{
		_delay_ms(1); // 1ms is compile-time constant
	}
}

/*
 * BUTTON DEBOUNCING ALGORITHM
 *
 * PURPOSE: Filter mechanical switch bounce
 * LEARNING: Shows software debouncing techniques
 *
 * ALGORITHM:
 * 1. Read button state
 * 2. Wait debounce delay
 * 3. Read again
 * 4. If both readings match, button state is stable
 * 5. Otherwise, repeat
 *
 * DEBOUNCE TIMING:
 * - Typical switch bounce: 5-20ms
 * - Safe debounce delay: 20-50ms
 * - Tradeoff: longer delay = more reliable, but slower response
 */
unsigned char button_pressed_debounced(unsigned char button_number)
{
	if (button_number > 7)
		return 0;

	unsigned char pin_mask = (1 << button_number);

	/* First reading */
	unsigned char state1 = (PIND & pin_mask) ? 0 : 1; // Active LOW

	/* Wait debounce delay */
	delay_ms_variable(debounce_delay_ms);

	/* Second reading */
	unsigned char state2 = (PIND & pin_mask) ? 0 : 1;

	/* Return stable state if both readings match */
	if (state1 == state2)
	{
		return state2;
	}

	/* Unstable - return not pressed */
	return 0;
}

/*
 * WAIT FOR BUTTON RELEASE (DEBOUNCED)
 *
 * PURPOSE: Prevent multiple detections of single press
 * LEARNING: Shows event-driven programming patterns
 */
void button_wait_release(unsigned char button_number)
{
	if (button_number > 7)
		return;

	unsigned char pin_mask = (1 << button_number);

	/* Wait while button is pressed */
	while (!(PIND & pin_mask))
	{
		_delay_ms(10);
	}

	/* Debounce the release */
	delay_ms_variable(debounce_delay_ms);

	/* Verify still released */
	while (!(PIND & pin_mask))
	{
		_delay_ms(10);
	}
}

/*
 * INTERNAL PULL-UP MANAGEMENT
 *
 * PURPOSE: Enable/disable internal pull-up resistors
 * LEARNING: Shows pull-up resistor concepts
 *
 * PULL-UP OPERATION:
 * - When pin is input (DDRx=0), setting PORTx=1 enables pull-up
 * - Pull-up pulls input to VCC (5V) through ~20-50kΩ resistor
 * - Essential for button inputs to prevent floating
 *
 * BUTTON CONNECTION:
 * - Button between pin and GND
 * - Pull-up enabled: pin reads HIGH when not pressed
 * - When pressed: pin reads LOW (active LOW)
 */
void port_enable_pullup(unsigned char pin_number)
{
	if (pin_number > 7)
		return;

	/* Enable pull-up on PORTD (typical for buttons) */
	PORTD |= (1 << pin_number);
}

void port_disable_pullup(unsigned char pin_number)
{
	if (pin_number > 7)
		return;

	PORTD &= ~(1 << pin_number);
}

/*
 * PORT MASKING OPERATIONS
 *
 * PURPOSE: Read/write specific bits without affecting others
 * LEARNING: Shows advanced bit manipulation
 *
 * MASKING TECHNIQUE:
 * - AND operation selects specific bits
 * - OR operation sets specific bits
 * - XOR operation toggles specific bits
 * - AND with NOT clears specific bits
 */
unsigned char port_read_masked(volatile unsigned char port_register, unsigned char mask)
{
	return (port_register & mask);
}

void port_write_masked(volatile unsigned char *port_register, unsigned char mask, unsigned char value)
{
	unsigned char temp;

	/* Read current value */
	temp = *port_register;

	/* Clear masked bits */
	temp &= ~mask;

	/* Set new value in masked bits */
	temp |= (value & mask);

	/* Write back */
	*port_register = temp;
}

/*
 * DEBOUNCE CONFIGURATION
 */
void port_set_debounce_delay(unsigned char delay_ms)
{
	if (delay_ms < 5)
		delay_ms = 5; // Minimum 5ms
	if (delay_ms > 100)
		delay_ms = 100; // Maximum 100ms
	debounce_delay_ms = delay_ms;
}

unsigned char port_get_debounce_delay(void)
{
	return debounce_delay_ms;
}

/*
 * ENHANCED BUTTON READING
 *
 * PURPOSE: Read all buttons with debouncing
 * LEARNING: Shows multi-channel input handling
 */
unsigned char read_buttons_debounced(void)
{
	unsigned char result = 0;

	/* Debounce each button */
	for (unsigned char i = 0; i < 8; i++)
	{
		if (button_pressed_debounced(i))
		{
			result |= (1 << i);
		}
	}

	return result;
}

/*
 * WAIT FOR ANY BUTTON (DEBOUNCED)
 *
 * PURPOSE: Wait for any button press and return which one
 * LEARNING: Shows event detection
 */
unsigned char wait_for_any_button_debounced(void)
{
	while (1)
	{
		for (unsigned char i = 0; i < 8; i++)
		{
			if (button_pressed_debounced(i))
			{
				/* Wait for release */
				button_wait_release(i);
				return i;
			}
		}
		_delay_ms(10);
	}
}
