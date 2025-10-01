#include "config.h"

/*
 * MODERNIZED MEMORY ACCESS DEMONSTRATIONS
 * Educational Framework: ATmega128 Memory Systems and Data Storage
 *
 * Learning Objectives:
 * 1. Master different memory types (Flash, EEPROM) and their applications
 * 2. Understand modernized EEPROM library for safe data storage
 * 3. Learn data persistence and non-volatile storage concepts
 * 4. Explore memory-based pattern generation and data visualization
 *
 * Memory Library Functions Used:
 * - EEPROM_write()/EEPROM_read(): Safe EEPROM operations with error checking
 * - EEPROM_write_array()/EEPROM_read_array(): Bulk data operations
 * - EEPROM_get_status(): Operation status monitoring
 * - Flash memory access: Efficient program space utilization
 *
 * Integration with Other Libraries:
 * - Port library: Button input for memory operations
 * - Timer2 library: Precise timing for memory access patterns
 * - GLCD library: Visual representation of memory data
 *
 * Hardware Connections:
 * - PORTB: LED indicators for memory operation status
 * - PORTD.0: Button for triggering memory operations
 * - LCD: Memory data display and operation status
 */

/*  Flash Memory Access with Educational Data Patterns */
#ifdef MEMORY_PROGRAM
/*
 * DEMONSTRATION 1: Flash Memory Access with Visual Data Patterns
 * Educational Focus: Program memory utilization and data visualization
 *
 * This example demonstrates:
 * - Efficient flash memory data storage and retrieval
 * - Visual representation of stored data patterns
 * - Memory-based LED pattern generation
 * - Educational data progression and display
 *
 * Learning Points:
 * 1. Flash memory provides efficient storage for constant data
 * 2. pgm_read_byte() enables safe program memory access
 * 3. Memory data can drive visual patterns and displays
 * 4. Data organization enhances educational progression
 *
 * Hardware Setup:
 * - LEDs connected to PORTB show memory data patterns
 * - LCD displays educational information and data values
 */

#include <avr/pgmspace.h>

// Educational lookup table stored in flash memory
const unsigned char PROGMEM educational_lookup[] =
	"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ!@#$%^&*()_+-=[]{}|;:,.<>?";

void main_memory_program(void)
{
	init_devices();

	// Initialize Timer2 for precise timing
	Timer2_init();
	Timer2_start();

	// Configure PORTB for visual feedback using modernized library
	Port_init_output(0xFF, 1);

	// Display educational information
	lcd_clear();
	lcd_string(0, 0, "Flash Memory Demo");
	lcd_string(0, 1, "Educational Data");
	lcd_string(0, 2, "Visual Patterns");
	lcd_string(0, 3, "Timer2 Precision");

	unsigned char current_data;
	unsigned int data_index = 0;
	unsigned long last_update = 0;
	const unsigned int LOOKUP_SIZE = sizeof(educational_lookup) - 1;

	while (1)
	{
		unsigned long current_time = Timer2_get_milliseconds();

		// Update display every 800ms for educational pacing
		if (current_time - last_update >= 800)
		{
			last_update = current_time;

			// Read data from flash memory
			current_data = pgm_read_byte(&educational_lookup[data_index]);

			// Generate LED pattern based on data value
			unsigned char led_pattern = ~(current_data - 0x30);
			Port_write(led_pattern, 1);

			// Update display with educational information
			lcd_clear();
			ScreenBuffer_clear();

			lcd_string(0, 0, "Flash Data Display");
			lcd_string(0, 1, "Index: ");
			GLCD_3DigitDecimal(data_index);

			lcd_string(0, 2, "Char: ");
			lcd_char(current_data);
			lcd_string(3, 2, " (0x");
			GLCD_2DigitHex(current_data);
			lcd_string(8, 2, ")");

			lcd_string(0, 3, "LED: 0x");
			GLCD_2DigitHex(led_pattern);

			// Progress indicator
			lcd_string(0, 4, "Progress: ");
			unsigned int progress = (data_index * 100) / LOOKUP_SIZE;
			GLCD_3DigitDecimal(progress);
			lcd_string(12, 4, "%");

			// Timing information
			lcd_string(0, 5, "Time: ");
			GLCD_4DigitDecimal(current_time / 1000);
			lcd_string(8, 5, "s");

			// Advance to next data element
			data_index = (data_index + 1) % LOOKUP_SIZE;
		}

		// Small delay for system responsiveness
		Timer2_delay_ms(50);
	}
}
#endif

/* EEPROM Access with Interactive Control and Data Persistence */
#ifdef MEMORY_EEPROM
/*
 * DEMONSTRATION 2: Advanced EEPROM Operations with Interactive Control
 * Educational Focus: Non-volatile data storage and interactive memory operations
 *
 * This example demonstrates:
 * - Modernized EEPROM library for safe data storage operations
 * - Interactive button control for write/read operations
 * - Data persistence across power cycles
 * - Real-time operation status monitoring
 *
 * Learning Points:
 * 1. EEPROM_write()/EEPROM_read() provide safe data persistence
 * 2. Interactive control demonstrates user-driven memory operations
 * 3. Status monitoring ensures reliable data operations
 * 4. Timer2 debouncing improves user interaction reliability
 *
 * Hardware Setup:
 * - Button connected to PORTD.0 for write/read control
 * - LEDs connected to PORTB for operation status indication
 * - LCD displays EEPROM data and operation status
 */

// Educational data array for EEPROM operations
unsigned char educational_data[] = "SOC3050-ATmega128-EEPROM-Demo-2025";
unsigned char read_buffer[40];			// Buffer for reading data back
unsigned int eeprom_base_address = 100; // Starting EEPROM address

void main_memory_eeprom(void)
{
	init_devices();

	// Initialize Timer2 for precise timing and debouncing
	Timer2_init();
	Timer2_start();

	// Initialize EEPROM library for safe operations
	EEPROM_init();

	// Configure ports using modernized libraries
	Port_init_input(0x01, 4);	 // PORTD.0 as input
	Port_set_pullup(0x01, 4, 1); // Enable pull-up
	Port_init_output(0xFF, 1);	 // PORTB as output for status

	// Display educational information
	lcd_clear();
	ScreenBuffer_clear();
	lcd_string(0, 0, "EEPROM Interactive");
	lcd_string(0, 1, "Button: Write/Read");
	lcd_string(0, 2, "PD0: Operation Ctrl");
	lcd_string(0, 3, "Persistent Storage");

	// Sound notification for system ready
	S_Start();

	unsigned char last_button_state = 1; // Start with button not pressed
	unsigned long last_debounce_time = 0;
	unsigned char operation_mode = 0; // 0=read, 1=write
	unsigned int operation_count = 0;
	const unsigned int DEBOUNCE_DELAY = 100; // 100ms debounce

	while (1)
	{
		unsigned long current_time = Timer2_get_milliseconds();

		// Read button state with debouncing
		unsigned char current_button_state = Port_read_pin(0, 4);

		// Button state change detection
		if (current_button_state != last_button_state)
		{
			last_debounce_time = current_time;
		}

		// Process button press after debounce period
		if ((current_time - last_debounce_time) > DEBOUNCE_DELAY)
		{
			// Button pressed (low due to pull-up)
			if (last_button_state == 1 && current_button_state == 0)
			{
				operation_count++;
				operation_mode = !operation_mode; // Toggle between read/write

				if (operation_mode) // Write operation
				{
					// Write educational data to EEPROM
					lcd_string(0, 4, "Writing to EEPROM...");
					Port_write(0x0F, 1); // LED pattern for write operation

					// Write data array using modernized EEPROM library
					for (unsigned int i = 0; i < sizeof(educational_data) - 1; i++)
					{
						EEPROM_write(eeprom_base_address + i, educational_data[i]);
						Timer2_delay_ms(5); // Small delay for EEPROM write completion
					}

					lcd_string(0, 4, "Write Complete!     ");
					Port_write(0xF0, 1); // Different LED pattern for completion
				}
				else // Read operation
				{
					// Read data from EEPROM
					lcd_string(0, 4, "Reading from EEPROM...");
					Port_write(0xAA, 1); // LED pattern for read operation

					// Read data using modernized EEPROM library
					for (unsigned int i = 0; i < sizeof(educational_data) - 1; i++)
					{
						read_buffer[i] = EEPROM_read(eeprom_base_address + i);
					}
					read_buffer[sizeof(educational_data) - 1] = '\\0'; // Null terminate

					lcd_string(0, 4, "Read Complete!      ");
					Port_write(0x55, 1); // Different LED pattern for completion

					// Display read data
					lcd_clear();
					ScreenBuffer_clear();
					lcd_string(0, 0, "EEPROM Data:");

					// Display data in chunks for LCD
					lcd_string(0, 1, "SOC3050-ATmega128");
					lcd_string(0, 2, "EEPROM-Demo-2025");
				}

				// Update operation statistics
				lcd_string(0, 5, "Ops: ");
				GLCD_3DigitDecimal(operation_count);
				lcd_string(6, 5, operation_mode ? " Write" : " Read ");

				// Display timing information
				lcd_string(0, 6, "Time: ");
				GLCD_4DigitDecimal(current_time / 1000);
				lcd_string(8, 6, "s");

				// Show EEPROM address range
				lcd_string(0, 7, "Addr: ");
				GLCD_3DigitDecimal(eeprom_base_address);
				lcd_string(6, 7, "-");
				GLCD_3DigitDecimal(eeprom_base_address + sizeof(educational_data) - 2);
			}
		}

		// Update button state for next iteration
		last_button_state = current_button_state;

		// Small delay for system responsiveness
		Timer2_delay_ms(10);
	}
}
#endif
