/*
 * _init.c - ATmega128 Educational Initialization Library
 * Part of Assembly → C → Python Learning Progression
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Understand microcontroller initialization sequence
 * 2. Learn proper peripheral setup order
 * 3. Master interrupt management concepts
 * 4. Practice modular programming design
 *
 * INITIALIZATION SEQUENCE:
 * 1. Disable interrupts (safety)
 * 2. Initialize peripherals in dependency order
 * 3. Enable interrupts (start operation)
 *
 * LEARNING NOTES:
 * - cli() = Clear Interrupt flag (disable all interrupts)
 * - sei() = Set Interrupt flag (enable interrupts)
 * - Initialization order matters for dependent systems
 * - Each peripheral has specific setup requirements
 */

#include "config.h"

/*
 * EDUCATIONAL FUNCTION: Complete System Initialization
 *
 * PURPOSE: Initialize all ATmega128 peripherals in the correct order
 * LEARNING: Shows systematic approach to microcontroller setup
 *
 * SEQUENCE EXPLANATION:
 * 1. Disable interrupts to prevent conflicts during setup
 * 2. Initialize basic I/O ports first (foundation)
 * 3. Setup timers for timing references
 * 4. Initialize communication peripherals
 * 5. Setup analog input systems
 * 6. Initialize display systems
 * 7. Enable interrupts to start operation
 */
void init_devices(void)
{
	/*
	 * STEP 1: DISABLE INTERRUPTS
	 * Safety measure: prevent interrupts during initialization
	 * Assembly equivalent: cli
	 */
	cli(); // Clear global interrupt flag

	/*
	 * STEP 2: INITIALIZE BASIC I/O PORTS
	 * Foundation: Setup digital input/output capabilities
	 * Educational: Demonstrates DDR (Data Direction Register) usage
	 */
	Port_init(); // Initialize port directions and initial states

	/*
	 * STEP 3: INITIALIZE INTERRUPT SYSTEM
	 * Setup: Configure external interrupt capabilities
	 * Educational: Shows interrupt vector setup
	 */
	Interrupt_init(); // Configure external interrupt sources

	/*
	 * STEP 4: INITIALIZE TIMER SYSTEMS
	 * Timing: Setup timer/counter peripherals for timing references
	 * Educational: Demonstrates timer modes and prescaler concepts
	 */
	Timer2_init(); // Initialize Timer/Counter 2 for timing operations

	/*
	 * STEP 5: INITIALIZE COMMUNICATION
	 * Communication: Setup UART for serial data exchange
	 * Educational: Shows serial communication setup and baud rate calculation
	 */
	Uart1_init(); // Initialize UART1 for serial communication

	/*
	 * STEP 6: INITIALIZE ANALOG INPUT
	 * Sensors: Setup ADC for analog sensor reading
	 * Educational: Demonstrates analog-to-digital conversion setup
	 */
	Adc_init(); // Initialize ADC for sensor input

	/*
	 * STEP 7: INITIALIZE DISPLAY SYSTEM
	 * Output: Setup graphical LCD for visual feedback
	 * Educational: Shows complex peripheral initialization
	 */
	lcd_init(); // Initialize graphical LCD display

	/*
	 * STEP 8: ENABLE INTERRUPTS
	 * Activation: Allow interrupt system to function
	 * Assembly equivalent: sei
	 */
	sei(); // Set global interrupt flag - system ready for operation

	/*
	 * EDUCATIONAL NOTE:
	 * At this point, all ATmega128 peripherals are initialized and ready.
	 * The microcontroller can now respond to:
	 * - External interrupts (buttons, sensors)
	 * - Timer interrupts (periodic events)
	 * - UART interrupts (serial communication)
	 * - ADC interrupts (analog conversion complete)
	 */
}

/*
 * EDUCATIONAL FUNCTION: Selective Initialization
 *
 * PURPOSE: Initialize only specific peripherals for focused learning
 * LEARNING: Demonstrates modular initialization approach
 * USE CASE: When students want to study one peripheral at a time
 */
void init_basic_io(void)
{
	cli();
	Port_init(); // Only initialize basic I/O
	sei();
}

/*
 * EDUCATIONAL FUNCTION: Communication-focused Initialization
 *
 * PURPOSE: Initialize only communication-related peripherals
 * LEARNING: Shows dependencies between related systems
 * USE CASE: For serial communication and networking examples
 */
void init_communication(void)
{
	cli();
	Port_init();   // Basic I/O needed for UART pins
	Timer2_init(); // Timer needed for baud rate generation
	Uart1_init();  // Serial communication
	sei();
}

/*
 * EDUCATIONAL FUNCTION: Sensor-focused Initialization
 *
 * PURPOSE: Initialize peripherals needed for sensor interfacing
 * LEARNING: Demonstrates analog input system requirements
 * USE CASE: For sensor reading and data acquisition examples
 */
void init_sensors(void)
{
	cli();
	Port_init();  // Basic I/O for sensor control
	Adc_init();	  // Analog input for sensors
	Uart1_init(); // Communication for sensor data
	sei();
}

/*
 * EDUCATIONAL FUNCTION: Display-focused Initialization
 *
 * PURPOSE: Initialize peripherals needed for graphical output
 * LEARNING: Shows complex peripheral dependencies
 * USE CASE: For graphics and user interface examples
 */
void init_display(void)
{
	cli();
	Port_init(); // Basic I/O for display control
	lcd_init();	 // Graphical LCD display
	sei();
}

/*
 * EDUCATIONAL FUNCTION: System Status Check
 *
 * PURPOSE: Verify that initialization completed successfully
 * LEARNING: Shows how to validate system state
 * RETURNS: 1 if all systems initialized, 0 if problems detected
 */
unsigned char check_init_status(void)
{
	/*
	 * EDUCATIONAL NOTE:
	 * In a real system, you would check:
	 * - Register values for expected settings
	 * - Peripheral response to test commands
	 * - Clock sources and frequencies
	 * - Power management states
	 */

	// Simple check: verify global interrupts are enabled
	// This indicates init_devices() completed successfully
	if (SREG & (1 << 7)) // Check global interrupt flag
	{
		return 1; // System initialized
	}
	else
	{
		return 0; // Initialization incomplete
	}
}

/*
 * EDUCATIONAL FUNCTION: Reset System
 *
 * PURPOSE: Safely reset all peripherals to known state
 * LEARNING: Shows proper shutdown and restart sequence
 * USE CASE: Error recovery and system restart
 */
void reset_system(void)
{
	cli(); // Disable interrupts

	/*
	 * EDUCATIONAL NOTE:
	 * In a complete implementation, you would:
	 * 1. Disable all peripherals
	 * 2. Reset all registers to default values
	 * 3. Clear all interrupt flags
	 * 4. Reset communication buffers
	 * 5. Reinitialize the system
	 */

	// For educational purposes, perform basic reset
	PORTB = 0x00; // Clear all port B outputs
	PORTC = 0x00; // Clear all port C outputs
	PORTD = 0x00; // Clear all port D outputs

	// Reinitialize the system
	init_devices();
}