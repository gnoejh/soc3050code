/*
 * _init.c - ATmega128 Educational Initialization Library
 * Part of Assembly → C → Python Learning Progression
 *
 * EDUCATIONAL OBJECTIVES:
 * - Understanding peripheral initialization sequences
 * - Learning sensor system requirements
 * - Demonstrating modular initialization patterns
 * - Master interrupt management concepts
 * - Practice modular programming design
 *
 * INITIALIZATION SEQUENCE:
 * 1. Disable interrupts (safety)
 * 2. Initialize peripherals in dependency order
 * 3. Enable interrupts (start operation)
 */

#include "config.h"

/*
 * EDUCATIONAL FUNCTION: Sensor-focused Initialization
 *
 * PURPOSE: Initialize peripherals needed for sensor interfacing
 * LEARNING: Demonstrates analog input system requirements
 * USE CASE: For sensor reading and data acquisition examples
 */
#if defined(ADC_BASIC_READING) || defined(ADC_POLLING) || defined(ADC_INTERRUPT) || defined(ENABLE_ADC) || defined(CDS) || defined(ACCELEROMETER) || defined(JOYSTICK)
void init_sensors(void)
{
	cli();
	Port_init();  // Basic I/O for sensor control
	Adc_init();	  // Analog input for sensors
	Uart1_init(); // Serial output for sensor data
	sei();
}
#endif

/*
 * EDUCATIONAL FUNCTION: Complete System Initialization
 *
 * PURPOSE: Initialize all system components in proper sequence
 * LEARNING: Demonstrates comprehensive system setup
 * USE CASE: For complex applications requiring multiple peripherals
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

#ifdef ENABLE_INTERRUPTS
	/*
	 * STEP 3: INITIALIZE INTERRUPT SYSTEM
	 * Setup: Configure external interrupt capabilities
	 * Educational: Shows interrupt vector setup
	 */
	Interrupt_init(); // Configure external interrupt sources
#endif

#ifdef ENABLE_TIMERS
	/*
	 * STEP 4: INITIALIZE TIMER SYSTEM
	 * Setup: Configure hardware timers for precise timing
	 * Educational: Demonstrates timer modes and prescalers
	 */
	Timer2_init(); // Initialize Timer2 for general timing
#endif

#ifdef ENABLE_UART
	/*
	 * STEP 5: INITIALIZE SERIAL COMMUNICATION
	 * Setup: Configure UART for data transmission
	 * Educational: Shows baud rate calculation and frame format
	 */
	Uart1_init(); // Initialize UART1 for serial communication
#endif

#ifdef ENABLE_ADC
	/*
	 * STEP 6: INITIALIZE ANALOG-TO-DIGITAL CONVERTER
	 * Setup: Configure ADC for sensor reading
	 * Educational: Demonstrates reference voltage and resolution
	 */
	Adc_init(); // Initialize ADC for analog input
#endif

#ifdef ENABLE_LCD
	/*
	 * STEP 7: INITIALIZE DISPLAY SYSTEM
	 * Setup: Configure LCD for visual output
	 * Educational: Shows parallel interface timing
	 */
	lcd_init(); // Initialize character LCD display
#endif

	/*
	 * STEP 8: ENABLE INTERRUPTS
	 * Final step: Allow interrupt processing to begin
	 * Assembly equivalent: sei
	 */
	sei(); // Set global interrupt flag
}

/*
 * EDUCATIONAL FUNCTION: Game-specific Initialization
 *
 * PURPOSE: Initialize peripherals needed for game applications
 * LEARNING: Demonstrates application-specific setup patterns
 * USE CASE: For interactive games and user interface applications
 */
#if defined(GAME_PONG) || defined(GAME_HANGMAN) || defined(GAME_WORD_PUZZLE) || defined(GAME_OBSTACLE) || defined(ENABLE_GAMES)
void init_game_devices(void)
{
	cli();
	Port_init();    // Basic I/O for buttons and LEDs
	Uart1_init();   // Serial communication for game control
	Timer2_init();  // Timing for game logic
	Interrupt_init(); // Button input handling
	sei();
}
#endif

/*
 * EDUCATIONAL FUNCTION: IoT-focused Initialization
 *
 * PURPOSE: Initialize peripherals needed for IoT applications
 * LEARNING: Demonstrates sensor integration and communication setup
 * USE CASE: For data collection and remote monitoring systems
 */
#if defined(IOT_BASIC) || defined(IOT_SENSORS) || defined(IOT_COMMUNICATION) || defined(ENABLE_IOT)
void init_iot_devices(void)
{
	cli();
	Port_init();    // Basic I/O for sensor interfaces
	Adc_init();     // Analog sensors (temperature, light, etc.)
	Uart1_init();   // Primary communication channel
	Timer2_init();  // Periodic sensor sampling
	Interrupt_init(); // Event-driven data collection
	sei();
}
#endif

/*
 * EDUCATIONAL FUNCTION: Communication-focused Initialization
 *
 * PURPOSE: Initialize peripherals optimized for serial communication
 * LEARNING: Demonstrates communication protocol setup
 * USE CASE: For data logging, remote control, and PC interface
 */
#if defined(SERIAL_POLLING_SINGLE_CHAR) || defined(SERIAL_POLLING_STRING) || defined(SERIAL_INTERRUPT_RX) || defined(SERIAL_INTERRUPT_TX) || defined(ENABLE_UART_FEATURES)
void init_communication_devices(void)
{
	cli();
	Port_init();  // Basic I/O for status indication
	Uart1_init(); // Primary serial communication
	#ifdef ENABLE_DUAL_UART
	Uart0_init(); // Secondary communication channel
	#endif
	Timer2_init(); // Communication timing and timeouts
	sei();
}
#endif

/*
 * EDUCATIONAL NOTES:
 *
 * INITIALIZATION ORDER IMPORTANCE:
 * - Always disable interrupts first (cli)
 * - Initialize hardware in dependency order
 * - Configure timers before interrupt-driven peripherals
 * - Enable interrupts last (sei)
 *
 * CONDITIONAL COMPILATION:
 * - Uses #ifdef to include only needed peripherals
 * - Reduces memory usage for simple applications
 * - Demonstrates modular programming concepts
 * - Allows project-specific optimization
 *
 * EDUCATIONAL VALUE:
 * - Shows real-world initialization patterns
 * - Demonstrates safety considerations
 * - Teaches peripheral interdependencies
 * - Provides reusable code templates
 */