#include "config.h"

// Only compile this file if any TIMER demo is enabled
#ifdef TIMER_COUNTER
#define TIMER_DEMO_ENABLED
#endif
#ifdef TIMER_CTC
#define TIMER_DEMO_ENABLED
#endif
#ifdef TIMER_FASTPWM
#define TIMER_DEMO_ENABLED
#endif
#ifdef TIMER_NORMAL
#define TIMER_DEMO_ENABLED
#endif

#ifdef TIMER_DEMO_ENABLED

/*
 * MODERNIZED TIMER DEMONSTRATIONS USING TIMER2 LIBRARY
 * Educational Framework: ATmega128 Timer Programming
 *
 * Learning Objectives:
 * 1. Understand timer modes and applications
 * 2. Learn real-time task scheduling concepts
 * 3. Master interrupt-driven timer operations
 * 4. Implement precision timing applications
 *
 * Timer2 Library Functions Used:
 * - Timer2_init(): Initialize 1ms precision timer system
 * - Timer2_start()/Timer2_stop(): Control timer operation
 * - Timer2_delay_ms(): Non-blocking delay functionality
 * - Timer2_get_milliseconds(): System uptime tracking
 * - Timer2_check_taskX(): Multi-task scheduling
 * - Timer2_set_prescaler(): Custom timing frequencies
 *
 * Key Educational Concepts:
 * - Real-time scheduling with multiple tasks
 * - Precise timing without blocking operations
 * - Timer-based event management
 * - Software PWM using timer interrupts
 * - System uptime and timing measurements
 *
 * Hardware Connections:
 * - PORTB.7: LED for visual timing feedback
 * - PORTB.4: PWM output demonstration (software-based)
 * - PORTD.6: External event input (for counter mode simulation)
 */

#ifdef TIMER_NORMAL
/*
 * DEMONSTRATION 1: Basic Timer Operation with Precision Delays
 * Educational Focus: Understanding timer-based delays vs busy waiting
 *
 * This example demonstrates:
 * - Timer2 library initialization and control
 * - Precision millisecond-based delays
 * - Non-blocking timer operations
 * - Visual feedback with LED blinking
 *
 * Learning Points:
 * 1. Timer2_delay_ms() provides precise, non-blocking delays
 * 2. System can perform other tasks during timer delays
 * 3. Timer precision is maintained across multiple delay cycles
 * 4. LED blink rate demonstrates actual timing accuracy
 *
 * Hardware Setup:
 * - LED connected to PORTB.7 for visual timing verification
 * - Observe consistent 1-second blink intervals
 */

void main_timer_normal(void)
{
	init_devices();

	// Initialize Timer2 system for 1ms precision timing
	Timer2_init();
	Timer2_start();

	// Configure PORTB.7 as output for LED timing indicator
	Port_init_output(7, 1); // Using modernized port library

	// Display educational information
	lcd_string(0, 0, "Timer2 Demo");
	lcd_string(0, 1, "Precision Delay");
	lcd_string(0, 2, "LED: 1sec blink");
	lcd_string(0, 3, "Non-blocking");

	unsigned long last_toggle_time = 0;
	unsigned int toggle_count = 0;

	while (1)
	{
		// Get current system time in milliseconds
		unsigned long current_time = Timer2_get_milliseconds();

		// Check if 1000ms (1 second) has elapsed
		if (current_time - last_toggle_time >= 1000)
		{
			// Toggle LED using modernized port functions
			Port_toggle_pin(7, 1); // Toggle PORTB.7

			// Update timing and counter
			last_toggle_time = current_time;
			toggle_count++;

			// Display timing statistics on LCD
			lcd_xy(0, 5);
			lcd_string(0, 5, "Toggles: ");
			GLCD_4DigitDecimal(toggle_count);

			lcd_xy(0, 6);
			lcd_string(0, 6, "Uptime: ");
			GLCD_4DigitDecimal(current_time / 1000); // Show seconds
			lcd_string(9, 6, "s");
		}

		// Demonstrate that system is responsive during timing
		// This loop can process other tasks without affecting timer precision

		// Example: Process button input or other sensors here
		// The timer operates independently in the background

		// Small delay to prevent LCD flicker (still non-blocking)
		Timer2_delay_ms(50);
	}
}
#endif

#ifdef TIMER_COUNTER
/*
 * DEMONSTRATION 2: Event Counter with Timer-Based Debouncing
 * Educational Focus: Combining timer precision with external event detection
 *
 * This example demonstrates:
 * - Timer-based button debouncing techniques
 * - Event counting with visual feedback
 * - Combining timer operations with GPIO input
 * - Real-time display updates without blocking
 *
 * Learning Points:
 * 1. Timer2 provides stable timing base for debouncing
 * 2. Event detection can be combined with timer operations
 * 3. Non-blocking event processing maintains system responsiveness
 * 4. Visual feedback shows both timer operation and event counting
 *
 * Hardware Setup:
 * - Push button connected to PORTD.6 (with pull-up)
 * - LED connected to PORTB.7 for event indication
 * - LCD displays counter and timing information
 */

void main_timer_counter(void)
{
	init_devices();

	// Initialize Timer2 system for precise timing
	Timer2_init();
	Timer2_start();

	// Configure GPIO using modernized port library
	Port_init(); // Initialize all ports (LEDs, buttons with pull-ups)

	// Display educational information
	lcd_string(0, 0, "Event Counter");
	lcd_string(0, 1, "Button: PD6");
	lcd_string(0, 2, "LED: PB7");
	lcd_string(0, 3, "Timer Debounce");

	unsigned int event_count = 0;
	unsigned char last_button_state = 1; // Start with button not pressed (pull-up)
	unsigned long last_debounce_time = 0;
	unsigned long last_led_time = 0;
	const unsigned int DEBOUNCE_DELAY = 50;	 // 50ms debounce time
	const unsigned int LED_BLINK_RATE = 200; // LED blink every 200ms when active

	while (1)
	{
		unsigned long current_time = Timer2_get_milliseconds();

		// Read current button state using modernized port functions
		unsigned char buttons = read_buttons();
		unsigned char current_button_state = (buttons & 0x40) ? 0 : 1; // Button 6 (PD6), inverted logic

		// Button debouncing using timer
		if (current_button_state != last_button_state)
		{
			last_debounce_time = current_time;
		}

		// Check if debounce period has elapsed
		if ((current_time - last_debounce_time) > DEBOUNCE_DELAY)
		{
			// If button state has changed and is now pressed (0 due to pull-up)
			if (last_button_state == 1 && current_button_state == 0)
			{
				// Button press detected - increment counter
				event_count++;

				// Update display immediately
				lcd_xy(0, 5);
				lcd_string(0, 5, "Events: ");
				GLCD_4DigitDecimal(event_count);

				// Visual feedback - brief LED flash
				led_on(7); // Turn on LED 7
				last_led_time = current_time;
			}
		}

		// Automatic LED turn-off after brief flash
		if ((current_time - last_led_time > LED_BLINK_RATE))
		{
			led_off(7); // Turn off LED 7
		}

		// Update button state for next iteration
		last_button_state = current_button_state;

		// Display system uptime
		lcd_xy(0, 6);
		lcd_string(0, 6, "Time: ");
		GLCD_4DigitDecimal(current_time / 1000);
		lcd_string(9, 6, "s");

		// Display timing statistics
		lcd_xy(0, 7);
		lcd_string(0, 7, "Rate: ");
		if (current_time > 0)
		{
			unsigned int events_per_minute = (event_count * 60000UL) / current_time;
			GLCD_4DigitDecimal(events_per_minute);
			lcd_string(9, 7, "/min");
		}

		// Small delay to prevent excessive polling
		Timer2_delay_ms(10);
	}
}
#endif

#ifdef TIMER_CTC
/*
 * DEMONSTRATION 3: Multi-Task Real-Time Scheduling
 * Educational Focus: Understanding real-time task management with timers
 *
 * This example demonstrates:
 * - Multi-task scheduling using Timer2 library
 * - Different task priorities and execution frequencies
 * - Non-blocking task execution patterns
 * - Real-time system design principles
 *
 * Learning Points:
 * 1. Timer2_check_taskX() functions enable precise task scheduling
 * 2. Multiple tasks can run at different frequencies simultaneously
 * 3. Task execution is deterministic and predictable
 * 4. System remains responsive to all tasks without blocking
 *
 * Task Schedule:
 * - Task 1: High frequency (100ms) - LED blinking and display updates
 * - Task 2: Medium frequency (500ms) - Sensor reading simulation
 * - Task 3: Low frequency (1000ms) - System statistics and logging
 *
 * Hardware Setup:
 * - LED connected to PORTB.7 for Task 1 visual feedback
 * - LCD display shows real-time task execution statistics
 */

void main_timer_ctc(void)
{
	init_devices();

	// Initialize Timer2 with default task intervals
	Timer2_init();
	Timer2_start();

	// Configure hardware using modernized libraries
	Port_init_output(7, 1); // PORTB.7 for LED indicator

	// Display educational information
	lcd_string(0, 0, "Multi-Task Demo");
	lcd_string(0, 1, "T1:100ms LED");
	lcd_string(0, 2, "T2:500ms Sensor");
	lcd_string(0, 3, "T3:1s Statistics");

	// Task execution counters
	unsigned int task1_count = 0;
	unsigned int task2_count = 0;
	unsigned int task3_count = 0;
	unsigned int sensor_value = 0;
	unsigned char led_state = 0;

	while (1)
	{
		// Task 1: High frequency (100ms) - LED control and fast updates
		if (Timer2_check_task2()) // Using Task2 for 100ms intervals
		{
			task1_count++;

			// Toggle LED state
			led_state = !led_state;
			Port_write_pin(7, 1, led_state);

			// Update fast-changing display
			lcd_xy(0, 4);
			lcd_string(0, 4, "T1: ");
			GLCD_4DigitDecimal(task1_count);

			// Simulate fast sensor reading (ADC-like operation)
			sensor_value = (sensor_value + 1) % 1024; // Simulate 10-bit ADC
		}

		// Task 2: Medium frequency (500ms) - Sensor processing
		if (Timer2_check_task1()) // Using Task1 for 500ms intervals
		{
			task2_count++;

			// Process sensor data (simulation)
			unsigned int processed_sensor = sensor_value / 4; // Scale to 8-bit

			// Update sensor display
			lcd_xy(0, 5);
			lcd_string(0, 5, "T2: ");
			GLCD_4DigitDecimal(task2_count);
			lcd_string(8, 5, " S:");
			GLCD_3DigitDecimal(processed_sensor);

			// Simulate sensor calibration or filtering here
		}

		// Task 3: Low frequency (1000ms) - System statistics
		if (Timer2_check_task3()) // Using Task3 for 1000ms intervals
		{
			task3_count++;

			// Calculate system uptime
			unsigned long uptime_seconds = Timer2_get_milliseconds() / 1000;

			// Update statistics display
			lcd_xy(0, 6);
			lcd_string(0, 6, "T3: ");
			GLCD_4DigitDecimal(task3_count);
			lcd_string(8, 6, " Up:");
			GLCD_4DigitDecimal(uptime_seconds);

			// Calculate task execution rates
			lcd_xy(0, 7);
			lcd_string(0, 7, "Rates: ");
			if (uptime_seconds > 0)
			{
				unsigned int t1_rate = task1_count / uptime_seconds; // Tasks per second
				unsigned int t2_rate = task2_count / uptime_seconds;
				GLCD_2DigitDecimal(t1_rate);
				lcd_string(2, 7, "/");
				GLCD_2DigitDecimal(t2_rate);
				lcd_string(5, 7, "/1");
			}
		}

		// Main loop can perform other non-time-critical tasks here
		// The timer-based task scheduling ensures precise timing

		// Small delay to prevent excessive polling (still responsive)
		Timer2_delay_ms(5);
	}
}
#endif

#ifdef TIMER_FASTPWM
/*
 * DEMONSTRATION 4: Software PWM Using Timer2 Precision
 * Educational Focus: Implementing PWM in software using timer interrupts
 *
 * This example demonstrates:
 * - Software-based PWM generation using Timer2
 * - Precise duty cycle control with timer precision
 * - Smooth brightness/intensity control algorithms
 * - Understanding PWM principles without hardware PWM
 *
 * Learning Points:
 * 1. Software PWM provides more flexibility than hardware PWM
 * 2. Timer2 precision enables smooth PWM generation
 * 3. Duty cycle can be dynamically adjusted in real-time
 * 4. Multiple PWM channels can be implemented simultaneously
 *
 * PWM Parameters:
 * - Frequency: Controlled by Timer2 precision (typically 1-10kHz)
 * - Duty Cycle: 0-100% with fine resolution
 * - Smooth transitions with gradual duty cycle changes
 *
 * Hardware Setup:
 * - LED connected to PORTB.4 for PWM output demonstration
 * - Observe smooth brightness transitions
 * - LCD displays current PWM parameters
 */

void main_timer_fastpwm(void)
{
	init_devices();

	// Initialize Timer2 for precise PWM timing
	Timer2_init();
	Timer2_start();

	// Configure PWM output pin using modernized port library
	Port_init_output(4, 1); // PORTB.4 as PWM output

	// Display educational information
	lcd_string(0, 0, "Software PWM");
	lcd_string(0, 1, "Output: PB4");
	lcd_string(0, 2, "Timer2 Precision");
	lcd_string(0, 3, "Smooth Control");

	// PWM parameters
	unsigned char duty_cycle = 0;			  // Current duty cycle (0-255)
	unsigned char target_duty = 255;		  // Target duty cycle
	unsigned char pwm_counter = 0;			  // PWM cycle counter
	unsigned long last_pwm_update = 0;		  // Last PWM cycle time
	unsigned long last_duty_change = 0;		  // Last duty cycle change time
	const unsigned char PWM_FREQUENCY_MS = 5; // PWM cycle time (5ms = 200Hz)
	const unsigned char DUTY_CHANGE_MS = 20;  // Duty cycle change rate

	// Direction control for smooth transitions
	char duty_direction = 1; // 1 for increasing, -1 for decreasing

	while (1)
	{
		unsigned long current_time = Timer2_get_milliseconds();

		// PWM Generation using Timer2 precision
		if (current_time - last_pwm_update >= PWM_FREQUENCY_MS)
		{
			last_pwm_update = current_time;
			pwm_counter = 0; // Reset PWM cycle
		}

		// Calculate PWM cycle position (0-255 over PWM_FREQUENCY_MS milliseconds)
		unsigned char cycle_position = (unsigned char)((current_time - last_pwm_update) * 255UL / PWM_FREQUENCY_MS);

		// Generate PWM output based on duty cycle
		if (cycle_position < duty_cycle)
		{
			Port_write_pin(4, 1, 1); // PWM HIGH
		}
		else
		{
			Port_write_pin(4, 1, 0); // PWM LOW
		}

		// Smooth duty cycle transitions
		if (current_time - last_duty_change >= DUTY_CHANGE_MS)
		{
			last_duty_change = current_time;

			// Gradual duty cycle changes for smooth transitions
			if (duty_direction > 0)
			{
				if (duty_cycle < target_duty)
				{
					duty_cycle += 5; // Increase gradually
				}
				else
				{
					// Reached maximum, start decreasing
					target_duty = 0;
					duty_direction = -1;
				}
			}
			else
			{
				if (duty_cycle > target_duty)
				{
					duty_cycle -= 5; // Decrease gradually
				}
				else
				{
					// Reached minimum, start increasing
					target_duty = 255;
					duty_direction = 1;
				}
			}

			// Update display with current PWM parameters
			lcd_xy(0, 5);
			lcd_string(0, 5, "Duty: ");
			GLCD_3DigitDecimal((duty_cycle * 100) / 255); // Show as percentage
			lcd_string(8, 5, "%");

			lcd_xy(0, 6);
			lcd_string(0, 6, "Raw: ");
			GLCD_3DigitDecimal(duty_cycle);
			lcd_string(8, 6, "/255");

			// Display PWM frequency
			lcd_xy(0, 7);
			lcd_string(0, 7, "Freq: ");
			GLCD_3DigitDecimal(1000 / PWM_FREQUENCY_MS);
			lcd_string(8, 7, "Hz");
		}

		// Display system timing information
		lcd_xy(0, 4);
		lcd_string(0, 4, "Time: ");
		GLCD_4DigitDecimal(current_time / 1000);
		lcd_string(9, 4, "s");

		// Very small delay to maintain PWM precision
		// Note: Real implementation might use Timer2 interrupts for even better precision
		Timer2_delay_ms(1);
	}
}
#endif

#endif // TIMER_DEMO_ENABLED
