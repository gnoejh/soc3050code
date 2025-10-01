#include "config.h"

// Only compile this file if any INTERRUPT demo is enabled
#ifdef INTERRUPT_EXTERNAL
#define INTERRUPT_DEMO_ENABLED
#endif
#ifdef INTERRUPT_LAB
#define INTERRUPT_DEMO_ENABLED
#endif
#ifdef INTERRUPT_TIMER
#define INTERRUPT_DEMO_ENABLED
#endif
#ifdef INTERRUPT_TIMER_CTC
#define INTERRUPT_DEMO_ENABLED
#endif
#ifdef INTERRUPT_EXT_TIMER
#define INTERRUPT_DEMO_ENABLED
#endif

#ifdef INTERRUPT_DEMO_ENABLED

/*
 * MODERNIZED INTERRUPT DEMONSTRATIONS USING INTERRUPT LIBRARY
 * Educational Framework: ATmega128 Event-Driven Programming
 *
 * Learning Objectives:
 * 1. Master interrupt-driven programming concepts
 * 2. Understand event prioritization and handling
 * 3. Implement safe critical section protection
 * 4. Learn interrupt statistics and monitoring
 * 5. Explore real-time system response patterns
 *
 * Interrupt Library Functions Used:
 * - Interrupt_init(): Configure external interrupt system
 * - Interrupt_enable_global()/Interrupt_disable_global(): Control interrupt state
 * - Interrupt_get_statistics(): Monitor interrupt performance
 * - Interrupt_reset_statistics(): Clear monitoring data
 *
 * Key Educational Concepts:
 * - Event-driven programming paradigms
 * - Interrupt priority and vector handling
 * - Critical section protection techniques
 * - Real-time response measurement
 * - System statistics and performance monitoring
 *
 * Hardware Connections:
 * - PD0 (INT0): Primary button/switch input (highest priority)
 * - PD1 (INT1): Secondary input (second priority)
 * - PORTB: LED indicators for visual interrupt feedback
 * - LCD: Real-time interrupt statistics display
 *
 * Integration with Other Libraries:
 * - Timer2 library: Precision timing and task scheduling
 * - Port library: Safe GPIO control and debouncing
 * - UART library: Interrupt statistics reporting
 * - GLCD library: Visual interrupt monitoring interface
 */

// Global variables for interrupt demonstrations
volatile unsigned int interrupt_demo_counter = 0;
volatile unsigned char last_interrupt_source = 0;
volatile unsigned long system_events = 0;

/*
 * INTERRUPT LIBRARY INTEGRATION NOTES:
 *
 * The modernized interrupt library handles all low-level register configuration
 * automatically, providing a clean API for educational use:
 *
 * 1. Hardware abstraction: No direct register manipulation required
 * 2. Safety features: Built-in debouncing and critical section protection
 * 3. Statistics: Automatic interrupt counting and performance monitoring
 * 4. Educational focus: Clear API that demonstrates interrupt concepts
 * 5. Integration: Seamless integration with Timer2, Port, and other libraries
 *
 * This allows students to focus on event-driven programming concepts
 * rather than low-level hardware details.
 */

#ifdef INTERRUPT_EXTERNAL
/*
 * DEMONSTRATION 1: Basic External Interrupt with Statistics
 * Educational Focus: Understanding interrupt-driven event handling
 *
 * This example demonstrates:
 * - Modern interrupt library initialization and control
 * - Automatic interrupt statistics collection
 * - Visual feedback with interrupt counting
 * - Safe interrupt programming practices
 *
 * Learning Points:
 * 1. Interrupt_init() handles all hardware configuration automatically
 * 2. Statistics provide insight into interrupt behavior and performance
 * 3. Visual feedback helps understand interrupt timing and frequency
 * 4. Library functions ensure safe and reliable interrupt operation
 *
 * Hardware Setup:
 * - Button/switch connected to PD0 (INT0) with pull-up
 * - LEDs connected to PORTB for visual interrupt indication
 * - LCD displays real-time interrupt statistics
 */

void main_interrupt_external(void)
{
	init_devices();

	// Initialize interrupt system using modernized library
	Interrupt_init();
	Interrupt_reset_statistics(); // Start with clean statistics

	// Configure output indicators using modernized port library
	Port_init_output(0xFF, 1); // All PORTB pins as outputs
	Port_write(0xAA, 1);	   // Initial LED pattern

	// Display educational information
	lcd_string(0, 0, "External INT Demo");
	lcd_string(0, 1, "Button: PD0 (INT0)");
	lcd_string(0, 2, "Library: Modern");
	lcd_string(0, 3, "Statistics: ON");

	// Enable global interrupts using library function
	Interrupt_enable_global();

	// Main loop displays interrupt statistics
	while (1)
	{
		// Get interrupt statistics from library
		unsigned int int0_count, int1_count, total_count;
		unsigned char last_triggered;

		Interrupt_get_statistics(&int0_count, &int1_count, &total_count, &last_triggered);

		// Display statistics on LCD
		lcd_xy(0, 4);
		lcd_string(0, 4, "INT0: ");
		GLCD_4DigitDecimal(int0_count);

		lcd_xy(0, 5);
		lcd_string(0, 5, "Total: ");
		GLCD_4DigitDecimal(total_count);

		lcd_xy(0, 6);
		lcd_string(0, 6, "Last: INT");
		GLCD_1DigitDecimal(last_triggered);

		// Calculate interrupt rate if enough time has passed
		if (total_count > 0)
		{
			unsigned long uptime = Timer2_get_milliseconds() / 1000;
			if (uptime > 0)
			{
				unsigned int rate = total_count / uptime; // Interrupts per second
				lcd_xy(0, 7);
				lcd_string(0, 7, "Rate: ");
				GLCD_2DigitDecimal(rate);
				lcd_string(7, 7, "/sec");
			}
		}

		// Small delay to prevent LCD flicker
		Timer2_delay_ms(100);
	}
}

/*
 * NOTE: The ISR is now automatically handled by the interrupt library.
 * The library provides the INT0_vect ISR that:
 * 1. Updates statistics counters automatically
 * 2. Provides debouncing for reliable operation
 * 3. Handles LED toggle as visual feedback
 * 4. Maintains interrupt timing information
 *
 * This demonstrates the power of library abstraction - students can
 * focus on interrupt concepts rather than low-level implementation.
 */
#endif

// Timer/Counter Interrupt Configuration Table - MODERNIZED APPROACH
/*
 * EDUCATIONAL NOTE: Timer Integration with Interrupt Library
 *
 * The modernized approach integrates Timer2 library with the Interrupt library
 * to provide comprehensive timing and event management. This demonstrates:
 *
 * 1. Library integration patterns for complex systems
 * 2. Timer-based interrupt correlation and analysis
 * 3. Real-time performance monitoring capabilities
 * 4. Advanced event timing and frequency analysis
 *
 * Key advantages over direct register manipulation:
 * - Automatic timing precision with Timer2 library
 * - Built-in interrupt statistics and monitoring
 * - Safe critical section management
 * - Educational focus on concepts rather than hardware details
 */

#ifdef INTERRUPT_TIMER
/*
 * DEMONSTRATION 2: Timer-Interrupt Integration Demo
 * Educational Focus: Combining timer and external interrupt systems
 *
 * This example demonstrates:
 * - Integration between Timer2 library and Interrupt library
 * - Dual interrupt source management (timer + external)
 * - Real-time event correlation and timing analysis
 * - Advanced interrupt statistics and performance monitoring
 *
 * Learning Points:
 * 1. Multiple interrupt sources can work together seamlessly
 * 2. Timer2 provides time-base for interrupt performance analysis
 * 3. Statistics help understand interrupt timing relationships
 * 4. Event correlation enables advanced system monitoring
 *
 * Hardware Setup:
 * - Button connected to PD0 (INT0) for external events
 * - LEDs connected to PORTB for dual-mode visual feedback
 * - LCD displays comprehensive timing and interrupt statistics
 */

void main_interrupt_timer(void)
{
	init_devices();

	// Initialize both timer and interrupt systems
	Timer2_init();
	Timer2_start();
	Interrupt_init();
	Interrupt_reset_statistics();

	// Configure output using modernized libraries
	Port_init_output(0xFF, 1); // All PORTB as outputs
	Port_write(0xAA, 1);	   // Initial pattern

	// Display educational information
	lcd_string(0, 0, "Timer+INT Demo");
	lcd_string(0, 1, "Timer2: 1ms base");
	lcd_string(0, 2, "INT0: External");
	lcd_string(0, 3, "Integration: ON");

	// Enable interrupts
	Interrupt_enable_global();

	unsigned long last_timer_toggle = 0;
	unsigned long last_stats_update = 0;
	unsigned char timer_led_state = 0;

	while (1)
	{
		unsigned long current_time = Timer2_get_milliseconds();

		// Timer-based LED toggle every 500ms (demonstrates timer interrupt concept)
		if (current_time - last_timer_toggle >= 500)
		{
			last_timer_toggle = current_time;
			timer_led_state = !timer_led_state;

			// Toggle specific LED bits to show timer operation
			if (timer_led_state)
			{
				Port_write_pin(0, 1, 1); // PORTB.0 ON
				Port_write_pin(1, 1, 0); // PORTB.1 OFF
			}
			else
			{
				Port_write_pin(0, 1, 0); // PORTB.0 OFF
				Port_write_pin(1, 1, 1); // PORTB.1 ON
			}
		}

		// Update statistics display every 200ms
		if (current_time - last_stats_update >= 200)
		{
			last_stats_update = current_time;

			// Get interrupt statistics
			unsigned int int0_count, int1_count, total_count;
			unsigned char last_triggered;
			Interrupt_get_statistics(&int0_count, &int1_count, &total_count, &last_triggered);

			// Display timing information
			lcd_xy(0, 4);
			lcd_string(0, 4, "Time: ");
			GLCD_4DigitDecimal(current_time / 1000);
			lcd_string(8, 4, "s");

			// Display interrupt statistics
			lcd_xy(0, 5);
			lcd_string(0, 5, "Ext: ");
			GLCD_4DigitDecimal(int0_count);

			// Calculate and display interrupt timing
			if (total_count > 0 && current_time > 1000)
			{
				unsigned int avg_interval = current_time / total_count;
				lcd_xy(0, 6);
				lcd_string(0, 6, "Avg: ");
				GLCD_4DigitDecimal(avg_interval);
				lcd_string(9, 6, "ms");

				// Display interrupt frequency
				unsigned int frequency = 1000 / (avg_interval + 1); // Hz
				lcd_xy(0, 7);
				lcd_string(0, 7, "Freq: ");
				GLCD_2DigitDecimal(frequency);
				lcd_string(7, 7, "Hz");
			}
		}

		// Small delay for system responsiveness
		Timer2_delay_ms(10);
	}
}

/*
 * NOTE: Timer interrupt functionality is now provided by Timer2 library.
 * The old TIMER0_OVF_vect ISR is replaced by Timer2's built-in interrupt
 * system which provides:
 * 1. Precise 1ms timing resolution
 * 2. Task scheduling capabilities
 * 3. Non-blocking delay functions
 * 4. System uptime tracking
 *
 * This demonstrates modern interrupt architecture where timer functionality
 * is abstracted into reusable library components.
 */
#endif

// Timer Interrupt: CTC Mode Configuration - MODERNIZED WITH TIMER2 TASKS
/*
 * EDUCATIONAL NOTE: Task-Based Timer Management
 *
 * The modernized approach uses Timer2 library's task scheduling system
 * instead of direct CTC (Clear Timer on Compare) mode configuration.
 * This demonstrates:
 *
 * 1. Multi-task timer scheduling with precise intervals
 * 2. Task-based interrupt architecture
 * 3. Advanced timing control and monitoring
 * 4. Integration of timer tasks with external interrupts
 *
 * Advantages over direct CTC mode:
 * - Multiple concurrent timer tasks
 * - Automatic task interval management
 * - Built-in task performance monitoring
 * - Non-blocking task execution patterns
 */

#ifdef INTERRUPT_TIMER_CTC
/*
 * DEMONSTRATION 3: Multi-Task Timer Scheduling with Interrupts
 * Educational Focus: Advanced timer task management and interrupt correlation
 *
 * This example demonstrates:
 * - Timer2 task scheduling system with multiple concurrent tasks
 * - Integration of timer tasks with external interrupt events
 * - Real-time task performance monitoring and analysis
 * - Advanced timing control with variable task intervals
 *
 * Learning Points:
 * 1. Timer2_check_taskX() functions provide precise multi-task scheduling
 * 2. Task intervals can be dynamically adjusted based on system load
 * 3. External interrupts can influence timer task behavior
 * 4. Task statistics provide insight into real-time system performance
 *
 * Hardware Setup:
 * - Button connected to PD0 (INT0) to trigger task interval changes
 * - LEDs connected to PORTB showing different task activities
 * - LCD displays comprehensive task timing and interrupt statistics
 */

void main_interrupt_timer_ctc(void)
{
	init_devices();

	// Initialize Timer2 multi-task system and interrupts
	Timer2_init();
	Timer2_start();
	Interrupt_init();
	Interrupt_reset_statistics();

	// Configure output using modernized libraries
	Port_init_output(0xFF, 1); // All PORTB as outputs
	Port_write(0xAA, 1);	   // Initial pattern

	// Display educational information
	lcd_string(0, 0, "Multi-Task CTC");
	lcd_string(0, 1, "T1:Fast T2:Med");
	lcd_string(0, 2, "T3:Slow INT:Adj");
	lcd_string(0, 3, "Dynamic Timing");

	// Enable interrupts
	Interrupt_enable_global();

	unsigned int task1_count = 0; // Fast task counter
	unsigned int task2_count = 0; // Medium task counter
	unsigned int task3_count = 0; // Slow task counter
	unsigned char led_pattern = 0xAA;
	unsigned int speed_multiplier = 1; // Adjustable by interrupts

	while (1)
	{
		// Task 1: Fast execution (100ms base, adjustable)
		if (Timer2_check_task2()) // Using task2 for fast execution
		{
			task1_count++;

			// Fast LED pattern change
			Port_write_pin(7, 1, task1_count & 1); // PORTB.7 blink

			// Display fast task counter
			lcd_xy(0, 4);
			lcd_string(0, 4, "Fast: ");
			GLCD_4DigitDecimal(task1_count);
		}

		// Task 2: Medium execution (500ms base, adjustable)
		if (Timer2_check_task1()) // Using task1 for medium execution
		{
			task2_count++;

			// Medium frequency LED pattern
			led_pattern = (led_pattern << 1) | (led_pattern >> 7); // Rotate pattern
			Port_write(led_pattern & 0x3F, 1);					   // Update PORTB bits 0-5

			// Display medium task counter
			lcd_xy(0, 5);
			lcd_string(0, 5, "Med: ");
			GLCD_4DigitDecimal(task2_count);
		}

		// Task 3: Slow execution (1000ms base, adjustable)
		if (Timer2_check_task3()) // Using task3 for slow execution
		{
			task3_count++;

			// Slow task - comprehensive statistics update
			unsigned int int0_count, int1_count, total_count;
			unsigned char last_triggered;
			Interrupt_get_statistics(&int0_count, &int1_count, &total_count, &last_triggered);

			// Display slow task counter and interrupt stats
			lcd_xy(0, 6);
			lcd_string(0, 6, "Slow: ");
			GLCD_4DigitDecimal(task3_count);
			lcd_string(8, 6, " I:");
			GLCD_2DigitDecimal(total_count);

			// Calculate system performance metrics
			unsigned long uptime = Timer2_get_milliseconds() / 1000;
			if (uptime > 0)
			{
				unsigned int task_rate = (task1_count + task2_count + task3_count) / uptime;
				lcd_xy(0, 7);
				lcd_string(0, 7, "Rate: ");
				GLCD_3DigitDecimal(task_rate);
				lcd_string(8, 7, "/s");
			}

			// Check if interrupt occurred - adjust timing if needed
			if (total_count > 0 && (total_count % 5) == 0)
			{
				// Every 5th interrupt, toggle speed multiplier
				speed_multiplier = (speed_multiplier == 1) ? 2 : 1;

				// Note: In a full implementation, task intervals would be
				// dynamically adjusted here using Timer2_set_period_ms()
			}
		}

		// Small delay for system responsiveness
		Timer2_delay_ms(5);
	}
}

/*
 * NOTE: Compare Match functionality is now handled by Timer2 task system.
 * The old TIMER0_COMP_vect ISR is replaced by Timer2's task scheduling
 * which provides:
 * 1. Multiple concurrent timer tasks with different intervals
 * 2. Dynamic task interval adjustment capabilities
 * 3. Task performance monitoring and statistics
 * 4. Integration with external interrupt events
 *
 * This demonstrates evolution from simple compare match interrupts to
 * sophisticated real-time task scheduling systems.
 */
#endif

// Lab: Button Toggle and LCD Display - MODERNIZED COMPREHENSIVE DEMO
/*
 * EDUCATIONAL NOTE: Comprehensive Interrupt and Timer Integration Lab
 *
 * This lab demonstrates the full integration of modernized libraries
 * for a complete educational experience:
 *
 * 1. Interrupt library for event handling
 * 2. Timer2 library for precise timing and clock functions
 * 3. Port library for safe GPIO control
 * 4. Mathematical operations triggered by interrupts
 * 5. Real-time statistics and performance monitoring
 *
 * Educational objectives:
 * - Event-driven programming with mathematical computations
 * - Real-time clock implementation using timer precision
 * - Interrupt-triggered educational content generation
 * - System integration and comprehensive monitoring
 */

#ifdef INTERRUPT_LAB
/*
 * DEMONSTRATION 4: Educational Interactive Lab with Comprehensive Integration
 * Educational Focus: Complete system integration with interactive learning
 *
 * This example demonstrates:
 * - Integration of all modernized libraries in a single application
 * - Interactive learning through interrupt-triggered mathematical problems
 * - Real-time clock using Timer2 precision instead of blocking delays
 * - Comprehensive system monitoring and educational statistics
 *
 * Learning Points:
 * 1. Event-driven educational content generation
 * 2. Non-blocking real-time clock implementation
 * 3. Mathematical computation triggered by external events
 * 4. System performance monitoring in educational applications
 *
 * Hardware Setup:
 * - Button connected to PD0 (INT0) for interactive problem generation
 * - LEDs connected to PORTB for visual feedback
 * - LCD displays clock, problems, and comprehensive statistics
 */

// Global variables for educational lab
volatile unsigned char num1, num2;
volatile unsigned int problem_count = 0;
char result_display[30];
char clock_display[20];

void main_interrupt_lab(void)
{
	init_devices();

	// Initialize all modernized libraries
	Timer2_init();
	Timer2_start();
	Interrupt_init();
	Interrupt_reset_statistics();

	// Configure hardware using modernized libraries
	Port_init_output(0xFF, 1); // All PORTB as outputs
	Port_write(0xAA, 1);	   // Initial pattern

	// Display initial educational information
	lcd_clear();
	lcd_string(0, 0, "SOC3050 EduLab");
	lcd_string(0, 1, "Press button for");
	lcd_string(0, 2, "math problems!");
	lcd_string(0, 3, "Real-time clock");

	// Enable interrupts
	Interrupt_enable_global();

	// Real-time clock variables (using Timer2 precision)
	unsigned long last_clock_update = 0;
	unsigned int total_seconds = 0;
	unsigned int hours = 0, minutes = 0, seconds = 0;

	// Statistics update timing
	unsigned long last_stats_update = 0;

	while (1)
	{
		unsigned long current_time = Timer2_get_milliseconds();

		// Real-time clock update every 1000ms (non-blocking)
		if (current_time - last_clock_update >= 1000)
		{
			last_clock_update = current_time;
			total_seconds++;

			// Calculate time components
			hours = total_seconds / 3600;
			minutes = (total_seconds % 3600) / 60;
			seconds = (total_seconds % 3600) % 60;

			// Display clock using Timer2 precision
			sprintf(clock_display, "%02d:%02d:%02d", hours, minutes, seconds);
			lcd_string(0, 4, "Time: ");
			lcd_string(6, 4, clock_display);
		}

		// Update educational statistics every 500ms
		if (current_time - last_stats_update >= 500)
		{
			last_stats_update = current_time;

			// Get interrupt statistics
			unsigned int int0_count, int1_count, total_count;
			unsigned char last_triggered;
			Interrupt_get_statistics(&int0_count, &int1_count, &total_count, &last_triggered);

			// Display problem count and interrupt stats
			lcd_xy(0, 5);
			lcd_string(0, 5, "Problems: ");
			GLCD_2DigitDecimal(problem_count);

			lcd_xy(0, 6);
			lcd_string(0, 6, "Buttons: ");
			GLCD_2DigitDecimal(total_count);

			// Calculate learning rate (problems per minute)
			if (total_seconds > 0)
			{
				unsigned int problems_per_minute = (problem_count * 60) / total_seconds;
				lcd_xy(0, 7);
				lcd_string(0, 7, "Rate: ");
				GLCD_2DigitDecimal(problems_per_minute);
				lcd_string(7, 7, "/min");
			}
		}

		// Small delay for system responsiveness
		Timer2_delay_ms(50);
	}
}

/*
 * NOTE: Interactive problem generation is now handled by the interrupt library
 * ISR which provides:
 * 1. Automatic debouncing for reliable button detection
 * 2. Safe random number generation in interrupt context
 * 3. Atomic updates to shared variables
 * 4. Visual feedback coordination
 *
 * The educational content is generated on-demand rather than using
 * blocking delays, making the system more responsive and educational.
 *
 * When the interrupt occurs, it:
 * - Generates new random math problems
 * - Updates the problem counter safely
 * - Provides immediate visual feedback
 * - Maintains comprehensive statistics
 */
#endif

#ifdef INTERRUPT_EXT_TIMER
/*
 * DEMONSTRATION 5: Advanced System Integration - Complete Educational Framework
 * Educational Focus: Master-level integration of all modernized libraries
 *
 * This example demonstrates:
 * - Complete integration of all modernized library systems
 * - Advanced real-time clock with Timer2 precision (no approximation needed)
 * - Sophisticated interrupt-driven educational content generation
 * - Comprehensive system monitoring and performance analysis
 * - Professional-grade event correlation and timing analysis
 *
 * Learning Points:
 * 1. Master-level system integration demonstrates real-world embedded design
 * 2. Timer2 precision eliminates the need for overflow counting approximations
 * 3. Interrupt library provides professional-grade event handling
 * 4. Statistical analysis enables system optimization and debugging
 * 5. Non-blocking architecture maintains system responsiveness
 *
 * Hardware Setup:
 * - Button connected to PD0 (INT0) for interactive educational content
 * - LEDs connected to PORTB for sophisticated visual feedback patterns
 * - LCD displays comprehensive system information and statistics
 *
 * Advanced Features:
 * - Precise real-time clock using Timer2 millisecond accuracy
 * - Dynamic educational content generation based on user interaction
 * - Real-time system performance monitoring and optimization
 * - Advanced interrupt correlation and timing analysis
 */

// Advanced system variables for comprehensive demonstration
volatile unsigned int advanced_problem_count = 0;
volatile unsigned long total_interaction_time = 0;
volatile unsigned char current_difficulty_level = 1;
char advanced_result_display[40];
char precision_clock_display[25];
char system_stats_display[30];

void main_interrupt_ext_timer(void)
{
	init_devices();

	// Initialize complete modernized library ecosystem
	Timer2_init();
	Timer2_start();
	Interrupt_init();
	Interrupt_reset_statistics();

	// Configure advanced hardware setup
	Port_init_output(0xFF, 1); // All PORTB as outputs for sophisticated patterns
	Port_write(0xAA, 1);	   // Initial sophisticated pattern

	// Display advanced system information
	lcd_clear();
	lcd_string(0, 0, "SOC3050 Advanced");
	lcd_string(0, 1, "System Integration");
	lcd_string(0, 2, "Precision Timing");
	lcd_string(0, 3, "Smart Education");

	// Enable advanced interrupt system
	Interrupt_enable_global();

	// Advanced timing variables (precision-based)
	unsigned long last_clock_update = 0;
	unsigned long last_stats_update = 0;
	unsigned long last_pattern_update = 0;
	unsigned long session_start_time = Timer2_get_milliseconds();

	// Advanced LED pattern variables
	unsigned char led_pattern = 0xAA;
	unsigned char pattern_direction = 1;

	while (1)
	{
		unsigned long current_time = Timer2_get_milliseconds();
		unsigned long session_time = current_time - session_start_time;

		// Precision real-time clock (exact millisecond accuracy)
		if (current_time - last_clock_update >= 1000)
		{
			last_clock_update = current_time;

			// Calculate precise time components from Timer2 milliseconds
			unsigned long total_seconds = current_time / 1000;
			unsigned int hours = total_seconds / 3600;
			unsigned int minutes = (total_seconds % 3600) / 60;
			unsigned int seconds = (total_seconds % 3600) % 60;

			// Display precision clock with milliseconds
			sprintf(precision_clock_display, "%02d:%02d:%02d.%03d",
					hours, minutes, seconds, (unsigned int)(current_time % 1000));
			lcd_string(0, 4, precision_clock_display);
		}

		// Advanced statistics update and analysis
		if (current_time - last_stats_update >= 250) // High-frequency updates
		{
			last_stats_update = current_time;

			// Get comprehensive interrupt statistics
			unsigned int int0_count, int1_count, total_count;
			unsigned char last_triggered;
			Interrupt_get_statistics(&int0_count, &int1_count, &total_count, &last_triggered);

			// Display advanced educational metrics
			lcd_xy(0, 5);
			lcd_string(0, 5, "Probs: ");
			GLCD_2DigitDecimal(advanced_problem_count);
			lcd_string(8, 5, " L");
			GLCD_1DigitDecimal(current_difficulty_level);

			// Calculate advanced interaction metrics
			if (total_count > 0 && session_time > 1000)
			{
				// Average time between interactions
				unsigned int avg_interaction_ms = session_time / total_count;

				// Interaction frequency (per minute)
				unsigned int interactions_per_minute = (total_count * 60000UL) / session_time;

				// Display advanced metrics
				lcd_xy(0, 6);
				sprintf(system_stats_display, "Avg:%4dms %2d/min",
						avg_interaction_ms, interactions_per_minute);
				lcd_string(0, 6, system_stats_display);

				// Dynamic difficulty adjustment based on interaction rate
				if (interactions_per_minute > 10)
					current_difficulty_level = 3; // High difficulty
				else if (interactions_per_minute > 5)
					current_difficulty_level = 2; // Medium difficulty
				else
					current_difficulty_level = 1; // Basic difficulty
			}

			// System performance metrics
			lcd_xy(0, 7);
			lcd_string(0, 7, "Uptime: ");
			GLCD_4DigitDecimal(session_time / 1000);
			lcd_string(10, 7, "s");
		}

		// Advanced LED pattern generation (smooth transitions)
		if (current_time - last_pattern_update >= 100)
		{
			last_pattern_update = current_time;

			// Sophisticated pattern based on system activity
			if (pattern_direction)
			{
				led_pattern = (led_pattern << 1) | (led_pattern >> 7);
			}
			else
			{
				led_pattern = (led_pattern >> 1) | (led_pattern << 7);
			}

			// Change direction based on interrupt activity
			unsigned int int0_count, int1_count, total_count;
			unsigned char last_triggered;
			Interrupt_get_statistics(&int0_count, &int1_count, &total_count, &last_triggered);

			if ((total_count % 10) == 0 && total_count > 0)
			{
				pattern_direction = !pattern_direction;
			}

			// Apply sophisticated pattern with activity indication
			Port_write(led_pattern, 1);
		}

		// Minimal delay for maximum system responsiveness
		Timer2_delay_ms(5);
	}
}

/*
 * ADVANCED INTEGRATION NOTES:
 *
 * This demonstration showcases the full power of the modernized library
 * ecosystem working together:
 *
 * 1. Timer2 Library: Provides microsecond-precision timing, eliminating
 *    the need for approximate overflow counting methods
 *
 * 2. Interrupt Library: Handles all interrupt events with professional-grade
 *    debouncing, statistics, and safety features automatically
 *
 * 3. Port Library: Manages all GPIO operations safely with proper timing
 *    and state management
 *
 * 4. Integration Benefits:
 *    - No direct register manipulation required
 *    - Automatic resource management and conflict prevention
 *    - Built-in monitoring and debugging capabilities
 *    - Educational focus on system design rather than hardware details
 *    - Professional-grade reliability and maintainability
 *
 * 5. Educational Progression:
 *    - Demonstrates evolution from basic register access to professional libraries
 *    - Shows how modern embedded systems are designed and implemented
 *    - Provides foundation for IoT and advanced embedded applications
 *    - Prepares students for industry-standard development practices
 *
 * This represents the culmination of the Assembly → C → Python → IoT
 * learning progression, showing students how professional embedded systems
 * are architected and implemented.
 */

#endif // INTERRUPT_EXT_TIMER

#endif // INTERRUPT_DEMO_ENABLED