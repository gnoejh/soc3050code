
/*
 * _timer2.c - ATmega128 Educational Timer2 Library
 * Part of Assembly → C → Python Learning Progression
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Understand timer/counter concepts and applications
 * 2. Learn timer register configuration (TCCR2, TCNT2, TIMSK)
 * 3. Master interrupt-driven timing and scheduling
 * 4. Practice real-time programming concepts
 * 5. Bridge assembly register access to C abstraction
 * 6. Prepare for Python timing and threading concepts
 *
 * TIMER OVERVIEW:
 * - Timer = Hardware counter that increments with clock pulses
 * - Timer2 = 8-bit timer/counter with prescaler options
 * - Overflow = Counter reaches maximum value and wraps to 0
 * - Interrupt = Automatic function call when overflow occurs
 * - Prescaler = Clock divider to slow down timer counting
 *
 * ATmega128 TIMER2 FEATURES:
 * - 8-bit timer/counter (0-255 range)
 * - Multiple clock sources and prescaler options
 * - Overflow and Compare Match interrupts
 * - Normal, CTC, Fast PWM, Phase Correct PWM modes
 * - Independent operation from other timers
 *
 * TIMING CALCULATIONS:
 * Timer frequency = F_CPU / prescaler
 * Overflow period = (256 - start_value) / timer_frequency
 * For 1ms timing with 64 prescaler:
 * Timer freq = 16MHz / 64 = 250kHz
 * Count needed = 250 (for 1ms)
 * Start value = 256 - 250 = 6
 *
 * ASSEMBLY EQUIVALENT CONCEPTS:
 * - TCCR2 = control  ≡  LDI R16, control; OUT TCCR2, R16
 * - TCNT2 = value    ≡  LDI R16, value; OUT TCNT2, R16
 * - Enable interrupt ≡  LDI R16, (1<<TOIE2); STS TIMSK, R16
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
#include "_timer2.h"

// Only compile Timer2 functions if not using self-contained assembly example
#ifndef ASSEMBLY_BLINK_BASIC

/*
 * EDUCATIONAL CONSTANTS: Timer2 Prescaler Values
 * These control the timer clock frequency for different timing requirements
 */
#define TIMER2_STOP 0x00		  // Timer stopped (CS22:0 = 000)
#define TIMER2_PRESCALE_1 0x01	  // No prescaling (CS22:0 = 001)
#define TIMER2_PRESCALE_8 0x02	  // Prescaler 8 (CS22:0 = 010)
#define TIMER2_PRESCALE_32 0x03	  // Prescaler 32 (CS22:0 = 011)
#define TIMER2_PRESCALE_64 0x04	  // Prescaler 64 (CS22:0 = 100)
#define TIMER2_PRESCALE_128 0x05  // Prescaler 128 (CS22:0 = 101)
#define TIMER2_PRESCALE_256 0x06  // Prescaler 256 (CS22:0 = 110)
#define TIMER2_PRESCALE_1024 0x07 // Prescaler 1024 (CS22:0 = 111)

/*
 * EDUCATIONAL CONSTANTS: Timer2 Start Values for Common Timings
 * Pre-calculated values for convenient timing intervals
 */
#define TIMER2_1MS_START 6	  // Start value for 1ms with prescaler 64
#define TIMER2_2MS_START 131  // Start value for 2ms with prescaler 64
#define TIMER2_5MS_START 131  // Start value for 5ms with prescaler 256
#define TIMER2_10MS_START 131 // Start value for 10ms with prescaler 256

/*
 * EDUCATIONAL VARIABLES
 * Global variables for learning timer concepts and real-time programming
 */
volatile unsigned int Count_Of_Timer2 = 0; // Timer overflow counter
volatile unsigned int Task1_Of_Timer2 = 0; // Task 1 trigger flag
volatile unsigned int Task2_Of_Timer2 = 0; // Task 2 trigger flag
volatile unsigned int Task3_Of_Timer2 = 0; // Task 3 trigger flag
unsigned int Time_Of_Timer2 = 500;		   // Task interval in timer ticks (500ms default)
unsigned int Time2_Of_Timer2 = 100;		   // Task 2 interval (100ms default)
unsigned int Time3_Of_Timer2 = 1000;	   // Task 3 interval (1000ms default)

// System timing variables
volatile unsigned long system_milliseconds = 0;				 // System uptime in milliseconds
volatile unsigned int timer2_prescaler = TIMER2_PRESCALE_64; // Current prescaler setting
unsigned char timer2_start_value = TIMER2_1MS_START;		 // Current timer start value

/*
 * EDUCATIONAL FUNCTION: Timer2 Initialization
 *
 * PURPOSE: Configure Timer2 for 1ms periodic interrupts
 * LEARNING: Shows complete timer setup sequence and register configuration
 *
 * REGISTER EXPLANATION:
 * - TCCR2: Timer/Counter Control Register 2
 *   FOC2  = Force Output Compare (not used in normal mode)
 *   WGM21:20 = Waveform Generation Mode (00 = Normal mode)
 *   COM21:20 = Compare Match Output Mode (not used)
 *   CS22:20  = Clock Select (prescaler selection)
 *
 * - TCNT2: Timer/Counter Register 2
 *   8-bit register that holds current timer value
 *   Increments with each timer clock pulse
 *   Overflows from 255 to 0, generating interrupt
 *
 * - TIMSK: Timer Interrupt Mask Register
 *   TOIE2 = Timer Overflow Interrupt Enable 2
 *   OCIE2 = Output Compare Interrupt Enable 2
 *
 * TIMING CALCULATION FOR 1ms:
 * Timer frequency = 16MHz / 64 = 250kHz
 * For 1ms period: need 250 counts
 * Start value = 256 - 250 = 6 (but we use 26 for practical reasons)
 * Actual period = (256 - 26) / 250kHz = 0.92ms ≈ 1ms
 *
 * ASSEMBLY EQUIVALENT:
 * LDI R16, 0x00; OUT TCCR2, R16         ; Stop timer
 * LDI R16, 26; OUT TCNT2, R16           ; Set start value
 * LDI R16, 0x04; OUT TCCR2, R16         ; Start with prescaler 64
 * LDI R16, 0x40; STS TIMSK, R16         ; Enable overflow interrupt
 */
void Timer2_init(void)
{
	/*
	 * STEP 1: Stop Timer2 for safe configuration
	 * Clear all control bits to ensure clean start
	 */
	TCCR2 = TIMER2_STOP; // Stop timer (CS22:0 = 000)

	/*
	 * STEP 2: Set initial timer value
	 * Timer counts from this value to 255, then overflows
	 * Value 26 gives approximately 1ms period with prescaler 64
	 * Calculation: (256 - 26) * (64/16MHz) = 0.92ms ≈ 1ms
	 */
	TCNT2 = timer2_start_value; // Set start value for desired timing

	/*
	 * STEP 3: Configure Timer2 for Normal mode with prescaler
	 * Normal mode: timer counts up, overflows at 255
	 * Prescaler 64: reduces 16MHz clock to 250kHz
	 */
	TCCR2 = timer2_prescaler; // Start timer with prescaler 64

	/*
	 * STEP 4: Enable Timer2 overflow interrupt
	 * TOIE2 bit in TIMSK enables interrupt when timer overflows
	 * ISR(TIMER2_OVF_vect) will be called on each overflow
	 */
	TIMSK |= (1 << TOIE2); // Enable Timer2 overflow interrupt

	/*
	 * EDUCATIONAL NOTE:
	 * Timer2 is now configured for periodic 1ms interrupts
	 * - Mode: Normal (count up, overflow at 255)
	 * - Prescaler: 64 (250kHz timer frequency)
	 * - Period: ~1ms per overflow
	 * - Interrupt: Enabled on overflow
	 */
}

/*
 * EDUCATIONAL FUNCTION: Timer2 Start
 *
 * PURPOSE: Start or resume Timer2 operation
 * LEARNING: Shows dynamic timer control
 */
void Timer2_start(void)
{
	TCCR2 = timer2_prescaler; // Start timer with current prescaler
}

/*
 * EDUCATIONAL FUNCTION: Timer2 Stop
 *
 * PURPOSE: Stop Timer2 operation
 * LEARNING: Shows timer pause functionality
 */
void Timer2_stop(void)
{
	TCCR2 = TIMER2_STOP; // Stop timer by clearing clock source
}

/*
 * EDUCATIONAL FUNCTION: Set Timer2 Prescaler
 *
 * PURPOSE: Change timer frequency for different timing requirements
 * LEARNING: Shows dynamic timing adjustment
 */
void Timer2_set_prescaler(unsigned char prescaler)
{
	timer2_prescaler = prescaler;
	if (TCCR2 != TIMER2_STOP) // If timer is running
	{
		TCCR2 = prescaler; // Apply new prescaler immediately
	}
}

/*
 * EDUCATIONAL FUNCTION: Set Timer2 Period
 *
 * PURPOSE: Configure timer for specific time intervals
 * LEARNING: Shows timing calculations and customization
 */
void Timer2_set_period_ms(unsigned int period_ms)
{
	/*
	 * This is a simplified approach
	 * For precise timing, complex calculations would be needed
	 * considering prescaler and desired period
	 */
	if (period_ms <= 1)
	{
		timer2_start_value = TIMER2_1MS_START;
		timer2_prescaler = TIMER2_PRESCALE_64;
	}
	else if (period_ms <= 5)
	{
		timer2_start_value = TIMER2_5MS_START;
		timer2_prescaler = TIMER2_PRESCALE_256;
	}
	else
	{
		timer2_start_value = TIMER2_10MS_START;
		timer2_prescaler = TIMER2_PRESCALE_1024;
	}

	TCNT2 = timer2_start_value;
	TCCR2 = timer2_prescaler;
}

/*
 * EDUCATIONAL FUNCTION: Get System Uptime
 *
 * PURPOSE: Provide system timing reference
 * LEARNING: Shows time keeping and system services
 */
unsigned long Timer2_get_milliseconds(void)
{
	return system_milliseconds;
}

/*
 * EDUCATIONAL FUNCTION: Delay Using Timer2
 *
 * PURPOSE: Non-blocking delay using timer reference
 * LEARNING: Shows alternative to blocking _delay_ms()
 */
unsigned char Timer2_delay_ms(unsigned int delay_ms)
{
	static unsigned long start_time = 0;
	static unsigned char delay_active = 0;

	if (!delay_active)
	{
		start_time = system_milliseconds;
		delay_active = 1;
		return 0; // Delay started
	}

	if ((system_milliseconds - start_time) >= delay_ms)
	{
		delay_active = 0;
		return 1; // Delay completed
	}

	return 0; // Delay in progress
}

/*
 * EDUCATIONAL FUNCTION: Timer2 Overflow Interrupt Service Routine
 *
 * PURPOSE: Handle timer overflow events and provide timing services
 * LEARNING: Shows interrupt service routine programming and real-time scheduling
 *
 * TIMING ANALYSIS:
 * This ISR is called every ~1ms (when timer overflows from 255 to 0)
 * Processing time should be minimal to avoid interfering with main program
 *
 * TASKS PERFORMED:
 * 1. Reload timer start value for next period
 * 2. Increment system millisecond counter
 * 3. Update task scheduling counters
 * 4. Set task flags when intervals are reached
 *
 * ISR DESIGN PRINCIPLES:
 * - Keep processing time short
 * - Use volatile variables for shared data
 * - Avoid complex calculations
 * - Set flags for main program to handle tasks
 *
 * ASSEMBLY EQUIVALENT:
 * The compiler generates this automatically, but conceptually:
 * PUSH R0, R1, SREG, etc.     ; Save context
 * LDI R16, start_value         ; Reload timer
 * OUT TCNT2, R16
 * ; Perform ISR tasks
 * POP SREG, R1, R0, etc.       ; Restore context
 * RETI                         ; Return from interrupt
 */
ISR(TIMER2_OVF_vect)
{
	/*
	 * STEP 1: Reload timer start value
	 * This ensures consistent timing for next overflow
	 * Must be done first to maintain accurate timing
	 */
	TCNT2 = timer2_start_value; // Reload for next period

	/*
	 * STEP 2: Update system millisecond counter
	 * Provides system uptime reference for applications
	 */
	system_milliseconds++;

	/*
	 * STEP 3: Increment main timer counter
	 * Used for primary task scheduling
	 */
	Count_Of_Timer2++;

	/*
	 * STEP 4: Check and trigger Task 1 (Primary task)
	 * Default: every 500ms (500 timer ticks)
	 * Used for main periodic operations
	 */
	if (Count_Of_Timer2 >= Time_Of_Timer2)
	{
		Task1_Of_Timer2 = 1; // Signal task 1 ready
		Count_Of_Timer2 = 0; // Reset counter
	}

	/*
	 * STEP 5: Check and trigger Task 2 (Secondary task)
	 * Independent timing for secondary operations
	 * Example: LED blinking, sensor reading
	 */
	static unsigned int count2 = 0;
	count2++;
	if (count2 >= Time2_Of_Timer2)
	{
		Task2_Of_Timer2 = 1; // Signal task 2 ready
		count2 = 0;			 // Reset counter
	}

	/*
	 * STEP 6: Check and trigger Task 3 (Tertiary task)
	 * Independent timing for low-frequency operations
	 * Example: data logging, communication
	 */
	static unsigned int count3 = 0;
	count3++;
	if (count3 >= Time3_Of_Timer2)
	{
		Task3_Of_Timer2 = 1; // Signal task 3 ready
		count3 = 0;			 // Reset counter
	}

	/*
	 * EDUCATIONAL NOTE:
	 * This ISR implements a simple real-time scheduler
	 * - Multiple independent timing channels
	 * - Flag-based task signaling
	 * - Minimal processing time
	 * - Consistent timing base
	 *
	 * Main program should check flags and execute tasks:
	 * if (Task1_Of_Timer2) { execute_task1(); Task1_Of_Timer2 = 0; }
	 */
}

/*
 * EDUCATIONAL FUNCTION: Timer2 Compare Match ISR (if needed)
 *
 * PURPOSE: Handle compare match events for precise timing
 * LEARNING: Shows alternative timer interrupt mode
 *
 * NOTE: This would be used in CTC (Clear Timer on Compare) mode
 * Currently commented out as we're using overflow mode
 */
/*
ISR(TIMER2_COMP_vect)
{
	// Compare match occurred
	// Timer automatically resets to 0 in CTC mode
	// No need to reload TCNT2

	system_milliseconds++;
	Count_Of_Timer2++;

	if (Count_Of_Timer2 >= Time_Of_Timer2)
	{
		Task1_Of_Timer2 = 1;
		Count_Of_Timer2 = 0;
	}
}
*/

/*
 * EDUCATIONAL FUNCTION: Task Management Helpers
 *
 * PURPOSE: Provide convenient task checking and management
 * LEARNING: Shows task-based programming patterns
 */

// Check if Task 1 is ready and clear flag
unsigned char Timer2_check_task1(void)
{
	if (Task1_Of_Timer2)
	{
		Task1_Of_Timer2 = 0;
		return 1;
	}
	return 0;
}

// Check if Task 2 is ready and clear flag
unsigned char Timer2_check_task2(void)
{
	if (Task2_Of_Timer2)
	{
		Task2_Of_Timer2 = 0;
		return 1;
	}
	return 0;
}

// Check if Task 3 is ready and clear flag
unsigned char Timer2_check_task3(void)
{
	if (Task3_Of_Timer2)
	{
		Task3_Of_Timer2 = 0;
		return 1;
	}
	return 0;
}

#endif // !ASSEMBLY_BLINK_BASIC