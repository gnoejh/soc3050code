/*
 * =============================================================================
 * EDUCATIONAL ATmega128 INTERRUPT MANAGEMENT LIBRARY - IMPLEMENTATION
 * =============================================================================
 *
 * COURSE: SOC 3050 - Embedded Systems and IoT
 * AUTHOR: Professor Kim
 *
 * PURPOSE:
 * Comprehensive interrupt management library for ATmega128 external interrupts.
 * This implementation provides educational framework for learning interrupt
 * programming, event-driven systems, and real-time response mechanisms.
 *
 * EDUCATIONAL OBJECTIVES:
 * 1. Master interrupt concepts and hardware mechanisms
 * 2. Understand interrupt vectors and priority systems
 * 3. Implement safe interrupt programming practices
 * 4. Learn event-driven programming paradigms
 * 5. Explore real-time response and timing considerations
 *
 * HARDWARE FEATURES:
 * - 8 External interrupts (INT0-INT7) on ATmega128
 * - Configurable trigger modes (level, edge)
 * - Interrupt priority levels and vector handling
 * - Asynchronous and synchronous operation modes
 *
 * LEARNING PROGRESSION:
 * Assembly → C → Python → IoT
 * Direct vectors → Structured handlers → Event systems → Message queues
 *
 * =============================================================================
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
#include "_interrupt.h"
#include "_uart.h"

/*
 * =============================================================================
 * EDUCATIONAL CONSTANTS AND INTERRUPT CONFIGURATION
 * =============================================================================
 */

/* ATmega128 External Interrupt Pin Assignments */
#define EXT_INT0_PIN PIND0 // External Interrupt 0 (PD0)
#define EXT_INT1_PIN PIND1 // External Interrupt 1 (PD1)
#define EXT_INT2_PIN PIND2 // External Interrupt 2 (PD2)
#define EXT_INT3_PIN PIND3 // External Interrupt 3 (PD3)
#define EXT_INT4_PIN PINE4 // External Interrupt 4 (PE4)
#define EXT_INT5_PIN PINE5 // External Interrupt 5 (PE5)
#define EXT_INT6_PIN PINE6 // External Interrupt 6 (PE6)
#define EXT_INT7_PIN PINE7 // External Interrupt 7 (PE7)

/* Interrupt Trigger Mode Definitions */
#define INT_TRIGGER_LOW_LEVEL 0	   // Low level generates interrupt
#define INT_TRIGGER_RESERVED 1	   // Reserved (do not use)
#define INT_TRIGGER_FALLING_EDGE 2 // Falling edge generates interrupt
#define INT_TRIGGER_RISING_EDGE 3  // Rising edge generates interrupt

/* Educational interrupt statistics */
static volatile unsigned int int0_count = 0;	   // INT0 trigger count
static volatile unsigned int int1_count = 0;	   // INT1 trigger count
static volatile unsigned int total_interrupts = 0; // Total interrupt count
static volatile unsigned char last_interrupt = 0;  // Last triggered interrupt

/*
 * =============================================================================
 * EDUCATIONAL FUNCTION: Initialize External Interrupt System
 * =============================================================================
 *
 * PURPOSE: Configure external interrupts with educational settings
 *
 * HARDWARE CONFIGURATION:
 * 1. EICRA (External Interrupt Control Register A): Controls INT0-INT3
 *    - Bits 1:0 (ISC01:ISC00): INT0 trigger mode
 *    - Bits 3:2 (ISC11:ISC10): INT1 trigger mode
 *    - Bits 5:4 (ISC21:ISC20): INT2 trigger mode
 *    - Bits 7:6 (ISC31:ISC30): INT3 trigger mode
 *
 * 2. EICRB (External Interrupt Control Register B): Controls INT4-INT7
 *    - Bits 1:0 (ISC41:ISC40): INT4 trigger mode
 *    - Bits 3:2 (ISC51:ISC50): INT5 trigger mode
 *    - Bits 5:4 (ISC61:ISC60): INT6 trigger mode
 *    - Bits 7:6 (ISC71:ISC70): INT7 trigger mode
 *
 * 3. EIMSK (External Interrupt Mask Register): Enable/disable interrupts
 *    - Bit n: Enable INTn interrupt (1=enabled, 0=disabled)
 *
 * 4. SREG (Status Register): Global interrupt enable
 *    - Bit 7 (I): Global interrupt enable flag
 *
 * TRIGGER MODE ENCODING:
 * ISCn1 ISCn0 | Description
 *   0     0   | Low level generates interrupt request
 *   0     1   | Reserved (do not use)
 *   1     0   | Falling edge generates interrupt request (asynchronous)
 *   1     1   | Rising edge generates interrupt request (asynchronous)
 *
 * EDUCATIONAL NOTES:
 * - Level-triggered interrupts require pin to remain in trigger state
 * - Edge-triggered interrupts respond to state transitions
 * - Asynchronous interrupts can wake CPU from sleep modes
 * - Priority: INT0 (highest) to INT7 (lowest)
 */
void Interrupt_init(void)
{
	/* Clear interrupt statistics */
	int0_count = 0;
	int1_count = 0;
	total_interrupts = 0;
	last_interrupt = 0;

	/*
	 * EDUCATIONAL STEP 1: Configure interrupt trigger modes
	 *
	 * EICRA configuration for INT0 and INT1:
	 * - INT0 (bits 1:0): Falling edge triggered (ISC01=1, ISC00=0)
	 * - INT1 (bits 3:2): Falling edge triggered (ISC11=1, ISC10=0)
	 * - INT2 and INT3: Not used (bits remain 0)
	 *
	 * Binary: 00001010 = 0x0A
	 */
	EICRA = (INT_TRIGGER_FALLING_EDGE << ISC01) | // INT0: falling edge
			(INT_TRIGGER_FALLING_EDGE << ISC11);  // INT1: falling edge

	/*
	 * EDUCATIONAL STEP 2: Configure higher interrupt pins (INT4-INT7)
	 *
	 * EICRB configuration:
	 * - All interrupts INT4-INT7 disabled for educational focus
	 * - Can be enabled later for advanced applications
	 */
	EICRB = 0x00; // INT4-INT7 unused in basic educational examples

	/*
	 * EDUCATIONAL STEP 3: Enable specific external interrupts
	 *
	 * EIMSK (External Interrupt Mask Register):
	 * - Bit 0 (INT0): Enable external interrupt 0
	 * - Bit 1 (INT1): Enable external interrupt 1
	 * - Other bits: Disabled for educational focus
	 */
	EIMSK = (1 << INT0) | (1 << INT1); // Enable INT0 and INT1

	/*
	 * EDUCATIONAL NOTE:
	 * Global interrupts are enabled separately using sei() or by setting
	 * the I-bit in SREG. This allows controlled interrupt activation.
	 */
}

/*
 * =============================================================================
 * EDUCATIONAL FUNCTION: Enable Global Interrupts
 * =============================================================================
 *
 * PURPOSE: Enable global interrupt processing with educational documentation
 *
 * HARDWARE OPERATION:
 * - Sets I-bit in Status Register (SREG)
 * - Enables interrupt vector execution
 * - Must be called after Interrupt_init() to activate interrupts
 */
void Interrupt_enable_global(void)
{
	sei(); // Set global interrupt enable flag

	/*
	 * EDUCATIONAL NOTE:
	 * After calling sei(), the processor can respond to:
	 * 1. Enabled external interrupts (INT0, INT1, etc.)
	 * 2. Timer/counter interrupts
	 * 3. UART interrupts
	 * 4. ADC completion interrupts
	 * 5. Other peripheral interrupts
	 */
}

/*
 * EDUCATIONAL FUNCTION: Disable Global Interrupts
 *
 * PURPOSE: Temporarily disable all interrupts for critical sections
 *
 * USAGE: Protect critical code sections from interrupt interference
 */
void Interrupt_disable_global(void)
{
	cli(); // Clear global interrupt enable flag

	/*
	 * EDUCATIONAL NOTE:
	 * Use cli() sparingly and for short durations:
	 * - Critical sections should be minimal
	 * - Long disabled periods affect real-time response
	 * - Always re-enable interrupts after critical section
	 */
}

/*
 * EDUCATIONAL FUNCTION: Get Interrupt Statistics
 *
 * PURPOSE: Provide interrupt occurrence counts for debugging and analysis
 * PARAMETERS: Pointers to receive statistical data
 */
void Interrupt_get_statistics(unsigned int *int0_triggers,
							  unsigned int *int1_triggers,
							  unsigned int *total_triggers,
							  unsigned char *last_triggered)
{
	/* Temporarily disable interrupts to ensure atomic read */
	unsigned char sreg_backup = SREG;
	cli();

	*int0_triggers = int0_count;
	*int1_triggers = int1_count;
	*total_triggers = total_interrupts;
	*last_triggered = last_interrupt;

	/* Restore interrupt state */
	SREG = sreg_backup;
}

/*
 * EDUCATIONAL FUNCTION: Reset Interrupt Statistics
 *
 * PURPOSE: Clear interrupt counters for fresh monitoring period
 */
void Interrupt_reset_statistics(void)
{
	unsigned char sreg_backup = SREG;
	cli();

	int0_count = 0;
	int1_count = 0;
	total_interrupts = 0;
	last_interrupt = 0;

	SREG = sreg_backup;
}

/*
 * =============================================================================
 * EDUCATIONAL INTERRUPT SERVICE ROUTINES (ISRs)
 * =============================================================================
 *
 * PURPOSE: Demonstrate proper ISR implementation with educational features
 *
 * ISR DESIGN PRINCIPLES:
 * 1. Keep ISRs short and fast
 * 2. Avoid blocking operations (delays, loops)
 * 3. Use volatile variables for shared data
 * 4. Minimize floating-point operations
 * 5. Consider interrupt nesting and priorities
 */

/*
 * EDUCATIONAL ISR: External Interrupt 0 Handler
 *
 * PURPOSE: Handle INT0 events with educational statistics tracking
 * TRIGGER: Falling edge on PD0 (configured in Interrupt_init)
 *
 * VECTOR: INT0_vect (highest priority external interrupt)
 */
ISR(INT0_vect)
{
	/* Update statistics atomically */
	int0_count++;
	total_interrupts++;
	last_interrupt = 0; // Indicate INT0 was last triggered

/* Optional: Send notification via UART for debugging */
#ifdef INTERRUPT_DEBUG_UART
	putch_USART1('A'); // Send 'A' to indicate INT0 triggered
#endif

	/*
	 * EDUCATIONAL NOTE:
	 * This ISR demonstrates:
	 * - Minimal processing time
	 * - Statistical data collection
	 * - Optional debugging output
	 * - Proper variable access (volatile globals)
	 */
}

/*
 * EDUCATIONAL ISR: External Interrupt 1 Handler
 *
 * PURPOSE: Handle INT1 events with educational statistics tracking
 * TRIGGER: Falling edge on PD1 (configured in Interrupt_init)
 *
 * VECTOR: INT1_vect (second priority external interrupt)
 */
ISR(INT1_vect)
{
	/* Update statistics atomically */
	int1_count++;
	total_interrupts++;
	last_interrupt = 1; // Indicate INT1 was last triggered

/* Optional: Send notification via UART for debugging */
#ifdef INTERRUPT_DEBUG_UART
	putch_USART1('B'); // Send 'B' to indicate INT1 triggered
#endif

	/*
	 * EDUCATIONAL NOTE:
	 * Multiple interrupt sources can be handled similarly:
	 * - Each ISR updates relevant statistics
	 * - Common pattern for event counting
	 * - Enables system monitoring and debugging
	 */
}

/*
 * =============================================================================
 * EDUCATIONAL SUMMARY AND LEARNING OBJECTIVES
 * =============================================================================
 *
 * This interrupt library demonstrates:
 *
 * 1. INTERRUPT CONCEPTS:
 *    - Hardware interrupt mechanisms and triggers
 *    - Interrupt vectors and priority levels
 *    - Event-driven programming paradigms
 *
 * 2. SAFE PROGRAMMING PRACTICES:
 *    - Atomic operations for shared data
 *    - Critical section protection
 *    - Minimal ISR processing time
 *
 * 3. REAL-TIME CONSIDERATIONS:
 *    - Interrupt latency and response time
 *    - Priority-based scheduling
 *    - System monitoring and statistics
 *
 * 4. PRACTICAL APPLICATIONS:
 *    - Button press detection
 *    - Emergency stop mechanisms
 *    - External sensor event handling
 *    - Communication protocol synchronization
 *
 * LEARNING PROGRESSION:
 * - Assembly: Direct vector programming and register manipulation
 * - C: Structured interrupt handling with safety features
 * - Python: Event-driven systems and callback mechanisms
 * - IoT: Message queues and distributed event processing
 *
 * =============================================================================
 */