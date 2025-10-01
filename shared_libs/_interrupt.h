/*
 * =============================================================================
 * EDUCATIONAL ATmega128 INTERRUPT MANAGEMENT LIBRARY - HEADER FILE
 * =============================================================================
 *
 * COURSE: SOC 3050 - Embedded Systems and IoT
 * AUTHOR: Professor Kim
 *
 * PURPOSE:
 * Educational header for ATmega128 external interrupt management. This library
 * provides comprehensive interrupt functionality with safety features and
 * educational documentation for learning event-driven programming.
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

#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

#include <avr/io.h>
#include <avr/interrupt.h>

/*
 * =============================================================================
 * INTERRUPT HARDWARE CONSTANTS AND DEFINITIONS
 * =============================================================================
 */

/* ATmega128 External Interrupt Pin Assignments */
#define INTERRUPT_PIN_INT0 PIND0 // External Interrupt 0 (PD0)
#define INTERRUPT_PIN_INT1 PIND1 // External Interrupt 1 (PD1)
#define INTERRUPT_PIN_INT2 PIND2 // External Interrupt 2 (PD2)
#define INTERRUPT_PIN_INT3 PIND3 // External Interrupt 3 (PD3)
#define INTERRUPT_PIN_INT4 PINE4 // External Interrupt 4 (PE4)
#define INTERRUPT_PIN_INT5 PINE5 // External Interrupt 5 (PE5)
#define INTERRUPT_PIN_INT6 PINE6 // External Interrupt 6 (PE6)
#define INTERRUPT_PIN_INT7 PINE7 // External Interrupt 7 (PE7)

/* Interrupt Trigger Mode Constants */
#define INTERRUPT_TRIGGER_LOW_LEVEL 0    // Low level generates interrupt
#define INTERRUPT_TRIGGER_RESERVED 1     // Reserved (do not use)
#define INTERRUPT_TRIGGER_FALLING_EDGE 2 // Falling edge generates interrupt
#define INTERRUPT_TRIGGER_RISING_EDGE 3  // Rising edge generates interrupt

/* Interrupt Priority Levels (hardware defined) */
#define INTERRUPT_PRIORITY_INT0 0 // Highest priority
#define INTERRUPT_PRIORITY_INT1 1
#define INTERRUPT_PRIORITY_INT2 2
#define INTERRUPT_PRIORITY_INT3 3
#define INTERRUPT_PRIORITY_INT4 4
#define INTERRUPT_PRIORITY_INT5 5
#define INTERRUPT_PRIORITY_INT6 6
#define INTERRUPT_PRIORITY_INT7 7 // Lowest priority

/*
 * =============================================================================
 * CORE INTERRUPT MANAGEMENT FUNCTIONS
 * =============================================================================
 */

/*
 * SYSTEM FUNCTION: Initialize External Interrupt System
 *
 * PURPOSE: Configure external interrupts with educational settings
 *
 * CONFIGURATION:
 * - INT0: Falling edge triggered (for button/switch input)
 * - INT1: Falling edge triggered (for secondary input)
 * - INT2-INT7: Disabled (for educational focus)
 * - Statistics: Enabled for monitoring and debugging
 *
 * EDUCATIONAL VALUE: Demonstrates proper interrupt initialization
 */
void Interrupt_init(void);

/*
 * CONTROL FUNCTION: Enable Global Interrupts
 *
 * PURPOSE: Activate interrupt processing system-wide
 *
 * OPERATION:
 * - Sets I-bit in Status Register (SREG)
 * - Enables all configured interrupt sources
 * - Must be called after Interrupt_init()
 *
 * EDUCATIONAL VALUE: Shows interrupt activation sequence
 */
void Interrupt_enable_global(void);

/*
 * CONTROL FUNCTION: Disable Global Interrupts
 *
 * PURPOSE: Temporarily disable all interrupts for critical sections
 *
 * USAGE:
 * - Protect critical code sections
 * - Ensure atomic operations on shared data
 * - Always re-enable after critical section
 *
 * EDUCATIONAL VALUE: Demonstrates critical section protection
 */
void Interrupt_disable_global(void);

/*
 * =============================================================================
 * MONITORING AND DIAGNOSTIC FUNCTIONS
 * =============================================================================
 */

/*
 * DIAGNOSTIC FUNCTION: Get Interrupt Statistics
 *
 * PURPOSE: Retrieve interrupt occurrence counts and status information
 *
 * PARAMETERS:
 *   int0_triggers - Pointer to receive INT0 count
 *   int1_triggers - Pointer to receive INT1 count
 *   total_triggers - Pointer to receive total interrupt count
 *   last_triggered - Pointer to receive last interrupt source
 *
 * EDUCATIONAL VALUE: System monitoring and performance analysis
 */
void Interrupt_get_statistics(unsigned int *int0_triggers,
                              unsigned int *int1_triggers,
                              unsigned int *total_triggers,
                              unsigned char *last_triggered);

/*
 * UTILITY FUNCTION: Reset Interrupt Statistics
 *
 * PURPOSE: Clear all interrupt counters for fresh monitoring period
 *
 * OPERATION:
 * - Atomically clears all statistical counters
 * - Preserves interrupt configuration
 * - Useful for periodic monitoring
 *
 * EDUCATIONAL VALUE: Demonstrates atomic data manipulation
 */
void Interrupt_reset_statistics(void);

/*
 * =============================================================================
 * INTERRUPT SERVICE ROUTINE DECLARATIONS
 * =============================================================================
 *
 * PURPOSE: External declarations for interrupt handlers
 *
 * EDUCATIONAL NOTES:
 * - ISRs are automatically called by hardware
 * - Vector names are predefined by avr-gcc
 * - ISR implementation is in the .c file
 * - No manual calling of ISRs required
 */

/*
 * ISR DECLARATION: External Interrupt 0 Handler
 *
 * VECTOR: INT0_vect (highest priority external interrupt)
 * TRIGGER: Falling edge on PD0
 * PURPOSE: Handle primary external events (buttons, sensors)
 */

/*
 * ISR DECLARATION: External Interrupt 1 Handler
 *
 * VECTOR: INT1_vect (second priority external interrupt)
 * TRIGGER: Falling edge on PD1
 * PURPOSE: Handle secondary external events
 */

/*
 * =============================================================================
 * EDUCATIONAL USAGE EXAMPLES AND LEARNING OBJECTIVES
 * =============================================================================
 *
 * BASIC INTERRUPT SETUP EXAMPLE:
 *   Interrupt_init();                    // Configure interrupt system
 *   Interrupt_enable_global();           // Enable interrupt processing
 *   // Main program continues...
 *   // Interrupts handled automatically by ISRs
 *
 * STATISTICS MONITORING EXAMPLE:
 *   unsigned int int0_count, int1_count, total_count;
 *   unsigned char last_int;
 *   Interrupt_get_statistics(&int0_count, &int1_count, &total_count, &last_int);
 *   printf("INT0: %u, INT1: %u, Total: %u\n", int0_count, int1_count, total_count);
 *
 * CRITICAL SECTION EXAMPLE:
 *   Interrupt_disable_global();         // Enter critical section
 *   shared_variable++;                  // Atomic operation
 *   Interrupt_enable_global();          // Exit critical section
 *
 * LEARNING OBJECTIVES ACHIEVED:
 * 1. ✓ Interrupt hardware configuration and control
 * 2. ✓ Event-driven programming concepts
 * 3. ✓ Critical section protection techniques
 * 4. ✓ Real-time system response mechanisms
 * 5. ✓ System monitoring and debugging methods
 * 6. ✓ Priority-based event handling
 * 7. ✓ Safe interrupt programming practices
 * 8. ✓ Integration with other system components
 *
 * =============================================================================
 */

#endif /* _INTERRUPT_H_ */
