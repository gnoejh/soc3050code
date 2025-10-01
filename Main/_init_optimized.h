/*
 * _init.h - ATmega128 System Initialization Library Header
 * Educational Version for Assembly→C→Python Learning Progression
 */

#ifndef _INIT_H_
#define _INIT_H_

/*
 * Master Initialization Functions
 */
void init_devices(void); // Initialize all systems (complete setup)

/*
 * Educational Subset Initialization Functions
 * These allow progressive learning by initializing only specific subsystems
 */
void init_basic_io(void);      // Basic I/O only (ports, no interrupts)
void init_communication(void); // Communication systems (UART)
void init_timing(void);        // Timing systems (timers, PWM)
void init_sensors(void);       // Sensor systems (ADC)
void init_display(void);       // Display systems (LCD, GLCD)

/*
 * System Status and Diagnostic Functions
 * Educational tools for understanding system state
 */
unsigned char get_interrupt_status(void); // Check if interrupts enabled (1=yes, 0=no)
void system_reset(void);                  // Software system reset
void wait_for_systems_ready(void);        // Wait for hardware stabilization

/*
 * Educational Constants for Initialization Phases
 * These help students understand initialization sequences
 */
#define INIT_PHASE_PORTS 1         // Phase 1: Port configuration
#define INIT_PHASE_INTERRUPTS 2    // Phase 2: Interrupt setup
#define INIT_PHASE_TIMERS 3        // Phase 3: Timer configuration
#define INIT_PHASE_COMMUNICATION 4 // Phase 4: UART setup
#define INIT_PHASE_SENSORS 5       // Phase 5: ADC setup
#define INIT_PHASE_DISPLAY 6       // Phase 6: LCD setup
#define INIT_PHASE_COMPLETE 7      // Phase 7: All systems ready

/*
 * System State Constants
 */
#define INTERRUPTS_DISABLED 0 // Interrupts are disabled
#define INTERRUPTS_ENABLED 1  // Interrupts are enabled

/*
 * Timing Constants for System Delays
 */
#define SYSTEM_STARTUP_DELAY_MS 100 // Standard startup delay
#define HARDWARE_SETTLE_DELAY_MS 50 // Hardware settling time
#define CRYSTAL_STARTUP_DELAY_MS 10 // Crystal oscillator startup

/*
 * EDUCATIONAL USAGE EXAMPLES:
 *
 * Basic I/O only (for register learning):
 *   init_basic_io();
 *   // Now safe to use PORTB, DDRA, etc.
 *
 * Communication learning:
 *   init_communication();
 *   puts_USART1("Hello World\r\n");
 *
 * Complete system:
 *   init_devices();
 *   // All systems ready for complex applications
 */

#endif // _INIT_H_