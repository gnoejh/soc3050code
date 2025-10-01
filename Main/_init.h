

/*
 * _init.h - ATmega128 Educational Initialization Library Header
 * Part of Assembly → C → Python Learning Progression
 *
 * EDUCATIONAL PURPOSE:
 * Provides systematic initialization functions for ATmega128 peripherals
 * with clear learning objectives and modular design
 */

#ifndef _INIT_H_
#define _INIT_H_

/*
 * COMPLETE SYSTEM INITIALIZATION
 * Initializes all ATmega128 peripherals in proper sequence
 * Use for: Complete applications requiring all systems
 */
void init_devices(void);

/*
 * SELECTIVE INITIALIZATION FUNCTIONS
 * Initialize only specific peripheral groups for focused learning
 */
void init_basic_io(void);      // Initialize only basic I/O ports
void init_communication(void); // Initialize communication peripherals
void init_sensors(void);       // Initialize sensor-related peripherals
void init_display(void);       // Initialize display systems

/*
 * SYSTEM MANAGEMENT FUNCTIONS
 * Utilities for system status and control
 */
unsigned char check_init_status(void); // Verify initialization success
void reset_system(void);               // Reset system to known state

#endif /* _INIT_H_ */
