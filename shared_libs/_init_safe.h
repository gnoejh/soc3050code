/*
 * =============================================================================
 * SAFE INITIALIZATION SYSTEM - Header File
 * =============================================================================
 * 
 * This header provides safe initialization functions that prevent conflicts
 * between different libraries when used together in projects.
 * 
 * =============================================================================
 */

#ifndef _INIT_SAFE_H_
#define _INIT_SAFE_H_

#include <stdint.h>

// System initialization flags
#define INIT_GLCD    0x01
#define INIT_UART    0x02
#define INIT_LEDS    0x04
#define INIT_ADC     0x08
#define INIT_ALL     0x0F

// Safe initialization functions
uint8_t check_port_conflict(uint8_t port, uint8_t pins);
uint8_t reserve_port_pins(uint8_t port, uint8_t pins);
void release_port_pins(uint8_t port, uint8_t pins);

// System-specific safe initialization
void init_safe_for_glcd(void);
void init_safe_for_uart(void);
void init_safe_for_leds(void);
void init_safe_for_adc(void);
void init_safe_combined(uint8_t systems);

// Resource management functions
uint8_t get_resource_usage(void);
uint8_t is_system_safe_for(uint8_t system_flags);

#endif /* _INIT_SAFE_H_ */
