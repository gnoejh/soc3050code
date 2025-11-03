/*
 * Configuration Header - Assembly (Inline Assembly Teaching Module)
 * ATmega128 Educational Framework
 * Inline Assembly Language Integration
 *
 * This project teaches INLINE ASSEMBLY concepts:
 * - Inline assembly syntax (__asm__ volatile)
 * - Input/output constraints
 * - Clobber lists
 * - Register allocation
 * - C and Assembly integration
 * - Performance optimization techniques
 * - Direct hardware manipulation
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#ifndef F_CPU
#define F_CPU 16000000UL // 16MHz crystal
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Include shared libraries for comparison demonstrations
#include "../../shared_libs/_uart.h"
#include "../../shared_libs/_init.h"

#endif /* CONFIG_H_ */
