/*
 * config.h - Assembly Blink Basic Project Configuration
 *
 * PURPOSE: Simple LED blinking using assembly language
 * LEARNING: Direct register manipulation, basic I/O control
 * HARDWARE: LEDs connected to PORTB
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// Core system configuration
#ifndef F_CPU
#define F_CPU 7372800UL // 7.3728 MHz crystal oscillator
#endif

// Standard AVR includes
#include <avr/io.h>
#include <avr/interrupt.h>

// Project-specific library includes
#include "../../shared_libs/_port.h"
#include "../../shared_libs/_init.h"

#endif /* CONFIG_H_ */