/*
 * config.h - Serial Polling Single Character Project Configuration
 *
 * PURPOSE: Basic UART communication with single character echo
 * LEARNING: Serial communication setup, polling vs interrupts
 * HARDWARE: UART1 connected to computer via USB/serial adapter
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// Core system configuration
#define F_CPU 16000000UL // 16 MHz crystal oscillator
#define BAUD 9600        // Standard baud rate for education

// Standard AVR includes
#include <avr/io.h>
#include <avr/interrupt.h>

// Project-specific library includes
#include "../../shared_libs/_port.h"
#include "../../shared_libs/_uart.h"
#include "../../shared_libs/_init.h"

#endif /* CONFIG_H_ */