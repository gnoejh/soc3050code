/*
 * Configuration Header - Timer1 Input Capture
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 16000000UL // 16 MHz crystal
#define BAUD 9600

// AVR Standard Libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdio.h>

// Include shared library headers
#include "../../shared_libs/_uart.h"
#include "../../shared_libs/_init.h"

#endif /* CONFIG_H_ */
