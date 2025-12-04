/*
 * Configuration Header - Timer0 Overflow LED Blink
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 16000000UL
#define BAUD 9600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

// Include shared library headers
#include "_uart.h"
#include "_init.h"

#endif /* CONFIG_H_ */
