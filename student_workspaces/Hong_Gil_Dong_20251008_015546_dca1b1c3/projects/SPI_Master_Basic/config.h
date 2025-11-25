/*
 * Configuration Header - SPI Master Basic
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 16000000UL
#define BAUD 9600

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

// Include shared library headers
#include "_uart.h"

#endif /* CONFIG_H_ */
