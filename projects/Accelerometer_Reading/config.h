/*
 * Configuration Header - Accelerometer Reading
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 7372800UL

#include <avr/io.h>
#include <util/delay.h>

// Include shared library headers
#include "_adc.h"
#include "_uart.h"
#include "_init.h"

// External function declaration
extern void main_accelerometer(void);

#endif /* CONFIG_H_ */