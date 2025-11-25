/*
 * Configuration Header - PWM Servo Motor Control
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 7372800UL
#define BAUD 9600

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>

// Include shared library headers
#include "_uart.h"
#include "_adc.h"
#include "_init.h"

#endif /* CONFIG_H_ */
