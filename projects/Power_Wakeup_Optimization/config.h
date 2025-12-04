/*
 * Configuration Header for Wake-up Optimization
 * ATmega128 Educational Framework
 * Note: Not using _uart.h due to ISR conflict
 */

#ifndef CONFIG_H
#define CONFIG_H

#define F_CPU 7372800UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#endif
