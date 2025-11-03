/*
 * Configuration Header - Timer Basic
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Include shared library headers
#include "_timer2.h"
#include "_init.h"

// External function declaration
extern void main_timer_normal(void);

#endif /* CONFIG_H_ */