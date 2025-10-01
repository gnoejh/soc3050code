/*
 * Configuration Header - Assembly Blink Full
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 7372800UL

#include <avr/io.h>
#include <util/delay.h>

// External function declaration
extern void main_blink_asm(void);

#endif /* CONFIG_H_ */