/*
 * Configuration Header - Graphics Display
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 7372800UL
#define BAUD 9600

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <avr/interrupt.h>

// Include shared library headers from Main folder
#include "../../Main/_port.h"
#include "../../Main/_init.h"
#include "../../Main/_glcd.h"

#endif /* CONFIG_H_ */
