/*
 * Configuration Header - ADC Basic Reading
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 7372800UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>  // For sprintf
#include <stdint.h> // For uint16_t types

// Include shared library headers
#include "_adc.h"
#include "_uart.h"
#include "_init.h"

#endif /* CONFIG_H_ */