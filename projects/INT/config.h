/*
 * Configuration Header for 14_Interrupt_Basic
 * ATmega128 Educational Framework
 */

#ifndef CONFIG_H
#define CONFIG_H

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "../../shared_libs/_port.h"
#include "../../shared_libs/_glcd.h"
#include "../../shared_libs/_init.h"

#endif
