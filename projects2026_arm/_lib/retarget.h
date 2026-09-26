/*
 * retarget.h - printf to the serial monitor (USART2 on PA2/PA3, the Nucleo VCP)
 *
 * Link with LIBS=retarget.  Call uart2_init() once, then printf() just works.
 */
#ifndef RETARGET_H
#define RETARGET_H

#include <stdint.h>

void uart2_init(uint32_t pclk, uint32_t baud);

#endif
