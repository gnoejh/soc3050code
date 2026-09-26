/*
 * spi.h - SPI1 master, 16-bit frames, for the MAX7219   (SOC3050 lesson 09)
 *   SCK PA5 (D13) - which is also LD4, so LD4 flickers with the clock
 *   MOSI PA7 (D11), chip select PB0 (D10) as a plain GPIO
 */
#ifndef SPI_H
#define SPI_H

#include <stdint.h>

void spi_init(void);
int  spi_send16(uint16_t frame);     /* CS low, 16 bits out, CS high; 0 = ok */

#endif
