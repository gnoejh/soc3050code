/*
 * i2c.h - I2C1 master on PB8 (SCL) / PB9 (SDA)   (SOC3050 lesson 09)
 */
#ifndef I2C_H
#define I2C_H

#include <stddef.h>
#include <stdint.h>

enum { I2C_OK = 0, I2C_NACK = -1, I2C_TIMEOUT = -2 };

void i2c_init(void);

/* Write `wn` bytes, then (if rn > 0) read `rn` bytes, as one transaction
 * with a repeated START between - the shape of every register read. */
int  i2c_write_read(uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn);

/* Does anything answer at `addr`?  An address byte and nothing else. */
int  i2c_probe(uint8_t addr);

#endif
