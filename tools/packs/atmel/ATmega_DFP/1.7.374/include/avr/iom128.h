#ifndef _AVR_ATmega128_H_
#define _AVR_ATmega128_H_ 1

/* ATmega128 Device Definitions */
#ifndef __ASSEMBLER__
#include <stdint.h>

/* Memory layout */
#define RAMSTART 0x0100
#define RAMSIZE 4096
#define RAMEND (RAMSTART + RAMSIZE - 1)

#define XRAMSTART 0x1100
#define XRAMSIZE 61440
#define XRAMEND (XRAMSTART + XRAMSIZE - 1)

#define E2START 0
#define E2SIZE 4096
#define E2END (E2SIZE - 1)

#define FLASHEND 0x1FFFF

/* Standard AVR include */
#include <avr/iom128.h>

#endif /* __ASSEMBLER__ */
#endif /* _AVR_ATmega128_H_ */