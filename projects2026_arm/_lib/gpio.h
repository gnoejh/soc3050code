/*
 * gpio.h - the pin helpers lesson 05 wrote, shared from lesson 06 on
 *
 * Header only: every function is static inline, so there is nothing to link
 * and nothing to add to LIBS.  #include "gpio.h" is enough.
 *
 * Each pin owns a 2-bit field in MODER and PUPDR at bit (pin * 2), a 4-bit
 * field in AFR[pin / 8] at bit ((pin % 8) * 4), and one bit in IDR, ODR and
 * BSRR.  Lesson 05, slide 4 has the table.  The port's clock gate in
 * RCC->IOPENR is NOT opened here - that is the caller's job, done once, first.
 */
#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include "stm32c031xx.h"

enum { MODE_INPUT = 0u, MODE_OUTPUT = 1u, MODE_AF = 2u, MODE_ANALOG = 3u };
enum { PULL_NONE  = 0u, PULL_UP     = 1u, PULL_DOWN = 2u };

static inline void pin_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode)
{
    port->MODER = (port->MODER & ~(3u << (pin * 2u))) | (mode << (pin * 2u));
}

static inline void pin_pull(GPIO_TypeDef *port, uint32_t pin, uint32_t pull)
{
    port->PUPDR = (port->PUPDR & ~(3u << (pin * 2u))) | (pull << (pin * 2u));
}

/* Hand the pin to a peripheral.  The AF number comes from the datasheet's
 * alternate-function table, and it is different for every pin: TIM3_CH1 is
 * AF1 on PA6 but AF12 on PB6.  A wrong number gives a pin that is electrically
 * fine and carries nothing - silent.  AFR is written before MODER so the pin
 * never spends a moment connected to the wrong peripheral. */
static inline void pin_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af)
{
    uint32_t shift = (pin % 8u) * 4u;
    port->AFR[pin / 8u] = (port->AFR[pin / 8u] & ~(0xFu << shift)) | (af << shift);
    pin_mode(port, pin, MODE_AF);
}

/* BSRR: one store, no read - lesson 05, slides 12-13. */
static inline void pin_high(GPIO_TypeDef *port, uint32_t pin) { port->BSRR = 1u << pin; }
static inline void pin_low (GPIO_TypeDef *port, uint32_t pin) { port->BSRR = 1u << (pin + 16u); }

static inline uint32_t pin_read(GPIO_TypeDef *port, uint32_t pin)
{
    return (port->IDR >> pin) & 1u;
}

/* Is this exception's vector still the weak default?  A strong handler with
 * the exact CMSIS name replaces it at link time; a misspelt one does not, and
 * nothing else will tell you (lesson 05, slide 30).  exc = 16 + IRQn. */
extern void (* const vectors[])(void);   /* startup.c */
void Default_Handler(void);              /* startup.c */

static inline int handler_installed(int32_t irqn)
{
    return vectors[16 + irqn] != Default_Handler;
}

#endif
