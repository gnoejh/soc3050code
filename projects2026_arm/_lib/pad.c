/*
 * pad.c - the app board's controls   LIBS=pad adc
 *
 * Nothing new here: lesson 05's pull-up buttons and debouncer, lesson 09's
 * ADC.  It exists so the application lessons spend their time on the
 * application.
 */
#include "stm32c031xx.h"
#include "gpio.h"
#include "adc.h"
#include "pad.h"

#define SEL_PIN  3u        /* PB3 */
#define A_PIN    4u        /* PB4 */
#define B_PIN    5u        /* PB5 */
#define DEAD     8         /* dead zone, in -100..100 units: a released stick reads 0 */

static uint8_t adc_ok;
static uint8_t last_raw, stable;

int pad_init(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;
    const uint32_t buttons[] = {SEL_PIN, A_PIN, B_PIN};
    for (uint32_t i = 0; i < 3u; i++) {
        pin_mode(GPIOB, buttons[i], MODE_INPUT);
        pin_pull(GPIOB, buttons[i], PULL_UP);
    }
    pin_mode(GPIOA, 0u, MODE_ANALOG);
    pin_mode(GPIOA, 1u, MODE_ANALOG);
    int rc = adc_init();                        /* also sets PA4 analog */
    adc_ok = (rc == 0);
    return rc;
}

/* 0..4095 -> -100..100 around the middle, with a dead zone so a released
 * stick is exactly 0.  `invert` flips the axis. */
static int16_t axis(int32_t raw, int invert)
{
    if (raw < 0) { return 0; }                  /* ADC timeout: centred, not wild */
    int32_t v = ((raw - 2048) * 100) / 2048;
    if (invert) { v = -v; }
    if (v > -DEAD && v < DEAD) { return 0; }
    if (v > 100)  { v = 100; }
    if (v < -100) { v = -100; }
    return (int16_t)v;
}

void pad_read(pad_t *p)
{
    uint8_t raw = 0;
    if (!pin_read(GPIOB, A_PIN))   { raw |= PAD_A; }
    if (!pin_read(GPIOB, B_PIN))   { raw |= PAD_B; }
    if (!pin_read(GPIOB, SEL_PIN)) { raw |= PAD_SEL; }

    /* A bit changes state only when two reads in a row agree on it. */
    uint8_t agree = (uint8_t)~(raw ^ last_raw);
    uint8_t now   = (uint8_t)((stable & ~agree) | (raw & agree));
    last_raw = raw;

    p->pressed  = (uint8_t)(now & ~stable);
    p->released = (uint8_t)(stable & ~now);
    p->down     = now;
    stable      = now;

    p->adc_ok = adc_ok;
    if (adc_ok) {
        p->x    = axis(adc_read(ADC_CH_PA0), 1);    /* HORZ: 0 V is RIGHT in Wokwi */
        p->y    = axis(adc_read(ADC_CH_PA1), 0);
        int32_t k = adc_read(ADC_CH_PA4);
        p->knob = (int16_t)(k < 0 ? 0 : (k * 1000) / 4095);
    } else {
        p->x = p->y = p->knob = 0;
    }
}
