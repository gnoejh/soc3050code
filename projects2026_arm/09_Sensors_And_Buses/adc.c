/*
 * adc.c - the 12-bit SAR ADC   (SOC3050 lesson 09)
 *
 * The peripheral model again - clock gate, control, status, data - but with
 * something new: a START-UP SEQUENCE the hardware insists on, in order:
 *
 *   1. clock        RCC->APBENR2 ADCEN; ADC clock = PCLK/2 = 24 MHz (CFGR2)
 *   2. regulator    CR.ADVREGEN = 1, then wait 20 us for it to settle
 *   3. calibrate    CR.ADCAL = 1, wait for the hardware to clear it
 *   4. enable       CR.ADEN = 1, wait for ISR.ADRDY
 *
 * Skip a step or reorder two and the ADC still returns numbers - just wrong
 * ones, or none.  The delays and their values are ST's, from the C0 LL
 * driver: LL_ADC_DELAY_INTERNAL_REGUL_STAB_US = 20 and
 * LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES = 2.
 *
 * Every wait below has a limit.  A flag that never comes - a missing clock, a
 * simulator that does not model it - becomes an error code, not a hang.
 */
#include "stm32c031xx.h"
#include "gpio.h"
#include "adc.h"

/* How long is "never"?  The slowest step here - calibration - takes a few
 * microseconds, and a conversion about 7 us.  2000 polls is well over 100 us:
 * ten times the worst case, and still short.  The first version allowed
 * 200 000 (20 ms), and in a simulator without CCRDY every failed read then
 * cost a whole 20 ms period - the sensor task never slept, and starved every
 * task below it.  A timeout's length is part of the design, not a formality. */
#define SPIN_LIMIT  2000u

/* "At least `us` microseconds".  The ADC's delays are minimums - waiting
 * longer is harmless - so a deliberately generous busy loop is enough: each
 * pass of a volatile counter loop costs several cycles, so 16 passes are
 * comfortably over 1 us at 48 MHz.
 *
 * The first version timed this with SysTick's VAL register and looped until
 * enough counts had passed.  In a simulator that reported SysTick as enabled
 * but never counting, it never returned.  A delay that can hang on a wrong
 * assumption is the bug this file's other waits exist to prevent. */
static void delay_us(uint32_t us)
{
    for (volatile uint32_t i = 0; i < us * 16u; i++) { }
}

static int wait_set(volatile uint32_t *reg, uint32_t mask)
{
    for (uint32_t i = 0; i < SPIN_LIMIT; i++) { if (*reg & mask) { return 0; } }
    return -1;
}

static int wait_clear(volatile uint32_t *reg, uint32_t mask)
{
    for (uint32_t i = 0; i < SPIN_LIMIT; i++) { if (!(*reg & mask)) { return 0; } }
    return -1;
}

int adc_init(void)
{
    RCC->IOPENR  |= RCC_IOPENR_GPIOAEN;
    RCC->APBENR2 |= RCC_APBENR2_ADCEN;
    pin_mode(GPIOA, 4u, MODE_ANALOG);            /* PA4: disconnect the digital input */

    /* Synchronous clock, PCLK / 2 = 24 MHz.  CKMODE may only change while
     * the ADC is disabled - which, at this point, it is. */
    ADC1->CFGR2 = (ADC1->CFGR2 & ~ADC_CFGR2_CKMODE) | ADC_CFGR2_CKMODE_0;

    ADC1->CR |= ADC_CR_ADVREGEN;                 /* 2. regulator on ...           */
    delay_us(20);                                /*    ... and let it settle      */

    ADC1->CR |= ADC_CR_ADCAL;                    /* 3. calibrate                  */
    if (wait_clear(&ADC1->CR, ADC_CR_ADCAL)) { return -1; }
    delay_us(1);                                 /*    >= 2 ADC cycles before ADEN */

    ADC1->ISR = ADC_ISR_ADRDY;                   /* 4. enable: clear, set, wait   */
    ADC1->CR |= ADC_CR_ADEN;
    if (wait_set(&ADC1->ISR, ADC_ISR_ADRDY)) { return -2; }

    /* Sampling time: the longest, 160.5 ADC cycles = 6.7 us.  The internal
     * reference needs several microseconds to charge the sampling capacitor;
     * a potentiometer is happy with less, but one setting serves both. */
    ADC1->SMPR = (ADC1->SMPR & ~ADC_SMPR_SMP1) | ADC_SMPR_SMP1;
    return 0;
}

int32_t adc_read(uint32_t channel)
{
    if (channel == ADC_CH_VREFINT && !(ADC1_COMMON->CCR & ADC_CCR_VREFEN)) {
        ADC1_COMMON->CCR |= ADC_CCR_VREFEN;              /* switch the reference on ...   */
        delay_us(12);                            /* ... and wait 12 us (ST's LL)  */
    }

    /* On this ADC family a new channel selection is not instant: write
     * CHSELR, then wait for CCRDY before starting.  Start too soon and the
     * conversion uses the OLD channel - a silent, plausible wrong answer. */
    ADC1->ISR    = ADC_ISR_CCRDY;
    ADC1->CHSELR = 1u << channel;
    if (wait_set(&ADC1->ISR, ADC_ISR_CCRDY)) { return -3; }

    ADC1->CR |= ADC_CR_ADSTART;                  /* one conversion              */
    if (wait_set(&ADC1->ISR, ADC_ISR_EOC)) { return -4; }
    return (int32_t)(ADC1->DR & 0xFFFu);         /* reading DR clears EOC       */
}
