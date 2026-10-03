/*
 * adc.h - the 12-bit ADC, one channel at a time   (SOC3050 lesson 09; shared copy in _lib/, LIBS=adc)
 */
#ifndef ADC_H
#define ADC_H

#include <stdint.h>

#define ADC_CH_PA0      0u        /* app board: joystick HORZ (lessons 12-19) */
#define ADC_CH_PA1      1u        /* app board: joystick VERT                 */
#define ADC_CH_PA4      4u        /* the potentiometer                       */
#define ADC_CH_VREFINT  10u       /* the chip's own 1.2 V-ish reference      */

int      adc_init(void);          /* 0 = ready; negative = a step timed out  */
int32_t  adc_read(uint32_t channel);   /* 0..4095, or negative on timeout    */

#endif
