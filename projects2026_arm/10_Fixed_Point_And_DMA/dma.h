/*
 * dma.h - DMA1 on the STM32C031: an ADC that fills RAM by itself, and a
 *         memory-to-memory copy   (SOC3050 lesson 10)
 */
#ifndef DMA_H
#define DMA_H

#include <stdint.h>

#define SCAN_CH     3u          /* PA0 joystick HORZ, PA1 VERT, PA4 knob     */
#define DMAREQ_ADC1 5u          /* DMAMUX request ID: ST LL_DMAMUX_REQ_ADC1  */
#define DMAREQ_MEM  0u          /* LL_DMAMUX_REQ_MEM2MEM                     */

/* The RAM the DMA writes: scan_buf[0] = PA0 (IN0), [1] = PA1 (IN1),
 * [2] = PA4 (IN4) - lowest channel first, because SCANDIR = 0.  volatile:
 * the hardware changes it behind the compiler's back. */
extern volatile uint16_t scan_buf[SCAN_CH];

/* Start the ADC scanning IN0, IN1, IN4 continuously, with DMA1 channel 1 in
 * circular mode copying each result into scan_buf.  Then WATCH it: return 0
 * only if all three entries were written within the time limit, -1 if the
 * DMA never moved (and everything is stopped again).  adc_init() first. */
int  scan_start(void);
void scan_stop(void);           /* ADSTP, DMA off; the ADC is free for adc_read() */
int  scan_running(void);

/* One reading of all three channels: from scan_buf if the DMA is running,
 * else three polled adc_read() calls.  Returns 0, or negative on an ADC error.
 * If the ADC overran (OVR) the scan is restarted and *overruns counts it. */
int  scan_read(uint16_t out[SCAN_CH], uint32_t *overruns);

/* Copy `words` 32-bit words with DMA1 channel 2 in memory-to-memory mode.
 * The CPU polls for completion, counting how many times it looked: work it
 * could have done instead.  Returns 0, or -1 if the transfer never finished. */
int  dma_copy(uint32_t *dst, const uint32_t *src, uint32_t words, uint32_t *spins);

#endif
