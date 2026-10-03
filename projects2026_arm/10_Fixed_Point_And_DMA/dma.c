/*
 * dma.c - DMA1 + DMAMUX on the STM32C031   (SOC3050 lesson 10)
 *
 * DMA = direct memory access: a second bus master beside the CPU that copies
 * words from one address to another when a peripheral asks it to.  The C031
 * has ONE DMA controller with THREE channels, and a DMAMUX in front of it
 * that decides which peripheral's request reaches which channel:
 *
 *   ADC1 ---request 5---> DMAMUX channel 0 ---> DMA1 channel 1 ---> RAM
 *
 * "DMAMUX channel 0 to 6 are mapped to DMA1 channel 1 to 7" - ST's own
 * comment in stm32c0xx_ll_dma.h; the C031 has the first three.  Request ID 5
 * is LL_DMAMUX_REQ_ADC1 in stm32c0xx_ll_dmamux.h.  Neither number is in the
 * CMSIS device header: they come from ST's LL driver and RM0490.
 *
 * Two jobs here:
 *   scan_*()   the ADC converts PA0, PA1, PA4 forever; the DMA drops each
 *              result into scan_buf[]; the CPU just reads RAM.
 *   dma_copy() memory to memory, no peripheral at all, timed against memcpy.
 *
 * WOKWI CAVEAT.  Whether Wokwi's STM32C0 models DMA1 and DMAMUX has not been
 * confirmed.  So nothing here is trusted: scan_start() watches the buffer and
 * gives up cleanly if it never fills, and dma_copy() has a time limit.
 */
#include "stm32c031xx.h"
#include "adc.h"
#include "dma.h"

volatile uint16_t scan_buf[SCAN_CH];

static int dma_on;

/* "Never" is 200 000 polls: tens of milliseconds at 48 MHz, while one whole
 * scan takes about 22 us.  Generous on purpose - this is only the give-up. */
#define SPIN_LIMIT  200000u

static void adc_stop(void)
{
    if (ADC1->CR & ADC_CR_ADSTART) {
        ADC1->CR |= ADC_CR_ADSTP;                      /* ask it to stop ...      */
        for (uint32_t i = 0; i < SPIN_LIMIT && (ADC1->CR & ADC_CR_ADSTART); i++) { }
    }                                                  /* ... ADSTART clears when done */
}

void scan_stop(void)
{
    adc_stop();
    /* CFGR1 may only be written while ADSTART = 0 - which adc_stop() made
     * sure of.  Back to single conversions, so adc_read() works again. */
    ADC1->CFGR1 &= ~(ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG | ADC_CFGR1_CONT);
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    dma_on = 0;
}

int scan_running(void) { return dma_on; }

int scan_start(void)
{
    scan_stop();
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;      /* one gate for DMA1 and DMAMUX: AHBENR has no other DMA bit */

    /* A 12-bit ADC can never write 0xFFFF.  Fill the buffer with it, and any
     * entry that changes was written by the DMA - proof, not hope. */
    for (uint32_t i = 0; i < SCAN_CH; i++) { scan_buf[i] = 0xFFFFu; }

    /* ---- the DMA channel: from where, to where, how many, how ------------- */
    DMA1->IFCR = DMA_IFCR_CGIF1;                       /* clear old flags         */
    DMA1_Channel1->CPAR  = (uint32_t)&ADC1->DR;        /* source: the ADC result  */
    DMA1_Channel1->CMAR  = (uint32_t)scan_buf;         /* dest: our array         */
    DMA1_Channel1->CNDTR = SCAN_CH;                    /* 3 transfers, then wrap  */
    DMAMUX1_Channel0->CCR = DMAREQ_ADC1;               /* ADC1's request -> DMA ch 1 */
    DMA1_Channel1->CCR =
          DMA_CCR_PSIZE_0          /* read 16 bits from DR                        */
        | DMA_CCR_MSIZE_0          /* write 16 bits to RAM                        */
        | DMA_CCR_MINC             /* RAM address steps +2 each time; DR stays    */
        | DMA_CCR_CIRC             /* at the end reload CNDTR and CMAR: forever   */
        | DMA_CCR_PL_1             /* priority high                               */
        | DMA_CCR_EN;              /* DIR = 0: peripheral -> memory               */

    /* ---- the ADC: three channels, continuous, asking for DMA --------------- */
    ADC1->CFGR1 = (ADC1->CFGR1 & ~(ADC_CFGR1_SCANDIR | ADC_CFGR1_CHSELRMOD))
                | ADC_CFGR1_DMAEN          /* raise a DMA request at each EOC        */
                | ADC_CFGR1_DMACFG         /* ... and keep raising them (circular)   */
                | ADC_CFGR1_CONT;          /* start the next scan without being told */
    ADC1->ISR    = ADC_ISR_CCRDY;
    ADC1->CHSELR = ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL4;
    for (uint32_t i = 0; i < SPIN_LIMIT && !(ADC1->ISR & ADC_ISR_CCRDY); i++) { }

    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
    ADC1->CR |= ADC_CR_ADSTART;                        /* go - and never again   */

    /* ---- now watch: did all three entries get written? --------------------- */
    for (uint32_t i = 0; i < SPIN_LIMIT; i++) {
        if (scan_buf[0] != 0xFFFFu && scan_buf[1] != 0xFFFFu && scan_buf[2] != 0xFFFFu) {
            dma_on = 1;
            return 0;
        }
    }
    scan_stop();                                       /* stuck: undo it all      */
    return -1;
}

int scan_read(uint16_t out[SCAN_CH], uint32_t *overruns)
{
    if (dma_on) {
        /* OVR: a conversion finished before the DMA had taken the previous
         * one.  With DMA, ST's HAL reports this as an error whatever OVRMOD
         * says, and the channel order in scan_buf can no longer be trusted.
         * So: count it, restart, and use this round's values anyway. */
        if (ADC1->ISR & ADC_ISR_OVR) {
            if (overruns) { (*overruns)++; }
            (void)scan_start();
        }
        for (uint32_t i = 0; i < SCAN_CH; i++) { out[i] = scan_buf[i]; }
        return 0;
    }
    static const uint32_t ch[SCAN_CH] = { ADC_CH_PA0, ADC_CH_PA1, ADC_CH_PA4 };
    for (uint32_t i = 0; i < SCAN_CH; i++) {
        int32_t v = adc_read(ch[i]);                   /* ~7 us of busy-waiting each */
        if (v < 0) { return (int)v; }
        out[i] = (uint16_t)v;
    }
    return 0;
}

int dma_copy(uint32_t *dst, const uint32_t *src, uint32_t words, uint32_t *spins)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel2->CCR   = 0;                          /* must be off to configure */
    DMA1->IFCR           = DMA_IFCR_CGIF2;
    DMA1_Channel2->CPAR  = (uint32_t)src;              /* DIR = 0: "peripheral" is the source */
    DMA1_Channel2->CMAR  = (uint32_t)dst;
    DMA1_Channel2->CNDTR = words;
    DMAMUX1_Channel1->CCR = DMAREQ_MEM;                /* no peripheral request    */
    DMA1_Channel2->CCR =
          DMA_CCR_MEM2MEM          /* run flat out, no request needed             */
        | DMA_CCR_PSIZE_1          /* 32-bit reads                                */
        | DMA_CCR_MSIZE_1          /* 32-bit writes                               */
        | DMA_CCR_PINC | DMA_CCR_MINC
        | DMA_CCR_EN;              /* starts immediately                          */

    uint32_t n = 0;
    while (!(DMA1->ISR & (DMA_ISR_TCIF2 | DMA_ISR_TEIF2))) {
        if (++n >= SPIN_LIMIT) { DMA1_Channel2->CCR = 0; *spins = n; return -1; }
    }
    *spins = n;
    int err = (DMA1->ISR & DMA_ISR_TEIF2) ? -1 : 0;   /* a bus error: bad address */
    DMA1->IFCR = DMA_IFCR_CGIF2;
    DMA1_Channel2->CCR = 0;
    return err;
}
