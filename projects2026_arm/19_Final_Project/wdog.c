/*
 * wdog.c - IWDG start/refresh and the reset-cause flags
 *
 * The sequence is ST's HAL_IWDG_Init() (stm32c0xx_hal_iwdg.c), with ST's key
 * values (stm32c0xx_hal_iwdg.h):
 *
 *   KR = 0xCCCC   start the watchdog (this also switches the LSI on)
 *   KR = 0x5555   unlock PR and RLR - they ignore writes otherwise
 *   PR = 3        prescaler /32:  32 000 Hz / 32 = 1000 counts per second
 *   RLR = 999     count 999..0:   1000 counts = 1.000 s, nominal
 *   wait SR == 0  PVU/RVU/WVU: the new values crossing into the LSI domain
 *   KR = 0xAAAA   refresh: reload the counter from RLR
 *
 * "Nominal" matters: LSI_VALUE is 32000 as ST's TYPICAL value, and ST's own
 * comment says the real one varies with voltage and temperature.  Design the
 * refresh period with a wide margin - here the health task refreshes every
 * 250 ms against a 1000 ms timeout, four times over.
 */
#include "stm32c031xx.h"
#include "wdog.h"

#define KEY_START   0xCCCCu
#define KEY_UNLOCK  0x5555u
#define KEY_RELOAD  0xAAAAu
#define PR_DIV32    (IWDG_PR_PR_1 | IWDG_PR_PR_0)       /* = 3, HAL's IWDG_PRESCALER_32 */

int wdog_start(void)
{
    /* Freeze the IWDG while a debugger halts the core.  Without this,
     * stopping at a breakpoint for a second resets the board under GDB. */
    RCC->APBENR1 |= RCC_APBENR1_DBGEN;
    DBG->APBFZ1  |= DBG_APB_FZ1_DBG_IWDG_STOP;

    IWDG->KR  = KEY_START;
    IWDG->KR  = KEY_UNLOCK;
    IWDG->PR  = PR_DIV32;
    IWDG->RLR = WDOG_TIMEOUT_MS - 1u;                    /* 12-bit field: <= 4095 */

    /* Every wait bounded (lesson 09).  ST's HAL allows a few LSI periods. */
    for (uint32_t n = 0; n < 200000u; n++) {
        if ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU)) == 0u) {
            IWDG->KR = KEY_RELOAD;
            return 0;
        }
    }
    IWDG->KR = KEY_RELOAD;
    return -1;
}

void wdog_feed(void) { IWDG->KR = KEY_RELOAD; }

uint32_t wdog_reset_flags(void)
{
    uint32_t f = RCC->CSR2;
    RCC->CSR2 |= RCC_CSR2_RMVF;                         /* clear for next boot */
    return f;
}

const char *wdog_reset_cause(uint32_t f)
{
    /* Several flags can be set at once - `reset` prints the raw word too;
     * this reports the most specific. */
    if (f & RCC_CSR2_IWDGRSTF) { return "IWDG - the watchdog fired"; }
    if (f & RCC_CSR2_WWDGRSTF) { return "WWDG"; }
    if (f & RCC_CSR2_SFTRSTF)  { return "software (NVIC_SystemReset)"; }
    if (f & RCC_CSR2_LPWRRSTF) { return "low-power"; }
    if (f & RCC_CSR2_OBLRSTF)  { return "option-byte load"; }
    if (f & RCC_CSR2_PWRRSTF)  { return "power-on or brown-out"; }
    if (f & RCC_CSR2_PINRSTF)  { return "reset pin (NRST)"; }
    return "none recorded - a simulator, most likely";
}
