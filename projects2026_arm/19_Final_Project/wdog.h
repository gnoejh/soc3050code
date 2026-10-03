/*
 * wdog.h - the independent watchdog (IWDG) and the reset-cause flags
 *
 * The IWDG counts down from RLR on its own clock - the ~32 kHz LSI, not the
 * 48 MHz system clock - so a crashed clock tree or a core stuck in a loop
 * cannot stop it.  Reach zero and it resets the chip.  Writing the reload key
 * puts it back at RLR.  Once started, nothing but a reset stops it.
 *
 * NOTE: Wokwi's Nucleo-C031C6 does NOT implement the IWDG (docs.wokwi.com,
 * board page, "IWDG: not implemented").  The register writes go nowhere there;
 * the health task's HEALTH_FORCE path stands in for it in the simulator.
 */
#ifndef WDOG_H
#define WDOG_H

#include <stdint.h>

#define WDOG_TIMEOUT_MS  1000u    /* LSI 32 kHz / 32 = 1 kHz; RLR = 999       */

/* Starts the IWDG.  0 = configured; -1 = the update flags never cleared
 * (the registers may not have taken the new values). */
int  wdog_start(void);
void wdog_feed(void);

/* RCC->CSR2's reset flags as read at boot, then cleared with RMVF so the
 * next boot reports only its own cause.  Call once, early. */
uint32_t    wdog_reset_flags(void);
const char *wdog_reset_cause(uint32_t flags);

#endif
