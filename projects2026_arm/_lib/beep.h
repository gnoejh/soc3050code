/*
 * beep.h - square-wave tones on the app board's buzzer (PA6, TIM3_CH1)   LIBS=beep
 *
 * Lesson 06's PWM, pointed at a speaker: the timer's PERIOD sets the pitch
 * and a 50% duty cycle makes a square wave.  The hardware keeps the tone
 * going with no CPU at all; software only decides when to stop it, so call
 * beep_poll() from any loop or task that runs at least every few ms.
 *
 * TIM3 belongs to this module.  Lesson 06's servo used it too - one timer,
 * one job; an application that wants both moves one to another timer.
 */
#ifndef BEEP_H
#define BEEP_H

#include <stdint.h>

void beep_init(void);
void beep(uint32_t hz, uint32_t ms, uint32_t now_ms);   /* hz = 0: silence  */
void beep_poll(uint32_t now_ms);                        /* stops it on time */
int  beep_busy(void);

#endif
