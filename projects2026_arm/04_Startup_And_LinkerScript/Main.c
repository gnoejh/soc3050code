/*
 * Main.c - SOC3050 lesson 04: Startup and the Linker Script
 *          ST Nucleo-C031C6, STM32C031C6, Cortex-M0+ at 48 MHz
 *
 * The first program in this course that you build and run yourself.
 *
 * It does almost nothing on purpose.  It blinks the user LED and prints a
 * memory report - and every number in that report is MEASURED from a linker
 * symbol, never typed in.  That is the whole idea: link.ld decides where your
 * program's pieces live, startup.c moves them into place, and this program
 * reads back what those two files actually did.
 *
 * Change link.ld and the numbers change.  Break startup.c and the numbers go
 * wrong in a specific, readable way.  Lab.md walks you through both.
 *
 * Build:     build.bat
 * Simulate:  simulate.bat, then upload Main.elf in the Wokwi browser tab
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32c031xx.h"

#define LED_PIN   5u          /* PA5 - the user LED, hardwired on the Nucleo */
#define BAUD      115200u

void uart2_init(uint32_t pclk, uint32_t baud);   /* retarget.c */

/* ---- Symbols placed by link.ld -------------------------------------------
 * These are not variables.  Each one is an ADDRESS the linker decided on,
 * and the only correct way to use it is to take its address.  Reading one as
 * if it held a value gives you whatever bytes happen to live there.
 */
extern uint32_t _sidata;      /* .data's home in FLASH                       */
extern uint32_t _sdata, _edata;
extern uint32_t _sbss,  _ebss;
extern uint32_t _estack;      /* top of RAM - the initial stack pointer      */
extern char     end;          /* first free byte after .bss - the heap start */
extern void (* const vectors[])(void);   /* startup.c, section .isr_vector   */

/* ---- Three globals that prove startup.c ran -------------------------------
 * data_witness is initialised, so it lives in .data: a copy in FLASH that
 * Reset_Handler must move into RAM.  If that copy does not happen, this
 * reads as something other than 0x00C0FFEE.
 *
 * bss_witness is initialised to zero, so the compiler puts it in .bss and
 * stores nothing at all in FLASH.  Its zero is a promise that Reset_Handler
 * must keep.
 *
 * rodata_witness is const, so it never leaves FLASH - nothing to copy, and
 * it costs zero bytes of RAM.  Compare the three in the size report.
 */
volatile uint32_t data_witness  = 0x00C0FFEEu;
volatile uint32_t bss_witness   = 0u;
const    uint32_t rodata_witness = 0x5AFEu;

static uint32_t blink_count;      /* also .bss - see Lab exercise 2 */

static void led_init(void)
{
    RCC->IOPENR  |= RCC_IOPENR_GPIOAEN;      /* clock the port FIRST, always */
    GPIOA->MODER &= ~(3u << (LED_PIN * 2));
    GPIOA->MODER |=  (1u << (LED_PIN * 2));  /* 01 = general purpose output  */
}

/* A counted NOP loop, not a calibrated delay.  Lesson 07 replaces it with
 * SysTick, which is the right answer; this is honest about being crude. */
static void delay_crude(volatile uint32_t n)
{
    while (n--) { __asm__ volatile("nop"); }
}

static void memory_report(void)
{
    uint32_t data_sz = (uint32_t)((uintptr_t)&_edata - (uintptr_t)&_sdata);
    uint32_t bss_sz  = (uint32_t)((uintptr_t)&_ebss  - (uintptr_t)&_sbss);

    printf("\n");
    printf("=== SOC3050 lesson 04 - Startup and the Linker Script ===\n");
    printf("    STM32C031C6, Cortex-M0+\n\n");

    printf("-- what the clock actually is --\n");
    printf("  SystemCoreClock : %lu Hz\n", (unsigned long)SystemCoreClock);
    printf("    (reset value is 12 MHz; SystemInit() in startup.c clears\n");
    printf("     RCC->CR HSIDIV to get 48. If this says 12000000, that\n");
    printf("     write did not take.)\n\n");

    printf("-- where link.ld put things --\n");
    printf("  vector table    : 0x%08lX  (must be the base of FLASH)\n",
           (unsigned long)(uintptr_t)vectors);
    printf("  .data in FLASH  : 0x%08lX  _sidata, the copy source\n",
           (unsigned long)(uintptr_t)&_sidata);
    printf("  .data in RAM    : 0x%08lX .. 0x%08lX  (%lu bytes)\n",
           (unsigned long)(uintptr_t)&_sdata,
           (unsigned long)(uintptr_t)&_edata, (unsigned long)data_sz);
    printf("  .bss  in RAM    : 0x%08lX .. 0x%08lX  (%lu bytes)\n",
           (unsigned long)(uintptr_t)&_sbss,
           (unsigned long)(uintptr_t)&_ebss,  (unsigned long)bss_sz);
    printf("  heap starts at  : 0x%08lX  (first byte after .bss)\n",
           (unsigned long)(uintptr_t)&end);
    printf("  stack top       : 0x%08lX  _estack, loaded into SP by\n",
           (unsigned long)(uintptr_t)&_estack);
    printf("                    the HARDWARE before any code runs\n\n");

    printf("-- did startup.c do its four jobs? --\n");
    printf("  .data copied    : 0x%08lX  %s  (expect 0x00C0FFEE)\n",
           (unsigned long)data_witness,
           data_witness == 0x00C0FFEEu ? "OK  " : "WRONG");
    printf("  .bss  zeroed    : 0x%08lX  %s  (expect 0x00000000)\n",
           (unsigned long)bss_witness,
           bss_witness == 0u ? "OK  " : "WRONG");
    printf("  .rodata in FLASH: 0x%08lX  at 0x%08lX - never copied,\n",
           (unsigned long)rodata_witness,
           (unsigned long)(uintptr_t)&rodata_witness);
    printf("                    so it costs no RAM at all\n\n");

    printf("  Every address above came from a linker symbol.\n");
    printf("  Nothing here is hardcoded. Edit link.ld and watch them move.\n\n");
    printf("LESSON 04 OK - LED on PA5 should now be blinking\n\n");
}

int main(void)
{
    led_init();
    uart2_init(SystemCoreClock, BAUD);

    memory_report();

    for (;;) {
        GPIOA->ODR ^= (1u << LED_PIN);   /* BSRR, the atomic way, is lesson 05 */
        blink_count++;

        /* Every eighth blink, show the .bss counter still counting.  If
         * exercise 2 has broken the .bss zeroing, this starts somewhere odd. */
        if ((blink_count & 7u) == 0u) {
            printf("blink %lu\n", (unsigned long)blink_count);
        }
        delay_crude(400000);
    }
}
