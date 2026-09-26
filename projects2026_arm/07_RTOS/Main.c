/*
 * Main.c - SOC3050 lesson 07: RTOS
 *          ST Nucleo-C031C6, STM32C031C6, Cortex-M0+ at 48 MHz
 *
 * The kernel is os.c, beside this file - read it first.  This file is two
 * programs that use it, chosen by SCENARIO:
 *
 *   SCENARIO 1 - a small system.  A task that NEVER yields (it hunts for
 *                primes) shares the CPU with an LED bar that keeps perfect
 *                time, two buttons and a report.  In lesson 06's superloop,
 *                one job like that would have stopped everything else.
 *                Button A changes the bar's speed; button B switches the
 *                hog on and off.  Every second the report prints each task's
 *                share of the CPU and how deep its stack has ever been.
 *
 *   SCENARIO 2 - priority inversion, measured.  A high-priority task needs a
 *                mutex a low-priority task holds, while a medium-priority
 *                task hogs the CPU.  The report prints how long the high
 *                task waited.  Then OS_PRIORITY_INHERIT in os.h decides
 *                whether it waits 30 ms or 150.
 *
 * Build:     build.bat
 * Simulate:  simulate.bat, paste diagram.json, upload Main.elf.
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32c031xx.h"
#include "retarget.h"
#include "gpio.h"
#include "os.h"

#define SCENARIO     1           /* 1 = small system, 2 = priority inversion */

#define BAUD         115200u
#define BTN_A_PIN    0u          /* PA0 - speed                                */
#define BTN_B_PIN    1u          /* PA1 - hog on / off                         */

#define STACK(name, words) static uint32_t name[words] __attribute__((aligned(8)))

/* ============================================================================
 *  A HardFault that says where
 * ============================================================================
 * On a fault the hardware stacks the same eight-word frame as for any
 * exception - on PSP if a task was running, MSP if a handler was.  Bit 2 of
 * EXC_RETURN (in lr on entry) says which.  Word 6 of the frame is the pc of
 * the instruction that faulted.  This is lesson 11's subject in miniature.
 */
__attribute__((used)) void fault_report(const uint32_t *frame)
{
    os_task_t *t = os_current_task();
    printf("\n*** HardFault in task '%s'\n", t ? t->name : "(none)");
    printf("    pc   = 0x%08lX   <- the instruction that faulted\n", (unsigned long)frame[6]);
    printf("    lr   = 0x%08lX   <- where it would have returned to\n", (unsigned long)frame[5]);
    printf("    xPSR = 0x%08lX\n", (unsigned long)frame[7]);
    printf("    Find the line:  arm-none-eabi-addr2line -e Main.elf 0x%08lX\n", (unsigned long)frame[6]);
    for (;;) { }
}

__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "   .syntax unified       \n"
        "   movs r0, #4           \n"
        "   mov  r1, lr           \n"
        "   tst  r0, r1           \n"     /* EXC_RETURN bit 2: which stack?  */
        "   beq  1f               \n"
        "   mrs  r0, psp          \n"     /* a task faulted                  */
        "   b    2f               \n"
        "1: mrs  r0, msp          \n"     /* a handler faulted               */
        "2: ldr  r1, =fault_report\n"
        "   bx   r1               \n"
        "   .align 2              \n"
        "   .ltorg                \n"
    );
}

/* ============================================================================
 *  The report - the same in both scenarios
 * ============================================================================ */
static void print_tasks(uint32_t *prev_ticks, uint32_t *prev_switches)
{
    static const char state[] = { 'R', 'S', 'B' };
    uint32_t sw = os_switches();

    printf("t=%6lu ms   switches/s %lu\n", (unsigned long)os_ticks(),
           (unsigned long)(sw - *prev_switches));
    *prev_switches = sw;

    printf("  task      prio  st  cpu%%   stack used/size\n");
    for (uint32_t i = 0; i < os_task_count(); i++) {
        os_task_t *t = os_task(i);
        uint32_t used = os_stack_used(t);
        printf("  %-8s %2u/%-2u  %c  %4lu    %4lu / %-4lu%s\n",
               t->name, (unsigned)t->base_prio, (unsigned)t->prio, state[t->state],
               (unsigned long)((t->ticks - prev_ticks[i]) / 10u),   /* ticks per 1000 -> % */
               (unsigned long)used, (unsigned long)t->stack_words,
               used >= t->stack_words ? "  OVERFLOW" : "");
        prev_ticks[i] = t->ticks;
    }
}

#if SCENARIO == 1
/* ============================================================================
 *  Scenario 1: a small system
 * ============================================================================ */
STACK(stk_bar, 128);  STACK(stk_button, 128);  STACK(stk_hog, 128);  STACK(stk_report, 512);

static volatile uint32_t bar_ms = 60;       /* frame time, set by button A   */
static volatile uint8_t  hog_on = 1;        /* set by button B               */
static volatile uint32_t primes;            /* the hog's output              */

/* Priority 2.  Lesson 04's knight rider, written the obvious way - a loop
 * with a delay in it - which a superloop could never afford.  os_delay_until
 * keeps it on a fixed grid, whatever else is running. */
static void task_bar(void *arg)
{
    (void)arg;
    uint32_t last = os_ticks(), pos = 0;
    int dir = 1;
    for (;;) {
        GPIOB->BSRR = ((uint32_t)(uint8_t)~(1u << pos) << 16) | (1u << pos);
        if (pos == 7u) { dir = -1; } else if (pos == 0u) { dir = 1; }
        pos = (uint32_t)((int)pos + dir);
        os_delay_until(&last, bar_ms);
    }
}

/* Priority 3, the highest: it runs for microseconds every 5 ms, and a person
 * pressing a button should never wait for anything else. */
static void task_button(void *arg)
{
    (void)arg;
    static const uint32_t speeds[] = { 120, 60, 30 };
    uint32_t last = os_ticks(), speed = 1;
    uint8_t  a_prev = 0, b_prev = 0;
    for (;;) {
        os_delay_until(&last, 5);                 /* sample every 5 ms: debounce */
        uint8_t a = (uint8_t)!pin_read(GPIOA, BTN_A_PIN);
        uint8_t b = (uint8_t)!pin_read(GPIOA, BTN_B_PIN);
        if (a && !a_prev) { speed = (speed + 1u) % 3u; bar_ms = speeds[speed]; }
        if (b && !b_prev) { hog_on = (uint8_t)!hog_on; }
        a_prev = a;
        b_prev = b;
    }
}

/* Priority 1.  Never blocks while it is on: pure computation, forever.  In a
 * superloop this would be the end of everything else. */
static void task_hog(void *arg)
{
    (void)arg;
    uint32_t n = 3;
    for (;;) {
        if (!hog_on) { os_delay(100); continue; }
        uint32_t d = 3;
        while (d * d <= n && n % d != 0u) { d += 2u; }
        if (d * d > n) { primes++; }
        n += 2u;
    }
}

/* Priority 1 as well: a report can be late without harm, so it shares time
 * with the hog rather than interrupting the bar. */
static void task_report(void *arg)
{
    (void)arg;
    uint32_t prev_ticks[OS_MAX_TASKS] = { 0 }, prev_sw = 0, prev_primes = 0;
    uint32_t last = os_ticks();
    for (;;) {
        os_delay_until(&last, 1000);
        uint32_t p = primes;
        printf("\n");
        print_tasks(prev_ticks, &prev_sw);
        printf("  bar %lu ms/frame   hog %s, %lu primes/s\n",
               (unsigned long)bar_ms, hog_on ? "ON " : "off", (unsigned long)(p - prev_primes));
        prev_primes = p;
    }
}

static void create_tasks(void)
{
    os_task_create("bar",    task_bar,    0, stk_bar,    128, 2);
    os_task_create("button", task_button, 0, stk_button, 128, 3);
    os_task_create("hog",    task_hog,    0, stk_hog,    128, 1);
    os_task_create("report", task_report, 0, stk_report, 512, 1);
}

#elif SCENARIO == 2
/* ============================================================================
 *  Scenario 2: priority inversion
 * ============================================================================
 *   low  (1)  holds the mutex for 30 ms at a time
 *   mid  (2)  every 250 ms, computes for 120 ms - never touches the mutex
 *   high (3)  every 100 ms, needs the mutex briefly, and times the wait
 */
STACK(stk_low, 128);  STACK(stk_mid, 128);  STACK(stk_high, 128);  STACK(stk_report, 512);

static os_mutex_t shared;
static volatile uint32_t h_locks, h_max_wait, h_long_waits;

static void busy_ms(uint32_t ms)              /* burn CPU, but preemptibly */
{
    uint32_t t0 = os_ticks();
    while (os_ticks() - t0 < ms) { }
}

static void task_low(void *arg)
{
    (void)arg;
    for (;;) {
        os_mutex_lock(&shared);
        busy_ms(30);                          /* the critical section       */
        os_mutex_unlock(&shared);
        os_delay(10);
    }
}

static void task_mid(void *arg)
{
    (void)arg;
    for (;;) {
        os_delay(250);
        busy_ms(120);                         /* unrelated work, no mutex   */
    }
}

static void task_high(void *arg)
{
    (void)arg;
    uint32_t last = os_ticks();
    for (;;) {
        os_delay_until(&last, 100);
        uint32_t t0 = os_ticks();
        os_mutex_lock(&shared);
        uint32_t wait = os_ticks() - t0;
        os_mutex_unlock(&shared);
        h_locks++;
        if (wait > h_max_wait) { h_max_wait = wait; }
        if (wait > 40u)        { h_long_waits++; }   /* longer than low's 30 ms can explain */
    }
}

static void task_report(void *arg)
{
    (void)arg;
    uint32_t prev_ticks[OS_MAX_TASKS] = { 0 }, prev_sw = 0;
    uint32_t last = os_ticks();
    for (;;) {
        os_delay_until(&last, 1000);
        printf("\n");
        print_tasks(prev_ticks, &prev_sw);
        printf("  high: %lu locks, worst wait %lu ms, waits over 40 ms: %lu"
               "   (inheritance %s)\n",
               (unsigned long)h_locks, (unsigned long)h_max_wait,
               (unsigned long)h_long_waits, OS_PRIORITY_INHERIT ? "ON" : "OFF");
    }
}

static void create_tasks(void)
{
    os_task_create("low",    task_low,    0, stk_low,    128, 1);
    os_task_create("mid",    task_mid,    0, stk_mid,    128, 2);
    os_task_create("high",   task_high,   0, stk_high,   128, 3);
    os_task_create("report", task_report, 0, stk_report, 512, 4);
}
#endif

/* ============================================================================
 *                                  main
 * ============================================================================
 * main() sets things up on MSP, starts the kernel, and is never seen again:
 * the first PendSV returns into a task on PSP.
 */
int main(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;
    for (uint32_t pin = 0; pin < 8u; pin++) { pin_mode(GPIOB, pin, MODE_OUTPUT); }
    pin_mode(GPIOA, BTN_A_PIN, MODE_INPUT);  pin_pull(GPIOA, BTN_A_PIN, PULL_UP);
    pin_mode(GPIOA, BTN_B_PIN, MODE_INPUT);  pin_pull(GPIOA, BTN_B_PIN, PULL_UP);

    uart2_init(SystemCoreClock, BAUD);

    printf("\n=== SOC3050 lesson 07 - RTOS, scenario %d ===\n", SCENARIO);
    printf("    STM32C031C6, Cortex-M0+, %lu Hz, tick %lu Hz\n",
           (unsigned long)SystemCoreClock, (unsigned long)OS_TICK_HZ);
    printf("    PendSV_Handler %s, SysTick_Handler %s, HardFault_Handler %s\n",
           handler_installed(PendSV_IRQn)  ? "installed" : "NOT INSTALLED",
           handler_installed(SysTick_IRQn) ? "installed" : "NOT INSTALLED",
           handler_installed(HardFault_IRQn) ? "installed" : "NOT INSTALLED");

    create_tasks();
    printf("    %lu tasks created; starting the kernel.\n", (unsigned long)os_task_count());

    os_start();                                   /* never returns */
}
