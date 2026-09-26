# RTOS: Writing the Thing That Runs Your Tasks
## SOC3050 ARM Edition — Part 1, Instances

**Reference**: [STM32 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0490-stm32c0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) ·
**Board manual**: [UM2953, STM32 Nucleo-64 boards (MB1717)](https://www.st.com/resource/en/user_manual/um2953-stm32c0-nucleo64-board-mb1717-stmicroelectronics.pdf)

**A preemptive kernel in about 300 lines, and every one of them is yours.**

Lesson 06 ended at the superloop's ceiling: every job must be short, or it
delays all the others. This week you lift the ceiling by building what
FreeRTOS, Zephyr and every other embedded kernel are built on — a context
switch — and then use it to meet the most famous scheduling bug in the history
of embedded systems.

Figures marked **measured** come from running this lesson's kernel, unchanged,
on Renode's Cortex-M0 model (slide 20 says exactly how). None has been watched
in Wokwi yet.

---

## Slide 1: The Job That Never Yields

Put this in lesson 06's superloop:

```c
for (;;) {                               /* find primes, forever */
    uint32_t d = 3;
    while (d * d <= n && n % d != 0u) { d += 2u; }
    if (d * d > n) { primes++; }
    n += 2u;
}
```

It never returns to the loop, so nothing else ever runs again: the LED bar
freezes, the buttons die, the report stops. The superloop only works while
every job **agrees** to be short. That is cooperation, and one uncooperative
job ends it.

What we want instead: the hog runs whenever nothing more urgent needs the CPU,
and **is stopped** — mid-calculation, without its consent — the instant
something more urgent does. Then carried on later from exactly where it was.

That is **preemptive multitasking**, and on a Cortex-M it takes one interrupt,
one exception, and a second stack pointer.

---

## Slide 2: A Task Is a Stack and Nine Numbers

A C function that is running *is*, at any instant:

- its **stack** — local variables, return addresses, saved registers;
- its **registers** — `r0`–`r12`, `lr`, `pc` and `xPSR`;
- its **stack pointer** — where on that stack it currently is.

Save all of that, and you can put the function away. Restore it, and it
continues as if nothing happened — because, from its point of view, nothing
did. That saved state is the task's **context**; swapping one for another is a
**context switch**.

```c
typedef struct os_task {
    uint32_t   *sp;          /* MUST be first - the assembly reads [tcb + 0] */
    const char *name;
    uint8_t     prio, base_prio, state;
    uint32_t    wake;
    ...
} os_task_t;
```

That struct is the **task control block**. Each task also gets its own
stack — a plain array. The registers live *on* that stack while the task is
switched out, so the TCB only needs to remember one number: where the stack
pointer was.

---

## Slide 3: Two Stack Pointers

The Cortex-M has **two** stack pointers, and you have been using one of them
without noticing:

| | Used by | In this kernel |
|---|---|---|
| **MSP** — main stack pointer | every exception handler; `main()` after reset | handlers only |
| **PSP** — process stack pointer | Thread mode, if `CONTROL.SPSEL = 1` | every task, each on its own stack |

The split is what makes the design clean. Interrupt handlers always run on
MSP, a stack no task owns, so a handler never needs space on a task's stack
beyond the eight words the hardware pushes on entry. Each task's stack only has
to hold *that task*.

The switch between them is automatic. On exception return, `lr` holds an
**EXC_RETURN** value, and one of its values means *go back to Thread mode and
use PSP*:

```
0xFFFFFFFD    return to Thread mode, restore from PSP, keep using PSP
```

Every switch in this kernel returns with exactly that value.

---

## Slide 4: PendSV — an Exception You Pend Yourself

A switch must never happen *inside* another interrupt handler: that handler's
frame is on MSP, half-finished, and swapping the task beneath it would return
it into the wrong world. So switches happen in one place only — **PendSV**, a
system exception with two properties:

- **It is triggered by software.** `SCB->ICSR = SCB_ICSR_PENDSVSET_Msk` sets its
  pending bit. Nothing else ever raises it.
- **This kernel gives it the lowest priority**, 3. So it runs only when *no
  other handler is active* — after every other interrupt has finished, on the
  way back to a task.

```svg
<svg viewBox="0 0 580 170" role="img" aria-label="SysTick decides a switch is needed and pends PendSV; PendSV runs after SysTick returns">
  <defs><marker id="r4" markerWidth="8" markerHeight="8" refX="7" refY="3.4" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.5 0.8 L7 3.4 L0.5 6 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="10" y="60" width="110" height="40" rx="5"/>
  <text x="65" y="85" text-anchor="middle" class="mono">task A</text>
  <path class="wire" d="M120 80 H150" marker-end="url(#r4)"/>
  <rect class="reg" x="154" y="60" width="130" height="40" rx="5"/>
  <text x="219" y="78" text-anchor="middle" class="mono">SysTick_Handler</text>
  <text x="219" y="93" text-anchor="middle" class="lbl">decides; pends PendSV</text>
  <path class="wire" d="M284 80 H314" marker-end="url(#r4)"/>
  <rect class="hifill" x="318" y="60" width="130" height="40" rx="5"/>
  <text x="383" y="78" text-anchor="middle" class="mono">PendSV_Handler</text>
  <text x="383" y="93" text-anchor="middle" class="lbl">tail-chained: switches</text>
  <path class="wire" d="M448 80 H478" marker-end="url(#r4)"/>
  <rect class="box" x="482" y="60" width="90" height="40" rx="5"/>
  <text x="527" y="85" text-anchor="middle" class="mono">task B</text>
  <text x="290" y="140" text-anchor="middle" class="lbl">Deciding and switching are separate, and switching always waits for the calm.</text>
</svg>
```

Deciding *who* runs next happens anywhere — the tick, a mutex unlock, a queue
put from a UART interrupt. **Doing** the switch happens only in PendSV.

---

## Slide 5: The Switch, Drawn

When PendSV starts, the hardware has already pushed half of task A's context
onto A's stack (lesson 03, slide 2). PendSV pushes the other half beside it:

```svg
<svg viewBox="0 0 580 250" role="img" aria-label="Saved context layout on a task stack: r4-r7, r8-r11 saved by PendSV, then the hardware frame">
  <text x="100" y="20" text-anchor="middle" class="lbl">task A's stack, after the save</text>
  <rect class="hifill" x="30" y="30" width="140" height="30"/>
  <text x="100" y="50" text-anchor="middle" class="mono">r4 r5 r6 r7</text>
  <rect class="hifill" x="30" y="60" width="140" height="30"/>
  <text x="100" y="80" text-anchor="middle" class="mono">r8 r9 r10 r11</text>
  <rect class="box" x="30" y="90" width="140" height="30"/>
  <text x="100" y="110" text-anchor="middle" class="mono">r0 r1 r2 r3</text>
  <rect class="box" x="30" y="120" width="140" height="30"/>
  <text x="100" y="140" text-anchor="middle" class="mono">r12 lr pc xPSR</text>
  <rect class="dash" x="30" y="150" width="140" height="40"/>
  <text x="100" y="175" text-anchor="middle" class="lbl">A's own locals</text>
  <text x="180" y="50" class="mono">&lt;- tcb->sp</text>
  <text x="200" y="80" class="lbl">PendSV saves these (8 words)</text>
  <text x="200" y="125" class="lbl">the hardware saved these (8 words)</text>
  <text x="300" y="175" class="lbl">low addresses at the top; stacks grow up the page</text>
  <text x="290" y="228" text-anchor="middle" class="hi">Sixteen words and one pointer: that is a whole paused program.</text>
</svg>
```

Then the reverse for task B: load B's saved `sp`, pop *its* `r4`–`r11`, set
PSP to B's hardware frame, return with `0xFFFFFFFD`. The hardware pops the
rest — including B's `pc` — and B resumes.

---

## Slide 6: The Switch, as Compiled

`PendSV_Handler` in `os.c` is a `naked` function: no compiler prologue, every
instruction written out. From this lesson's `Main.elf`:

```
08000700 <PendSV_Handler>:
 cpsid  i
 ldr    r3, =os_current      ; r1 = the task being left
 ldr    r1, [r3, #0]
 cmp    r1, #0               ; the very first switch has nothing to save
 beq    1f
 mrs    r0, PSP
 subs   r0, #32
 str    r0, [r1, #0]         ; tcb->sp = psp - 32
 stmia  r0!, {r4, r5, r6, r7}
 mov    r4, r8  ...  mov r7, fp
 stmia  r0!, {r4, r5, r6, r7}
1: ...                       ; os_current = os_next; switch count++
 ldr    r0, [r1, #0]         ; r0 = the new task's saved sp
 adds   r0, #16
 ldmia  r0!, {r4, r5, r6, r7}
 mov    r8, r4  ...  mov fp, r7
 msr    PSP, r0
 subs   r0, #32
 ldmia  r0!, {r4, r5, r6, r7}
 cpsie  i
 ldr    r0, =0xFFFFFFFD
 bx     r0
```

**34 instructions, 88 bytes** with its literal pool. That is the whole of
multitasking.

---

## Slide 7: Why `r8`–`r11` Go the Long Way Round

`stmia r0!, {r4-r11}` would do it in one instruction — on a Cortex-M3. On
this core it does not exist.

The M0+ runs **Thumb-1 plus six** (lesson 01, slide 17). Its `stm` and `ldm`
reach only the **low registers**, `r0`–`r7`. So `r8`–`r11` are saved by
copying them into `r4`–`r7` — which have *already* been saved and are free —
and storing those. Restoring runs the same trick backwards, which is why the
code restores `r8`–`r11` **first** and `r4`–`r7` last.

Two more details the listing shows:

- `mov r4, r8` is legal: `mov` between low and high registers is one of the
  few instructions Thumb-1 allows on `r8`–`r12`.
- `sl` and `fp` in the disassembly are `r10` and `r11` — their old APCS names.
  Same registers.

The assembler rejected the first draft outright: `subs r0, #32` is not
accepted in GCC's default inline-assembly syntax for this core. One line,
`.syntax unified`, at the top of the block fixed it. Silent failures get
slides in this course; loud ones deserve a sentence.

---

## Slide 8: The First Switch — Forging a Frame

A new task has never run, so it has never been preempted, so there is no
saved context to restore. `os_task_create()` **builds one**: the stack a task
would have if it had just been interrupted at its first instruction.

```c
*--sp = 0x01000000u;                   /* xPSR: only the Thumb bit     */
*--sp = (uint32_t)fn & ~1u;            /* pc:   the function's start   */
*--sp = (uint32_t)task_exit;           /* lr:   where it goes if it returns */
*--sp = 0u; *--sp = 0u; *--sp = 0u; *--sp = 0u;   /* r12, r3, r2, r1  */
*--sp = (uint32_t)arg;                 /* r0:   its argument (AAPCS)   */
for (int i = 0; i < 8; i++) *--sp = 0u;           /* r11..r4           */
```

- **xPSR's T bit must be 1.** Returning with it clear means switching to ARM
  state, which the M0+ does not have: a HardFault on the first switch.
- **`arg` goes in the `r0` slot**, because that is where AAPCS puts a
  function's first argument (lesson 01, slide 18). The task receives it as a
  normal parameter.
- **`lr` is `task_exit`**, so a task function that returns does not wander
  off into memory — it is parked for good.

To the switch, a brand-new task and a task preempted a second ago look
identical. That is the trick.

---

## Slide 9: Who Runs Next

```c
static os_task_t *pick(void)
{
    uint32_t cur = os_current ? (uint32_t)(os_current - tasks) : 0u;
    os_task_t *best = 0;
    for (uint32_t i = 1; i <= n_tasks; i++) {
        os_task_t *t = &tasks[(cur + i) % n_tasks];
        if (t->state == OS_READY && (best == 0 || t->prio > best->prio)) best = t;
    }
    return best;
}
```

- **Fixed-priority preemptive**: the highest-priority `READY` task runs, always.
- **Round-robin among equals**: the search starts *after* the current task,
  and only a strictly higher priority replaces a find, so tasks of equal
  priority take turns — one tick each.
- **Idle**, priority 0, is always ready, so there is always an answer.

**Priority numbers here run the other way from the NVIC.** In this kernel —
as in FreeRTOS — a *higher* number is *more* urgent. In the NVIC, 0 is most
urgent. Two numbering schemes on one chip: always say which one you mean.

---

## Slide 10: The Tick, and Sleeping

`SysTick_Handler` is now the kernel's heartbeat:

```c
ticks++;
if (os_current) os_current->ticks++;           /* who was running: CPU use */
for each task: if SLEEPING and (int32_t)(ticks - wake) >= 0 -> READY;
schedule();                                     /* pend PendSV if needed    */
```

A task that wants to wait calls `os_delay(ms)`: it marks itself `SLEEPING`,
asks for a switch, and is gone until the tick wakes it. **The CPU is not
waiting for it** — someone else runs.

`os_delay_until(&last, period)` is lesson 06's `at += period` turned into a
call: the task wakes on a fixed grid however long its own work took. The LED
bar uses it, so its frames stay exactly 60 ms apart while the hog runs flat
out underneath.

The per-task `ticks` counter is **statistical profiling**: at each tick, charge
one millisecond to whoever was running. Over a second, it is each task's CPU
share — the table the report prints.

---

## Slide 11: Measured — a Hog That Does Not Hurt

Scenario 1, one second's report, **measured** (hog on, then after button B):

```
t=  3001 ms   switches/s 419                t=  5001 ms   switches/s 429
  task      prio  st  cpu%   stack            task      prio  st  cpu%
  bar       2/2   S     0     24 / 128          bar       2/2   S     0
  button    3/3   S     0     30 / 128          button    3/3   S     0
  hog       1/1   R   100     20 / 128          hog       1/1   S     0
  report    1/1   R     0    115 / 512          report    1/1   R     0
  idle      0/0   R     0     16 / 64           idle      0/0   R   100
```

The hog takes **every spare cycle** — 100% — and yet the bar and the buttons,
at higher priority, are serviced on time: each wakes, preempts the hog within a
tick, does a few microseconds of work, and sleeps again. That is why they show
0% and still work.

Press B and the hog's share passes, whole, to **idle**. Nothing else changed.
In lesson 06's superloop, the left-hand column is impossible.

~420 switches a second: the button task wakes 200 times a second and the bar
about 17, and each visit is two switches — in and out.

---

## Slide 12: Sharing — the Mutex

Two tasks that touch the same thing — a buffer, a peripheral, the UART — need
**mutual exclusion**, as in lesson 03. Masking interrupts works, but for a long
critical section it freezes the whole system. A **mutex** blocks only the tasks
that want *this* resource:

```c
void os_mutex_lock(os_mutex_t *m)
{
    uint32_t pm = enter();                       /* PRIMASK, as always   */
    while (m->owner != 0) {
        os_current->state      = OS_BLOCKED;     /* wait - off the CPU   */
        os_current->waiting_on = m;
        schedule(); leave(pm);  pm = enter();    /* switched out here    */
    }
    m->owner = os_current;
    leave(pm);
}
```

Look at what it is made of: the kernel's own bookkeeping is protected by
**masking interrupts for a few instructions** — lesson 03's tool, unchanged.
The mutex is a way to make a *long* critical section cheap for everyone who
does not need it.

---

## Slide 13: Priority Inversion

Three tasks. **High** needs the mutex briefly every 100 ms. **Low** holds it for
30 ms at a time. **Mid** never touches it, but computes for 120 ms every
250 ms.

```svg
<svg viewBox="0 0 580 200" role="img" aria-label="Priority inversion timeline: high blocks on low's mutex, mid preempts low, high waits for mid">
  <text x="10" y="40" class="mono">high</text>
  <text x="10" y="90" class="mono">mid</text>
  <text x="10" y="140" class="mono">low</text>
  <rect class="box" x="60" y="125" width="60" height="22"/>
  <text x="90" y="140" text-anchor="middle" class="lbl">lock</text>
  <rect class="hifill" x="120" y="25" width="16" height="22"/>
  <text x="128" y="20" text-anchor="middle" class="lbl">wants it</text>
  <path class="dash" d="M136 36 H470"/>
  <text x="300" y="30" text-anchor="middle" class="hi">BLOCKED — waiting for low</text>
  <rect class="box" x="136" y="125" width="20" height="22"/>
  <rect class="hifill" x="156" y="75" width="280" height="22"/>
  <text x="296" y="90" text-anchor="middle" class="mono">mid runs 120 ms</text>
  <rect class="box" x="436" y="125" width="34" height="22"/>
  <text x="453" y="162" text-anchor="middle" class="lbl">unlock</text>
  <rect class="box" x="470" y="25" width="40" height="22"/>
  <text x="290" y="190" text-anchor="middle" class="lbl">high waits for mid — a task it outranks, that shares nothing with it.</text>
</svg>
```

High is blocked by low, which is legitimate — low holds what high needs. But
then mid preempts **low**, and while mid runs, low cannot finish and release
the mutex. **High is now waiting for mid**, a lower-priority task it has
nothing to do with. The priorities have been turned upside down.

This happened on Mars. In 1997 the Pathfinder lander's computer kept
resetting, because a high-priority bus task was starved through exactly this
pattern. Engineers found it by reproducing it on the ground, and fixed it by
turning on one flag, remotely — the one on the next slide.

---

## Slide 14: Priority Inheritance — Measured

The fix: while low holds a mutex that high is waiting for, **low borrows
high's priority**. Mid can no longer preempt it; low finishes its 30 ms,
unlocks, drops back to priority 1, and high goes.

```c
#if OS_PRIORITY_INHERIT
    if (m->owner->prio < os_current->prio) m->owner->prio = os_current->prio;
#endif
```

and `os_mutex_unlock()` restores `base_prio`. Scenario 2, **measured**, five
seconds each:

| `OS_PRIORITY_INHERIT` | High's worst wait | Waits over 40 ms |
|---|---|---|
| **1** | **10 ms** | **0** |
| 0 | **110 ms** | 6 in 49 locks |

Same tasks, same code, one `#define`. Low's critical section is 30 ms, so any
wait over 40 ms is time high spent waiting for *mid*. With inheritance there
are none.

Limitation, stated plainly: this kernel's inheritance is right for one mutex
at a time. A task holding two, with different waiters, needs a real
implementation — FreeRTOS's is several hundred lines.

---

## Slide 15: Queues — Data Between Tasks, and From Interrupts

A mutex protects shared data. Often the better design is **not to share it**:
give it to someone. A queue copies items in and out, and blocks a reader while
it is empty and a writer while it is full:

```c
os_queue_put(&q, &item);            /* task: blocks if full   */
os_queue_get(&q, &item);            /* task: blocks if empty  */
os_queue_put_from_isr(&q, &item);   /* ISR: never blocks - returns 0 if full */
```

The ISR version is the important one. An interrupt handler **must never
block** — there is no task to put to sleep, only a handler on MSP. So it
reports "full" and lets the handler decide. And because it calls `schedule()`,
a task waiting on the queue can be made ready *from inside the interrupt*; the
switch to it happens the moment the last handler returns.

That is exactly lesson 08's design: the UART receive interrupt puts bytes in a
queue, and a protocol task blocks on it — using no CPU at all until a byte
arrives.

---

## Slide 16: Sizing a Stack — the High-Water Mark

Every task needs its own stack, and RAM is 12 KB. How big?

`os_task_create()` fills each stack with `0xDEADBEEF`. The stack grows down, so
the words that still say `0xDEADBEEF` at the bottom were never reached. Count
them:

```
  task      stack used/size       (measured)
  bar          24 / 128
  button       30 / 128
  hog          20 / 128
  report      115 / 512      <- printf
  idle         16 / 64
```

`printf` is the expensive one: 115 words — 460 bytes — for newlib-nano's
formatter and our `_write`. Every task that prints needs room for that.

The method matters more than the numbers: **run the program, exercise every
path, read the high-water mark, add a margin.** A stack sized by guessing is
wrong in one of two directions, and only one of them is visible.

---

## Slide 17: When It Is Too Small — Measured

Give the report task 96 words instead of 512. It needs 115. **Measured**:

```
  report    1/1   R     0      96 / 96    OVERFLOW
  bar 60 ms/frame   hog ON , 5539 primes/s
  ...
  bar 60 ms/frame   hog ON , 536871010 primes/s
```

**Nothing crashed.** The report task wrote 19 words below its stack, and the
linker had put `primes` — another task's variable — right there, at
`0x20000084`, the word under `stk_report` at `0x20000088`. 536 871 010 is
`0x20000062`: a stack address, now sitting in a prime counter.

That is what a stack overflow usually looks like on a microcontroller: not a
crash, but *somebody else's data quietly wrong*, far from the cause. The
high-water mark caught it; without one you would be debugging the hog.

---

## Slide 18: A HardFault That Says Where

When something does fault, the hardware stacks the usual eight words and
enters `HardFault_Handler` — on PSP's stack if a task was running. Bit 2 of
`lr` says which stack; word 6 of the frame is the `pc` that faulted. This
lesson's handler prints them. **Measured**, with `__builtin_trap()` placed in
the hog:

```
*** HardFault in task 'hog'
    pc   = 0x08000422   <- the instruction that faulted
    lr   = 0x08000415   <- where it would have returned to
    xPSR = 0x61000000
    Find the line:  arm-none-eabi-addr2line -e Main.elf 0x08000422
```

and `addr2line` answers with the exact line of `Main.c`. The instruction at
that address is `udf #255` — *undefined*, deliberately.

The M0+ has one fault handler and no fault-status registers; the reason is
lost. On the Cortex-M4 in lesson 11, `CFSR` says *why* as well as *where*.

---

## Slide 19: What You Built, and What a Real RTOS Adds

| This kernel | FreeRTOS / Zephyr add |
|---|---|
| fixed-priority preemptive, round-robin | tickless idle, deadline scheduling (Zephyr) |
| sleep, mutex with inheritance, queue | semaphores, event groups, timers, message buffers |
| one mutex's inheritance | nested inheritance, recursive mutexes |
| high-water marks by pattern | overflow checks on every switch, MPU guard regions |
| 34-instruction PendSV | the same switch — with FPU state on an M4F |

The core is the same, instruction for instruction. That is the point of
writing it: `xTaskCreate`, `vTaskDelayUntil` and `xQueueSendFromISR` are no
longer names — they are `os_task_create`, `os_delay_until` and
`os_queue_put_from_isr`, and you know what is under each.

**From here on, lessons are written as tasks.** Lesson 08's UART is an
interrupt feeding a queue feeding a task.

---

## Slide 20: How the Measurements Were Made

Wokwi could not be driven from the command line, so the kernel was run in
**Renode 1.17.0** on its STM32F072 platform — a Cortex-M0, the same ARMv6-M
architecture as this chip, with SysTick, PendSV and the NVIC modelled.

- `os.c` and `Main.c` were compiled **unchanged**.
- A **test-only** `startup.c` skipped the C0's clock and flash-latency waits,
  which the F0 model does not answer the same way.
- The C0's GPIO block was mapped as plain memory so button presses could be
  written in.
- The F0's USART2 sits at the C0's USART2 address with the same registers, so
  `printf` needed nothing.

One difference was found and matters: **Renode does not fault on an unaligned
word access**; ARMv6-M silicon does. So the lab faults with
`__builtin_trap()`, which faults everywhere. Whether Wokwi faults on the
unaligned access is Lab Part 6's open question.
