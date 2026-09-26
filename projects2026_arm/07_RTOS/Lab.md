# Lab 07 — RTOS

**SOC3050 ARM Edition · Nucleo-C031C6 · allow 2 hours**

**Reference**: [RM0490, STM32C0x1 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0490-stm32c0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) ·
**Board**: [ST Nucleo-C031C6 on Wokwi](https://docs.wokwi.com/parts/board-st-nucleo-c031c6) ·
**Board manual**: [UM2953, STM32 Nucleo-64 boards (MB1717)](../_docs/UM2953_Nucleo64_MB1717.pdf)

---

## What this lab is

**A guided walkthrough, not a test.** `os.c` is a working preemptive kernel.
You will watch it schedule, starve a system on purpose, reproduce the bug that
reset a spacecraft, overflow a stack and see where the damage lands, and fault
the CPU and make it tell you the line. Then you add a task of your own.

Unlike earlier labs, **most answers below were measured**: the kernel was run,
unchanged, on Renode's Cortex-M0 model (slide 20). Where Wokwi may differ, the
part says so.

**Put every change back before the next part** unless told otherwise.

---

## Part 0 — Build it, run it, read the table (20 min)

```
build.bat
simulate.bat
```

Zero warnings:

```
           FLASH:        7472 B        32 KB     22.80%
             RAM:        6128 B        12 KB     49.87%
```

RAM has jumped from ~2 KB to ~6 KB. **Guess first:** where did 4 KB go?

> Stacks. Four tasks at 128, 128, 128 and 512 words, plus idle's 64, is
> 960 words — 3840 bytes. Every task pays for a stack whether it uses it or
> not; that is the price of the model, and Part 3 is about paying the right
> amount.

Paste `diagram.json` (the bar and buttons A and B, as in lesson 05), upload
`Main.elf`. The LED bar bounces; every second the table prints:

```
t=  3001 ms   switches/s 419
  task      prio  st  cpu%   stack used/size
  bar       2/2   S     0      24 / 128
  button    3/3   S     0      30 / 128
  hog       1/1   R   100      20 / 128
  report    1/1   R     0     115 / 512
  idle      0/0   R     0      16 / 64
  bar 60 ms/frame   hog ON , 2446 primes/s
```

**Read it:**

> - `st`: `R` ready/running, `S` sleeping, `B` blocked.
> - `hog` 100%: it never sleeps, so it gets every tick nobody else wants.
> - `bar` and `button` 0%, and **both still work** — press A and the bar
>   speeds up. They wake, preempt the hog within one tick, work for a few
>   microseconds and sleep again. The profiler samples once a millisecond, so
>   work that short never shows.
> - `primes/s` falls from second to second: the numbers are getting bigger,
>   and trial division gets slower. Not a bug.

**Press B.** The hog switches off.

> Measured: the next table shows `hog ... S 19` (it ran for part of that
> second) and `idle 80`, and the one after, `idle 100`. The CPU time the hog
> gave up went, whole, to the idle task.

---

## Part 1 — Priority is a promise to starve (10 min)

In `create_tasks()`, give the hog the highest priority in the system:

```c
    os_task_create("hog",    task_hog,    0, stk_hog,    128, 4);   /* was 1 */
```

**Guess first:** what still works?

> **Nothing.** Measured: the banner prints, and then not one more line. The
> bar freezes, the buttons are dead, the report never runs.
>
> The kernel is doing exactly what it promised: the highest-priority ready
> task runs. The hog is always ready. So nothing else ever runs.
>
> A preemptive kernel does not make a CPU-bound task harmless; it makes it
> harmless **at the right priority**. The rule of slide 15 in lesson 06 is
> the rule here too: priority follows deadline. The hog has no deadline at
> all — it belongs at the bottom.

---

## Part 2 — Forge the frame wrong (10 min)

In `os_task_create()` in `os.c`, clear the Thumb bit in the forged `xPSR`:

```c
    *--sp = 0x00000000u;          /* was 0x01000000u */
```

**Guess first:** when does it go wrong — at build, at reset, or later?

> Measured, immediately after the banner:
>
> ```
> *** HardFault in task 'button'
>     pc   = 0x08000228   <- the instruction that faulted
>     xPSR = 0x00000000
> ```
>
> `0x08000228` is the **first instruction of `task_button`** — the
> highest-priority task, so the first one switched to. PendSV returned into
> it with T = 0, which asks for ARM state; the M0+ has no ARM state, and
> faulted before executing a single instruction of it.
>
> Notice that `pc` is where the task *would* have started. The forged frame
> did its job perfectly — except for one bit.

---

## Part 3 — Priority inversion, and the fix (20 min)

At the top of `Main.c`:

```c
#define SCENARIO     2
```

Rebuild, run, and let it go for five seconds. Then in `os.h`:

```c
#define OS_PRIORITY_INHERIT   0
```

and run again.

**Guess first:** low holds the mutex for 30 ms. How long can high wait?

> Measured over five seconds each:
>
> ```
> high: 49 locks, worst wait 10 ms,  waits over 40 ms: 0   (inheritance ON)
> high: 49 locks, worst wait 110 ms, waits over 40 ms: 6   (inheritance OFF)
> ```
>
> Without inheritance, high waited up to **110 ms** for a 30 ms critical
> section. The other 80 ms it was waiting for **mid** — which has lower
> priority than high and never touches the mutex. That is slide 13's diagram,
> measured.
>
> With inheritance, low is lifted to high's priority while high waits, mid
> cannot preempt it, and high's worst wait drops to 10 ms.
>
> Look at the `mid` row in the table while you are there: mid's CPU share is
> the same either way. Inheritance does not take time from mid; it changes
> **when** mid gets it.

Set both back to 1.

---

## Part 4 — A stack that is too small (15 min)

Part 0's table says the report task, which calls `printf`, has used **115**
words of its 512. Give it 96: change **both** the `STACK(stk_report, 512)`
declaration and the `512` in its `os_task_create()` line.

**Guess first:** crash, or not?

> Measured — **no crash**:
>
> ```
>   report    1/1   R     0      96 / 96    OVERFLOW
>   bar 60 ms/frame   hog ON , 5539 primes/s
>   ...
>   bar 60 ms/frame   hog ON , 536871010 primes/s
> ```
>
> The high-water mark says the whole stack was used. And the next report's
> prime count is nonsense. Ask the linker why:
>
> ```
> ..\..\tools\arm-toolchain\bin\arm-none-eabi-nm.exe -n Main.elf | findstr "primes stk_report"
> ```
>
> `primes` sits at `0x20000084`, the word directly below `stk_report` at
> `0x20000088`. The report task wrote past the bottom of its stack and landed
> in the hog's counter. 536 871 010 is `0x20000062` — a stack address.
>
> The symptom appeared in a **different task's data**. Without the high-water
> mark you would be debugging the hog. This is why real kernels check stacks
> on every switch, and why the Cortex-M33 added a hardware stack limit
> register.

Put it back to 512.

---

## Part 5 — Make it fault, and make it say where (15 min)

In `task_hog()`, after the `primes++` line, add:

```c
        if (primes == 3000u) { __builtin_trap(); }
```

Rebuild, upload, wait about half a second.

> Measured:
>
> ```
> *** HardFault in task 'hog'
>     pc   = 0x08000422   <- the instruction that faulted
>     lr   = 0x08000415   <- where it would have returned to
>     xPSR = 0x61000000
>     Find the line:  arm-none-eabi-addr2line -e Main.elf 0x08000422
> ```
>
> (Your addresses may differ by a few bytes if your edit differs.) Run the
> command it prints, from this folder:
>
> ```
> ..\..\tools\arm-toolchain\bin\arm-none-eabi-addr2line.exe -e Main.elf 0x08000422
> ```
>
> It prints `Main.c:` and the line number of your `__builtin_trap()`. The
> instruction there is `udf` — *undefined* — which is what `__builtin_trap()`
> compiles to, precisely so that it faults on every Arm core.

**The open question.** Replace the trap with a realistic bug, an unaligned
word store:

```c
        if (primes == 3000u) { *(volatile uint32_t *)0x20002801u = 1u; }
```

On real ARMv6-M silicon that is a HardFault. **Renode does not fault on it**
— measured: the reports carry on as if nothing happened. What does Wokwi do?
Nobody on this course has checked yet. Your answer decides whether this lab
can use the realistic bug next year.

---

## Part 6 — A task that waits for events (open-ended)

Add a **logger** task that prints a line for every button press:

```
button A at 1300 ms
button B at 1750 ms
```

Rules: the button task must not call `printf` (it has the smallest stack and
the highest priority), and the logger must use **no CPU** while nobody is
pressing anything.

> A queue does it (slide 15). Give the logger its own stack — how big, given
> Part 0's table? — a queue of small structs, and `os_queue_get()` in a loop.
> The button task does `os_queue_put()` on each press.
>
> Measured, one working version: the logger's row reads `log 2/2 B 0` — **B**,
> blocked, 0% — until a press, and its stack high-water mark is 89 words
> after its first `printf`. It builds with zero warnings.
>
> What you have built is the shape of lesson 08: an event source, a queue,
> and a task that sleeps until there is something to do.

---

## Where to go next

- Set `bar` and `hog` to the **same** priority, 1. The bar still runs — why?
  (Slide 9: round-robin.) Does it keep perfect time now?
- Count the instructions in `PendSV_Handler` with
  `arm-none-eabi-objdump --disassemble=PendSV_Handler Main.elf`. At 48 MHz,
  roughly how long is a switch? What fraction of the CPU do ~420 switches a
  second cost?
- Read FreeRTOS's `port.c` for the Cortex-M0 (`portable/GCC/ARM_CM0`). Find its
  `xPortPendSVHandler`. How much of it do you now recognise?
