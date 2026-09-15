# Execution and Concurrency: When Two Things Run at Once
## SOC3050 ARM Edition — Part 0, Models

**Reference**: [STM32 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0490-stm32c0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

The last of four lessons with **no code to write**.

You have the machine, its instructions, and the toolchain. One model is left,
and it is the one that produces the hardest bugs in this course: what happens
when the hardware interrupts your program.

---

## Slide 1: Three Ways to Meet the Hardware

A peripheral finishes when it finishes. Your program has to find out. There are
exactly three arrangements, and every embedded design is built from them.

```svg
<svg viewBox="0 0 560 250" role="img" aria-label="Polling, interrupt and DMA compared">
  <defs><marker id="c1" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <text x="90" y="20" text-anchor="middle" class="lbl">POLL</text>
  <rect class="box" x="20" y="32" width="140" height="34" rx="5"/>
  <text x="90" y="49" text-anchor="middle" class="mono">ready yet?</text>
  <path class="wire" d="M160 49 H178 V86 H20 V66" marker-end="url(#c1)"/>
  <text x="90" y="112" text-anchor="middle" class="lbl">CPU does nothing else</text>
  <text x="90" y="130" text-anchor="middle" class="lbl">simplest to get right</text>
  <text x="280" y="20" text-anchor="middle" class="lbl">INTERRUPT</text>
  <rect class="box" x="210" y="32" width="140" height="34" rx="5"/>
  <text x="280" y="49" text-anchor="middle" class="mono">useful work</text>
  <path class="hi" d="M355 49 V74" marker-end="url(#c1)"/>
  <rect class="hifill" x="210" y="78" width="140" height="30" rx="5"/>
  <text x="280" y="93" text-anchor="middle" class="mono">handler</text>
  <text x="280" y="130" text-anchor="middle" class="lbl">CPU works until told</text>
  <text x="470" y="20" text-anchor="middle" class="lbl">DMA</text>
  <rect class="box" x="400" y="32" width="140" height="34" rx="5"/>
  <text x="470" y="49" text-anchor="middle" class="mono">useful work</text>
  <rect class="reg" x="400" y="78" width="140" height="30" rx="5"/>
  <text x="470" y="93" text-anchor="middle" class="lbl">hardware moves bytes</text>
  <text x="470" y="130" text-anchor="middle" class="lbl">CPU not involved at all</text>
  <text x="280" y="182" text-anchor="middle" class="lbl">Cost rises left to right in complexity, and falls in CPU time.</text>
  <text x="280" y="206" text-anchor="middle" class="lbl">Polling is not the beginner option — it is the right answer whenever</text>
  <text x="280" y="224" text-anchor="middle" class="lbl">the wait is shorter than the cost of being interrupted.</text>
</svg>
```

---

## Slide 2: An Interrupt Is a Hardware-Forced Function Call

No scheduler, no operating system. The hardware does this between two
instructions of whatever was running:

```svg
<svg viewBox="0 0 560 230" role="img" aria-label="Interrupt entry: stack the frame, look up the vector, run the handler, return">
  <defs><marker id="c2" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="20" y="30" width="150" height="30" rx="4"/>
  <text x="95" y="45" text-anchor="middle" class="mono">main() running</text>
  <path class="hi" d="M170 45 H206" marker-end="url(#c2)"/>
  <text x="188" y="26" text-anchor="middle" class="lbl">event</text>
  <rect class="hifill" x="210" y="30" width="150" height="30" rx="4"/>
  <text x="285" y="45" text-anchor="middle" class="lbl">push 8 registers</text>
  <path class="wire" d="M285 60 V82" marker-end="url(#c2)"/>
  <rect class="box" x="210" y="82" width="150" height="30" rx="4"/>
  <text x="285" y="97" text-anchor="middle" class="lbl">read the vector table</text>
  <path class="wire" d="M285 112 V134" marker-end="url(#c2)"/>
  <rect class="box" x="210" y="134" width="150" height="30" rx="4"/>
  <text x="285" y="149" text-anchor="middle" class="mono">handler runs</text>
  <path class="wire" d="M210 149 H120 V60" marker-end="url(#c2)"/>
  <text x="150" y="112" text-anchor="middle" class="lbl">pop and resume</text>
  <text x="450" y="60" class="lbl">R0-R3, R12,</text>
  <text x="450" y="78" class="lbl">LR, PC, xPSR</text>
  <text x="450" y="100" class="lbl">stacked by the</text>
  <text x="450" y="118" class="lbl">hardware itself</text>
  <text x="280" y="204" text-anchor="middle" class="lbl">Your main program never knows it happened. That is the problem.</text>
</svg>
```

The processor pushes eight registers for you — which is exactly the set AAPCS
lets a function clobber, so **an ISR can be an ordinary C function**. No special
keyword, no `ISR()` macro. You simply define a function with the right name and
the linker puts it in the vector table.

> Spell the name wrong and you have written an unused function. It compiles, it
> links, it runs, and the interrupt does nothing. No warning, anywhere.

---

## Slide 3: Now You Have Two Programs Sharing One Memory

This is the whole reason concurrency is a topic. Before interrupts, your
program had one thread of control. After, it has two — and they share every
global variable.

```svg
<svg viewBox="0 0 560 200" role="img" aria-label="Main and ISR contexts sharing a variable">
  <defs><marker id="c3" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <rect class="box" x="24" y="30" width="160" height="48" rx="6"/>
  <text x="104" y="50" text-anchor="middle" class="mono">main()</text>
  <text x="104" y="68" text-anchor="middle" class="lbl">runs when nothing else does</text>
  <rect class="hifill" x="376" y="30" width="160" height="48" rx="6"/>
  <text x="456" y="50" text-anchor="middle" class="mono">handler()</text>
  <text x="456" y="68" text-anchor="middle" class="lbl">runs whenever it likes</text>
  <path class="wire" d="M104 78 V116 H256" marker-end="url(#c3)"/>
  <path class="hi" d="M456 78 V116 H304" marker-end="url(#c3)"/>
  <rect class="reg" x="228" y="116" width="104" height="40" rx="5"/>
  <text x="280" y="136" text-anchor="middle" class="mono">counter</text>
  <text x="280" y="186" text-anchor="middle" class="lbl">Neither one can see what the other is doing.</text>
</svg>
```

Two distinct hazards follow, and they need different fixes. Students routinely
apply one fix to the other problem and conclude the language is broken.

---

## Slide 4: Hazard One — the Compiler Does Not Believe You

```c
while (flag == 0) { }        /* wait for the ISR to set it */
```

The compiler analyses `main()` and reasons, correctly by the rules of C: *this
loop does not modify `flag`, so `flag` cannot change; I will read it once and
loop on a register.* The result is an infinite loop that no amount of staring
at the source will explain.

**The general idea is the as-if rule.** A C compiler is not required to do what
you wrote; it is required to produce a program that *behaves* as if it had,
judged only by the observable behaviour the language defines. Everything else —
how many times memory is read, in what order, whether a variable exists at all —
it may change freely. That licence is where optimisation comes from, and it is
not an embedded matter: the same rule governs the compiler on your laptop.

What differs here is that **the hardware is an observer the language does not
know about.** A pin that changes, a status bit that clears when read, another
context writing a variable — none of these are in C's account of what is
observable, so the compiler is free to optimise them away and is not
misbehaving when it does.

**Fix: `volatile`.** It is C's way of extending the definition of observable:
this object changes for reasons outside the program, so every access written
must actually happen, all of them, in the order written.

```c
volatile uint8_t flag;
```

`volatile` is about the *compiler*. It is not a lock, it does not make anything
atomic, and it does not help with the next slide.

---

## Slide 5: Hazard Two — the Interrupt Lands Mid-Update

Recall from lesson 01 that touching memory is always three instructions. Here
is `counter++` in `main()`, interrupted at the worst moment:

```svg
<svg viewBox="0 0 560 250" role="img" aria-label="A lost update when an interrupt occurs between load and store">
  <text x="14" y="22" class="lbl">main()</text>
  <rect class="box" x="14" y="32" width="210" height="28" rx="4"/>
  <text x="119" y="47" text-anchor="middle" class="mono">ldr r0, [counter]   ; 5</text>
  <rect class="dash" x="14" y="68" width="210" height="28" rx="4"/>
  <text x="119" y="83" text-anchor="middle" class="lbl">— interrupted here —</text>
  <rect class="box" x="14" y="146" width="210" height="28" rx="4"/>
  <text x="119" y="161" text-anchor="middle" class="mono">adds r0, #1         ; 6</text>
  <rect class="box" x="14" y="182" width="210" height="28" rx="4"/>
  <text x="119" y="197" text-anchor="middle" class="mono">str r0, [counter]   ; 6</text>
  <text x="336" y="22" class="lbl">handler()</text>
  <rect class="hifill" x="336" y="68" width="210" height="28" rx="4"/>
  <text x="441" y="83" text-anchor="middle" class="mono">ldr  ; reads 5</text>
  <rect class="hifill" x="336" y="104" width="210" height="28" rx="4"/>
  <text x="441" y="119" text-anchor="middle" class="mono">adds ; 6</text>
  <rect class="hifill" x="336" y="140" width="210" height="28" rx="4"/>
  <text x="441" y="155" text-anchor="middle" class="mono">str  ; writes 6</text>
  <text x="280" y="234" text-anchor="middle" class="lbl">Two increments happened. The counter went from 5 to 6. One update vanished.</text>
</svg>
```

`volatile` does not help here — every access did happen, in order, exactly as
written. The problem is that the *sequence* was not indivisible.

**These are the standard names, and they are not embedded-specific.** What you
just watched is a **race condition**: the result depends on the relative timing
of two contexts. The region of code that must not be interleaved is a
**critical section**; the guarantee you need over it is **mutual exclusion** —
at most one context inside at a time. A sequence that cannot be observed
half-done is **atomic**. Every concurrent system in computing is built from
those four words, whether the two contexts are an ISR and `main()`, two threads
on a laptop, or two processes writing one file.

What changes between systems is only **how** mutual exclusion is obtained:

| Contexts | How you get mutual exclusion |
|---|---|
| ISR vs `main()`, one core | **turn interrupts off** — the other context cannot run if it cannot start |
| two threads, one OS | a mutex, which asks the scheduler not to switch |
| two cores | neither of the above works — an atomic read-modify-write instruction is required |

The third row is worth noticing. Masking interrupts stops the *other context on
this core*; it does nothing to a second core, which is still running. Larger ARM
cores provide `ldrex`/`strex` for that, and x86 has `lock cmpxchg`. **A Cortex-M0+
is single-core and has neither**, which is exactly why interrupt masking is the
right tool here and would be the wrong tool on a phone.

**This chip's answer: a critical section by masking.** Stop interrupts for the
three instructions:

```c
__disable_irq();
counter++;
__enable_irq();
```

Keep it short. Every cycle spent with interrupts off is a cycle in which the
chip cannot respond to anything.

> The two fixes are not interchangeable. `volatile` makes accesses *real*;
> a critical section makes a sequence *indivisible*. Most concurrency bugs in
> this course come from using the first where the second was needed.

---

## Slide 6: A Bug That Only Appears Sometimes

The diagram above required the interrupt to land in a two-instruction window.
On a 48 MHz chip that window is about 40 nanoseconds wide.

| If the interrupt fires | The bug appears |
|---|---|
| 1000 times a second | roughly once every 7 hours |
| 10000 times a second | roughly once every 40 minutes |

This is why concurrency bugs have a reputation. They survive your testing,
they survive the demo, and they appear in the field. **You cannot find them by
running the program more.** You find them by reasoning about which variables
are shared, and protecting those.

> A useful discipline: for every global, write down which contexts touch it.
> If the answer is more than one, it needs `volatile`, and probably a critical
> section too.

---

## Slide 7: Not Everything Needs Protecting

Turning interrupts off has a cost, so be precise about when it is needed.

| Situation | Safe? |
|---|---|
| ISR writes a `uint8_t`, main only reads it | **Yes** — single aligned access, one writer |
| ISR sets a flag, main reads and clears it | **Usually** — but the clear is read-modify-write |
| main does `counter++`, ISR also increments | **No** — the lost update above |
| main reads a `uint64_t` the ISR writes | **No** — more than one bus access; you can read half |
| Either side touches a multi-field struct | **No** — the reader can see it half-updated |

The pattern: **a single aligned word written by exactly one context is safe.**
Everything else needs thought.

The failure in rows four and five has a name — **tearing**. A reader that needs
two bus accesses to collect one logical value can be interrupted between them
and walk away with half of the old value and half of the new: a number that was
never stored by anybody. It is the same hazard that makes a 64-bit counter
unsafe on a 32-bit machine in any language, on any operating system, and the
same reason a shared struct needs protecting even when each field is small.

---

## Slide 8: Priority — Which Interrupt Wins

Several peripherals can want attention at once. The NVIC decides, by priority,
and it can also let a more urgent interrupt preempt a handler already running.

```svg
<svg viewBox="0 0 560 220" role="img" aria-label="A higher priority interrupt preempting a running handler">
  <defs><marker id="c4" markerWidth="7" markerHeight="7" refX="6.2" refY="3" orient="auto" markerUnits="userSpaceOnUse">
    <path d="M0.4 0.7 L6.2 3 L0.4 5.3 z" fill="currentColor"/></marker></defs>
  <text x="18" y="36" class="lbl">main</text>
  <path class="wire" d="M70 30 H160"/>
  <path class="dash" d="M160 30 H430"/>
  <path class="wire" d="M430 30 H540"/>
  <text x="18" y="100" class="lbl">timer</text>
  <path class="wire" d="M160 94 H230"/>
  <path class="dash" d="M230 94 H360"/>
  <path class="wire" d="M360 94 H430"/>
  <path class="hi" d="M160 36 V88" marker-end="url(#c4)"/>
  <path class="hi" d="M430 88 V36" marker-end="url(#c4)"/>
  <text x="18" y="164" class="lbl">uart</text>
  <rect class="hifill" x="230" y="144" width="130" height="26" rx="4"/>
  <text x="295" y="158" text-anchor="middle" class="lbl">higher priority</text>
  <path class="hi" d="M230 100 V140" marker-end="url(#c4)"/>
  <path class="hi" d="M360 140 V100" marker-end="url(#c4)"/>
  <text x="280" y="200" text-anchor="middle" class="lbl">The timer handler was itself interrupted. Everything it shares is now at risk too.</text>
</svg>
```

Two consequences worth carrying forward:

- **Handlers must be short.** Anything slow in a handler delays every interrupt
  of equal or lower priority. The usual shape is: set a flag or push to a
  buffer, return, and let `main()` do the work.
- **Preemption multiplies the sharing problem.** It is not only main-versus-ISR
  any more; it is ISR-versus-ISR.

The vocabulary here is shared with operating-system scheduling, because it is
the same problem. **Preemption** is one runnable context displacing another
mid-execution; **interrupt latency** is the gap between the hardware event and
the first instruction of your handler, and the longest critical section anywhere
in your program sets its worst case. There is even an inherited failure mode —
**priority inversion**, where a high-priority task waits on something a
low-priority one holds. It grounded a spacecraft once: Mars Pathfinder kept
resetting on the surface in 1997 for exactly this reason.

---

## Slide 9: The Model, Complete

Four lessons, one picture:

```svg
<svg viewBox="0 0 560 240" role="img" aria-label="The complete Part 0 model">
  <rect class="box" x="30" y="24" width="230" height="88" rx="7"/>
  <text x="145" y="46" text-anchor="middle">THE MACHINE</text>
  <text x="145" y="68" text-anchor="middle" class="lbl">one bus, one address space</text>
  <text x="145" y="86" text-anchor="middle" class="lbl">peripherals = control / status / data</text>
  <text x="145" y="104" text-anchor="middle" class="lbl">guarded by a clock gate and a mux</text>
  <rect class="box" x="300" y="24" width="230" height="88" rx="7"/>
  <text x="415" y="46" text-anchor="middle">THE PROGRAM</text>
  <text x="415" y="68" text-anchor="middle" class="lbl">load, modify, store</text>
  <text x="415" y="86" text-anchor="middle" class="lbl">sections placed by a linker</text>
  <text x="415" y="104" text-anchor="middle" class="lbl">startup code before main()</text>
  <rect class="hifill" x="30" y="134" width="500" height="56" rx="7"/>
  <text x="280" y="156" text-anchor="middle">AND THEY MEET ASYNCHRONOUSLY</text>
  <text x="280" y="178" text-anchor="middle" class="lbl">volatile makes accesses real · critical sections make them indivisible</text>
  <text x="280" y="222" text-anchor="middle" class="lbl">Everything from here is an instance of this.</text>
</svg>
```

---

## Slide 10: What You Should Be Able to Say

1. Name the three ways to meet hardware, and give a case where polling is the
   right choice.
2. Why can an ISR on this processor be an ordinary C function with no special
   keyword?
3. What exactly does `volatile` prevent? Name one thing it does **not** do.
4. Draw the instruction sequence that loses an update to a shared counter.
5. A shared variable is a single aligned `uint8_t`, written only by the ISR and
   only read by main. Does it need a critical section? Why?
6. Why must interrupt handlers be short?
7. Your program works for hours, then misbehaves once. What class of bug is
   this, and why will running it more not help?
8. State the as-if rule in one sentence, and say why hardware breaks the
   assumption it rests on.
9. Define *race condition*, *critical section*, *mutual exclusion* and *atomic*
   without mentioning ARM.
10. Why would `__disable_irq()` be an insufficient lock on a dual-core chip, and
    what mechanism replaces it there?
11. What is *tearing*, and which two rows of the table on slide 7 are instances
    of it?

---

## Part 0 is finished. What changes now.

From here every lesson has code, and every lesson follows the same three beats:

> **Model → Map → Measure**
>
> - **Model** — which part of Part 0 is this? *(you can already answer this)*
> - **Map** — onto this chip's registers, by reading the reference manual
> - **Measure** — write it, run it, and watch it work

The first is `04_Startup_And_Linker`, where you write the `startup.c` and
`link.ld` that lesson 02 described — and the chip boots because you made it.
