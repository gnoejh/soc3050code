# Lab 05 — GPIO and Interrupts

**SOC3050 ARM Edition · Nucleo-C031C6 · allow 2 hours**

**Reference**: [RM0490, STM32C0x1 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0490-stm32c0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) ·
**Board**: [ST Nucleo-C031C6 on Wokwi](https://docs.wokwi.com/parts/board-st-nucleo-c031c6) ·
**Board manual**: [UM2953, STM32 Nucleo-64 boards (MB1717)](../_docs/UM2953_Nucleo64_MB1717.pdf)

---

## What this lab is

**A guided walkthrough, not a test.** Two buttons do the same job — one polled,
one on an interrupt — and you measure what each one sees. Then you break the
interrupt path one hop at a time, because every one of those breaks builds with
zero warnings and you need to have seen each symptom once. The last part is
yours: a third button, wired and interrupting from scratch.

Each part says **Do this**, asks you to **guess**, then explains **what you see
and why**. Nothing is marked or collected.

**Put every change back before the next part** unless the part says otherwise.

---

## Part 0 — Build it, run it, read the banner (20 min)

### Step 1. Build and simulate

```
build.bat
simulate.bat
```

Zero warnings, and:

```
Building 05_GPIO_And_Interrupts  [target c031c6] ...
           FLASH:        7156 B        32 KB     21.84%
             RAM:        2008 B        12 KB     16.34%
Build OK: Main.elf, Main.hex and Main.bin created
```

This folder has **no `startup.c` and no `link.ld`**. You wrote those in lesson
04; from now on the engine links the shared copies in `_startup\c031c6\`. The
printf glue moved too, to `_lib\retarget.c` — that is what `set LIBS=retarget`
in `build.bat` asks for.

In the Wokwi tab, paste this folder's `diagram.json` into the `diagram.json`
tab (**required this time** — the buttons live there), then `F1` → **Upload
Firmware and Start Simulation…** → `Main.elf`.

### Step 2. Read the banner

Every value in it was **read back from a register**, not printed from the code:

```
-- how the port came out of reset --
  GPIOA->MODER    : 0x........   (RM0490 says 0xEBFFFFFF: ...
-- the pins, read back from MODER and PUPDR --
  PA0 button A : input   pull-up   polled every 1 ms
  PA1 button B : input   pull-up   EXTI line 1
  ...
-- the interrupt path, hop by hop --
  EXTICR[0] line 1 port : 0    (0 = port A)
  RTSR1 / FTSR1 bit 1   : 1 / 1 (rising / falling edge)
  IMR1 bit 1            : 1    (1 = unmasked)
  NVIC IRQ 5 enabled    : 1    priority 2 of 0..3
  vector 21             : 0x0800023D  EXTI0_1_IRQHandler - yours
```

**Check the first line against slide 6.** If it prints `0xEBFFFFFF`, the
simulator models the reset state faithfully. If it prints something else, write
it down — that is a difference between Wokwi and silicon, and knowing where
your simulator is lenient is part of using one.

**Check the last line against slide 27.** `0x0800023D` is odd — the Thumb bit —
and `0x0800023C` is where `nm` says your handler starts.

### Step 3. Press the buttons

Click them, or click the diagram once and hold the **A** or **B** key. A moves
the lit LED left, B moves it right, and LD4 toggles every half second
throughout — the proof that `main()` is still running.

**Guess first:** for one press of A and one press of B, which prints the bigger
number?

> Something like:
>
> ```
> A polled     press     1   level changes seen at 1 kHz: 1
> A polled     release   1   level changes seen at 1 kHz: 1
> B interrupt  press     1   edges caught by EXTI:        23
> B interrupt  release   1   edges caught by EXTI:        17
> ```
>
> Your B numbers will differ press to press — that is bounce, and it is random.
> A stays at 1, occasionally 2 or 3.
>
> **Both buttons bounce the same way.** EXTI catches edges microseconds apart;
> the 1 kHz poll samples once a millisecond, so almost every bounce falls
> between two samples and is never seen. Polling is a low-pass filter you get
> for free. The interrupt shows you the truth, and then `main()` has to filter
> it itself — the quiet-time rule on slide 31.

---

## Part 1 — Forget input mode (10 min)

In `gpio_init()`, delete:

```c
    pin_mode(GPIOA, BTN_A_PIN, MODE_INPUT);
```

Rebuild (it builds clean) and re-upload. **Don't press anything yet.**

**Guess first:** slide 6 says what PA0 is now. What does the program think
button A is doing?

> The banner says `PA0 button A : ANALOG`, and on silicon the idle level reads
> `A=0` — an analog pin's input buffer is off, and `IDR` reads 0. For an
> active-low button, 0 means **pressed**.
>
> So 20 ms after reset you get a press nobody made:
>
> ```
> A polled     press     1   level changes seen at 1 kHz: 1
> ```
>
> and the LED moves one place left. After that **button A is dead**: the
> program believes it is being held down forever, and pressing it changes
> nothing, because nothing reaches `IDR`.
>
> If your simulator shows `A=1` and a working button, Wokwi is being kinder
> than silicon — the chip would not be. Either way, the rule stands: **write
> the mode you want; never inherit the reset state.**

---

## Part 2 — Switch the bounce off (10 min)

In `diagram.json`, add `"bounce": "0"` to button B's `attrs`:

```json
"attrs": { "color": "green", "label": "B  PA1 interrupt", "key": "b", "bounce": "0" }
```

Paste the edited diagram into Wokwi, restart the simulation, press B a few
times. No rebuild needed — the firmware did not change, the button did.

**Guess first:** how many edges per press now?

> **Exactly 1 per press, 1 per release.** A clean switch changes level once
> each way, and `RTSR1`/`FTSR1` catch each change once.
>
> That makes every number from Part 0 above 1 a bounce — so you have just
> *measured* bounce, as a count, on a button you could not otherwise see
> inside. Try the same on button A: it barely changes, because the poll was
> already hiding most of it.

Put the diagram back afterwards; the rest of the lab wants a bouncing button.

---

## Part 3 — Forget to clear the pending bit (15 min)

In `EXTI0_1_IRQHandler()`, delete the two clearing lines:

```c
        EXTI->RPR1 = mask;
        EXTI->FPR1 = mask;
```

Rebuild — clean — and re-upload. Watch LD4, then press **B** once.

**Guess first:** what happens to LD4? To button A?

> **Everything stops on the first press.** LD4 freezes on or off, button A
> does nothing, no line is ever printed for B.
>
> The first edge sets `FPR1` bit 1. The handler runs, counts, and returns —
> but the bit is still set, so the NVIC sees line 1 still pending and
> **tail-chains straight back into the handler** (slide 29). It never lets
> `main()` run again. The chip is not crashed; it is busier than it has ever
> been, doing nothing useful, forever.
>
> This is the most common first-interrupt bug on any microcontroller, and it
> looks exactly like a hang. The give-away is that it starts precisely on the
> first event, not at reset.

---

## Part 4 — Misspell the handler (15 min)

Change one letter of the handler's name:

```c
void EXTI0_1_IRQhandler(void)      /* lower-case h */
```

Rebuild.

**Guess first:** does it build? With a warning?

> ```
>            FLASH:        7120 B        32 KB     21.73%
> Build OK: Main.elf, Main.hex and Main.bin created
> ```
>
> **Clean. Zero warnings.** And 36 bytes *smaller*.
>
> Ask the linker what it did:
>
> ```
> ..\..\tools\arm-toolchain\bin\arm-none-eabi-nm.exe Main.elf | findstr IRQ
> ```
>
> ```
> 080005d0 W EXTI0_1_IRQHandler
> 080005d0 W EXTI2_3_IRQHandler
> 080005d0 W EXTI4_15_IRQHandler
> ...
> ```
>
> `EXTI0_1_IRQHandler` is `W` — the weak alias from `startup.c`, at the same
> address as all the other unused handlers: `Default_Handler`. Your
> `EXTI0_1_IRQhandler` is **not in the list at all**. Nothing called it, so
> `--gc-sections` removed it as dead code. Those are the 36 bytes.

Now upload it.

> The banner catches it before you press anything:
>
> ```
>   vector 21             : 0x080005D1  Default_Handler - YOUR HANDLER IS NOT INSTALLED
> ```
>
> Press B and the board hangs — the first edge jumps to `Default_Handler`,
> which is an infinite loop by design, so that a debugger finds the chip parked
> somewhere obvious instead of wandering.
>
> **Keep the habit this part teaches:** after writing any handler, run `nm` and
> look for a `T`. It takes five seconds and it is the only check that sees this.

---

## Part 5 — Forget the NVIC (15 min)

In `button_b_irq_init()`, comment out:

```c
    NVIC_EnableIRQ(EXTI0_1_IRQn);
```

Rebuild, re-upload, press B.

> The banner already told you: `NVIC IRQ 5 enabled    : 0`. B does nothing at
> all; A still works. Unlike Part 3 and Part 4, nothing hangs — the interrupt
> simply never arrives.

**Is EXTI still working?** Find out. In `main()`'s heartbeat block, after the
`if (beat_on)` line, add:

```c
            printf("FPR1 bit 1 = %lu\n", (unsigned long)((EXTI->FPR1 >> 1) & 1u));
```

Rebuild, re-upload, watch a few lines, then press B.

> ```
> FPR1 bit 1 = 0
> FPR1 bit 1 = 0
> FPR1 bit 1 = 1      <- you pressed B
> FPR1 bit 1 = 1
> FPR1 bit 1 = 1
> ```
>
> **EXTI saw the edge and latched it.** Hops one and two of slide 23 worked.
> The pending bit is sitting there, set, and the NVIC is not letting it in,
> because nobody enabled IRQ 5. It stays 1 forever, because the only code that
> clears it is the handler that never runs.
>
> That is the method for every "my interrupt doesn't fire": **walk the path
> hop by hop and read the pending bit at each one.** Where the bit stops
> moving is where the fault is.

Remove the `printf` and put the `NVIC_EnableIRQ` back.

---

## Part 6 — `BSRR` against `ODR`, in the listing (15 min)

`bar_write()` writes the whole LED bar. Look at what it compiled to:

```
..\..\tools\arm-toolchain\bin\arm-none-eabi-objdump.exe --disassemble=bar_write Main.elf
```

```
08000228 <bar_write>:
 mvns  r3, r0              ; ~leds
 uxtb  r3, r3              ; keep 8 bits
 ldr   r2, [pc, #8]        ; r2 = GPIOB = 0x50000400
 lsls  r3, r3, #16         ; into the reset half
 orrs  r3, r0              ; set half = leds
 str   r3, [r2, #24]       ; BSRR   <- the only access to GPIOB
 bx    lr
```

**One store, no load.** Now change its body to lesson 04's style:

```c
    GPIOB->ODR = leds;
```

Rebuild and disassemble again.

> It builds 8 bytes smaller and it is *also* one store — `str r0, [r3, #20]`,
> offset `0x14`, `ODR`. So which is better?
>
> The size says `ODR`. **The behaviour says `BSRR`.** `ODR = leds` writes all 16
> bits of the port: it drives PB0–PB7 as asked and **forces PB8–PB15 to 0**.
> Nothing is on PB8–PB15 today. The day something is — an SPI chip select in
> lesson 09, say — this line silently switches it off every frame.
>
> The version that does not clobber is `ODR = (ODR & ~0xFF) | leds`, which is a
> load, an AND, an OR and a store: lesson 03's lost-update window, back again.
> `BSRR` is the only way to change eight pins, leave eight alone, and not read
> the port. That is why the hardware has it.

Put `bar_write()` back.

---

## Part 7 — A third button, from scratch (open-ended)

Add button **C** on **PC13**, as an interrupt. When it is pressed, send the lit
LED home to PB0.

This touches every hop of slide 23, and two of them differently from button B.
Work through them in order:

1. **Wire it.** In `diagram.json`, copy `btnB`'s part and its two connections;
   give the copy id `btnC` and wire `btnC:1.r` to `nucleo:PC13`. The name is on
   the **Board pins** page in the top bar.
2. **Clock the port.** PC13 is on **GPIOC**, and nothing has opened that gate
   yet. Which bit of `RCC->IOPENR`?
3. **Configure the pin.** Input, pull-up — the helpers take any port.
4. **Select the port in EXTI.** Line 13. Which of the four `EXTICR` registers,
   which byte in it, and which code for port C? (Slide 24 works line 13 as its
   example — try it before you look.)
   *Try it once without this step first.* Button B worked without thinking
   about `EXTICR`; this one will not. Why?
5. **Pick edges, clear stale pending, unmask** — the same three lines as B,
   for line 13.
6. **Enable the right NVIC line.** Not IRQ 5. Slide 26 says which IRQ serves
   line 13, and ST's header gives it a name ending `_IRQn`.
7. **Write the handler.** Its name comes from the same place. It serves lines
   4 **to 15**, so check `FPR1` for bit 13 — never assume which line fired.
8. **Tell `main()`**, and move the LED there — not in the handler.

Then prove it before you press anything: the banner pattern works for any
vector, and `nm` should show your new handler with a `T`.

> **Why step 4 was needed this time:** the reset value of every `EXTICR` field
> is 0, which means **port A**. Button B is on port A, so forgetting `EXTICR`
> for B works by accident. With the field left at 0, line 13 watches **PA13** —
> the debug pin — and PC13 goes unheard. The code that forgot was always
> wrong; port A was hiding it.
>
> A worked version of this part was built while writing the lab: 7316 B FLASH,
> 2016 B RAM, zero warnings, and `nm` shows `T EXTI4_15_IRQHandler`. If yours
> is close to that, you are on track.

**Bring it to the next class** — lesson 06 adds time to the picture, and a
third input is exactly what a stopwatch needs.

---

## Part 8 — Reach both paths from the debugger (optional, 20 min)

Needs the Wokwi VS Code extension and `F5` — see "Debugging a lesson in VS
Code" in the ARM `README.md`. Everything else in this lab runs in a browser tab;
this part does not.

**Why clicking the buttons does not work under a debugger.** Three reasons,
and knowing them is most of the skill:

1. A click is a press *and* a release, over in a fraction of a second. While
   the chip is halted at a breakpoint or being stepped, **simulated time is
   frozen**, so a click made then is finished before the program ever samples
   the pin.
2. A breakpoint on `b_edges++` stops on **every bounce edge** — ten to a
   hundred stops per press.
3. A breakpoint on the first line of `debounce_step()` stops **a thousand
   times a second**, and the accepted press is never reached. A conditional
   breakpoint reaches it, but crawls: GDB halts the chip to test the condition
   each time.

The rule for all three: **never step with the button in your hand.** Latch the
input, let the chip run, and let the program reach the breakpoint in its own
simulated time.

### Step 1. Latch button A; break where the press is accepted

Set the breakpoint (`F9`) on this line inside `debounce_step()` — the one that
runs once per *accepted* press or release, not the function's first line:

```c
            d->stable = raw;
```

Build, open `diagram.json`, Play, `F5`, and let it run. Now **Ctrl-click
button A** (Cmd-click on a Mac). Wokwi documents this as a sticky press: the
button stays down until the next click. Holding the **A** key does the same for
as long as you hold it.

**Guess first:** how long after the click does the breakpoint hit, and what are
`raw` and `held_ms` when it does?

> About 20 ms of simulated time later — `DEBOUNCE_MS` samples in a row at
> 1 kHz — with `raw = 1`, `held_ms = 20` and `changes` up by one. Step Over
> past the accept and the function returns `+1`; Continue, and `a_count++` and
> `bar_write()` run, so the LED moves. Click A again for the release: the same
> line hits with `raw = 0`.
>
> If you click while the chip is halted and nothing happens after Continue,
> the click was not delivered to the simulation. Continue first, then click.

### Step 2. Break in the handler; see the bounce one edge at a time

Move the breakpoint to `b_edges++`. Continue, then Ctrl-click **B**.

**Guess first:** how many times does it stop for one press?

> As many times as Part 0's "edges caught by EXTI" column said — every stop is
> one bounce, and `-exec p b_edges` in the Debug Console counts them for you.
> To stop **once** per press, either switch the bounce off as in Part 2, or put
> the breakpoint on `b_count++` in `main()`, where the press has been
> *decided*. That is the general rule: break where the event is decided, not
> where the signal arrives.

### Step 3. Fire the interrupt with no button at all

Keep the breakpoint on `b_edges++`. In VS Code's Debug Console, commands
prefixed `-exec` go straight to GDB:

```
-exec x/3wx 0x40021808
-exec set *(unsigned int *)0x40021808 = 2
-exec x/3wx 0x40021808
```

then Continue.

**Guess first:** what is at `0x40021808`, and does the handler run?

> `EXTI_BASE` is `0x40021800` in `stm32c031xx.h`, and offset `0x08` is
> **`SWIER1`**, the software interrupt event register. RM0490: writing 1 to
> `SWIx` sets bit *x* of `RPR1` and raises the line exactly as a rising edge
> would. The three words printed are `SWIER1`, `RPR1`, `FPR1`; after the write,
> `RPR1` shows bit 1 set, and the breakpoint hits with `b_edges` up by exactly
> one — a single clean edge, no bounce, no button, no mouse.
>
> The build uses `-g3`, so GDB also knows the header's macros; from a frame in
> `Main.c`, `-exec set EXTI->SWIER1 = 2` may work as well. Try it.
>
> **Then `main()` prints nothing.** The quiet-time rule waits 20 ms, reads the
> pin, and finds it released — as it was. The *interrupt* fired; the *press*
> never happened. That is the division of labour on slide 31, seen from the
> other side: the handler counts edges, `main()` decides what they meant.

### Step 4. Pend the NVIC directly, and skip EXTI

Now move the breakpoint to the handler's `if` line:

```c
    if ((EXTI->RPR1 | EXTI->FPR1) & mask)
```

and in the Debug Console:

```
-exec set *(unsigned int *)0xE000E200 = 32
```

then Continue.

**Guess first:** the handler is entered — is anything counted?

> `0xE000E200` is the NVIC's `ISPR`, and bit 5 pends IRQ 5. The breakpoint
> hits: the vector was taken. But `RPR1` and `FPR1` are both 0, the `if` is
> false, and `b_edges` does not move. The handler was called by the NVIC with
> no EXTI event behind it — and it was right not to believe the vector. Part 7
> step 7 said the same thing for a handler that serves lines 4 to 15: **check
> the pending bit; never assume which line fired, or that one fired at all.**
>
> Between them, Steps 3 and 4 let you *poke* every hop of slide 23 as well as
> read it, which is what Part 5 did. Any interrupt in the rest of the course
> can be tested this way before its hardware source exists.

**Not yet watched.** Steps 1 and 2 rest on Wokwi's documented pushbutton
behaviour (Ctrl-click latching, the `key` hold). Steps 3 and 4 write to
peripheral and core registers through Wokwi's GDB server, and whether its
STM32C0 model implements `SWIER1`, or its GDB server accepts writes there at
all, has not been confirmed. If `set` is refused or nothing changes, write down
exactly what you saw: that is a fact about the simulator, and the chip would
have obeyed.

---

## Where to go next

- Change `DEBOUNCE_MS` to 1, then to 200. At 1, does A ever report a double
  press? At 200, can you press fast enough for the program to miss one?
- Give button B **only** `FTSR1` (delete the `RTSR1` line). With Wokwi's
  default bounce, presses and releases are still reported. Now set
  `"bounce": "0"` as in Part 2: B works **once**, then never again. Trace
  `b_down` through `main()` to see why — and then explain how bounce let a
  falling-edge-only interrupt see releases at all.
- Read `tick_wait()`. Every `printf` above holds the loop for several
  milliseconds, and `COUNTFLAG` can only remember one missed tick. Estimate how
  far `ms_now` drifts per printed line. Lesson 06 fixes it.
