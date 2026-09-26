# Sensors and Buses: Three Ways the World Gets In
## SOC3050 ARM Edition — Part 1, Instances

**Reference**: [STM32 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0490-stm32c0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) ·
**Board manual**: [UM2953, STM32 Nucleo-64 boards (MB1717)](https://www.st.com/resource/en/user_manual/um2953-stm32c0-nucleo64-board-mb1717-stmicroelectronics.pdf)

**A ball that rolls when you tilt the board — and the three buses it
takes.**

A potentiometer is a voltage: the **ADC** reads it. An accelerometer is a chip
with registers at an address: the **I²C** bus reaches it. A display driver is a
chip that just listens: the **SPI** bus feeds it. Every sensor and actuator in
Part 3 — the drone's IMU, a robot's motor driver — arrives through one of
these three doors.

Figures marked **measured** come from running this lesson's firmware in Renode
(lesson 07, slide 20), which models all three buses but none of these three
devices; slide 17 says exactly what that could and could not show.

---

## Slide 1: Three Doors, One Model

| | ADC | I²C | SPI |
|---|---|---|---|
| **what comes in** | a voltage | bytes from a chip at an address | bytes from a chip you select |
| **wires** | 1 per input | 2, shared by up to ~100 devices | 3–4, plus 1 select per device |
| **here** | pot on PA4 | MPU6050 at `0x68`, PB8/PB9 | MAX7219, PA5/PA7, CS PB0 |
| **speed here** | ~7 µs per conversion | 400 kHz | 6 MHz |
| **clock gate** | `APBENR2.ADCEN` | `APBENR1.I2C1EN` | `APBENR2.SPI1EN` |

Each is the Part 0 peripheral model once more — gate, control, status, data —
and each adds exactly one new idea: the ADC a **start-up sequence**, I²C
**addresses and acknowledgement**, SPI **selection by wire**.

---

## Slide 2: The ADC — a Voltage Becomes a Number

The STM32C0's ADC is **successive approximation**: it compares the input with
a DAC's output, halving the range each step, twelve steps for twelve bits.

```
code = input / VREF+ × 4095          0 V -> 0      VREF+ -> 4095
```

On this board VREF+ is the supply, **VDDA**, nominally 3.3 V, so one step is
about **0.8 mV**. Two consequences:

- The answer is only as good as your knowledge of VDDA. Slide 5 measures it.
- The number is an integer, and so is every calculation on it here — the
  M0+ has no FPU (lesson 06): `mV = code * vdda_mv / 4095` in `int32_t`.

In Wokwi the potentiometer is the input: `adc_read(ADC_CH_PA4)` and its knob
sets the MAX7219's brightness, `code >> 8`, 0..15.

---

## Slide 3: A Start-Up Sequence You Must Follow

The ADC will not simply turn on:

```c
RCC->APBENR2 |= RCC_APBENR2_ADCEN;             /* 1. clock                     */
ADC1->CFGR2   = ... ADC_CFGR2_CKMODE_0;         /*    PCLK/2 = 24 MHz           */
ADC1->CR     |= ADC_CR_ADVREGEN;                /* 2. internal regulator on     */
delay_us(20);                                   /*    let it settle             */
ADC1->CR     |= ADC_CR_ADCAL;                   /* 3. self-calibrate            */
while (ADC1->CR & ADC_CR_ADCAL) ;               /*    hardware clears the bit   */
ADC1->CR     |= ADC_CR_ADEN;                    /* 4. enable                    */
while (!(ADC1->ISR & ADC_ISR_ADRDY)) ;          /*    ready                     */
```

The **20 µs** is ST's number (`LL_ADC_DELAY_INTERNAL_REGUL_STAB_US` in the C0
LL driver), as is the rule that at least 2 ADC cycles separate calibration from
enable. Calibration measures the converter's own offset and stores a
correction — skip it and every reading is shifted by a few counts, forever,
with no error anywhere.

`CKMODE` can only be changed while the ADC is off, which is why it is step 1.

---

## Slide 4: Choosing a Channel — and Waiting for It

One ADC, many inputs: `CHSELR` has a bit per channel.

```c
ADC1->ISR    = ADC_ISR_CCRDY;          /* clear                              */
ADC1->CHSELR = 1u << channel;          /* select                             */
while (!(ADC1->ISR & ADC_ISR_CCRDY)) ; /* WAIT: the new selection settles    */
ADC1->CR    |= ADC_CR_ADSTART;         /* now convert                        */
while (!(ADC1->ISR & ADC_ISR_EOC)) ;
code = ADC1->DR;                       /* reading DR clears EOC              */
```

On this ADC family a new `CHSELR` is not instant. Start before `CCRDY` and the
conversion runs on the **old** channel — a plausible number from the wrong
input. It is the ADC edition of lesson 05's rule: the silent failures are the
ones that return a value.

Sampling time is set once, to the longest (160.5 ADC cycles ≈ 6.7 µs). The
internal reference needs microseconds to charge the sampling capacitor; a knob
does not care. One setting serves both.

---

## Slide 5: 3.3 V Is a Guess — Measure It

Inside the chip is a reference of about 1.2 V, **VREFINT**, on ADC channel 10.
At the factory, ST measured it with VDDA at exactly 3.000 V and wrote the result
into system memory — at **`0x1FFF756A`** on the C0 (ST's LL driver:
`VREFINT_CAL_ADDR`). Measure VREFINT now, and the ratio is the real supply:

```
VDDA = 3000 mV × VREFINT_CAL / VREFINT_now
```

That turns every other reading from "percent of whatever the supply is today"
into millivolts. A battery-powered device that sags from 3.3 V to 3.0 V would
otherwise report every sensor 10% high.

The shell's `vref` command does it — and **checks the calibration word first**.
In a simulator there may be no factory value at that address; **measured**: in
Renode it reads `0`, and `vref` reports *no usable factory value* instead of
dividing by it. What Wokwi returns there is Lab Part 2's question.

---

## Slide 6: I²C — Two Wires, Many Chips

```svg
<svg viewBox="0 0 580 190" role="img" aria-label="I2C bus: SCL and SDA with pull-up resistors, a controller and several addressed devices">
  <text x="20" y="40" class="mono">SDA</text>
  <text x="20" y="80" class="mono">SCL</text>
  <path class="wire" d="M60 36 H560"/>
  <path class="wire" d="M60 76 H560"/>
  <rect class="reg" x="80" y="4" width="20" height="26"/>
  <rect class="reg" x="110" y="4" width="20" height="26"/>
  <text x="130" y="0" class="lbl">pull-ups</text>
  <rect class="hifill" x="60" y="110" width="110" height="46" rx="5"/>
  <text x="115" y="130" text-anchor="middle" class="mono">STM32</text>
  <text x="115" y="146" text-anchor="middle" class="lbl">controller</text>
  <rect class="box" x="230" y="110" width="110" height="46" rx="5"/>
  <text x="285" y="130" text-anchor="middle" class="mono">MPU6050</text>
  <text x="285" y="146" text-anchor="middle" class="lbl">0x68</text>
  <rect class="box" x="400" y="110" width="110" height="46" rx="5"/>
  <text x="455" y="130" text-anchor="middle" class="mono">another</text>
  <text x="455" y="146" text-anchor="middle" class="lbl">0x3C, 0x77 ...</text>
  <path class="wire" d="M100 110 V36 M130 110 V76 M270 110 V36 M300 110 V76 M440 110 V36 M470 110 V76"/>
  <text x="290" y="182" text-anchor="middle" class="lbl">Every device sees every byte; each answers only to its own 7-bit address.</text>
</svg>
```

- **Open drain**: a device can only pull a line **low**; resistors pull it
  high. Two devices can never fight — a low always wins, harmlessly. The code
  sets `OTYPER` for PB8/PB9; real breakout boards carry 4.7 kΩ pull-ups.
- **Addresses**: 7 bits, sent first in every transfer, with a direction bit.
- **ACK**: after every byte the receiver pulls SDA low for one clock. **No
  ACK means nobody is there** — or nobody willing.

---

## Slide 7: Reading a Register — One Transaction, Two Directions

Almost every I²C sensor works the same way: *write the register number, then
read from it*:

```
S | 0x68+W | A | 0x3B | A | Sr | 0x68+R | A | d0 A d1 A ... d7 N | P
    address       register      repeated     8 data bytes      stop
                                START
```

The **repeated START** (`Sr`) turns the bus around without releasing it, so no
other controller can slip in between "which register" and "read it".

`i2c_write_read(0x68, &reg, 1, buf, 8)` is exactly that. At 400 kHz it is about
99 bit-times — **~250 µs** — and the MPU6050 auto-increments its register
pointer, so one transaction reads accelerometer X, Y, Z and temperature.

---

## Slide 8: The C0's I²C Block Does the Bit-Banging

Software never toggles SCL. It loads `CR2` with *what* to do, and then feeds or
drains one byte at a time as `ISR` asks:

```c
I2C1->CR2 = (addr << 1) | (n << I2C_CR2_NBYTES_Pos) | I2C_CR2_START [| AUTOEND] [| RD_WRN];
```

| `ISR` flag | Means | Software |
|---|---|---|
| `TXIS` | ready for the next byte | write `TXDR` |
| `RXNE` | a byte arrived | read `RXDR` |
| `TC` | `NBYTES` done, no STOP (AUTOEND = 0) | issue the repeated START |
| `STOPF` | STOP sent — the transfer is over | clear with `ICR` |
| `NACKF` | not acknowledged | clear, and end with STOP |

The bus timing lives in one register, `TIMINGR`. This lesson uses
**`0x0090273D`** — not derived here, but **ST's own value** from its
NUCLEO-C031C6 examples, computed by CubeMX for Fast mode, 400 kHz, with a
48 MHz I²C clock. Deriving it is a page of arithmetic with rise and fall times;
using the manufacturer's is the professional choice, provided you say so.

---

## Slide 9: When Nobody Answers

A missing device and a broken bus look different — and the code must survive
both:

| Situation | Silicon shows | Driver returns |
|---|---|---|
| device present | ACK after every byte | `I2C_OK` |
| no device at that address | **NACK** after the address | `I2C_NACK` |
| bus held low, no pull-ups, controller confused | **nothing** — no flag ever | `I2C_TIMEOUT` |

Every wait in `i2c.c` has a limit. And after a NACK the transfer must still be
**closed with a STOP**, or the bus stays claimed and the next START fails.

**A bus scan** is the most useful I²C tool there is: probe every address with
an address byte and nothing else (`NBYTES = 0`, START, AUTOEND) and list who
acknowledges. That is precisely what ST's own `HAL_I2C_IsDeviceReady()` does.
The shell's `scan` command is it.

---

## Slide 10: The MPU6050

| | |
|---|---|
| address | `0x68` (`0x69` with AD0 high) |
| `WHO_AM_I`, reg `0x75` | reads `0x68` — the first thing to check |
| `PWR_MGMT_1`, reg `0x6B` | **SLEEP set at power-up** — write 0 or it reads zeros |
| data, reg `0x3B` on | ax, ay, az, temp, gx, gy, gz — 16-bit, **big-endian** |
| accel scale | ±2 g default: **16384 counts per g** |
| temperature | °C = raw / 340 + 36.53 |

`mpu_init()` checks `WHO_AM_I` before anything else: the right chip, answering,
at the right address. Then it wakes it. A driver that skips the wake reads a
sensor that is perfectly connected, acknowledges every byte — and returns
zeros.

Big-endian means the **high** byte comes first: `(raw[0] << 8) | raw[1]`, cast to
`int16_t` so negative tilts stay negative.

---

## Slide 11: SPI — Select, Then Shift

```svg
<svg viewBox="0 0 580 170" role="img" aria-label="SPI: controller drives SCK and MOSI to all devices and one chip-select per device">
  <rect class="hifill" x="20" y="50" width="110" height="70" rx="5"/>
  <text x="75" y="80" text-anchor="middle" class="mono">STM32</text>
  <text x="75" y="98" text-anchor="middle" class="lbl">controller</text>
  <path class="wire" d="M130 62 H430"/>
  <path class="wire" d="M130 80 H430"/>
  <path class="hi" d="M130 100 H280 V140"/>
  <text x="140" y="58" class="mono lbl">SCK</text>
  <text x="140" y="76" class="mono lbl">MOSI</text>
  <text x="140" y="96" class="mono lbl">CS (PB0)</text>
  <rect class="box" x="240" y="140" width="120" height="26" rx="4"/>
  <text x="300" y="158" text-anchor="middle" class="mono">MAX7219</text>
  <path class="wire" d="M300 140 V62 M320 140 V80"/>
  <text x="470" y="80" class="lbl">no address:</text>
  <text x="470" y="98" class="lbl">CS low = "you"</text>
</svg>
```

| | I²C | SPI |
|---|---|---|
| wires | 2 | SCK, MOSI, MISO + one CS each |
| who is addressed | address byte | CS line |
| acknowledgement | every byte | **none** |
| speed | 100–400 kHz (1 MHz+) | MHz — 6 MHz here |
| direction | half duplex | **full duplex**: a byte in for every byte out |

No acknowledgement means **no way to know the MAX7219 is even there**. The
banner says so honestly: *it sends nothing back to check*.

---

## Slide 12: The C0's SPI Block

```c
SPI1->CR2 = 0xF << SPI_CR2_DS_Pos;                /* 16-bit frames         */
SPI1->CR1 = SPI_CR1_MSTR | (2 << SPI_CR1_BR_Pos)  /* master, 48/8 = 6 MHz  */
          | SPI_CR1_SSM | SPI_CR1_SSI;            /* CS is ours, as a GPIO */
SPI1->CR1 |= SPI_CR1_SPE;
```

Sending one 16-bit command, and the three things that go wrong silently:

```c
pin_low(GPIOB, CS_PIN);
*(volatile uint16_t *)&SPI1->DR = frame;   /* 1. 16-bit ACCESS, not 32      */
while (!(SPI1->SR & SPI_SR_RXNE)) ;
(void)*(volatile uint16_t *)&SPI1->DR;     /* 2. read what came back        */
while (SPI1->SR & SPI_SR_BSY) ;            /* 3. wait for the LAST bit      */
pin_high(GPIOB, CS_PIN);                   /*    the rising edge latches it */
```

1. This SPI packs data by the **width of the access**, so the pointer type
   matters.
2. Full duplex: something always comes back. Leave it and the receive FIFO
   fills.
3. Raise CS while the last bits are still shifting and the MAX7219 latches half
   a command.

SCK is PA5 — which is also **LD4**. On a real Nucleo the green LED flickers
with the SPI clock. Arduino's pin D13 has always been both.

---

## Slide 13: The MAX7219

It takes 16-bit commands: register in the high byte, value in the low.

| Register | Does | Set to |
|---|---|---|
| `0x0F` display test | all LEDs on | 0 |
| `0x09` decode mode | 7-segment font | 0 — raw bits |
| `0x0B` scan limit | how many rows | 7 — all eight |
| `0x0A` intensity | brightness, 0..15 | the pot |
| `0x0C` shutdown | **starts in shutdown** | 1 |
| `0x01`–`0x08` | one row each | a byte of pixels |

Like the MPU6050, it **powers up asleep**. The pattern is common enough to be a
rule: after any external chip's reset, check its datasheet for a power-down
bit.

Redrawing only when the ball moves to a new pixel costs two commands —
about 5 µs of SPI at 6 MHz — instead of eight.

---

## Slide 14: A Ball, in Integers

```c
vx += r.ax / 1024;   vy += r.ay / 1024;     /* tilt accelerates         */
vx -= vx / 16;       vy -= vy / 16;         /* friction                 */
bx += vx;            by += vy;              /* move                     */
if (bx < 0) { bx = 0; vx = -vx / 2; } ...   /* bounce, losing half      */
```

Position and velocity are in **1/256ths of a pixel**, so the ball can move a
fraction of a pixel per 20 ms step with no floating point. 1 g of tilt
(16384 counts) adds 16/256 px per step; friction caps the speed at about one
pixel per step.

This is the smallest physics simulation you will write, and it has the shape of
every one after it: **read sensors, update state by the equations of motion,
drive outputs, repeat at a fixed rate**. Lesson 13's drone does exactly this —
with gyroscopes, quaternions and four motors instead of one dot.

---

## Slide 15: One Bus, Two Tasks

The sensor task reads the MPU6050 every 20 ms. The shell's `scan` and `who`
use the same I²C bus. Two tasks, one peripheral:

```c
os_mutex_lock(&bus_lock);
rc = mpu_read(MPU_ACCEL_X, raw, sizeof raw);
os_mutex_unlock(&bus_lock);
```

Without the lock, a `scan` could start a transfer in the middle of the sensor
task's register read — two STARTs, one bus, garbage for both.

And the reading itself — seven fields — goes to the telemetry task through a
struct copied inside a two-line critical section, because a half-updated reading
(new X, old Y) is lesson 03's *tearing*.

Two tools, two jobs: a **mutex** for a resource held for hundreds of
microseconds, a **critical section** for a copy that takes a few instructions.

---

## Slide 16: The Silent-Failure Checklist, Bus Edition

| Forgot | Symptom |
|---|---|
| ADC calibration | every reading off by a few counts, forever |
| waiting for `CCRDY` | a reading from the previous channel |
| PA4 in analog mode | digital input buffer loads the signal |
| I²C open-drain / pull-ups | bus never goes high; timeout |
| waking the MPU6050 | ACKs perfectly, returns zeros |
| a STOP after NACK | the next transfer fails |
| a limit on every wait | one missing chip hangs the product |
| SPI access width | wrong frames on the wire |
| `BSY` before CS high | half-latched commands |
| waking the MAX7219 | a dark display that accepted everything |

Three of these return plausible data, and the rest fail silently. None raises an
error on its own. Every one is a line in this lesson's code, commented.

---

## Slide 17: What Was Measured — and What Could Not Be

Renode models the C0's I²C, SPI and ADC blocks at the same addresses, but has
no MPU6050 and no MAX7219. What running there showed:

| Test | Result |
|---|---|
| ADC start-up sequence | `calibrated, enabled` |
| MPU6050 absent | Renode stays **silent** (silicon would NACK): **`bus timeout`**, and the program carries on |
| same driver vs Renode's LSM6DSO IMU at `0x6A` | `WHO_AM_I = 0x6C` — correct, via write + repeated START + read |
| 13 MAX7219 commands over SPI | all complete; nothing listening, nothing hung |
| `vref` | calibration word reads 0 → *no usable factory value* |

Two things the simulator got wrong, both recorded rather than worked around:
its F0-derived ADC has **no `CCRDY`**, so channel reads time out (`pot -3`);
and a zero-length probe of a *present* device **crashed Renode's I²C model**
(an internal index error) — though it is exactly what ST's HAL does on silicon.

And one lesson the simulator taught by accident: with a 20 ms time limit on
`CCRDY`, every failed read ate the whole 20 ms period and **the sensor task
starved everything below it**. The limit is now 2000 polls. *How long* a
timeout waits is design, not boilerplate.

---

## Slide 18: What Carries Forward

- **Polling a bus wastes the CPU.** An 8-byte I²C read spins for ~250 µs; the
  sensor task does it 50 times a second — over 1% of the CPU, watching a flag.
  **Lesson 10 moves this to DMA**: start the transfer, sleep, get one interrupt
  when it is done.
- **The IMU is the drone's first sensor.** Lesson 13 reads the gyroscope too,
  and fuses both into an attitude estimate — the ball becomes an aircraft.
- **Every driver here is the model.** Lesson 12's OLED is an I²C device;
  Part 3's motor drivers are SPI or PWM. You will not need a new lesson to use
  them — only their datasheets.
