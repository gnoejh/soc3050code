/*
 * Main.c - SOC3050 lesson 09: Sensors and Buses
 *          ST Nucleo-C031C6, STM32C031C6, Cortex-M0+ at 48 MHz
 *
 * A ball that rolls when you tilt the board.
 *
 *   ADC  (adc.c)   a potentiometer on PA4 sets the display's brightness
 *   I2C  (i2c.c)   an MPU6050 accelerometer at address 0x68 on PB8/PB9
 *   SPI  (spi.c)   a MAX7219 driving an 8x8 LED matrix on PA5/PA7, CS PB0
 *
 * Every 20 ms the sensor task reads the tilt, moves the ball - integer
 * physics, no FPU - and redraws it.  Every 100 ms an $IMU frame goes to the
 * serial port for host.py.  A small shell answers: scan, who, vref, stats.
 *
 * In Wokwi, click the MPU6050 while it runs: sliders set its acceleration.
 *
 * Build:     build.bat
 * Simulate:  simulate.bat, paste diagram.json, upload Main.elf.
 * Host:      python ..\08_UART_And_Python_Host\host.py --file capture.txt --chart IMU,2,-16384,16384
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "stm32c031xx.h"
#include "gpio.h"
#include "os.h"
#include "uart.h"
#include "proto.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"

#define BAUD          115200u
#define MPU_ADDR      0x68u       /* AD0 low; 0x69 with AD0 high           */
#define MPU_PWR_MGMT1 0x6Bu       /* bit 6 SLEEP is SET at power-up         */
#define MPU_ACCEL_X   0x3Bu       /* ax, ay, az, temp: 8 bytes, big-endian */
#define MPU_WHO_AM_I  0x75u       /* reads 0x68                             */

#define STACK(name, words) static uint32_t name[words] __attribute__((aligned(8)))
STACK(stk_sense, 384);  STACK(stk_tel, 384);  STACK(stk_shell, 384);

/* ---- shared between tasks --------------------------------------------------
 * The sensor task writes a whole reading; the telemetry task reads a whole
 * reading.  Several fields = tearing risk (lesson 03 slide 7), so both sides
 * copy the struct inside a two-line critical section. */
typedef struct { int16_t ax, ay, az, temp10; int32_t pot; uint32_t t; } reading_t;
static reading_t latest;
static volatile uint32_t i2c_errors, readings;
static volatile uint8_t  mpu_ok;

static os_mutex_t print_lock;     /* whole lines, as in lesson 08          */
static os_mutex_t bus_lock;       /* I2C1: the sensor task and the shell   */

static void say(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    os_mutex_lock(&print_lock);
    vprintf(fmt, ap);
    os_mutex_unlock(&print_lock);
    va_end(ap);
}

static void send_frame(const char *fmt, ...)
{
    char body[80];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(body, sizeof body, fmt, ap);
    va_end(ap);
    if (n < 0) { return; }
    if ((size_t)n >= sizeof body) { n = (int)sizeof body - 1; }
    say("$%s*%02X\n", body, (unsigned)proto_checksum(body, (size_t)n));
}

/* ============================================================================
 *  The MPU6050
 * ============================================================================ */
static int mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t w[2] = { reg, val };
    return i2c_write_read(MPU_ADDR, w, 2, 0, 0);
}

static int mpu_read(uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_write_read(MPU_ADDR, &reg, 1, buf, n);   /* write the register, read on */
}

static int mpu_init(void)
{
    uint8_t who = 0;
    int rc = mpu_read(MPU_WHO_AM_I, &who, 1);
    if (rc != I2C_OK) { return rc; }
    if (who != 0x68u) { return -10; }
    return mpu_write(MPU_PWR_MGMT1, 0x00);   /* clear SLEEP: it starts asleep */
}

/* ============================================================================
 *  The MAX7219: 16-bit commands, register in the high byte
 * ============================================================================ */
enum { MAX_DIGIT0 = 0x01, MAX_DECODE = 0x09, MAX_INTENSITY = 0x0A,
       MAX_SCANLIMIT = 0x0B, MAX_SHUTDOWN = 0x0C, MAX_TEST = 0x0F };

static void max_cmd(uint8_t reg, uint8_t val) { spi_send16((uint16_t)((reg << 8) | val)); }

static void max_init(void)
{
    max_cmd(MAX_TEST, 0);          /* not the all-on display test          */
    max_cmd(MAX_DECODE, 0);        /* raw bits, not 7-segment digits       */
    max_cmd(MAX_SCANLIMIT, 7);     /* all eight rows                       */
    max_cmd(MAX_INTENSITY, 4);
    max_cmd(MAX_SHUTDOWN, 1);      /* wake: it powers up in shutdown       */
    for (uint8_t r = 0; r < 8; r++) { max_cmd((uint8_t)(MAX_DIGIT0 + r), 0); }
}

/* ============================================================================
 *  The sensor task: read, move, draw - every 20 ms
 * ============================================================================
 * Position and velocity are in 1/256ths of a pixel, so the ball can move a
 * fraction of a pixel per step with integers only.  1 g of tilt (16384 raw at
 * the MPU6050's default +-2 g range) accelerates it 16/256 px per step. */
static void task_sense(void *arg)
{
    (void)arg;
    int32_t bx = 3 << 8, by = 3 << 8, vx = 0, vy = 0;
    uint8_t shown_row = 0xFF, shown_col = 0xFF;
    uint32_t last = os_ticks();

    for (;;) {
        os_delay_until(&last, 20);

        int32_t pot = adc_read(ADC_CH_PA4);
        uint8_t raw[8];
        os_mutex_lock(&bus_lock);
        int rc = mpu_ok ? mpu_read(MPU_ACCEL_X, raw, sizeof raw) : I2C_NACK;
        os_mutex_unlock(&bus_lock);
        if (rc != I2C_OK) { i2c_errors++; continue; }

        reading_t r;
        r.ax     = (int16_t)((raw[0] << 8) | raw[1]);
        r.ay     = (int16_t)((raw[2] << 8) | raw[3]);
        r.az     = (int16_t)((raw[4] << 8) | raw[5]);
        int16_t traw = (int16_t)((raw[6] << 8) | raw[7]);
        r.temp10 = (int16_t)(traw * 10 / 340 + 365);   /* datasheet: raw/340 + 36.53 C */
        r.pot    = pot;
        r.t      = os_ticks();
        __disable_irq(); latest = r; __enable_irq();
        readings++;

        /* physics: accelerate, rub, move, bounce off the walls */
        vx += r.ax / 1024;  vy += r.ay / 1024;
        vx -= vx / 16;      vy -= vy / 16;
        bx += vx;           by += vy;
        if (bx < 0)      { bx = 0;      vx = -vx / 2; }
        if (bx > 7 << 8) { bx = 7 << 8; vx = -vx / 2; }
        if (by < 0)      { by = 0;      vy = -vy / 2; }
        if (by > 7 << 8) { by = 7 << 8; vy = -vy / 2; }

        uint8_t col = (uint8_t)((bx + 128) >> 8), row = (uint8_t)((by + 128) >> 8);
        if (row != shown_row || col != shown_col) {     /* redraw only on change */
            if (shown_row < 8) { max_cmd((uint8_t)(MAX_DIGIT0 + shown_row), 0); }
            max_cmd((uint8_t)(MAX_DIGIT0 + row), (uint8_t)(1u << col));
            shown_row = row;  shown_col = col;
        }
        if (pot >= 0) { max_cmd(MAX_INTENSITY, (uint8_t)(pot >> 8)); }   /* 0..15 */
    }
}

/* ---- telemetry: $IMU,<ms>,<ax>,<ay>,<az>,<temp x10>,<pot> every 100 ms ---- */
static void task_tel(void *arg)
{
    (void)arg;
    uint32_t last = os_ticks();
    for (;;) {
        os_delay_until(&last, 100);
        if (!mpu_ok) { continue; }
        reading_t r;
        __disable_irq(); r = latest; __enable_irq();
        send_frame("IMU,%lu,%d,%d,%d,%d,%ld", (unsigned long)r.t,
                   r.ax, r.ay, r.az, r.temp10, (long)r.pot);
    }
}

/* ============================================================================
 *  The shell: scan  who  vref  stats
 * ============================================================================ */
static void cmd_scan(void)
{
    say("I2C scan, 0x08..0x77:");
    int found = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        os_mutex_lock(&bus_lock);
        int rc = i2c_probe(a);
        os_mutex_unlock(&bus_lock);
        if (rc == I2C_OK)           { say(" 0x%02X", a); found++; }
        else if (rc == I2C_TIMEOUT) { say(" (0x%02X: timeout - bus stuck?)", a); break; }
    }
    say(found ? "\n" : " nothing answered\n");
}

static void cmd_vref(void)
{
    /* VREFINT is ~1.2 V, trimmed at the factory; ST writes what it reads at
     * VDDA = 3.000 V into system memory at 0x1FFF756A (ST's C0 LL driver).
     * Measure it now, and the ratio gives the real supply voltage. */
    const volatile uint16_t *cal_addr = (const volatile uint16_t *)0x1FFF756AUL;
    say("reading VREFINT_CAL at 0x1FFF756A ... ");
    uint16_t cal = *cal_addr;
    int32_t  raw = adc_read(ADC_CH_VREFINT);
    say("cal %u, now %ld\n", (unsigned)cal, (long)raw);
    if (cal == 0u || cal == 0xFFFFu || raw <= 0) {
        say("no usable factory value here - a simulator, most likely\n");
        return;
    }
    say("VDDA = 3000 * %u / %ld = %ld mV\n", (unsigned)cal, (long)raw, 3000L * cal / raw);
}

static void task_shell(void *arg)
{
    (void)arg;
    char line[32];
    uint32_t n = 0;
    for (;;) {
        char c = (char)uart_getc();
        if (c != '\r' && c != '\n') { if (n < sizeof line - 1u) { line[n++] = c; } continue; }
        if (n == 0) { continue; }
        line[n] = '\0';
        n = 0;
        if      (strcmp(line, "scan") == 0) { cmd_scan(); }
        else if (strcmp(line, "vref") == 0) { cmd_vref(); }
        else if (strcmp(line, "who") == 0) {
            uint8_t who = 0;
            os_mutex_lock(&bus_lock);
            int rc = mpu_read(MPU_WHO_AM_I, &who, 1);
            os_mutex_unlock(&bus_lock);
            say(rc == I2C_OK ? "WHO_AM_I = 0x%02X\n" : "no answer (%d)\n", rc == I2C_OK ? who : rc);
        } else if (strcmp(line, "stats") == 0) {
            say("readings %lu, i2c errors %lu, pot %ld\n", (unsigned long)readings,
                (unsigned long)i2c_errors, (long)adc_read(ADC_CH_PA4));
        } else {
            say("commands: scan  who  vref  stats\n");
        }
    }
}

int main(void)
{
    uart_init(SystemCoreClock, BAUD);
    printf("\n=== SOC3050 lesson 09 - Sensors and Buses ===\n");

    int a = adc_init();
    printf("  ADC      : %s\n", a == 0 ? "calibrated, enabled" : a == -1 ? "CALIBRATION TIMED OUT" : "ADRDY TIMED OUT");

    i2c_init();
    int m = mpu_init();
    mpu_ok = (uint8_t)(m == I2C_OK);
    printf("  MPU6050  : %s\n", m == I2C_OK ? "WHO_AM_I 0x68, awake"
                              : m == I2C_NACK ? "no ACK at 0x68 - not connected?"
                              : m == I2C_TIMEOUT ? "bus timeout" : "wrong WHO_AM_I");

    spi_init();
    max_init();
    printf("  MAX7219  : initialised over SPI (it sends nothing back to check)\n");
    printf("  shell    : scan  who  vref  stats\n\n");

    os_task_create("sense", task_sense, 0, stk_sense, 384, 3);
    os_task_create("tel",   task_tel,   0, stk_tel,   384, 2);
    os_task_create("shell", task_shell, 0, stk_shell, 384, 1);
    os_start();
}
