/*
 * Main.c - SOC3050 lesson 08: UART and a Python host
 *          ST Nucleo-C031C6, STM32C031C6, Cortex-M0+ at 48 MHz
 *
 * One serial line, two kinds of listener:
 *
 *   a PERSON typing in the serial monitor:      led 170    rate 200    stats
 *   a PROGRAM (host.py) exchanging frames:      $LED,170*57    $TEL,...*75
 *
 * Both are handled by one task that blocks on the UART (uart.c) and wakes only
 * when a line arrives.  A second task sends telemetry frames on a schedule; a
 * third counts button presses.  Every frame carries an NMEA-style checksum
 * (proto.c), so host.py can tell a good line from a damaged one.
 *
 * Build:     build.bat
 * Simulate:  simulate.bat, paste diagram.json, upload Main.elf, then type
 *            "help" into the serial monitor's input box.
 * Host:      python host.py --help
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32c031xx.h"
#include "gpio.h"
#include "os.h"
#include "uart.h"
#include "proto.h"

#define BAUD         115200u
#define BTN_A_PIN    0u
#define BTN_B_PIN    1u
#define LINE_MAX     64u

#define STACK(name, words) static uint32_t name[words] __attribute__((aligned(8)))
STACK(stk_proto, 384);  STACK(stk_tel, 384);  STACK(stk_btn, 128);

/* ---- state the commands change ------------------------------------------ */
static volatile uint32_t tel_ms  = 1000;     /* telemetry period             */
static volatile uint8_t  tel_on  = 1;
static volatile uint8_t  bar     = 0x01;
static volatile uint32_t presses_a, presses_b;
static volatile uint32_t frames_ok, frames_bad, lines_in;

/* ---- one printer at a time ------------------------------------------------
 * Two tasks print.  printf() is not re-entrant in newlib-nano, and even if it
 * were, two lines interleaved character by character are garbage to host.py.
 * Lesson 07's mutex makes each whole line atomic. */
static os_mutex_t print_lock;

static void say(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    os_mutex_lock(&print_lock);
    vprintf(fmt, ap);
    os_mutex_unlock(&print_lock);
    va_end(ap);
}

/* A frame: "$" body "*" checksum.  The body is formatted first so the
 * checksum can be computed over exactly the bytes that will be sent. */
static void send_frame(const char *fmt, ...)
{
    char body[LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(body, sizeof body, fmt, ap);
    va_end(ap);
    if (n < 0) { return; }
    if ((size_t)n >= sizeof body) { n = (int)sizeof body - 1; }
    say("$%s*%02X\n", body, (unsigned)proto_checksum(body, (size_t)n));
}

static void set_bar(uint8_t v)
{
    bar = v;
    GPIOB->BSRR = ((uint32_t)(uint8_t)~v << 16) | v;     /* lesson 05: one store */
}

static int set_rate(long ms)
{
    if (ms < 20 || ms > 5000) { return 0; }
    tel_ms = (uint32_t)ms;
    return 1;
}

/* ============================================================================
 *  Frames from a program:  $LED,170*CS   $RATE,200*CS   $PING*CS   $STAT*CS
 * ============================================================================ */
static void handle_frame(const char *line)
{
    const char *body;
    size_t len;
    int r = proto_check(line, &body, &len);
    if (r != PROTO_OK) {
        frames_bad++;
        send_frame("NAK,%s", r == PROTO_MISMATCH ? "CHECKSUM"
                           : r == PROTO_NO_STAR  ? "NOSTAR" : "FORMAT");
        return;
    }
    frames_ok++;

    char cmd[LINE_MAX];
    memcpy(cmd, body, len);
    cmd[len] = '\0';
    char *arg = strchr(cmd, ',');
    if (arg) { *arg++ = '\0'; }

    if (strcmp(cmd, "LED") == 0 && arg) {
        set_bar((uint8_t)strtol(arg, 0, 0));
        send_frame("ACK,LED,%u", (unsigned)bar);
    } else if (strcmp(cmd, "RATE") == 0 && arg) {
        if (set_rate(strtol(arg, 0, 0))) { send_frame("ACK,RATE,%lu", (unsigned long)tel_ms); }
        else                             { send_frame("NAK,RANGE"); }
    } else if (strcmp(cmd, "PING") == 0) {
        send_frame("ACK,PING,%lu", (unsigned long)os_ticks());
    } else if (strcmp(cmd, "STAT") == 0) {
        const volatile uart_stats_t *s = uart_stats();
        send_frame("STAT,%lu,%lu,%lu,%lu,%lu,%lu",
                   (unsigned long)s->rx_bytes, (unsigned long)frames_ok,
                   (unsigned long)frames_bad, (unsigned long)s->rx_overruns,
                   (unsigned long)s->rx_dropped, (unsigned long)s->tx_waits);
    } else {
        send_frame("NAK,UNKNOWN");
    }
}

/* ============================================================================
 *  Lines from a person:  help  led 170  rate 200  quiet  stats
 * ============================================================================ */
static void handle_shell(char *line)
{
    char *arg = strchr(line, ' ');
    if (arg) { *arg++ = '\0'; }

    if (strcmp(line, "help") == 0) {
        say("commands: led <0-255>   rate <20-5000 ms>   quiet   stats   help\n"
            "frames:   $LED,<n>*CS   $RATE,<ms>*CS   $PING*CS   $STAT*CS\n");
    } else if (strcmp(line, "led") == 0 && arg) {
        set_bar((uint8_t)strtol(arg, 0, 0));
        say("bar = 0x%02X\n", (unsigned)bar);
    } else if (strcmp(line, "rate") == 0 && arg) {
        say(set_rate(strtol(arg, 0, 0)) ? "telemetry every %lu ms\n"
                                        : "rate must be 20..5000 (still %lu ms)\n",
            (unsigned long)tel_ms);
    } else if (strcmp(line, "quiet") == 0) {
        tel_on = (uint8_t)!tel_on;
        say("telemetry %s\n", tel_on ? "on" : "off");
    } else if (strcmp(line, "stats") == 0) {
        const volatile uart_stats_t *s = uart_stats();
        say("rx %lu bytes, %lu lines; frames %lu ok, %lu bad\n"
            "rx overruns %lu, rx dropped %lu; tx %lu bytes, tx waits %lu\n",
            (unsigned long)s->rx_bytes, (unsigned long)lines_in,
            (unsigned long)frames_ok, (unsigned long)frames_bad,
            (unsigned long)s->rx_overruns, (unsigned long)s->rx_dropped,
            (unsigned long)s->tx_bytes, (unsigned long)s->tx_waits);
    } else {
        say("unknown command '%s' - try help\n", line);
    }
}

/* Priority 3.  Blocks inside uart_getc() - 0% CPU - until a byte arrives. */
static void task_proto(void *arg)
{
    (void)arg;
    char line[LINE_MAX];
    uint32_t n = 0;
    uint8_t  overflow = 0;
    for (;;) {
        char c = (char)uart_getc();
        if (c == '\r' || c == '\n') {
            if (n > 0 && !overflow) {
                line[n] = '\0';
                lines_in++;
                if (line[0] == '$') { handle_frame(line); } else { handle_shell(line); }
            } else if (overflow) {
                say("line too long - ignored\n");
            }
            n = 0;
            overflow = 0;
        } else if (n < LINE_MAX - 1u) {
            line[n++] = c;
        } else {
            overflow = 1;                        /* keep reading to the end of it */
        }
    }
}

/* Priority 2.  $TEL,<ms>,<seq>,<tri>,<A presses>,<B presses>*CS
 * `tri` is a 0..1000 triangle with a 4 s period - something to plot before
 * lesson 09 brings real sensors.  `seq` lets host.py spot a lost line. */
static void task_tel(void *arg)
{
    (void)arg;
    uint32_t last = os_ticks(), seq = 0;
    for (;;) {
        os_delay_until(&last, tel_ms);
        if (!tel_on) { continue; }
        uint32_t t  = os_ticks();
        uint32_t ph = t % 4000u;
        uint32_t tri = ph < 2000u ? ph / 2u : (4000u - ph) / 2u;
        send_frame("TEL,%lu,%lu,%lu,%lu,%lu", (unsigned long)t, (unsigned long)seq++,
                   (unsigned long)tri, (unsigned long)presses_a, (unsigned long)presses_b);
    }
}

/* Priority 4.  Polls the buttons every 5 ms and counts presses. */
static void task_btn(void *arg)
{
    (void)arg;
    uint32_t last = os_ticks();
    uint8_t a_prev = 0, b_prev = 0;
    for (;;) {
        os_delay_until(&last, 5);
        uint8_t a = (uint8_t)!pin_read(GPIOA, BTN_A_PIN);
        uint8_t b = (uint8_t)!pin_read(GPIOA, BTN_B_PIN);
        if (a && !a_prev) { presses_a++; }
        if (b && !b_prev) { presses_b++; }
        a_prev = a;
        b_prev = b;
    }
}

int main(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;
    for (uint32_t pin = 0; pin < 8u; pin++) { pin_mode(GPIOB, pin, MODE_OUTPUT); }
    pin_mode(GPIOA, BTN_A_PIN, MODE_INPUT);  pin_pull(GPIOA, BTN_A_PIN, PULL_UP);
    pin_mode(GPIOA, BTN_B_PIN, MODE_INPUT);  pin_pull(GPIOA, BTN_B_PIN, PULL_UP);
    set_bar(bar);

    uart_init(SystemCoreClock, BAUD);

    printf("\n=== SOC3050 lesson 08 - UART and a Python host ===\n");
    printf("    USART2 %lu baud, BRR = %lu, RX and TX interrupt-driven, handler %s\n",
           (unsigned long)BAUD, (unsigned long)USART2->BRR,
           handler_installed(USART2_IRQn) ? "installed" : "NOT INSTALLED");
    printf("    type 'help'.  Telemetry frames follow, once a second.\n\n");

    os_task_create("proto", task_proto, 0, stk_proto, 384, 3);
    os_task_create("tel",   task_tel,   0, stk_tel,   384, 2);
    os_task_create("btn",   task_btn,   0, stk_btn,   128, 4);
    os_start();
}
