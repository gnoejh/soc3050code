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
 * A person on a raw terminal (the VS Code extension, or miniterm) also gets
 * keys: the arrows arrive as three-byte escape sequences that keys.c decodes.
 * Left/Right move the lit LED along the bar, Up/Down change the telemetry
 * rate, Esc cancels a half-typed line, and every other key is spelt out.
 * Typed characters are echoed - but never a $ frame, because host.py would
 * read its own command coming back as if the board had sent it.
 *
 * Build:     build.bat
 * Simulate:  simulate.bat, paste diagram.json, upload Main.elf, then type
 *            "help" into the serial monitor's input box.
 *            For the keys: in VS Code, press Play on diagram.json and type
 *            into the Wokwi terminal - it sends each key as it is pressed.
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
#include "keys.h"

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
static volatile uint8_t  typing;             /* a person is part-way through a line */

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
            "frames:   $LED,<n>*CS   $RATE,<ms>*CS   $PING*CS   $STAT*CS\n"
            "keys:     Left/Right move the bar   Up/Down rate   Esc cancels a line\n");
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

/* ============================================================================
 *  Keys from a person:  Left  Right  Up  Down  Esc  and anything else, spelt
 * ============================================================================ */
static keydec_t keys;

/* `typed` is the half-typed line, if any: the key's report goes on a line of
 * its own, then the half-typed text is put back so the person can carry on. */
static void handle_key(keycode_t k, const char *typed, uint32_t n)
{
    char spelt[4u * KEY_SEQ_MAX];
    if (n > 0) { say("\n"); }
    switch (k) {
    case KEY_LEFT:                                   /* PB7 is at the left end */
    case KEY_RIGHT: {
        uint8_t b = bar ? bar : 0x01u;
        set_bar(k == KEY_LEFT ? (uint8_t)((b << 1) | (b >> 7))
                              : (uint8_t)((b >> 1) | (b << 7)));
        say("%-6s bar = 0x%02X\n", key_name(k), (unsigned)bar);
        break;
    }
    case KEY_UP:
    case KEY_DOWN: {
        int ok = set_rate(k == KEY_UP ? (long)(tel_ms / 2u) : (long)(tel_ms * 2u));
        say(ok ? "%-6s telemetry every %lu ms\n" : "%-6s rate stays %lu ms (20..5000)\n",
            key_name(k), (unsigned long)tel_ms);
        break;
    }
    case KEY_ESC:
        say("Esc    line cancelled\n");
        return;                                      /* nothing to put back */
    default:
        key_spell(&keys, spelt, sizeof spelt);
        say("key    %s  (no name for this one)\n", spelt);
        break;
    }
    if (n > 0) { say("%.*s", (int)n, typed); }
}

/* Priority 3.  Blocks inside uart_getc() - 0% CPU - until a byte arrives.
 *
 * Each byte goes through the key decoder first.  Most come straight back as
 * KEY_CHAR and are collected into a line; an escape sequence is swallowed
 * until it is complete, then acted on as one key. */
static void task_proto(void *arg)
{
    (void)arg;
    char line[LINE_MAX];
    uint32_t n = 0;
    uint8_t  overflow = 0;
    char     prev = 0;
    for (;;) {
        char c = (char)uart_getc();
        uint32_t now = os_ticks();

        /* A lone Esc shows itself only by the silence after it, so it is
         * noticed late - when the NEXT byte arrives, which is then handled
         * as usual below. */
        keycode_t late = key_timeout(&keys, now);
        if (late != KEY_NONE) {
            handle_key(late, line, typing ? n : 0);
            if (late == KEY_ESC) { n = 0; overflow = 0; typing = 0; }
        }

        keycode_t k = key_feed(&keys, (uint8_t)c, now);
        if (k == KEY_NONE) { continue; }             /* inside a sequence */
        if (k != KEY_CHAR) {                         /* a whole key       */
            handle_key(k, line, typing ? n : 0);
            continue;
        }

        /* An ordinary byte.  A frame (first byte '$') is never echoed. */
        uint8_t frame = (uint8_t)(n > 0 && line[0] == '$');
        if (c == '\n' && prev == '\r') {
            /* CR LF is one Enter, not two */
        } else if (c == '\r' || c == '\n') {
            if (!frame) { say("\n"); }
            if (n > 0 && !overflow) {
                line[n] = '\0';
                lines_in++;
                if (frame) { handle_frame(line); } else { handle_shell(line); }
            } else if (overflow) {
                say("line too long - ignored\n");
            }
            n = 0;
            overflow = 0;
        } else if (c == 0x7F || c == '\b') {         /* Backspace: two codes in use */
            if (n > 0 && !overflow) {
                n--;
                if (!frame) { say("\b \b"); }        /* back, blank it, back again */
            }
        } else if ((uint8_t)c < 0x20u || (uint8_t)c > 0x7Eu) {
            /* other control bytes: not part of any command */
        } else if (n < LINE_MAX - 1u) {
            line[n++] = c;
            if (line[0] != '$') { say("%c", c); }
        } else {
            overflow = 1;                        /* keep reading to the end of it */
        }
        prev = c;
        typing = (uint8_t)(n > 0 && line[0] != '$');
    }
}

/* Priority 2.  $TEL,<ms>,<seq>,<tri>,<A presses>,<B presses>*CS
 * `tri` is a 0..1000 triangle with a 4 s period - something to plot before
 * lesson 09 brings real sensors.  `seq` lets host.py spot a lost line.
 *
 * It holds off while a person is half-way through typing a line: the echo has
 * left their text on the screen with no newline, so a frame sent now would
 * be glued onto it - "le$TEL,..." - and host.py would not recognise it.
 * `seq` is not advanced for a skipped frame, so host.py sees no gap. */
static void task_tel(void *arg)
{
    (void)arg;
    uint32_t last = os_ticks(), seq = 0;
    for (;;) {
        os_delay_until(&last, tel_ms);
        if (!tel_on || typing) { continue; }
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
