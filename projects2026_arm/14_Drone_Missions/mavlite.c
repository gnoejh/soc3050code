/*
 * mavlite.c - parse mission frames, build telemetry frames.  See mavlite.h.
 *
 * A parser that faces the outside world is the most attacked code in any
 * vehicle, so this one trusts nothing: the checksum first, then the length
 * and the field count, then every number strictly (no "12abc"), then the
 * range - and only then the flight computer, which checks once more against
 * the fence.
 *
 * Every frame is built IN PLACE in the caller's buffer - no 96-byte scratch
 * copy on the stack.  That matters: the link task's stack is 320 words, and
 * vsnprintf alone takes about a hundred of them (slide 20).
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "mavlite.h"
#include "proto.h"

#define MAX_FIELDS  6
#define MAX_BODY    63              /* longest body accepted; longer = refused */

static ml_stats_t stats;
const ml_stats_t *ml_stats(void) { return &stats; }

static long rnd(float x) { return (long)(x + (x >= 0.0f ? 0.5f : -0.5f)); }

static int vframe(char *out, size_t n, const char *fmt, va_list ap)
{
    static const char hex[] = "0123456789ABCDEF";
    if (n < 6u) { if (n) { out[0] = '\0'; } return 0; }
    /* body at out+1, leaving room for "*CS\n" and the NUL */
    int len = vsnprintf(out + 1, n - 5u, fmt, ap);
    if (len < 0 || (size_t)len >= n - 5u) { out[0] = '\0'; return 0; }
    uint8_t cs = proto_checksum(out + 1, (size_t)len);
    out[0] = '$';
    char *t = out + 1 + len;
    t[0] = '*';  t[1] = hex[cs >> 4];  t[2] = hex[cs & 15u];  t[3] = '\n';  t[4] = '\0';
    return len + 5;
}

int ml_frame(char *out, size_t n, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int w = vframe(out, n, fmt, ap);
    va_end(ap);
    return w;
}

/* Strict decimal: optional '-', at least one digit, nothing else. */
static int to_int(const char *s, int32_t *v)
{
    int neg = 0;
    int32_t x = 0;
    if (*s == '-') { neg = 1; s++; }
    if (*s == '\0') { return 0; }
    for (; *s; s++) {
        if (*s < '0' || *s > '9') { return 0; }
        if (x > 100000000) { return 0; }                  /* no overflow games */
        x = x * 10 + (*s - '0');
    }
    *v = neg ? -x : x;
    return 1;
}

static const char *const mode_words[] = { "HOLD", "MISSION", "RTL", "LAND" };
static const uint8_t     mode_ids[]   = { M_HOLD, M_MISSION, M_RTL, M_LAND };

/* Reply frames are appended to the caller's buffer. */
typedef struct { char *buf; size_t n, used; } out_t;

static void put(out_t *o, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int w = vframe(o->buf + o->used, o->n - o->used, fmt, ap);
    va_end(ap);
    o->used += (size_t)w;
}

static void ack(out_t *o, const char *cmd, int err)
{
    if (err == FC_OK) { put(o, "ACK,%s,OK", cmd); return; }
    stats.refused++;
    put(o, "ACK,%s,ERR,%s", cmd, fc_error_name(err));
}

int ml_handle(const char *line, char *reply, size_t n, ml_sim_param_fn sim)
{
    out_t o = { reply, n, 0 };
    if (n) { reply[0] = '\0'; }
    if (line[0] != '$') { return ML_NOT_FRAME; }

    const char *body;
    size_t len;
    if (proto_check(line, &body, &len) != PROTO_OK) {
        stats.bad++;
        ack(&o, "?", FC_E_CHECKSUM);                        /* a NAK: "say again" */
        return ML_BAD;
    }
    stats.good++;
    if (len > MAX_BODY) { ack(&o, "?", FC_E_RANGE); return ML_FRAME; }  /* never truncate */

    /* Copy the body so it can be cut into NUL-terminated fields in place. */
    char b[MAX_BODY + 1];
    memcpy(b, body, len);
    b[len] = '\0';
    char *f[MAX_FIELDS];
    int nf = 0;
    for (char *p = b; nf < MAX_FIELDS; ) {
        f[nf++] = p;
        p = strchr(p, ',');
        if (!p) { break; }
        *p++ = '\0';
    }

    char cmd[20];                                           /* names the ACK */
    snprintf(cmd, sizeof cmd, "%.8s", f[0]);
    int32_t v[4] = { 0 };

    if (strcmp(f[0], "WP") == 0) {
        int ok = nf == 5;
        for (int i = 0; ok && i < 4; i++) { ok = to_int(f[i + 1], &v[i]); }
        if (ok) { snprintf(cmd, sizeof cmd, "WP,%ld", (long)v[0]); }
        if (!ok || v[0] < 0 || v[0] >= (int32_t)FC_MAX_WP
                || v[1] < -10000 || v[1] > 10000 || v[2] < -10000 || v[2] > 10000
                || v[3] < 0 || v[3] > 2000) {
            ack(&o, cmd, FC_E_RANGE);
        } else {
            ack(&o, cmd, fc_wp_set((uint8_t)v[0], v[1] * 0.1f, v[2] * 0.1f, v[3] * 0.1f));
        }
    } else if (strcmp(f[0], "MISSION") == 0 && nf == 2) {
        if      (strcmp(f[1], "CLEAR") == 0) { fc_mission_clear(); ack(&o, cmd, FC_OK); }
        else if (strcmp(f[1], "START") == 0) { ack(&o, cmd, fc_mission_start()); }
        else                                 { ack(&o, cmd, FC_E_RANGE); }
    } else if (strcmp(f[0], "MODE") == 0 && nf == 2) {
        int err = FC_E_RANGE;
        for (unsigned i = 0; i < sizeof mode_ids; i++) {
            if (strcmp(f[1], mode_words[i]) == 0) { err = fc_set_mode(mode_ids[i], R_CMD); }
        }
        ack(&o, cmd, err);
    } else if (strcmp(f[0], "ARM") == 0 && nf == 1) {
        ack(&o, cmd, fc_arm());
    } else if (strcmp(f[0], "DISARM") == 0 && nf == 1) {
        ack(&o, cmd, fc_disarm());
    } else if (strcmp(f[0], "PING") == 0 && nf == 1) {
        ack(&o, cmd, FC_OK);
    } else if (strcmp(f[0], "PARAM") == 0 && (nf == 2 || nf == 3)) {
        snprintf(cmd, sizeof cmd, "PARAM,%.10s", f[1]);
        if (nf == 2) {                                      /* read one */
            if (fc_param_get(f[1], &v[0]) == FC_OK) { put(&o, "PARAM,%s,%ld", f[1], (long)v[0]); }
            else                                    { ack(&o, cmd, FC_E_PARAM); }
        } else if (!to_int(f[2], &v[0])) {
            ack(&o, cmd, FC_E_RANGE);
        } else {                                            /* set one */
            int err = fc_param_set(f[1], v[0]);
            if (err == FC_E_PARAM && sim) { err = sim(f[1], v[0]); }
            ack(&o, cmd, err);
        }
    } else {
        stats.unknown++;
        ack(&o, cmd, FC_E_UNKNOWN);
    }
    return ML_FRAME;
}

/* ---- telemetry --------------------------------------------------------- */
int ml_pos(char *out, size_t n, const fc_t *f)
{
    long yaw = rnd(f->yaw * 57.29578f);
    return ml_frame(out, n, "POS,%lu,%s,%ld,%ld,%ld,%ld,%d,%ld",
                    (unsigned long)f->t_ms, fc_mode_name(f->mode),
                    rnd(f->px * 10.0f), rnd(f->py * 10.0f), rnd(f->pz * 10.0f),
                    yaw, f->mission_active ? (int)f->cur : -1, rnd(f->battery));
}

int ml_tru(char *out, size_t n, uint32_t t_ms, float x, float y, float z, float wind)
{
    return ml_frame(out, n, "TRU,%lu,%ld,%ld,%ld,%ld", (unsigned long)t_ms,
                    rnd(x * 100.0f), rnd(y * 100.0f), rnd(z * 100.0f), rnd(wind * 10.0f));
}

int ml_evt(char *out, size_t n, const fc_event_t *e)
{
    switch (e->kind) {
    case EV_MODE:
        return ml_frame(out, n, "EVT,%lu,MODE,%s,%s", (unsigned long)e->t_ms,
                        fc_mode_name(e->a), fc_reason_name(e->b));
    case EV_WP:
        return ml_frame(out, n, "EVT,%lu,WP,%u", (unsigned long)e->t_ms, (unsigned)e->a);
    default:
        return ml_frame(out, n, "EVT,%lu,MISSION,%u", (unsigned long)e->t_ms, (unsigned)e->a);
    }
}

int ml_mis(char *out, size_t n, const fc_t *f, uint8_t i)
{
    return ml_frame(out, n, "MIS,%u,%u,%ld,%ld,%ld", (unsigned)i, (unsigned)f->n_wp,
                    rnd(f->wp[i].x * 10.0f), rnd(f->wp[i].y * 10.0f), rnd(f->wp[i].z * 10.0f));
}
