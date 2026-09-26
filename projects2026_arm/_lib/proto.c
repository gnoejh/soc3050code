/*
 * proto.c - shared copy of lesson 08's NMEA-style framing.  LIBS=proto
 *
 * The checksum is one XOR per byte.  It catches every single-bit error and
 * any odd number of flipped bits in the same bit position; it misses two
 * errors that cancel.  A CRC would catch far more - MAVLink, the drone
 * protocol of lessons 13-14, uses one - but XOR is what NMEA chose, it fits in two hex digits a person
 * can check by eye, and host.py can compare it with a real GPS sentence.
 */
#include "proto.h"

uint8_t proto_checksum(const char *body, size_t len)
{
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) { cs ^= (uint8_t)body[i]; }
    return cs;
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') { return c - '0'; }
    if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
    if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
    return -1;
}

int proto_check(const char *line, const char **body, size_t *len)
{
    if (line[0] != '$') { return PROTO_NO_DOLLAR; }

    const char *b = line + 1;
    size_t n = 0;
    while (b[n] != '\0' && b[n] != '*') { n++; }
    if (b[n] != '*') { return PROTO_NO_STAR; }

    int hi = hexval(b[n + 1]);
    int lo = (hi < 0) ? -1 : hexval(b[n + 2]);
    if (hi < 0 || lo < 0) { return PROTO_BAD_HEX; }

    if (proto_checksum(b, n) != (uint8_t)((hi << 4) | lo)) { return PROTO_MISMATCH; }

    *body = b;
    *len  = n;
    return PROTO_OK;
}
