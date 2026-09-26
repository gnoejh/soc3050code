/*
 * proto.h - the line protocol between the board and host.py   (lesson 08)
 *
 *   $<BODY>*<CS>\r\n
 *
 * BODY is comma-separated ASCII, CS is two hex digits: the XOR of every byte
 * of BODY.  That is exactly the NMEA 0183 sentence format GPS receivers have
 * used since the 1980s - chosen because it is printable (you can read it, type
 * it, and copy it out of a serial monitor) and because host.py can check it
 * against a real GPS sentence.
 *
 * Pure C, no hardware: the same file would compile on a PC.
 */
#ifndef PROTO_H
#define PROTO_H

#include <stddef.h>
#include <stdint.h>

enum { PROTO_OK = 0, PROTO_NO_DOLLAR = -1, PROTO_NO_STAR = -2,
       PROTO_BAD_HEX = -3, PROTO_MISMATCH = -4 };

uint8_t proto_checksum(const char *body, size_t len);

/* Checks a received line.  On PROTO_OK, *body points just after the '$' and
 * *len is the body's length (up to, not including, the '*'). */
int proto_check(const char *line, const char **body, size_t *len);

#endif
