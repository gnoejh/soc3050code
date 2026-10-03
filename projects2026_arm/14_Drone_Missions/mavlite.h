/*
 * mavlite.h - "MAVLink-lite": the mission protocol between host and drone
 *
 * NOT MAVLink.  Real MAVLink is binary, CRC-16 checked, and its mission
 * upload is a handshake (the vehicle asks for each item in turn).  This is
 * lesson 08's printable  $BODY*CS  frame carrying the same ideas, so you can
 * read every byte, type it into a serial monitor, and check it by eye.
 * Slide 17 maps each frame onto the MAVLink message it stands for.
 *
 *   host -> drone                         drone -> host
 *   $WP,idx,x_dm,y_dm,alt_dm              $ACK,WP,OK          or  $ACK,WP,ERR,FENCE
 *   $MISSION,CLEAR | START                $ACK,MISSION,OK ...
 *   $MODE,HOLD | MISSION | RTL | LAND     $ACK,MODE,OK ...
 *   $ARM   $DISARM   $PING                $ACK,ARM,OK ...
 *   $PARAM,NAME[,value]                   $PARAM,NAME,value
 *                                         $POS,t,mode,x_dm,y_dm,alt_dm,yaw_deg,wp,bat   5 Hz
 *                                         $TRU,t,x_cm,y_cm,alt_cm,wind_dms             5 Hz
 *                                         $EVT,t,MODE,name,reason  /  $EVT,t,WP,idx
 *                                         $MIS,idx,count,x_dm,y_dm,alt_dm  (on START)
 *
 * Positions are integers in DECIMETRES east/north/up of home: no floats on
 * the wire, and a decimetre is finer than the GPS can tell.
 *
 * Pure C: host/sitl.c feeds mission.py's output straight into ml_handle().
 */
#ifndef MAVLITE_H
#define MAVLITE_H

#include <stddef.h>
#include <stdint.h>
#include "flight.h"

enum { ML_FRAME = 0, ML_NOT_FRAME = 1, ML_BAD = 2 };

/* Simulator settings ($PARAM,WIND,... etc.) are not the flight computer's
 * business, so the caller supplies them.  Return an FC_* code. */
typedef int (*ml_sim_param_fn)(const char *name, int32_t value);

typedef struct { uint32_t good, bad, unknown, refused; } ml_stats_t;

/* Handle one received line.  Any reply lines ("$...*CS\n") are written to
 * `reply` (always NUL-terminated).  Returns ML_FRAME, ML_NOT_FRAME (not a
 * $-line: the caller's text shell can have it) or ML_BAD (checksum). */
int  ml_handle(const char *line, char *reply, size_t n, ml_sim_param_fn sim);
const ml_stats_t *ml_stats(void);

/* Build "$body*CS\n" from a printf format.  Returns the length, or 0. */
int  ml_frame(char *out, size_t n, const char *fmt, ...)
     __attribute__((format(printf, 3, 4)));

/* Telemetry, as frames - the firmware and host/sitl.c print the same bytes. */
int  ml_pos(char *out, size_t n, const fc_t *f);
int  ml_tru(char *out, size_t n, uint32_t t_ms, float x, float y, float z, float wind);
int  ml_evt(char *out, size_t n, const fc_event_t *e);
int  ml_mis(char *out, size_t n, const fc_t *f, uint8_t i);

#endif
