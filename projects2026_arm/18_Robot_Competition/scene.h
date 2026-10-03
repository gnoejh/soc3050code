/*
 * scene.h - what the OLED shows, as a plain struct, and the code that draws it
 *
 * The simulation task fills a snap_t (a SNAPSHOT) every 20 ms; the display
 * task copies it and draws it.  The two never share anything else, so the
 * display can run slowly - 12 frames a second - without ever seeing half of
 * one step and half of the next.
 *
 * Pure C on top of oled.h: host/league.c --draw renders the same picture as
 * text, which is how the drawing code is checked without a screen.
 */
#ifndef SCENE_H
#define SCENE_H

#include <stdint.h>
#include "strategy.h"

enum { SCR_MENU, SCR_MATCH, SCR_RESULT };

typedef struct {
    uint8_t     screen;             /* SCR_*                                  */
    /* the match */
    float       x[2], y[2], th[2];  /* metres, radians                        */
    uint16_t    dist[2][N_DIST];    /* each robot's distance readings          */
    const char *name[2];
    uint8_t     round, won[2], phase, cones;
    int32_t     t_ms;               /* negative: countdown                    */
    uint8_t     speed;              /* 1, 2, 4, 8, or 0 = as fast as it goes  */
    char        msg[11];            /* "GO!", "OUT!" ...                      */
    /* the menu */
    const char *opp_name, *opp_style;
    uint8_t     joystick;           /* you drive, instead of student.c        */
    uint8_t     league;             /* the menu item is the tournament        */
    /* the tournament */
    uint8_t     tour_on, tour_done, tour_n;
    uint8_t     W, D, L;
} snap_t;

void scene_draw(const snap_t *s);   /* into the OLED framebuffer; no flush    */

#endif
