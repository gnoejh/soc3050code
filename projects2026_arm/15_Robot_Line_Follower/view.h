/*
 * view.h - the race on a 128x64 OLED
 *
 *   y  0..7   OVAL L2 12.34 B 9.87      track, laps, this lap, best lap
 *   y  9..54  the whole track, scaled to fit, and the robot as a triangle
 *   y 56..63  |||||||  A1000 RUN        the seven sensors as bars, mode,
 *                                       speed setting, race state
 *
 * Pure C on top of oled.h's framebuffer, so host/sitl.c draws the very same
 * frame on a PC and prints it as text for the slides.
 */
#ifndef VIEW_H
#define VIEW_H

#include <stdint.h>
#include "world.h"

void view_menu(int selected);
void view_track(void);                         /* clear, fit, draw the track    */
void view_robot(float x, float y, float th);   /* move the triangle (XOR)       */
void view_hud(const world_t *w, const sensors_t *s, int manual, int knob);

#endif
