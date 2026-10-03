/*
 * view.h - draw the robot's map, its plan and both poses on the OLED
 *
 * Harness code: it shows the TRUTH (the real pose, the real walls if asked)
 * beside the robot's BELIEF, which is the point of the picture.  Pure C over
 * oled.h's framebuffer, so host/sitl.c renders the same frame as ASCII art.
 */
#ifndef VIEW_H
#define VIEW_H

#include <stdint.h>

#define VIEW_PX   4               /* pixels per cell: 32 x 14 cells = 128 x 56 */

/* Redraw the whole framebuffer (no bus traffic; call oled_flush() after). */
void view_draw(int show_truth, uint32_t run_ms);

#endif
