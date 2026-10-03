/*
 * view.h - what the OLED shows: the robot from the side, and a tilt chart
 *
 * Pure C on top of oled.h's framebuffer: it never touches the bus, so the
 * host test draws a frame on the PC and prints it as text.
 */
#ifndef VIEW_H
#define VIEW_H

#include <stdint.h>

#define VIEW_HIST 36          /* tilt history samples: one per frame */

typedef struct {
    const char *mode;         /* controller name                          */
    float th;                 /* true tilt, rad - the world, as an eye sees it */
    float x, xd;              /* axle position m, speed m/s               */
    float x_ref;              /* where the controller wants to be, m      */
    float u;                  /* motor volts                              */
    float payload;            /* kg                                       */
    uint8_t fallen;
    float cam;                /* camera position, m: follows the robot    */
    int8_t hist[VIEW_HIST];   /* tilt, degrees, clipped to +-30           */
    uint8_t head;             /* next slot to write                       */
} view_t;

void view_history(view_t *v, float th);   /* add one tilt sample, rad      */
void view_draw(view_t *v);                /* clear and draw the whole frame */

#endif
