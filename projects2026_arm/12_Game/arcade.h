/*
 * arcade.h - the menu, pause, game-over and high-score screens.   PURE C.
 *
 * A state machine with four states, stepped at the same fixed tick as the
 * games:
 *
 *        A                   game over
 *   MENU ---> PLAY ----------------------> OVER --A--> PLAY (same game again)
 *    ^        |  ^                           |
 *    |      B |  | B                         B
 *    |        v  |                           |
 *    |        PAUSE                          |
 *    +-------------------------------------- +
 */
#ifndef ARCADE_H
#define ARCADE_H

#include <stddef.h>
#include <stdint.h>
#include "pad.h"

enum { ARC_MENU, ARC_PLAY, ARC_PAUSE, ARC_OVER };

void    arcade_init(void);
void    arcade_step(const pad_t *in);       /* one 20 ms tick                 */
void    arcade_draw(void);                  /* framebuffer only               */
int     arcade_state(void);
int     arcade_game(void);                  /* index of the selected game     */
int32_t arcade_high(int game);

/* The platform hook: called once per finished game.  Main.c prints it as a
 * $SCORE frame; the host test checks it. */
void on_score(const char *game, int32_t score);

/* "$SCORE,SNAKE,42*1C" into out (no newline).  Returns its length. */
int score_frame(char *out, size_t n, const char *game, int32_t score);

#endif
