/*
 * fm.h - fmath.h's functions, compiled ONCE and called
 *
 * fmath.h makes them `static inline`, which is right for a lesson that calls
 * each twice.  This one calls sin, cos, atan2 and sqrt from four files and
 * dozens of places, and the compiler gave each file its own copy of each -
 * ten copies, about 1.3 KB.  On a 32 KB chip that is a real price, so here
 * they are compiled once in fm.c and called.  A call costs ~4 cycles; the
 * function itself ~300-600 (soft float).  Slide 21 has the numbers.
 *
 * f_clamp and f_abs stay inline: they are smaller than a call.
 */
#ifndef FM_H
#define FM_H

float m_sin(float a);
float m_cos(float a);
float m_atan2(float y, float x);
float m_sqrt(float x);
float m_wrap(float a);

#endif
