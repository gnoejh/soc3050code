/*
 * mathx.h - fmath.h's functions, compiled ONCE
 *
 * fmath.h's f_sin, f_cos, f_atan2 and f_sqrt are `static inline`: fast, but
 * every call site gets its own copy of the polynomial - about 230 bytes for
 * f_sin.  nav.c calls them a dozen times, and this lesson's first build came
 * out at 34.9 KB on a 32 KB chip.  One out-of-line copy of each, called
 * everywhere, saved 552 B at the price of a function call (a few cycles)
 * each time.  Inline is a speed/size trade, not free speed.
 */
#ifndef MATHX_H
#define MATHX_H

float m_sin(float a);
float m_cos(float a);
float m_atan2(float y, float x);
float m_sqrt(float x);
float m_wrap(float a);

#endif
