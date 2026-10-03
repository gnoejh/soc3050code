/*
 * fm.c - one copy each of fmath.h's sin, cos, atan2, sqrt and wrap.  See fm.h.
 */
#include "fm.h"
#include "fmath.h"

float m_sin(float a)            { return f_sin(a); }
float m_cos(float a)            { return f_cos(a); }
float m_atan2(float y, float x) { return f_atan2(y, x); }
float m_sqrt(float x)           { return f_sqrt(x); }
float m_wrap(float a)           { return f_wrap(a); }
