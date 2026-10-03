/* mathx.c - one copy of each of fmath.h's functions; see mathx.h */
#include "fmath.h"
#include "mathx.h"

float m_sin(float a)              { return f_sin(a); }
float m_cos(float a)              { return m_sin(a + F_PI_2); }
float m_atan2(float y, float x)   { return f_atan2(y, x); }
float m_sqrt(float x)             { return f_sqrt(x); }
float m_wrap(float a)             { return f_wrap(a); }
