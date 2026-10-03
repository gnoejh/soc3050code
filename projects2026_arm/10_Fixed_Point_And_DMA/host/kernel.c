/*
 * kernel.c - one piece of float maths, compiled for two different CPUs
 *
 * SOC3050 lesson 10.  Not part of the firmware: fpu_compare.bat / .sh compile
 * this file twice and disassemble both results side by side.
 *
 *   Cortex-M0+  (our STM32C031C6)   -mcpu=cortex-m0plus -mfloat-abi=soft
 *   Cortex-M4F  (e.g. an STM32F446) -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
 *
 * The C is identical.  What the compiler can do with it is not: the M4F has a
 * floating-point unit, so a float multiply is one instruction; the M0+ has
 * none, so the same line becomes a call into libgcc's software float.
 */

/* Rotate the point (x, y) by an angle whose cosine and sine are c and s:
 * the 2D rotation the benchmark arena races in every number format. */
void rotate(float c, float s, float *x, float *y)
{
    float nx = c * *x - s * *y;
    float ny = s * *x + c * *y;
    *x = nx;
    *y = ny;
}

/* One step of a PI controller - the shape of every control loop in Part 3. */
float pi_step(float err, float *integ, float kp, float ki, float dt)
{
    *integ += err * dt;
    return kp * err + ki * *integ;
}
