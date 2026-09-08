/*
 * hal_sim.c - host backend for the application track
 * SOC3050 ATmega128 Educational Framework, 2026 AVR edition
 *
 * Compiled by _build/build-host.bat into control.dll and loaded by
 * _sim/harness.py through ctypes. The lesson's control.c is linked in
 * unchanged - exactly the same source that goes to the AVR.
 *
 * WHY ctypes AND NOT A SOCKET
 * ---------------------------
 * An early sketch had the world talk to the controller over a socket. Direct
 * calls are better here: no port to collide, no partial reads to frame, no
 * ordering to get wrong, and a step costs microseconds instead of milliseconds
 * - which matters when a scoring run is 20 000 steps and a class of students
 * each want several. The C stays C and the Python stays Python either way.
 *
 * THE ONE RISK, AND HOW IT IS HANDLED
 * -----------------------------------
 * ctypes must lay its structures out byte-for-byte the way this compiler did.
 * It does follow native alignment rules, but "should match" is not evidence,
 * so the sizes are exported and harness.py refuses to run if they disagree.
 * A silent layout mismatch would show up as physics that is subtly wrong
 * rather than as an error, which is the worst way for anything to fail.
 */

#include "control.h"

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

/* A lesson may define control_init(); if it does not, this empty one is used. */
__attribute__((weak)) void control_init(void) {}

/* --- layout check ------------------------------------------------------- */

EXPORT int sim_sensors_size(void) { return (int)sizeof(sensors_t); }
EXPORT int sim_actuators_size(void) { return (int)sizeof(actuators_t); }
EXPORT int sim_channels(void) { return CTRL_CHANNELS; }

/* Offsets too: two structs can share a size and still disagree on padding. */
EXPORT int sim_offset_q(void) { return (int)__builtin_offsetof(sensors_t, q); }
EXPORT int sim_offset_sensor(void) { return (int)__builtin_offsetof(sensors_t, sensor); }

/* --- the loop ----------------------------------------------------------- */

EXPORT void sim_init(void) { control_init(); }

/**
 * One control step. The world fills @p in, this calls the student's code, and
 * the world reads @p out.
 *
 * The efforts are clamped here as well as in the lesson, so a controller that
 * forgets to saturate cannot hand the physics an impossible torque and appear
 * to succeed. On the board the PWM register would have clipped it anyway; the
 * simulator has to impose the same limit or it is not the same system.
 */
EXPORT void sim_step(const sensors_t *in, actuators_t *out)
{
    out->led = 0;
    for (int i = 0; i < CTRL_CHANNELS; i++)
        out->effort[i] = 0;

    control_step(in, out);

    for (int i = 0; i < CTRL_CHANNELS; i++)
        out->effort[i] = ctrl_clamp(out->effort[i]);
}
