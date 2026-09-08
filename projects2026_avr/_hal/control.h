/*
 * control.h - the contract every application-track lesson implements
 * SOC3050 ATmega128 Educational Framework, 2026 AVR edition
 *
 * One function is the whole lesson:
 *
 *     void control_step(const sensors_t *in, actuators_t *out);
 *
 * It is compiled twice from the same source, against two backends:
 *
 *   hal_avr.c   in  <- ADC, encoder, I2C          out -> Timer1 PWM
 *               built by _build/build-lesson.bat, runs on the real board
 *
 *   hal_sim.c   in  <- the Python world           out -> the Python world
 *               built by _build/build-host.bat into control.dll, loaded by
 *               _sim/harness.py through ctypes
 *
 * This is the SITL (software-in-the-loop) pattern that ArduPilot and PX4 are
 * developed with: the controller is the real firmware, only the physics moves
 * off-chip. Students never edit the Python.
 *
 * WHY EVERYTHING IS AN INTEGER
 * ----------------------------
 * The ATmega128 has no floating-point unit. A single float multiply costs
 * hundreds of cycles and drags in several kilobytes of soft-float library, so
 * a control loop written in float will run on the host and miss its deadline
 * on the board - and you would not find out until the last week of term.
 *
 * So every quantity here is a fixed-point integer in milli-units: an angle of
 * 1.500 rad is 1500, a torque of -0.250 Nm is -250. The same arithmetic runs
 * identically in both backends, and the AVR does it in a few cycles.
 *
 * RELATION TO THE UNITREE G1 (course SOC4180GH)
 * ---------------------------------------------
 * The G1 commands each of its 29 joints at 500 Hz - 1 kHz with exactly this
 * shape: a target position, a target velocity, gains, and a feedforward
 * torque, from which the motor computes
 *
 *     tau = kp * (q_d - q) + kd * (dq_d - dq) + tau_ff
 *
 * That is the PD-with-feedforward law of lesson 30, written in the notation the
 * humanoid uses. SOC3050 builds one joint; SOC4180GH coordinates twenty-nine.
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Four channels is enough for every lesson in the track: one joint, a
 * differential-drive pair, or a quadcopter's four rotors. */
#define CTRL_CHANNELS 4

/* Actuator command range. Full reverse to full forward, so the units are the
 * same whether the output is a torque, a duty cycle or an ESC setpoint. */
#define CTRL_EFFORT_MIN (-1000)
#define CTRL_EFFORT_MAX (1000)

typedef struct
{
    uint16_t dt_ms;              /* milliseconds since the previous call     */
    uint32_t t_ms;               /* milliseconds since the run started       */
    int16_t q[CTRL_CHANNELS];    /* position: milli-radians, or milli-metres */
    int16_t dq[CTRL_CHANNELS];   /* velocity: milli-units per second         */
    int16_t target[CTRL_CHANNELS];  /* commanded position                    */
    int16_t dtarget[CTRL_CHANNELS]; /* commanded velocity                    */
    int16_t sensor[CTRL_CHANNELS];  /* line error, range, IMU - per lesson   */
} sensors_t;

typedef struct
{
    int16_t effort[CTRL_CHANNELS]; /* clamped to CTRL_EFFORT_MIN..MAX */
    uint8_t led;                   /* mirrored to PORT B on the board */
} actuators_t;

/**
 * @brief One control step. THE function students write.
 *
 * Called at a fixed rate by whichever backend is running. It must not block,
 * must not allocate, and must be deterministic: the same inputs must always
 * produce the same outputs, or the scoring harness cannot reproduce a run.
 */
void control_step(const sensors_t *in, actuators_t *out);

/**
 * @brief Optional one-time setup, called before the first control_step().
 *
 * Provided so a lesson can zero its integrator or precompute a table. The
 * backends call it if it is present.
 */
void control_init(void);

/* --- helpers, shared by every lesson ------------------------------------ */

/**
 * @brief Clamp an effort into the actuator range.
 *
 * Saturation is not a detail. An unclamped controller that asks for more than
 * the motor can give will wind up its integrator while the output is pinned,
 * and then overshoot badly on the way back - the classic integral windup that
 * lesson 28 makes visible.
 */
static inline int16_t ctrl_clamp(int32_t v)
{
    if (v > CTRL_EFFORT_MAX)
        return (int16_t)CTRL_EFFORT_MAX;
    if (v < CTRL_EFFORT_MIN)
        return (int16_t)CTRL_EFFORT_MIN;
    return (int16_t)v;
}

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H_ */
