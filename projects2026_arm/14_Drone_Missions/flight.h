/*
 * flight.h - the flight computer: estimator, position control, modes,
 *            missions and failsafes.  It implements control.h's
 *            control_step() and owns ONE flight state, fc_get().
 *
 * Pure C: the same file flies in the firmware and in host/sitl.c.
 *
 * Nothing here is thread-safe on its own.  In the firmware every call is
 * made with Main.c's fc_lock held - the control task, the link task and the
 * UI task all reach this state.
 */
#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdint.h>
#include "control.h"

#define FC_MAX_WP   16u
#define FC_HIST     16u              /* estimator history, >= GPS delay + 1   */
#define FC_EVENTS   8u

/* Flight modes - the drone's whole behaviour is one of these at a time.  */
enum { M_DISARMED, M_TAKEOFF, M_HOLD, M_MISSION, M_RTL, M_LAND, M_COUNT };

/* Why the mode last changed - telemetry and the OLED both show it.       */
enum { R_CMD, R_BUTTON, R_TKOFF_DONE, R_MISSION_DONE, R_HOME, R_LANDED,
       R_FENCE, R_BATTERY, R_BATTERY_CRIT, R_COUNT };

/* Results of commands - each becomes the reason in an $ACK.              */
enum { FC_OK = 0, FC_E_RANGE, FC_E_FENCE, FC_E_EMPTY, FC_E_STATE,
       FC_E_PREARM, FC_E_PARAM, FC_E_CHECKSUM, FC_E_UNKNOWN, FC_E_COUNT };

/* Events, queued for the telemetry task to turn into $EVT frames.        */
enum { EV_MODE, EV_WP, EV_MISSION };

typedef struct { float x, y, z; } vec3_t;

typedef struct {
    uint32_t t_ms;
    uint8_t  kind, a, b;             /* EV_MODE: a = mode, b = reason;
                                        EV_WP: a = index reached;
                                        EV_MISSION: a = count                */
} fc_event_t;

/* Tuning - every one settable at run time with $PARAM,NAME,value
 * (values are integers x1000 unless the table in flight.c says otherwise). */
typedef struct {
    float kp_pos;      /* 1/s     position error -> velocity setpoint       */
    float kp_vel;      /* 1/s     velocity error -> acceleration            */
    float ki_vel;      /* 1/s^2   the integral: it learns the wind          */
    float kp_z, kv_z, ki_z;
    float cruise;      /* m/s     mission speed                             */
    float hold_vmax;   /* m/s     fastest a HOLD correction may fly         */
    float tilt_max;    /* rad     acceleration limit = g tan(tilt_max)      */
    float accept;      /* m       waypoint acceptance radius                */
    float fence_r;     /* m       geofence radius around home               */
    float fence_alt;   /* m       geofence ceiling                          */
    float rtl_alt;     /* m       RTL climbs to at least this               */
    float tkoff_alt;   /* m                                                 */
    float bat_low;     /* %       -> RTL                                    */
    float bat_crit;    /* %       -> LAND where it is                       */
    float line;        /* 1 = follow the line between waypoints, 0 = aim at the waypoint */
    float est_a, est_b;/*         GPS correction gains (alpha, beta)        */
} fc_params_t;

typedef struct {
    /* ---- the estimate: all the flight computer believes ---------------- */
    float px, py, pz, vx, vy, vz, yaw;
    float hx[FC_HIST], hy[FC_HIST];  /* where it believed it was, last 16 steps */
    uint8_t hi, gps_fixes;
    /* ---- control state ------------------------------------------------- */
    float spx, spy, spz;             /* position setpoint (HOLD, TAKEOFF, LAND) */
    float ix, iy, iz;                /* velocity-loop integrators            */
    float nudge_x, nudge_y;          /* joystick, m/s, HOLD only             */
    float yaw_sp;
    uint8_t mode, reason, rtl_phase;
    uint32_t t_ms, t_mode, t_still;
    float battery;
    /* ---- the mission --------------------------------------------------- */
    vec3_t   wp[FC_MAX_WP];
    uint16_t wp_set;                 /* bit i: wp[i] was loaded              */
    uint8_t  n_wp, cur, mission_active;
    vec3_t   leg_a;                  /* where the current leg starts         */
    /* ---- what it commanded last (for telemetry and the display) -------- */
    actuators_t out;
    float vsp_x, vsp_y;
    /* ---- events -------------------------------------------------------- */
    fc_event_t ev[FC_EVENTS];
    uint8_t ev_head, ev_tail, ev_lost;
    fc_params_t p;
} fc_t;

fc_t       *fc_get(void);
void        fc_reset(void);                        /* power-up state       */

int         fc_arm(void);                          /* takeoff, then HOLD   */
int         fc_disarm(void);                       /* only on the ground   */
int         fc_set_mode(uint8_t mode, uint8_t reason);
int         fc_wp_set(uint8_t idx, float x, float y, float z);
void        fc_mission_clear(void);
int         fc_mission_start(void);                /* arms if needed       */
void        fc_mission_default(void);              /* the button-A mission */
void        fc_nudge(float vx, float vy);
int         fc_param_set(const char *name, int32_t value);
int         fc_param_get(const char *name, int32_t *value);
const char *fc_param_name(unsigned i);             /* 0 at the end         */
int         fc_event_pop(fc_event_t *e);

const char *fc_mode_name(uint8_t mode);
const char *fc_reason_name(uint8_t reason);
const char *fc_error_name(int err);

#endif
