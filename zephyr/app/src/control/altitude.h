/*
 * altitude.h - Cascaded Altitude Hold Controller (Outer Position + Inner Velocity PID)
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Airframe: S500 Quadcopter (~2.5 kg AUW)
 * Rate: 50 Hz (20 ms loop period)
 */

#ifndef ALTITUDE_H_
#define ALTITUDE_H_

#include "types.h"
#include "config.h"
#include "pid.h"
#include <stdbool.h>

/* Altitude Controller Configuration Parameters */
#define ALT_HOLD_LOOP_FREQ_HZ         50
#define ALT_HOLD_LOOP_PERIOD_MS       (1000 / ALT_HOLD_LOOP_FREQ_HZ) /* 20 ms */
#define ALT_HOLD_DT_S                 (1.0f / (float)ALT_HOLD_LOOP_FREQ_HZ) /* 0.02 s */

#define ALT_STICK_CENTER              0.50f
#define ALT_STICK_DEADBAND            0.08f    /* Deadband [0.42, 0.58] maintains altitude */
#define ALT_MAX_CLIMB_RATE_MPS        2.00f    /* Maximum climb velocity in m/s */
#define ALT_MAX_DESCENT_RATE_MPS      1.50f    /* Maximum descent velocity in m/s */

#define ALT_HOVER_THROTTLE_DEFAULT    0.50f    /* Nominal hover throttle for S500 quadcopter */
#define ALT_THROTTLE_MIN              0.15f    /* Minimum active altitude hold throttle */
#define ALT_THROTTLE_MAX              0.85f    /* Maximum active altitude hold throttle */

/* Altitude Controller Full State Structure */
typedef struct {
	float target_altitude_m;       /* Desired altitude above ground (m) */
	float current_altitude_m;      /* Measured filtered altitude (m) */
	float target_velocity_mps;     /* Desired vertical climb/descent rate (m/s) */
	float current_velocity_mps;    /* Measured vertical climb/descent rate (m/s) */
	float hover_throttle;          /* Learned/baseline hover throttle [0.0, 1.0] */
	float throttle_output;         /* Computed collective throttle output [0.0, 1.0] */
	bool is_active;                /* True when Altitude Hold is engaged */
	pid_controller_t pid_position; /* Outer Loop: Altitude Error -> Target Velocity */
	pid_controller_t pid_velocity; /* Inner Loop: Velocity Error -> Throttle Adjustment */
} altitude_controller_t;

/**
 * @brief Initialize the cascaded altitude controller with tuning gains.
 */
void altitude_controller_init(void);

/**
 * @brief Reset altitude controller state and set target altitude to current measurement.
 *
 * @param current_altitude_m Current measured altitude in meters.
 */
void altitude_controller_reset(float current_altitude_m);

/**
 * @brief Execute the 50 Hz altitude control step.
 * Implements stick deadband, outer altitude loop, and inner velocity PID loop.
 *
 * @param stick_throttle Raw normalized RC throttle stick [0.0f, 1.0f].
 * @param current_alt_m Measured filtered altitude in meters from BMP280.
 * @param current_vario_mps Measured vertical velocity in m/s from BMP280.
 * @param is_alt_hold_mode True if system flight mode is MODE_ALT_HOLD.
 * @param dt_s Actual elapsed loop time step in seconds (0.02s).
 * @return float Computed collective throttle output [0.0f, 1.0f].
 */
float altitude_controller_update(float stick_throttle, float current_alt_m,
				 float current_vario_mps, bool is_alt_hold_mode,
				 float dt_s);

/**
 * @brief Retrieve current altitude controller internal state.
 *
 * @param state Pointer to output altitude_controller_t structure.
 */
void altitude_controller_get_state(altitude_controller_t *state);

#endif /* ALTITUDE_H_ */
