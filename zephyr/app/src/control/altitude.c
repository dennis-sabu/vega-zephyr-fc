/*
 * altitude.c - Cascaded Altitude Hold Controller Implementation
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Airframe: S500 Quadcopter (~2.5 kg AUW)
 *
 * Implements:
 * 1. RC throttle center deadband ([0.42, 0.58]) for altitude hold.
 * 2. Outer Loop: Altitude Error -> Target Vertical Velocity.
 * 3. Inner Loop: Velocity Error -> Collective Throttle Output.
 */

#include "altitude.h"
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <math.h>

LOG_MODULE_REGISTER(altitude, CONFIG_LOG_DEFAULT_LEVEL);

/* Outer Position Loop Gains */
#define POS_KP  1.20f
#define POS_KI  0.00f
#define POS_KD  0.00f

/* Inner Velocity Loop Gains */
#define VEL_KP  0.22f
#define VEL_KI  0.15f
#define VEL_KD  0.008f

static altitude_controller_t s_state;
static struct k_spinlock s_alt_lock;
static bool s_was_active_prev = false;

void altitude_controller_init(void)
{
	k_spinlock_key_t key = k_spin_lock(&s_alt_lock);

	s_state.target_altitude_m = 0.0f;
	s_state.current_altitude_m = 0.0f;
	s_state.target_velocity_mps = 0.0f;
	s_state.current_velocity_mps = 0.0f;
	s_state.hover_throttle = ALT_HOVER_THROTTLE_DEFAULT;
	s_state.throttle_output = 0.0f;
	s_state.is_active = false;

	/* Initialize Outer Loop PID (Position Z to Velocity Z) */
	pid_init(&s_state.pid_position, POS_KP, POS_KI, POS_KD,
		 -ALT_MAX_DESCENT_RATE_MPS, ALT_MAX_CLIMB_RATE_MPS,
		 0.0f, 0.0f,
		 0.0f, ALT_HOLD_DT_S, false);

	/* Initialize Inner Loop PID (Velocity Z to Throttle Offset) */
	pid_init(&s_state.pid_velocity, VEL_KP, VEL_KI, VEL_KD,
		 -0.35f, 0.35f,
		 -0.20f, 0.20f,
		 20.0f, ALT_HOLD_DT_S, true);

	s_was_active_prev = false;
	k_spin_unlock(&s_alt_lock, key);

	LOG_INF("Cascaded Altitude Controller initialized (50 Hz).");
}

void altitude_controller_reset(float current_altitude_m)
{
	k_spinlock_key_t key = k_spin_lock(&s_alt_lock);

	s_state.target_altitude_m = (current_altitude_m > 0.0f) ? current_altitude_m : 0.0f;
	s_state.current_altitude_m = current_altitude_m;
	s_state.target_velocity_mps = 0.0f;
	s_state.current_velocity_mps = 0.0f;
	s_state.throttle_output = s_state.hover_throttle;

	pid_reset(&s_state.pid_position);
	pid_reset(&s_state.pid_velocity);

	k_spin_unlock(&s_alt_lock, key);
}

float altitude_controller_update(float stick_throttle, float current_alt_m,
				 float current_vario_mps, bool is_alt_hold_mode,
				 float dt_s)
{
	if (dt_s <= 0.0f) {
		dt_s = ALT_HOLD_DT_S;
	}

	/* Manual Throttle Pass-Through when Altitude Hold is disengaged */
	if (!is_alt_hold_mode) {
		k_spinlock_key_t key = k_spin_lock(&s_alt_lock);
		s_state.is_active = false;
		s_state.current_altitude_m = current_alt_m;
		s_state.current_velocity_mps = current_vario_mps;
		s_state.throttle_output = stick_throttle;
		s_was_active_prev = false;
		k_spin_unlock(&s_alt_lock, key);
		return stick_throttle;
	}

	k_spinlock_key_t key = k_spin_lock(&s_alt_lock);

	/* Mode Transition: Lock target altitude on engagement */
	if (!s_was_active_prev) {
		s_state.target_altitude_m = (current_alt_m > 0.0f) ? current_alt_m : 0.0f;
		pid_reset(&s_state.pid_position);
		pid_reset(&s_state.pid_velocity);
		s_was_active_prev = true;
		LOG_INF("Altitude Hold ENGAGED! Target locked at %.2f m", s_state.target_altitude_m);
	}

	s_state.is_active = true;
	s_state.current_altitude_m = current_alt_m;
	s_state.current_velocity_mps = current_vario_mps;

	/* Emergency / Ground Safety: Throttle pulled to absolute bottom */
	if (stick_throttle < 0.05f) {
		s_state.throttle_output = 0.0f;
		s_state.target_velocity_mps = 0.0f;
		pid_reset(&s_state.pid_velocity);
		k_spin_unlock(&s_alt_lock, key);
		return 0.0f;
	}

	/*
	 * 1. RC Throttle Center Deadband Logic ([0.42, 0.58]):
	 * - Deadband center holds locked target altitude.
	 * - Above deadband requests climb up to ALT_MAX_CLIMB_RATE_MPS.
	 * - Below deadband requests descent down to ALT_MAX_DESCENT_RATE_MPS.
	 */
	float manual_climb_cmd_mps = 0.0f;
	float deadband_high = ALT_STICK_CENTER + ALT_STICK_DEADBAND; /* 0.58 */
	float deadband_low  = ALT_STICK_CENTER - ALT_STICK_DEADBAND; /* 0.42 */

	if (stick_throttle > deadband_high) {
		float ratio = (stick_throttle - deadband_high) / (1.0f - deadband_high);
		manual_climb_cmd_mps = ratio * ALT_MAX_CLIMB_RATE_MPS;
		/* Ramp target altitude up */
		s_state.target_altitude_m += manual_climb_cmd_mps * dt_s;
	} else if (stick_throttle < deadband_low) {
		float ratio = (deadband_low - stick_throttle) / deadband_low;
		manual_climb_cmd_mps = -ratio * ALT_MAX_DESCENT_RATE_MPS;
		/* Ramp target altitude down */
		s_state.target_altitude_m += manual_climb_cmd_mps * dt_s;
		if (s_state.target_altitude_m < 0.0f) {
			s_state.target_altitude_m = 0.0f;
		}
	} else {
		/* Inside deadband: zero commanded vertical velocity */
		manual_climb_cmd_mps = 0.0f;
	}

	/*
	 * 2. Outer Loop: Altitude Error -> Target Vertical Velocity
	 */
	float pos_correction_mps = pid_update(&s_state.pid_position,
					      s_state.target_altitude_m,
					      current_alt_m, dt_s);

	float target_velocity = manual_climb_cmd_mps + pos_correction_mps;

	/* Saturate target vertical velocity */
	if (target_velocity > ALT_MAX_CLIMB_RATE_MPS) {
		target_velocity = ALT_MAX_CLIMB_RATE_MPS;
	} else if (target_velocity < -ALT_MAX_DESCENT_RATE_MPS) {
		target_velocity = -ALT_MAX_DESCENT_RATE_MPS;
	}
	s_state.target_velocity_mps = target_velocity;

	/*
	 * 3. Inner Loop: Vertical Velocity Error -> Collective Throttle Output
	 */
	float throttle_correction = pid_update(&s_state.pid_velocity,
					       target_velocity,
					       current_vario_mps, dt_s);

	float total_throttle = s_state.hover_throttle + throttle_correction;

	/* Saturate collective throttle within safe flight envelope */
	if (total_throttle > ALT_THROTTLE_MAX) {
		total_throttle = ALT_THROTTLE_MAX;
	} else if (total_throttle < ALT_THROTTLE_MIN) {
		total_throttle = ALT_THROTTLE_MIN;
	}

	s_state.throttle_output = total_throttle;

	k_spin_unlock(&s_alt_lock, key);
	return total_throttle;
}

void altitude_controller_get_state(altitude_controller_t *state)
{
	if (state != NULL) {
		k_spinlock_key_t key = k_spin_lock(&s_alt_lock);
		*state = s_state;
		k_spin_unlock(&s_alt_lock, key);
	}
}
