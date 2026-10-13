/*
 * failsafe.c - Flight Safety Supervisor Implementation
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 *
 * Implements Failsafe Matrix:
 * 1. RC Link Loss (> 500 ms): Auto-level pitch/roll, smooth 5s throttle ramp, disarm.
 * 2. Critical Battery (< 3.3V/cell for > 2s): Forced autoland descent.
 * 3. High-Angle Disarm (> 75 deg tilt): Immediate zero throttle and motor cutoff.
 */

#include "failsafe.h"
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <math.h>

LOG_MODULE_REGISTER(failsafe, CONFIG_LOG_DEFAULT_LEVEL);

static failsafe_state_t s_failsafe = {
	.active = false,
	.reason_mask = FAILSAFE_REASON_NONE,
	.trigger_timestamp_ms = 0,
	.descent_throttle = 0.0f
};

static uint32_t s_bat_crit_duration_ms = 0;
static struct k_spinlock s_fs_lock;

void failsafe_init(void)
{
	k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
	s_failsafe.active = false;
	s_failsafe.reason_mask = FAILSAFE_REASON_NONE;
	s_failsafe.trigger_timestamp_ms = 0;
	s_failsafe.descent_throttle = 0.0f;
	s_bat_crit_duration_ms = 0;
	k_spin_unlock(&s_fs_lock, key);

	LOG_INF("Failsafe supervisor initialized.");
}

bool failsafe_can_arm(const sbus_channels_t *rc, const euler_angles_t *att,
		      bool imu_healthy, bool bat_crit)
{
	if (rc == NULL || att == NULL) {
		LOG_WRN("Arming blocked: Null pointer passed to safety check.");
		return false;
	}

	/* 1. Throttle stick must be completely closed (< 5%) */
	if (rc->throttle_norm > 0.05f) {
		LOG_WRN("Arming blocked: Throttle stick not at zero (%.1f%%)",
			rc->throttle_norm * 100.0f);
		return false;
	}

	/* 2. Vehicle must be level on the ground (tilt < 10 degrees) */
	if (fabsf(att->roll) > ARMING_MAX_TILT_RAD || fabsf(att->pitch) > ARMING_MAX_TILT_RAD) {
		LOG_WRN("Arming blocked: Excessive tilt angle (Roll: %.1f deg, Pitch: %.1f deg)",
			att->roll * (180.0f / 3.14159f), att->pitch * (180.0f / 3.14159f));
		return false;
	}

	/* 3. RC link must be healthy without lost frames or failsafe flags */
	if (rc->failsafe_active || rc->frame_lost) {
		LOG_WRN("Arming blocked: RC link reports failsafe or frame loss.");
		return false;
	}

	/* 4. IMU must be functional and responding */
	if (!imu_healthy) {
		LOG_ERR("Arming blocked: IMU sensor reports communication failure!");
		return false;
	}

	/* 5. Battery must not be in critical state (< 3.3V/cell) */
	if (bat_crit) {
		LOG_ERR("Arming blocked: Battery voltage is CRITICAL (< 3.3V/cell)!");
		return false;
	}

	return true;
}

flight_state_t failsafe_update(flight_state_t current_state, const sbus_channels_t *rc,
			       const euler_angles_t *att, bool imu_healthy,
			       bool bat_crit, uint32_t dt_ms)
{
	uint32_t now = (uint32_t)k_uptime_get_32();

	/* Track sustained low battery duration */
	if (bat_crit) {
		s_bat_crit_duration_ms += dt_ms;
	} else {
		s_bat_crit_duration_ms = 0;
	}

	switch (current_state) {
	case STATE_DISARMED:
		if (rc != NULL && rc->arm_switch) {
			if (failsafe_can_arm(rc, att, imu_healthy, bat_crit)) {
				LOG_INF("Arming checks passed: SYSTEM ARMED!");
				return STATE_ARMED;
			}
		}
		return STATE_DISARMED;

	case STATE_ARMED:
		/* 1. Pilot requested disarm via switch */
		if (rc != NULL && !rc->arm_switch) {
			LOG_INF("Disarm switch toggled: SYSTEM DISARMED.");
			return STATE_DISARMED;
		}

		/* 2. High-Angle Disarm (> 75 degrees tilt crash detection) */
		if (att != NULL &&
		    (fabsf(att->roll) > CRASH_TILT_LIMIT_RAD || fabsf(att->pitch) > CRASH_TILT_LIMIT_RAD)) {
			LOG_ERR("CRASH DETECTED! Tilt exceeded 75 deg. IMMEDIATE MOTOR CUTOFF!");
			k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
			s_failsafe.active = true;
			s_failsafe.reason_mask |= FAILSAFE_REASON_EXCESSIVE_TILT;
			k_spin_unlock(&s_fs_lock, key);
			return STATE_EMERGENCY_STOP;
		}

		/* 3. IMU failure detection */
		if (!imu_healthy) {
			LOG_ERR("IMU FAILURE during flight! Emergency stop.");
			k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
			s_failsafe.active = true;
			s_failsafe.reason_mask |= FAILSAFE_REASON_IMU_FAILURE;
			k_spin_unlock(&s_fs_lock, key);
			return STATE_EMERGENCY_STOP;
		}

		/* 4. RC Signal Loss (> 500 ms without valid packet) */
		if (rc == NULL || rc->failsafe_active || rc->frame_lost ||
		    (now - rc->last_packet_time_ms) > RC_TIMEOUT_MS) {
			LOG_WRN("RC SIGNAL LOST (> 500 ms)! Entering controlled FAILSAFE descent...");
			k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
			s_failsafe.active = true;
			s_failsafe.reason_mask |= FAILSAFE_REASON_RC_LOST;
			s_failsafe.trigger_timestamp_ms = now;
			s_failsafe.descent_throttle = FAILSAFE_DESCENT_THROTTLE;
			k_spin_unlock(&s_fs_lock, key);
			return STATE_FAILSAFE;
		}

		/* 5. Critical Battery Voltage sustained for > 2 seconds */
		if (s_bat_crit_duration_ms >= BATTERY_CRITICAL_TIMEOUT_MS) {
			LOG_WRN("CRITICAL BATTERY SUSTAINED (> 2s)! Initiating forced autoland descent.");
			k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
			s_failsafe.active = true;
			s_failsafe.reason_mask |= FAILSAFE_REASON_LOW_BATTERY;
			s_failsafe.trigger_timestamp_ms = now;
			s_failsafe.descent_throttle = FAILSAFE_DESCENT_THROTTLE;
			k_spin_unlock(&s_fs_lock, key);
			return STATE_FAILSAFE;
		}

		return STATE_ARMED;

	case STATE_FAILSAFE:
		/* Immediate pilot manual override disarm */
		if (rc != NULL && !rc->arm_switch) {
			LOG_INF("Disarm switch toggled during failsafe: DISARMING.");
			k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
			s_failsafe.active = false;
			k_spin_unlock(&s_fs_lock, key);
			return STATE_DISARMED;
		}

		/* High-angle crash cutoff during descent (> 75 degrees) */
		if (att != NULL &&
		    (fabsf(att->roll) > CRASH_TILT_LIMIT_RAD || fabsf(att->pitch) > CRASH_TILT_LIMIT_RAD)) {
			LOG_ERR("Vehicle inverted during failsafe descent. IMMEDIATE CUTOFF.");
			return STATE_EMERGENCY_STOP;
		}

		/* Smooth throttle ramp down over FAILSAFE_DESCENT_TIME_MS (5 seconds) then disarm */
		{
			k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
			uint32_t elapsed = now - s_failsafe.trigger_timestamp_ms;

			if (elapsed >= FAILSAFE_DESCENT_TIME_MS) {
				LOG_INF("Failsafe descent completed (5s elapsed). Disarming motors.");
				s_failsafe.active = false;
				s_failsafe.descent_throttle = 0.0f;
				k_spin_unlock(&s_fs_lock, key);
				return STATE_DISARMED;
			}

			/* Linear ramp down to 0 over 5 seconds */
			float progress = (float)elapsed / (float)FAILSAFE_DESCENT_TIME_MS;
			s_failsafe.descent_throttle = FAILSAFE_DESCENT_THROTTLE * (1.0f - progress);
			k_spin_unlock(&s_fs_lock, key);
		}
		return STATE_FAILSAFE;

	case STATE_EMERGENCY_STOP:
		/* Require manual switch flip to disarm before system can reset */
		if (rc != NULL && !rc->arm_switch) {
			LOG_INF("Emergency stopped cleared by disarm switch.");
			k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
			s_failsafe.active = false;
			s_failsafe.reason_mask = FAILSAFE_REASON_NONE;
			k_spin_unlock(&s_fs_lock, key);
			return STATE_DISARMED;
		}
		return STATE_EMERGENCY_STOP;

	default:
		return STATE_DISARMED;
	}
}

void failsafe_get_state(failsafe_state_t *fs)
{
	if (fs != NULL) {
		k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
		*fs = s_failsafe;
		k_spin_unlock(&s_fs_lock, key);
	}
}

float failsafe_get_throttle(void)
{
	k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
	float thr = s_failsafe.descent_throttle;
	k_spin_unlock(&s_fs_lock, key);
	return thr;
}

static uint32_t s_eval_rclost_trigger_ms = 0;
static uint32_t s_eval_lowbatt_duration_ms = 0;
static uint32_t s_eval_last_eval_time_ms = 0;

failsafe_condition_t failsafe_evaluate(uint32_t now_ms,
				       uint32_t last_rc_time_ms,
				       float cell_voltage,
				       float roll_deg,
				       float pitch_deg,
				       motor_output_t *override_motors)
{
	if (override_motors == NULL) {
		return FAILSAFE_NONE;
	}

	uint32_t dt_ms = (s_eval_last_eval_time_ms > 0 && now_ms > s_eval_last_eval_time_ms) ?
			 (now_ms - s_eval_last_eval_time_ms) : 4;
	s_eval_last_eval_time_ms = now_ms;

	/* 1. Crash Detection (> 75 degrees roll or pitch) */
	if (fabsf(roll_deg) > 75.0f || fabsf(pitch_deg) > 75.0f) {
		for (int i = 0; i < 4; i++) {
			override_motors->norm[i] = 0.0f;
			override_motors->pulse_us[i] = PWM_PULSE_DISARMED_US; /* 900 us */
		}
		return FAILSAFE_CRASH;
	}

	/* 2. RC Link Loss (> 500 ms) */
	if ((now_ms - last_rc_time_ms) > 500U) {
		if (s_eval_rclost_trigger_ms == 0) {
			s_eval_rclost_trigger_ms = now_ms;
		}

		uint32_t elapsed = now_ms - s_eval_rclost_trigger_ms;
		if (elapsed >= 5000U) {
			/* Completed 5-second descent ramp -> disarm to 900 us */
			for (int i = 0; i < 4; i++) {
				override_motors->norm[i] = 0.0f;
				override_motors->pulse_us[i] = PWM_PULSE_DISARMED_US;
			}
		} else {
			/* Linear descent ramp down to zero over 5 seconds */
			float progress = (float)elapsed / 5000.0f;
			float descent_thr = FAILSAFE_DESCENT_THROTTLE * (1.0f - progress);
			uint32_t pulse = PWM_PULSE_MIN_US + (uint32_t)(descent_thr * (float)(PWM_PULSE_MAX_US - PWM_PULSE_MIN_US));
			for (int i = 0; i < 4; i++) {
				override_motors->norm[i] = descent_thr;
				override_motors->pulse_us[i] = pulse;
			}
		}
		return FAILSAFE_RCLOST;
	} else {
		s_eval_rclost_trigger_ms = 0;
	}

	/* 3. Low Battery (< 3.3V/cell sustained for > 2 seconds) */
	if (cell_voltage > 0.0f && cell_voltage < 3.30f) {
		s_eval_lowbatt_duration_ms += dt_ms;
		if (s_eval_lowbatt_duration_ms >= 2000U) {
			/* Force controlled descent throttle (35%) */
			float descent_thr = FAILSAFE_DESCENT_THROTTLE;
			uint32_t pulse = PWM_PULSE_MIN_US + (uint32_t)(descent_thr * (float)(PWM_PULSE_MAX_US - PWM_PULSE_MIN_US));
			for (int i = 0; i < 4; i++) {
				override_motors->norm[i] = descent_thr;
				override_motors->pulse_us[i] = pulse;
			}
			return FAILSAFE_LOWBATT;
		}
	} else {
		s_eval_lowbatt_duration_ms = 0;
	}

	return FAILSAFE_NONE;
}

