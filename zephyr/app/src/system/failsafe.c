/*
 * failsafe.c - Flight Safety Supervisor Implementation
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 *
 * Implements arming safety checks, RC loss detection, battery critical failsafe,
 * crash angle cutoff (> 65 degrees), and controlled descent timing.
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

	/* 5. Battery must not be in critical state */
	if (bat_crit) {
		LOG_ERR("Arming blocked: Battery voltage is CRITICAL!");
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
		/* Pilot requested disarm via switch */
		if (rc != NULL && !rc->arm_switch) {
			LOG_INF("Disarm switch toggled: SYSTEM DISARMED.");
			return STATE_DISARMED;
		}

		/* Crash / Inversion detection (> 65 degrees tilt) */
		if (att != NULL &&
		    (fabsf(att->roll) > CRASH_TILT_LIMIT_RAD || fabsf(att->pitch) > CRASH_TILT_LIMIT_RAD)) {
			LOG_ERR("CRASH DETECTED! Tilt exceeded 65 deg. EMERGENCY MOTOR CUT!");
			k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
			s_failsafe.active = true;
			s_failsafe.reason_mask |= FAILSAFE_REASON_EXCESSIVE_TILT;
			k_spin_unlock(&s_fs_lock, key);
			return STATE_EMERGENCY_STOP;
		}

		/* IMU failure detection */
		if (!imu_healthy) {
			LOG_ERR("IMU FAILURE during flight! Emergency stop.");
			k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
			s_failsafe.active = true;
			s_failsafe.reason_mask |= FAILSAFE_REASON_IMU_FAILURE;
			k_spin_unlock(&s_fs_lock, key);
			return STATE_EMERGENCY_STOP;
		}

		/* RC Signal Loss detection */
		if (rc == NULL || rc->failsafe_active || rc->frame_lost ||
		    (now - rc->last_packet_time_ms) > RC_TIMEOUT_MS) {
			LOG_WRN("RC SIGNAL LOST! Entering controlled FAILSAFE descent...");
			k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
			s_failsafe.active = true;
			s_failsafe.reason_mask |= FAILSAFE_REASON_RC_LOST;
			s_failsafe.trigger_timestamp_ms = now;
			s_failsafe.descent_throttle = FAILSAFE_DESCENT_THROTTLE;
			k_spin_unlock(&s_fs_lock, key);
			return STATE_FAILSAFE;
		}

		/* Sustained Critical Battery detection */
		if (s_bat_crit_duration_ms >= BATTERY_CRITICAL_TIMEOUT_MS) {
			LOG_WRN("SUSTAINED CRITICAL BATTERY! Initiating forced descent.");
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
		/* Allow pilot to disarm immediately with RC switch */
		if (rc != NULL && !rc->arm_switch) {
			LOG_INF("Disarm switch toggled during failsafe: DISARMING.");
			k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
			s_failsafe.active = false;
			k_spin_unlock(&s_fs_lock, key);
			return STATE_DISARMED;
		}

		/* Immediate cutoff on ground impact / roll over */
		if (att != NULL &&
		    (fabsf(att->roll) > CRASH_TILT_LIMIT_RAD || fabsf(att->pitch) > CRASH_TILT_LIMIT_RAD)) {
			LOG_ERR("Vehicle tipped over during failsafe. EMERGENCY CUT.");
			return STATE_EMERGENCY_STOP;
		}

		/* Ramp down throttle over FAILSAFE_DESCENT_TIME_MS */
		{
			k_spinlock_key_t key = k_spin_lock(&s_fs_lock);
			uint32_t elapsed = now - s_failsafe.trigger_timestamp_ms;

			if (elapsed >= FAILSAFE_DESCENT_TIME_MS) {
				LOG_INF("Failsafe descent duration expired. Disarming motors.");
				s_failsafe.active = false;
				s_failsafe.descent_throttle = 0.0f;
				k_spin_unlock(&s_fs_lock, key);
				return STATE_DISARMED;
			}

			float progress = (float)elapsed / (float)FAILSAFE_DESCENT_TIME_MS;
			/* Ramp throttle down from initial descent throttle */
			s_failsafe.descent_throttle = FAILSAFE_DESCENT_THROTTLE * (1.0f - (progress * 0.75f));
			k_spin_unlock(&s_fs_lock, key);
		}
		return STATE_FAILSAFE;

	case STATE_EMERGENCY_STOP:
		/* Require physical disarm switch toggle to clear emergency state */
		if (rc != NULL && !rc->arm_switch) {
			LOG_INF("Emergency cleared by disarm switch.");
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
