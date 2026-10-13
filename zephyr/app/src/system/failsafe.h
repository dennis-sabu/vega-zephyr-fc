/*
 * failsafe.h - Flight Safety Supervisor and Arming Interlock State Machine
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Zephyr RTOS Application
 */

#ifndef FAILSAFE_H_
#define FAILSAFE_H_

#include "types.h"
#include "config.h"

/**
 * @brief Initialize the safety manager and reset all failsafe timers.
 */
void failsafe_init(void);

/**
 * @brief Evaluate all pre-flight safety interlocks to determine arming permission.
 *
 * Checks:
 * 1. Throttle stick at zero (< 5%)
 * 2. Airframe level (tilt < 10 deg)
 * 3. RC receiver signal active and healthy
 * 4. IMU calibrated and healthy
 * 5. Battery voltage not critical
 *
 * @param rc Pointer to current RC channels.
 * @param att Pointer to current attitude Euler angles (radians).
 * @param imu_healthy IMU communication health boolean.
 * @param bat_crit Battery critical state boolean.
 * @return true if all checks pass and vehicle may safely arm; false otherwise.
 */
bool failsafe_can_arm(const sbus_channels_t *rc, const euler_angles_t *att,
		      bool imu_healthy, bool bat_crit);

/**
 * @brief Update the safety state machine, evaluate trigger conditions,
 * and execute failsafe landing ramps or emergency motor stops.
 *
 * @param current_state Current system flight state.
 * @param rc Pointer to current RC channels.
 * @param att Pointer to current attitude Euler angles (radians).
 * @param imu_healthy IMU communication health boolean.
 * @param bat_crit Battery critical state boolean.
 * @param dt_ms Time elapsed since last update in milliseconds.
 * @return flight_state_t Next system flight state.
 */
flight_state_t failsafe_update(flight_state_t current_state, const sbus_channels_t *rc,
			       const euler_angles_t *att, bool imu_healthy,
			       bool bat_crit, uint32_t dt_ms);

/**
 * @brief Retrieve current detailed failsafe status and trigger mask.
 *
 * @param fs Pointer to output failsafe_state_t structure.
 */
void failsafe_get_state(failsafe_state_t *fs);

/**
 * @brief Get controlled descent throttle during active failsafe landing.
 *
 * @return float Normalized throttle [0.0f, 1.0f].
 */
float failsafe_get_throttle(void);

#endif /* FAILSAFE_H_ */
