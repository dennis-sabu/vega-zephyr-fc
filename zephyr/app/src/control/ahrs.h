/*
 * ahrs.h - Attitude and Heading Reference System (Madgwick Filter)
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Zephyr RTOS Application
 */

#ifndef AHRS_H_
#define AHRS_H_

#include "types.h"
#include "config.h"

/**
 * @brief Initialize the AHRS filter with sample rate and divergence beta.
 *
 * @param sample_freq Loop sampling frequency in Hz (e.g., 500.0f).
 * @param beta Filter gain determining rate of accelerometer convergence.
 */
void ahrs_init(float sample_freq, float beta);

/**
 * @brief Update orientation quaternion using latest gyro and accelerometer data.
 *
 * @param gyro Pointer to angular rates vector (rad/s in body frame).
 * @param accel Pointer to linear acceleration vector (m/s^2 in body frame).
 * @param dt Time step in seconds.
 */
void ahrs_update(const vector3f_t *gyro, const vector3f_t *accel, float dt);

/**
 * @brief Get the current orientation quaternion.
 *
 * @param q Pointer to quaternion output structure.
 */
void ahrs_get_quaternion(quaternion_t *q);

/**
 * @brief Get the current Euler angles in radians (Roll, Pitch, Yaw).
 *
 * @param euler Pointer to Euler angles output structure.
 */
void ahrs_get_euler_angles(euler_angles_t *euler);

/**
 * @brief Get the current Euler angles in degrees (Roll, Pitch, Yaw).
 *
 * @param euler Pointer to Euler angles output structure.
 */
void ahrs_get_euler_degrees(euler_angles_t *euler);

/**
 * @brief Reset AHRS quaternion to identity [1, 0, 0, 0].
 */
void ahrs_reset(void);

#endif /* AHRS_H_ */
