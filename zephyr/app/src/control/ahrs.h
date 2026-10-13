/*
 * ahrs.h - Attitude and Heading Reference System
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Zephyr RTOS Application
 *
 * Implements:
 * 1. Complementary Filter: theta = alpha*(theta + omega*dt) + (1 - alpha)*theta_acc
 * 2. Madgwick gradient-descent quaternion fusion
 */

#ifndef AHRS_H_
#define AHRS_H_

#include "types.h"
#include "config.h"

/**
 * @brief Initialize the AHRS filter with sample rate and complementary alpha.
 *
 * @param sample_freq Loop sampling frequency in Hz (250.0f).
 * @param alpha Complementary filter ratio (0.98f).
 */
void ahrs_init(float sample_freq, float alpha);

/**
 * @brief Update attitude estimation using latest gyro and accelerometer data.
 * Executes the discrete complementary filter equation:
 * theta = alpha * (theta + omega * dt) + (1 - alpha) * theta_acc
 *
 * @param gyro Pointer to angular rates vector (rad/s in body frame).
 * @param accel Pointer to linear acceleration vector (m/s^2 in body frame).
 * @param dt Time step in seconds (0.004 s @ 250 Hz).
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
 * @brief Get the latest angular rates in rad/s.
 *
 * @param rates Pointer to vector3f_t output structure.
 */
void ahrs_get_rates(vector3f_t *rates);

/**
 * @brief Reset AHRS orientation to level [0, 0, 0].
 */
void ahrs_reset(void);

#endif /* AHRS_H_ */
