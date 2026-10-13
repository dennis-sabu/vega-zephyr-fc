/*
 * mixer.h - Multirotor Flight Mixer and Actuator Driver
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Airframe: S500 Quadcopter Quad-X Configuration
 *
 * Motor Geometry & Rotation (Top View):
 *           Front
 *    M4 (CW)     M1 (CCW)   [Ch 3, Ch 0]
 *        \        /
 *         \      /
 *          [AIR]
 *         /      \
 *        /        \
 *    M3 (CCW)    M2 (CW)    [Ch 2, Ch 1]
 *           Rear
 *
 * Equations:
 * - M1 = Throttle - Pitch - Roll - Yaw
 * - M2 = Throttle + Pitch - Roll + Yaw
 * - M3 = Throttle + Pitch + Roll - Yaw
 * - M4 = Throttle - Pitch + Roll + Yaw
 */

#ifndef MIXER_H_
#define MIXER_H_

#include "types.h"
#include "config.h"

/**
 * @brief Initialize the PWM0 peripheral and configure 400 Hz ESC channels.
 * Asserts the THEJAS32 global enable register at 0x80.
 *
 * @return 0 on success, negative error code on failure.
 */
int mixer_init(void);

/**
 * @brief Compute individual motor throttle demands using Quad-X mixing equations.
 * M1 = Throttle - Pitch - Roll - Yaw
 * M2 = Throttle + Pitch - Roll + Yaw
 * M3 = Throttle + Pitch + Roll - Yaw
 * M4 = Throttle - Pitch + Roll + Yaw
 *
 * Constrains outputs strictly between 1000 us (idle) and 2000 us (max).
 * If disarmed, forces all outputs to 900 us.
 *
 * @param throttle Normalized collective throttle [0.0f, 1.0f].
 * @param roll_cmd Normalized roll demand [-1.0f, +1.0f].
 * @param pitch_cmd Normalized pitch demand [-1.0f, +1.0f].
 * @param yaw_cmd Normalized yaw demand [-1.0f, +1.0f].
 * @param is_armed System arming state.
 * @param output Output structure containing normalized and pulse microsecond values.
 */
void mixer_compute(float throttle, float roll_cmd, float pitch_cmd, float yaw_cmd,
		   bool is_armed, motor_output_t *output);

/**
 * @brief Apply pulse widths to PWM0 hardware channels 0-3 via thejas32 PWM driver.
 *
 * @param output Pointer to calculated motor output structure.
 * @return 0 on success, negative error code on failure.
 */
int mixer_apply_output(const motor_output_t *output);

/**
 * @brief Force all motors to safe disarmed state (900 us / 0% duty).
 */
void mixer_disarm(void);

#endif /* MIXER_H_ */
