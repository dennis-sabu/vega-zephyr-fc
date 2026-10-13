/*
 * mixer.h - Multirotor Flight Mixer and Actuator Driver
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Airframe: S500 Quadcopter Quad-X Configuration
 *
 * Motor Layout (View from Top looking Down):
 *           Front
 *     M4(CW)     M1(CCW)
 *        \        /
 *         \      /
 *          [AIR]
 *         /      \
 *        /        \
 *     M3(CCW)    M2(CW)
 *           Rear
 *
 * Channels 0-3 on PWM0 (0x10400000) @ 400 Hz ESC update rate.
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
 * @brief Compute individual motor throttle demands using Quad-X mixing matrix
 * and dynamic desaturation / attitude priority logic.
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
 * @brief Apply pulse widths to PWM0 hardware channels 0-3.
 *
 * @param output Pointer to calculated motor output structure.
 * @return 0 on success, negative error code on failure.
 */
int mixer_apply_output(const motor_output_t *output);

/**
 * @brief Force all motors to safe idle / disarm state (1000 us).
 */
void mixer_disarm(void);

#endif /* MIXER_H_ */
