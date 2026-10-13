/*
 * pid.h - Discrete PID Controller with Low-Pass Filtered Derivative and Anti-Windup
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Zephyr RTOS Application
 */

#ifndef PID_H_
#define PID_H_

#include "types.h"
#include <stdbool.h>

/**
 * @brief Initialize a PID controller instance with full gains and limits.
 *
 * @param pid Pointer to PID controller structure.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 * @param out_min Minimum total output limit.
 * @param out_max Maximum total output limit.
 * @param int_min Minimum integrator limit (anti-windup).
 * @param int_max Maximum integrator limit (anti-windup).
 * @param d_cutoff_hz Low pass cutoff frequency for derivative filter in Hz.
 * @param dt Nominal sample time step in seconds.
 * @param deriv_on_measurement If true, compute derivative on measurement instead of error.
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
	      float out_min, float out_max, float int_min, float int_max,
	      float d_cutoff_hz, float dt, bool deriv_on_measurement);

/**
 * @brief Reset integrator and derivative state variables to zero.
 *
 * @param pid Pointer to PID controller structure.
 */
void pid_reset(pid_controller_t *pid);

/**
 * @brief Update gains for an active PID controller.
 *
 * @param pid Pointer to PID controller structure.
 * @param kp New proportional gain.
 * @param ki New integral gain.
 * @param kd New derivative gain.
 */
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

/**
 * @brief Update output and anti-windup saturation limits.
 *
 * @param pid Pointer to PID controller structure.
 * @param out_min Minimum output.
 * @param out_max Maximum output.
 * @param int_min Minimum integrator value.
 * @param int_max Maximum integrator value.
 */
void pid_set_limits(pid_controller_t *pid, float out_min, float out_max,
		    float int_min, float int_max);

/**
 * @brief Configure the derivative 1st-order low-pass filter beta coefficient.
 * beta = tau / (tau + dt), where tau = 1 / (2 * pi * cutoff_hz)
 *
 * @param pid Pointer to PID controller structure.
 * @param cutoff_hz Cutoff frequency in Hz.
 * @param dt Time step in seconds.
 */
void pid_set_d_filter(pid_controller_t *pid, float cutoff_hz, float dt);

/**
 * @brief Compute PID control output for the current time step.
 *
 * @param pid Pointer to PID controller structure.
 * @param setpoint Desired target value.
 * @param measurement Current measured value.
 * @param dt Actual elapsed time step in seconds.
 * @return float Clamped control effort output.
 */
float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt);

#endif /* PID_H_ */
