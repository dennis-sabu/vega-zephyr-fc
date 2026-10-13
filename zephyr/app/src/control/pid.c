/*
 * pid.c - Discrete PID Controller Implementation
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 *
 * Implements:
 * 1. Proportional term: P[k] = Kp * e[k]
 * 2. Derivative term with 1st-order low-pass filter:
 *    D[k] = beta * D[k-1] + (1 - beta) * (Kd * (e[k] - e[k-1]) / dt)
 * 3. Integral term with anti-windup clamping to prevent saturation.
 */

#include "pid.h"
#include <math.h>

#define TWO_PI 6.28318530717958647692f

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
	      float out_min, float out_max, float int_min, float int_max,
	      float d_cutoff_hz, float dt, bool deriv_on_measurement)
{
	if (pid == NULL) {
		return;
	}

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->out_min = out_min;
	pid->out_max = out_max;
	pid->int_min = int_min;
	pid->int_max = int_max;
	pid->integrator = 0.0f;
	pid->prev_measurement = 0.0f;
	pid->prev_error = 0.0f;
	pid->prev_derivative = 0.0f;
	pid->derivative_on_measurement = deriv_on_measurement;

	pid_set_d_filter(pid, d_cutoff_hz, dt);
}

void pid_reset(pid_controller_t *pid)
{
	if (pid == NULL) {
		return;
	}

	pid->integrator = 0.0f;
	pid->prev_measurement = 0.0f;
	pid->prev_error = 0.0f;
	pid->prev_derivative = 0.0f;
}

void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd)
{
	if (pid == NULL) {
		return;
	}

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

void pid_set_limits(pid_controller_t *pid, float out_min, float out_max,
		    float int_min, float int_max)
{
	if (pid == NULL) {
		return;
	}

	pid->out_min = out_min;
	pid->out_max = out_max;
	pid->int_min = int_min;
	pid->int_max = int_max;

	if (pid->integrator > int_max) {
		pid->integrator = int_max;
	} else if (pid->integrator < int_min) {
		pid->integrator = int_min;
	}
}

void pid_set_d_filter(pid_controller_t *pid, float cutoff_hz, float dt)
{
	if (pid == NULL) {
		return;
	}

	if (cutoff_hz > 0.0f && dt > 0.0f) {
		float tau = 1.0f / (TWO_PI * cutoff_hz);
		/* Discrete low-pass filter parameter: beta = tau / (tau + dt) */
		pid->d_beta = tau / (tau + dt);
	} else {
		pid->d_beta = 0.0f; /* No filtering (1 - beta = 1.0) */
	}
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt)
{
	if (pid == NULL || dt <= 0.0f) {
		return 0.0f;
	}

	float error = setpoint - measurement;

	/* 1. Proportional Term */
	float p_term = pid->kp * error;

	/* 2. Derivative Term with 1st-Order Low-Pass Filter */
	float d_raw = 0.0f;
	if (pid->derivative_on_measurement) {
		/* Derivative-on-measurement prevents setpoint step kick */
		d_raw = -pid->kd * (measurement - pid->prev_measurement) / dt;
	} else {
		/* Standard derivative on error */
		d_raw = pid->kd * (error - pid->prev_error) / dt;
	}

	/* D[k] = beta * D[k-1] + (1 - beta) * D_raw */
	float d_term = pid->d_beta * pid->prev_derivative + (1.0f - pid->d_beta) * d_raw;
	pid->prev_derivative = d_term;
	pid->prev_measurement = measurement;
	pid->prev_error = error;

	/*
	 * 3. Integral Term with Anti-Windup Clamping:
	 * Prevent integration if output is already saturated in error direction.
	 */
	float tentative_out = p_term + pid->integrator + d_term;
	bool saturate_high = (tentative_out >= pid->out_max) && (error > 0.0f);
	bool saturate_low  = (tentative_out <= pid->out_min) && (error < 0.0f);

	if (!saturate_high && !saturate_low) {
		pid->integrator += pid->ki * error * dt;
		if (pid->integrator > pid->int_max) {
			pid->integrator = pid->int_max;
		} else if (pid->integrator < pid->int_min) {
			pid->integrator = pid->int_min;
		}
	}

	/* 4. Combined Clamped Output */
	float output = p_term + pid->integrator + d_term;

	if (output > pid->out_max) {
		output = pid->out_max;
	} else if (output < pid->out_min) {
		output = pid->out_min;
	}

	return output;
}
