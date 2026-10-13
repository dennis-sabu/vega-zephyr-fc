/*
 * pid.c - Production PID Controller Implementation
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 *
 * Implements anti-windup integration clamping, derivative-on-measurement
 * kick prevention, and first-order low-pass filtering on derivative action.
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

	/* Enforce integrator bounds immediately */
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
		float rc = 1.0f / (TWO_PI * cutoff_hz);
		pid->d_lpf_alpha = dt / (rc + dt);
	} else {
		pid->d_lpf_alpha = 1.0f; /* No filtering */
	}
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt)
{
	if (pid == NULL || dt <= 0.0f) {
		return 0.0f;
	}

	float error = setpoint - measurement;

	/* Proportional term */
	float p_term = pid->kp * error;

	/* Derivative term computation */
	float d_raw = 0.0f;
	if (pid->derivative_on_measurement) {
		/*
		 * Derivative-on-measurement prevents "derivative kick" when setpoint
		 * steps abruptly: d(error)/dt = d(setpoint - measurement)/dt = -d(measurement)/dt
		 */
		d_raw = -(measurement - pid->prev_measurement) / dt;
	} else {
		d_raw = (error - (setpoint - pid->prev_measurement)) / dt;
	}

	/* 1st-order Low-Pass Filter on Derivative: prevents motor vibration amplification */
	float d_filtered = pid->prev_derivative + pid->d_lpf_alpha * (d_raw - pid->prev_derivative);
	pid->prev_derivative = d_filtered;
	pid->prev_measurement = measurement;

	float d_term = pid->kd * d_filtered;

	/*
	 * Conditional Integration / Anti-windup:
	 * Do not accumulate integral if the output is already saturated in the direction of error.
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

	/* Compute combined output */
	float output = p_term + pid->integrator + d_term;

	/* Saturate output to specified boundaries */
	if (output > pid->out_max) {
		output = pid->out_max;
	} else if (output < pid->out_min) {
		output = pid->out_min;
	}

	return output;
}
