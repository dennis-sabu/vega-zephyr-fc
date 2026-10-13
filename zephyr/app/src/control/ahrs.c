/*
 * ahrs.c - Madgwick IMU Attitude Estimation Algorithm Implementation
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 *
 * Implements gradient descent orientation filtering fusing 3-axis accelerometer
 * and 3-axis gyroscope into a normalized unit quaternion and Euler angles.
 */

#include "ahrs.h"
#include <math.h>

#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)

static quaternion_t s_q = { .w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f };
static float s_beta = MADGWICK_BETA;

void ahrs_init(float sample_freq, float beta)
{
	(void)sample_freq;
	s_beta = (beta > 0.0f) ? beta : MADGWICK_BETA;
	ahrs_reset();
}

void ahrs_reset(void)
{
	s_q.w = 1.0f;
	s_q.x = 0.0f;
	s_q.y = 0.0f;
	s_q.z = 0.0f;
}

void ahrs_update(const vector3f_t *gyro, const vector3f_t *accel, float dt)
{
	if (gyro == NULL || accel == NULL || dt <= 0.0f) {
		return;
	}

	float q0 = s_q.w;
	float q1 = s_q.x;
	float q2 = s_q.y;
	float q3 = s_q.z;

	/* Rate of change of quaternion from gyroscope */
	float gx = gyro->x;
	float gy = gyro->y;
	float gz = gyro->z;

	float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
	float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
	float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

	/* Compute feedback only if accelerometer measurement is valid */
	float ax = accel->x;
	float ay = accel->y;
	float az = accel->z;
	float accel_norm_sq = ax * ax + ay * ay + az * az;

	if (accel_norm_sq > 0.01f) {
		float recip_norm = 1.0f / sqrtf(accel_norm_sq);
		ax *= recip_norm;
		ay *= recip_norm;
		az *= recip_norm;

		/* Auxiliary variables to avoid repeated arithmetic */
		float _2q0 = 2.0f * q0;
		float _2q1 = 2.0f * q1;
		float _2q2 = 2.0f * q2;
		float _2q3 = 2.0f * q3;
		float _4q0 = 4.0f * q0;
		float _4q1 = 4.0f * q1;
		float _4q2 = 4.0f * q2;
		float _8q1 = 8.0f * q1;
		float _8q2 = 8.0f * q2;
		float q0q0 = q0 * q0;
		float q1q1 = q1 * q1;
		float q2q2 = q2 * q2;
		float q3q3 = q3 * q3;

		/* Gradient descent objective function and Jacobian */
		float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

		/* Normalize step gradient */
		float s_norm_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
		if (s_norm_sq > 0.000001f) {
			float recip_s = 1.0f / sqrtf(s_norm_sq);
			s0 *= recip_s;
			s1 *= recip_s;
			s2 *= recip_s;
			s3 *= recip_s;

			/* Apply feedback step */
			qDot0 -= s_beta * s0;
			qDot1 -= s_beta * s1;
			qDot2 -= s_beta * s2;
			qDot3 -= s_beta * s3;
		}
	}

	/* Integrate rate of change of quaternion */
	q0 += qDot0 * dt;
	q1 += qDot1 * dt;
	q2 += qDot2 * dt;
	q3 += qDot3 * dt;

	/* Normalize quaternion */
	float q_norm_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
	if (q_norm_sq > 0.000001f) {
		float recip_q = 1.0f / sqrtf(q_norm_sq);
		s_q.w = q0 * recip_q;
		s_q.x = q1 * recip_q;
		s_q.y = q2 * recip_q;
		s_q.z = q3 * recip_q;
	}
}

void ahrs_get_quaternion(quaternion_t *q)
{
	if (q != NULL) {
		*q = s_q;
	}
}

void ahrs_get_euler_angles(euler_angles_t *euler)
{
	if (euler == NULL) {
		return;
	}

	float q0 = s_q.w;
	float q1 = s_q.x;
	float q2 = s_q.y;
	float q3 = s_q.z;

	/* Roll (x-axis rotation): atan2(2(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2)) */
	euler->roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

	/* Pitch (y-axis rotation): asin(2(q0*q2 - q3*q1)) */
	float sinp = 2.0f * (q0 * q2 - q3 * q1);
	if (fabsf(sinp) >= 1.0f) {
		euler->pitch = copysignf(1.5707963267948966f, sinp); /* +/- pi/2 */
	} else {
		euler->pitch = asinf(sinp);
	}

	/* Yaw (z-axis rotation): atan2(2(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2)) */
	euler->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

void ahrs_get_euler_degrees(euler_angles_t *euler)
{
	if (euler == NULL) {
		return;
	}

	euler_angles_t rad;
	ahrs_get_euler_angles(&rad);

	euler->roll  = rad.roll * RAD_TO_DEG;
	euler->pitch = rad.pitch * RAD_TO_DEG;
	euler->yaw   = rad.yaw * RAD_TO_DEG;
}
