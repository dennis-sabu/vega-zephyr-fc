/*
 * ahrs.c - Attitude and Heading Reference System Implementation
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 *
 * Implements discrete Complementary Filter fusing 3-axis accelerometer
 * and 3-axis gyroscope data:
 * theta = alpha * (theta + omega * dt) + (1 - alpha) * theta_acc
 */

#include "ahrs.h"
#include <math.h>

#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
#define PI_F       3.14159265358979323846f
#define TWO_PI_F   6.28318530717958647692f

static euler_angles_t s_euler = { .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f };
static quaternion_t s_q = { .w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f };
static vector3f_t s_rates = { .x = 0.0f, .y = 0.0f, .z = 0.0f };
static float s_alpha = COMPLEMENTARY_ALPHA;

void ahrs_init(float sample_freq, float alpha)
{
	(void)sample_freq;
	s_alpha = (alpha > 0.0f && alpha < 1.0f) ? alpha : COMPLEMENTARY_ALPHA;
	ahrs_reset();
}

void ahrs_reset(void)
{
	s_euler.roll = 0.0f;
	s_euler.pitch = 0.0f;
	s_euler.yaw = 0.0f;

	s_q.w = 1.0f;
	s_q.x = 0.0f;
	s_q.y = 0.0f;
	s_q.z = 0.0f;

	s_rates.x = 0.0f;
	s_rates.y = 0.0f;
	s_rates.z = 0.0f;
}

void ahrs_update(const vector3f_t *gyro, const vector3f_t *accel, float dt)
{
	if (gyro == NULL || accel == NULL || dt <= 0.0f) {
		return;
	}

	s_rates = *gyro;

	/* Accelerometer-derived tilt angles */
	float ax = accel->x;
	float ay = accel->y;
	float az = accel->z;
	float accel_norm_sq = ax * ax + ay * ay + az * az;

	if (accel_norm_sq > 0.1f) {
		/*
		 * Roll from accelerometer: rotation around X axis (Y vs Z)
		 * Pitch from accelerometer: rotation around Y axis (-X vs sqrt(Y^2 + Z^2))
		 */
		float roll_acc = atan2f(ay, az);
		float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));

		/*
		 * Discrete Complementary Filter formula:
		 * theta = alpha * (theta + omega * dt) + (1 - alpha) * theta_acc
		 */
		s_euler.roll = s_alpha * (s_euler.roll + gyro->x * dt) + (1.0f - s_alpha) * roll_acc;
		s_euler.pitch = s_alpha * (s_euler.pitch + gyro->y * dt) + (1.0f - s_alpha) * pitch_acc;
	} else {
		/* Gyro integration only if accelerometer reading is invalid/freefall */
		s_euler.roll += gyro->x * dt;
		s_euler.pitch += gyro->y * dt;
	}

	/* Gyro integration for Yaw (heading) */
	s_euler.yaw += gyro->z * dt;

	/* Wrap Yaw to [-PI, +PI] */
	while (s_euler.yaw > PI_F) {
		s_euler.yaw -= TWO_PI_F;
	}
	while (s_euler.yaw < -PI_F) {
		s_euler.yaw += TWO_PI_F;
	}

	/* Compute Quaternion representation from updated Euler angles */
	float cr = cosf(s_euler.roll * 0.5f);
	float sr = sinf(s_euler.roll * 0.5f);
	float cp = cosf(s_euler.pitch * 0.5f);
	float sp = sinf(s_euler.pitch * 0.5f);
	float cy = cosf(s_euler.yaw * 0.5f);
	float sy = sinf(s_euler.yaw * 0.5f);

	s_q.w = cr * cp * cy + sr * sp * sy;
	s_q.x = sr * cp * cy - cr * sp * sy;
	s_q.y = cr * sp * cy + sr * cp * sy;
	s_q.z = cr * cp * sy - sr * sp * cy;
}

void ahrs_get_quaternion(quaternion_t *q)
{
	if (q != NULL) {
		*q = s_q;
	}
}

void ahrs_get_euler_angles(euler_angles_t *euler)
{
	if (euler != NULL) {
		*euler = s_euler;
	}
}

void ahrs_get_euler_degrees(euler_angles_t *euler)
{
	if (euler == NULL) {
		return;
	}

	euler->roll  = s_euler.roll * RAD_TO_DEG;
	euler->pitch = s_euler.pitch * RAD_TO_DEG;
	euler->yaw   = s_euler.yaw * RAD_TO_DEG;
}

void ahrs_get_rates(vector3f_t *rates)
{
	if (rates != NULL) {
		*rates = s_rates;
	}
}
