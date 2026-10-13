/*
 * mixer.c - Multirotor Flight Mixer and Actuator Driver Implementation
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Airframe: S500 Quadcopter Quad-X Configuration
 *
 * Peripherals:
 * - PWM0 @ 0x10400000 (8 Channels, Global Enable Register @ 0x80)
 * - 400 Hz ESC Pulse Repetition Frequency (Period: 2,500,000 ns)
 */

#include "mixer.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/sys_io.h>
#include <errno.h>

LOG_MODULE_REGISTER(mixer, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_EXISTS(DT_ALIAS(motor_pwm))
static const struct device *const pwm_dev = DEVICE_DT_GET(DT_ALIAS(motor_pwm));
#elif DT_NODE_EXISTS(DT_NODELABEL(pwm0))
static const struct device *const pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm0));
#else
#error "DeviceTree error: Neither alias 'motor_pwm' nor nodelabel 'pwm0' found."
#endif

int mixer_init(void)
{
	if (!device_is_ready(pwm_dev)) {
		LOG_ERR("PWM0 peripheral not ready on THEJAS32!");
		return -ENODEV;
	}

	LOG_INF("Initializing PWM0 @ 0x%08X (400 Hz, channels 0-3)...", THEJAS32_PWM0_BASE_ADDR);

	/* 1. Assert Global Enable Register on THEJAS32 PWM Controller (Offset 0x80) */
	sys_write32(THEJAS32_PWM_ENABLE_BIT,
		    THEJAS32_PWM0_BASE_ADDR + THEJAS32_PWM_GLOBAL_ENABLE_REG);

	/* Verify register write */
	uint32_t reg_val = sys_read32(THEJAS32_PWM0_BASE_ADDR + THEJAS32_PWM_GLOBAL_ENABLE_REG);
	if ((reg_val & THEJAS32_PWM_ENABLE_BIT) == 0) {
		LOG_WRN("PWM Global Enable readback did not confirm bit 0: 0x%08X", reg_val);
	}

	/* 2. Set all 4 motor ESC channels to disarmed state (900 us) */
	mixer_disarm();

	LOG_INF("Mixer initialized: Channels 0-3 set to 900 us disarmed pulse.");
	return 0;
}

void mixer_compute(float throttle, float roll_cmd, float pitch_cmd, float yaw_cmd,
		   bool is_armed, motor_output_t *output)
{
	if (output == NULL) {
		return;
	}

	/* If disarmed, force all outputs to 900 us */
	if (!is_armed) {
		for (int i = 0; i < 4; i++) {
			output->norm[i] = 0.0f;
			output->pulse_us[i] = PWM_PULSE_DISARMED_US;
		}
		return;
	}

	/*
	 * Quad-X Mixer Equations:
	 * M1 = Throttle - Pitch - Roll - Yaw   (Motor 1, Front-Right, CCW)
	 * M2 = Throttle + Pitch - Roll + Yaw   (Motor 2, Rear-Right, CW)
	 * M3 = Throttle + Pitch + Roll - Yaw   (Motor 3, Rear-Left, CCW)
	 * M4 = Throttle - Pitch + Roll + Yaw   (Motor 4, Front-Left, CW)
	 */
	float m1 = throttle - pitch_cmd - roll_cmd - yaw_cmd;
	float m2 = throttle + pitch_cmd - roll_cmd + yaw_cmd;
	float m3 = throttle + pitch_cmd + roll_cmd - yaw_cmd;
	float m4 = throttle - pitch_cmd + roll_cmd + yaw_cmd;

	/* Dynamic Desaturation: Preserve roll/pitch/yaw control authority when motors saturate */
	float max_m = m1;
	if (m2 > max_m) max_m = m2;
	if (m3 > max_m) max_m = m3;
	if (m4 > max_m) max_m = m4;

	if (max_m > 1.0f) {
		float excess = max_m - 1.0f;
		m1 -= excess;
		m2 -= excess;
		m3 -= excess;
		m4 -= excess;
	}

	float min_m = m1;
	if (m2 < min_m) min_m = m2;
	if (m3 < min_m) min_m = m3;
	if (m4 < min_m) min_m = m4;

	if (min_m < 0.0f) {
		float deficit = -min_m;
		m1 += deficit;
		m2 += deficit;
		m3 += deficit;
		m4 += deficit;
	}

	/* Store and constrain outputs strictly between 1000 us (idle) and 2000 us (max) */
	float motors[4] = {m1, m2, m3, m4};
	for (int i = 0; i < 4; i++) {
		if (motors[i] > 1.0f) {
			motors[i] = 1.0f;
		} else if (motors[i] < 0.0f) {
			motors[i] = 0.0f;
		}

		output->norm[i] = motors[i];

		uint32_t pulse = PWM_PULSE_MIN_US +
			(uint32_t)(motors[i] * (float)(PWM_PULSE_MAX_US - PWM_PULSE_MIN_US));

		if (pulse > PWM_PULSE_MAX_US) {
			pulse = PWM_PULSE_MAX_US;
		} else if (pulse < PWM_PULSE_MIN_US) {
			pulse = PWM_PULSE_MIN_US;
		}

		output->pulse_us[i] = pulse;
	}
}

int mixer_apply_output(const motor_output_t *output)
{
	if (output == NULL) {
		return -EINVAL;
	}

	int ret = 0;
	for (int ch = 0; ch < 4; ch++) {
		uint32_t pulse_ns = output->pulse_us[ch] * 1000U;
		int err = pwm_set(pwm_dev, ch, PWM_PERIOD_NS, pulse_ns, 0);
		if (err < 0) {
			LOG_ERR("Failed to update PWM Ch %d to %u ns (err %d)", ch, pulse_ns, err);
			ret = err;
		}
	}

	return ret;
}

void mixer_disarm(void)
{
	uint32_t disarm_pulse_ns = PWM_PULSE_DISARMED_US * 1000U;
	for (int ch = 0; ch < 4; ch++) {
		pwm_set(pwm_dev, ch, PWM_PERIOD_NS, disarm_pulse_ns, 0);
	}
}
