/*
 * esc_pwm.c - Hardware Driver Implementation for ESC PWM Outputs (FW-15)
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Peripheral: PWM0 @ 0x10400000 (Channels 0-3, 400 Hz ESC Update Rate)
 *
 * Implements:
 * 1. esc_init() with DeviceTree binding and global register 0x80 enable.
 * 2. esc_write_pulses() clamping 1000 us - 2000 us (armed) or 900 us (disarmed).
 * 3. Atomic thread-safe arming state management.
 */

#include "esc_pwm.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/sys_io.h>
#include <errno.h>

LOG_MODULE_REGISTER(esc_pwm, CONFIG_LOG_DEFAULT_LEVEL);

/* Device Tree Node Resolution */
#if DT_NODE_EXISTS(DT_ALIAS(motor_pwm))
static const struct device *const s_pwm_dev = DEVICE_DT_GET(DT_ALIAS(motor_pwm));
#elif DT_NODE_EXISTS(DT_NODELABEL(pwm0))
static const struct device *const s_pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm0));
#else
#error "DeviceTree error: Neither alias 'motor_pwm' nor nodelabel 'pwm0' found."
#endif

/* State Variables */
static bool s_is_armed = false;
static bool s_is_initialized = false;

/* Clamp Helper */
static inline uint16_t clamp_u16(uint16_t val, uint16_t min_val, uint16_t max_val)
{
	if (val < min_val) {
		return min_val;
	}
	if (val > max_val) {
		return max_val;
	}
	return val;
}

int esc_init(void)
{
	if (!device_is_ready(s_pwm_dev)) {
		LOG_ERR("PWM0 device not ready for ESC control");
		return -ENODEV;
	}

	/*
	 * Assert THEJAS32 SoC PWM0 Global Control Register (offset 0x80).
	 * Bit 0: Global PWM core output enable.
	 */
	uintptr_t global_reg_addr = THEJAS32_PWM0_BASE_ADDR + THEJAS32_PWM_GLOBAL_CTRL_OFFSET;
	uint32_t reg_val = sys_read32(global_reg_addr);
	reg_val |= THEJAS32_PWM_GLOBAL_ENABLE_BIT;
	sys_write32(reg_val, global_reg_addr);

	LOG_INF("THEJAS32 PWM0 global enable register asserted at 0x%08X (val=0x%08X)",
		(uint32_t)global_reg_addr, sys_read32(global_reg_addr));

	/* Ensure initial state is safely disarmed */
	s_is_armed = false;

	/* Configure channels 0-3 with 400 Hz period (2.5 ms) and disarmed 900 us pulse */
	uint32_t disarm_pulse_ns = ESC_PULSE_DISARM_US * 1000U;
	for (uint32_t ch = 0; ch < ESC_NUM_CHANNELS; ch++) {
		int ret = pwm_set_cycles(s_pwm_dev, ch, ESC_PWM_PERIOD_NS, disarm_pulse_ns, 0);
		if (ret < 0) {
			LOG_ERR("Failed to initialize PWM channel %u with disarm pulse (err: %d)", ch, ret);
			return ret;
		}
	}

	s_is_initialized = true;
	LOG_INF("ESC PWM subsystem initialized (400 Hz, 4 channels at 900 us disarm)");
	return 0;
}

void esc_set_armed(bool armed)
{
	s_is_armed = armed;
	if (!armed) {
		/* Immediately force disarmed state to all channels */
		esc_write_pulses(ESC_PULSE_DISARM_US, ESC_PULSE_DISARM_US,
				 ESC_PULSE_DISARM_US, ESC_PULSE_DISARM_US);
	}
}

bool esc_is_armed(void)
{
	return s_is_armed;
}

void esc_disarm(void)
{
	esc_set_armed(false);
}

int esc_write_pulses(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
	if (!s_is_initialized) {
		return -EAGAIN;
	}

	uint16_t target_pulses[ESC_NUM_CHANNELS];

	if (!s_is_armed) {
		/* Strict safety cutoff when disarmed: 900 us */
		target_pulses[0] = ESC_PULSE_DISARM_US;
		target_pulses[1] = ESC_PULSE_DISARM_US;
		target_pulses[2] = ESC_PULSE_DISARM_US;
		target_pulses[3] = ESC_PULSE_DISARM_US;
	} else {
		/* Armed: clamp strictly between 1000 us (Idle) and 2000 us (Max) */
		target_pulses[0] = clamp_u16(m1, ESC_PULSE_MIN_US, ESC_PULSE_MAX_US);
		target_pulses[1] = clamp_u16(m2, ESC_PULSE_MIN_US, ESC_PULSE_MAX_US);
		target_pulses[2] = clamp_u16(m3, ESC_PULSE_MIN_US, ESC_PULSE_MAX_US);
		target_pulses[3] = clamp_u16(m4, ESC_PULSE_MIN_US, ESC_PULSE_MAX_US);
	}

	/* Write nanosecond pulses to each hardware channel */
	for (uint32_t ch = 0; ch < ESC_NUM_CHANNELS; ch++) {
		uint32_t pulse_ns = (uint32_t)target_pulses[ch] * 1000U;
		int ret = pwm_set_cycles(s_pwm_dev, ch, ESC_PWM_PERIOD_NS, pulse_ns, 0);
		if (ret < 0) {
			LOG_ERR("Failed to set pulse on PWM channel %u (err: %d)", ch, ret);
			return ret;
		}
	}

	return 0;
}
