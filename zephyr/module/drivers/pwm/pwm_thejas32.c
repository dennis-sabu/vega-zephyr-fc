/*
 * Out-of-tree Zephyr PWM driver for the C-DAC THEJAS32 SoC.
 * NOT part of upstream Zephyr -- see zephyr/module/CMakeLists.txt.
 *
 * Register layout verified identically between vega-bsp's
 * cores/arduino/pwm.h and the native VEGA SDK's bsp/include/pwmc.h (via
 * cdac-vega.gitlab.io/vega-sdk) -- see the devicetree binding's header
 * comment for the exact bitfield cross-check.
 *
 *   PWMCreg(n) = PWMC_BASE_ADDR + n*0x10, n = 0..7
 *     +0x00  Control  16 used bits: Mode[1:0] AC[3:2] IE[4] OPC[5] RepeatCount[21:6]
 *     +0x04  Status
 *     +0x08  Period
 *     +0x0C  On_Off (duty)
 *   Global control register: PWMC_BASE_ADDR + 0x80, bit0 = global enable
 *   for ALL 8 channels together -- there is no per-channel enable bit in
 *   either source this project could read. This driver enables the
 *   global bit once, the first time any channel is configured, and never
 *   disables it again; Zephyr's pwm_set_cycles() has no "disable this
 *   channel but leave others running" semantics to map onto a
 *   coarser-grained hardware enable anyway.
 *
 * UNVERIFIED, stated plainly: neither source documents what clock feeds
 * the Period/On_Off counters, or whether it is divided from the CPU
 * clock. This driver assumes the counters run at the CPU clock
 * (devicetree cpu0 clock-frequency, 100 MHz on ARIES v3.0) because the
 * vendor's own RYG_LED_PWM.ino example (period=800000 counts for a
 * human-visible LED breathing effect) is only physically plausible at
 * that assumption -- at 100 MHz, 800000 counts is an 8 ms (125 Hz)
 * period, which is a sensible LED PWM rate; at, say, 1 MHz it would be
 * 800 ms, clearly wrong for the effect the example describes. This is an
 * inference from a physically-plausible example, not a confirmed
 * datasheet value. get_cycles_per_sec() reports this assumption
 * explicitly so a caller can tell what this driver believes, rather than
 * silently guessing inside pwm_set_cycles() alone.
 */

#define DT_DRV_COMPAT cdac_thejas32_pwm

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pwm_thejas32, CONFIG_PWM_LOG_LEVEL);

#define THEJAS32_PWM_CHANNEL_COUNT 8U
#define THEJAS32_PWM_CHANNEL_STRIDE 0x10UL
#define THEJAS32_PWM_GCR_OFFSET 0x80UL

#define THEJAS32_PWM_CTRL_OFFSET   0x00
#define THEJAS32_PWM_STATUS_OFFSET 0x04
#define THEJAS32_PWM_PERIOD_OFFSET 0x08
#define THEJAS32_PWM_ONOFF_OFFSET  0x0C

#define THEJAS32_PWM_CTRL_MODE(x)         ((x) & 0x3)
#define THEJAS32_PWM_CTRL_AC(x)           (((x) & 0x3) << 2)
#define THEJAS32_PWM_CTRL_IE(x)           (((x) & 0x1) << 4)
#define THEJAS32_PWM_CTRL_OPC(x)          (((x) & 0x1) << 5)
#define THEJAS32_PWM_CTRL_REPEATCOUNT(x)  (((x) & 0xFFFF) << 6)

#define THEJAS32_PWM_MODE_CONTINUOUS 2U
#define THEJAS32_PWM_ALIGN_LEFT      0U
#define THEJAS32_PWM_INTR_DISABLE    0U
#define THEJAS32_PWM_OPC_NORMAL      0U
#define THEJAS32_PWM_OPC_INVERTED    1U
#define THEJAS32_PWM_DEFAULT_REPEAT_COUNT 5U

#define THEJAS32_PWM_GCR_GPE BIT(0)
#define THEJAS32_PWM_GCR_GIE BIT(1)

struct pwm_thejas32_config {
	mem_addr_t base;
};

struct pwm_thejas32_data {
	bool global_enabled;
	bool channel_initialized[THEJAS32_PWM_CHANNEL_COUNT];
};

static inline mem_addr_t channel_base(mem_addr_t base, uint32_t channel)
{
	return base + channel * THEJAS32_PWM_CHANNEL_STRIDE;
}

static void thejas32_pwm_channel_init(mem_addr_t base, uint32_t channel, uint32_t opc)
{
	const uint32_t ctrl = THEJAS32_PWM_CTRL_MODE(THEJAS32_PWM_MODE_CONTINUOUS) |
			      THEJAS32_PWM_CTRL_AC(THEJAS32_PWM_ALIGN_LEFT) |
			      THEJAS32_PWM_CTRL_IE(THEJAS32_PWM_INTR_DISABLE) |
			      THEJAS32_PWM_CTRL_OPC(opc) |
			      THEJAS32_PWM_CTRL_REPEATCOUNT(THEJAS32_PWM_DEFAULT_REPEAT_COUNT);

	sys_write32(ctrl, channel_base(base, channel) + THEJAS32_PWM_CTRL_OFFSET);
	barrier_dmem_fence_full();
}

static int pwm_thejas32_set_cycles(const struct device *dev, uint32_t channel,
				    uint32_t period_cycles, uint32_t pulse_cycles,
				    pwm_flags_t flags)
{
	const struct pwm_thejas32_config *config = dev->config;
	struct pwm_thejas32_data *data = dev->data;

	if (channel >= THEJAS32_PWM_CHANNEL_COUNT) {
		return -EINVAL;
	}
	if (pulse_cycles > period_cycles) {
		return -EINVAL;
	}

	const uint32_t opc = (flags & PWM_POLARITY_INVERTED) ? THEJAS32_PWM_OPC_INVERTED
							       : THEJAS32_PWM_OPC_NORMAL;

	if (!data->channel_initialized[channel]) {
		thejas32_pwm_channel_init(config->base, channel, opc);
		data->channel_initialized[channel] = true;
	}

	sys_write32(period_cycles, channel_base(config->base, channel) + THEJAS32_PWM_PERIOD_OFFSET);
	sys_write32(pulse_cycles, channel_base(config->base, channel) + THEJAS32_PWM_ONOFF_OFFSET);
	barrier_dmem_fence_full();

	if (!data->global_enabled) {
		sys_write32(THEJAS32_PWM_GCR_GPE, config->base + THEJAS32_PWM_GCR_OFFSET);
		barrier_dmem_fence_full();
		data->global_enabled = true;
		LOG_WRN("global PWM enable bit set (hardware has no per-channel enable; "
			 "see driver header comment)");
	}

	return 0;
}

static int pwm_thejas32_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					    uint64_t *cycles)
{
	ARG_UNUSED(dev);

	if (channel >= THEJAS32_PWM_CHANNEL_COUNT) {
		return -EINVAL;
	}

	/* See the UNVERIFIED note in the file header. */
	*cycles = DT_PROP(DT_NODELABEL(cpu0), clock_frequency);
	return 0;
}

static DEVICE_API(pwm, pwm_thejas32_driver_api) = {
	.set_cycles = pwm_thejas32_set_cycles,
	.get_cycles_per_sec = pwm_thejas32_get_cycles_per_sec,
};

static int pwm_thejas32_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

#define THEJAS32_PWM_INIT(n)                                                                      \
	static const struct pwm_thejas32_config pwm_thejas32_config_##n = {                       \
		.base = DT_INST_REG_ADDR(n),                                                       \
	};                                                                                         \
	static struct pwm_thejas32_data pwm_thejas32_data_##n;                                    \
	DEVICE_DT_INST_DEFINE(n, pwm_thejas32_init, NULL, &pwm_thejas32_data_##n,                 \
			      &pwm_thejas32_config_##n, POST_KERNEL,                              \
			      CONFIG_PWM_INIT_PRIORITY, &pwm_thejas32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(THEJAS32_PWM_INIT)
