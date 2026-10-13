/*
 * Out-of-tree Zephyr ADC driver for the C-DAC THEJAS32 SoC.
 * NOT part of upstream Zephyr -- see zephyr/module/CMakeLists.txt.
 *
 * Verified from a single source only: the native VEGA SDK's
 * bsp/include/adc.h and bsp/drivers/adc/adc.c, read via
 * cdac-vega.gitlab.io/vega-sdk's Doxygen pages. Unlike the I2C/SPI/PWM
 * drivers in this module, the Arduino core (vega-bsp) has no usable ADC
 * source to cross-check against -- see dts/bindings/adc/
 * cdac,thejas32-adc.yaml and config/firmware_config.h in this
 * repository's earlier Arduino-based work for that gap.
 *
 *   ADC_BASE_ADDRESS = 0x10001000
 *     +0x00  address_reg  write 0x14+channel (channel 0..3) to start a conversion
 *     +0x04  status_reg   bit0 = conversion complete
 *     +0x08  data_reg     conversion result
 *
 * Resolution and reference voltage are NOT documented in the source this
 * project could read. This driver does not invent them: it masks the raw
 * data_reg value to whatever bit width the caller's adc_sequence asks
 * for, which only produces a meaningful value if that width happens to
 * match the real hardware resolution -- a fact this driver cannot
 * confirm. Treat any voltage you derive from a sample as unverified
 * until you have measured a known reference voltage on real hardware
 * and compared it to the raw code yourself.
 */

#define DT_DRV_COMPAT cdac_thejas32_adc

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_thejas32, CONFIG_ADC_LOG_LEVEL);

#define THEJAS32_ADC_CHANNEL_COUNT 4U
#define THEJAS32_ADC_CHANNEL_CODE_BASE 0x14U

#define THEJAS32_ADC_ADDRESS_OFFSET 0x00
#define THEJAS32_ADC_STATUS_OFFSET  0x04
#define THEJAS32_ADC_DATA_OFFSET    0x08

#define THEJAS32_ADC_STATUS_READY BIT(0)

struct adc_thejas32_config {
	mem_addr_t base;
};

struct adc_thejas32_data {
	struct k_mutex lock;
};

static int adc_thejas32_channel_setup(const struct device *dev,
				       const struct adc_channel_cfg *channel_cfg)
{
	ARG_UNUSED(dev);

	if (channel_cfg->channel_id >= THEJAS32_ADC_CHANNEL_COUNT) {
		LOG_ERR("channel %u does not exist (hardware has 4 channels, 0-3)",
			channel_cfg->channel_id);
		return -EINVAL;
	}
	if (channel_cfg->differential) {
		return -ENOTSUP;
	}
	/* gain/reference/acquisition_time are accepted without validation:
	 * this driver has no documented way to configure any of them in
	 * hardware (see file header), so every value is equally
	 * "unverified" rather than some being rejected arbitrarily.
	 */
	return 0;
}

static int read_one_channel(mem_addr_t base, uint8_t channel, uint32_t *out_raw)
{
	const uint32_t max_cycles = k_us_to_cyc_ceil32(CONFIG_ADC_THEJAS32_TIMEOUT_US);
	const uint32_t start = k_cycle_get_32();

	sys_write32(THEJAS32_ADC_CHANNEL_CODE_BASE + channel, base + THEJAS32_ADC_ADDRESS_OFFSET);
	barrier_dmem_fence_full();

	while ((sys_read32(base + THEJAS32_ADC_STATUS_OFFSET) & THEJAS32_ADC_STATUS_READY) == 0U) {
		if ((k_cycle_get_32() - start) > max_cycles) {
			return -ETIMEDOUT;
		}
	}

	*out_raw = sys_read32(base + THEJAS32_ADC_DATA_OFFSET);
	return 0;
}

static int adc_thejas32_read(const struct device *dev, const struct adc_sequence *sequence)
{
	const struct adc_thejas32_config *config = dev->config;
	struct adc_thejas32_data *data = dev->data;
	uint8_t *buf = sequence->buffer;
	size_t remaining = sequence->buffer_size;
	int rc = 0;

	if (sequence->resolution == 0U || sequence->resolution > 32U) {
		return -EINVAL;
	}
	const uint32_t mask = (sequence->resolution == 32U)
				       ? 0xFFFFFFFFU
				       : ((1UL << sequence->resolution) - 1UL);

	k_mutex_lock(&data->lock, K_FOREVER);

	for (uint8_t channel = 0; channel < THEJAS32_ADC_CHANNEL_COUNT; channel++) {
		if ((sequence->channels & BIT(channel)) == 0U) {
			continue;
		}
		if (remaining < sizeof(uint16_t)) {
			rc = -ENOMEM;
			break;
		}

		uint32_t raw;

		rc = read_one_channel(config->base, channel, &raw);
		if (rc != 0) {
			break;
		}

		const uint16_t sample = (uint16_t)(raw & mask);

		memcpy(buf, &sample, sizeof(sample));
		buf += sizeof(sample);
		remaining -= sizeof(sample);
	}

	k_mutex_unlock(&data->lock);

	return rc;
}

static DEVICE_API(adc, adc_thejas32_driver_api) = {
	.channel_setup = adc_thejas32_channel_setup,
	.read = adc_thejas32_read,
};

static int adc_thejas32_init(const struct device *dev)
{
	struct adc_thejas32_data *data = dev->data;

	k_mutex_init(&data->lock);
	return 0;
}

#define THEJAS32_ADC_INIT(n)                                                                      \
	static const struct adc_thejas32_config adc_thejas32_config_##n = {                       \
		.base = DT_INST_REG_ADDR(n),                                                       \
	};                                                                                         \
	static struct adc_thejas32_data adc_thejas32_data_##n;                                    \
	DEVICE_DT_INST_DEFINE(n, adc_thejas32_init, NULL, &adc_thejas32_data_##n,                 \
			      &adc_thejas32_config_##n, POST_KERNEL,                              \
			      CONFIG_ADC_INIT_PRIORITY, &adc_thejas32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(THEJAS32_ADC_INIT)
