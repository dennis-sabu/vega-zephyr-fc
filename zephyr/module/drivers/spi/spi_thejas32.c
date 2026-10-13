/*
 * Out-of-tree Zephyr SPI driver for the C-DAC THEJAS32 SoC.
 * NOT part of upstream Zephyr -- see zephyr/module/CMakeLists.txt.
 *
 * Register layout and protocol cross-checked between vega-bsp's
 * libraries/SPI/src/SPI.h and the native VEGA SDK's bsp/include/spi.h +
 * bsp/drivers/spi/spi.c (via cdac-vega.gitlab.io/vega-sdk); both agree
 * exactly:
 *
 *   offset  field      role
 *   0x00    Control    16-bit: PeriphCS[1:0], Periph[2], DataOrder[3],
 *                       Mode[5:4], SPIrxTxIntr[7:6], CSAAT[8], Dbits[12:9]
 *   0x04    Status     bit2=RX_INT_STATUS bit3=TX_INT_STATUS bit4=BUSY
 *                       bit5=OVERRUN bit6=RX_COMPLETE bit7=TX_HOLD_EMPTY
 *   0x08    Baudrate   divider code << 4
 *   0x0C    TxData     write triggers the shift; bits16/17 also carry
 *                       the chip-select override for "variable peripheral"
 *                       mode (not used here -- see CSAAT discussion below)
 *   0x10    RxData     shifted-in byte, valid once RX_COMPLETE is set
 *
 * This driver implements only the 8-bit, "fixed peripheral" mode the
 * native SDK itself defaults to in SPI_init(): the chip-select index
 * (spi_config->slave) is latched into Control.PeriphCS once per
 * transceive() call and the Control.CSAAT bit is held set for the
 * duration of the whole spi_buf_set so the hardware keeps the selected
 * peripheral addressed across every byte of one logical transfer, then
 * cleared again at the end. Neither source document describes the exact
 * electrical behavior CSAAT controls beyond "Chip Select Active After
 * Transfer" -- this is this project's best-effort, documented
 * interpretation, not a claim verified against a logic analyzer.
 *
 * All register-status spin-waits are bounded by
 * CONFIG_SPI_THEJAS32_TIMEOUT_US; the native driver's own spi.c uses
 * unbounded while() loops for the same waits.
 */

#define DT_DRV_COMPAT cdac_thejas32_spi

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_thejas32, CONFIG_SPI_LOG_LEVEL);

#define THEJAS32_SPI_CONTROL  0x00
#define THEJAS32_SPI_STATUS   0x04
#define THEJAS32_SPI_BAUDRATE 0x08
#define THEJAS32_SPI_TXDATA   0x0C
#define THEJAS32_SPI_RXDATA   0x10

#define THEJAS32_SPI_STATUS_RXINT  BIT(2)
#define THEJAS32_SPI_STATUS_TXINT  BIT(3)
#define THEJAS32_SPI_STATUS_BUSY   BIT(4)
#define THEJAS32_SPI_STATUS_OVERR  BIT(5)
#define THEJAS32_SPI_STATUS_RXC    BIT(6)
#define THEJAS32_SPI_STATUS_TXE    BIT(7)

#define THEJAS32_SPI_CTRL_PCS(x)    ((x) & 0x3)
#define THEJAS32_SPI_CTRL_PERIPH(x) (((x) & 0x1) << 2)
#define THEJAS32_SPI_CTRL_LSB(x)    (((x) & 0x1) << 3)
#define THEJAS32_SPI_CTRL_MODE(x)   (((x) & 0x3) << 4)
#define THEJAS32_SPI_CTRL_CSAAT(x)  (((x) & 0x1) << 8)
#define THEJAS32_SPI_CTRL_DBITS(x)  (((x) & 0xF) << 9)

#define THEJAS32_SPI_FIXED_PERIPHERAL 0
#define THEJAS32_SPI_DBITS_8 0

struct spi_thejas32_config {
	mem_addr_t base;
};

struct spi_thejas32_data {
	struct k_mutex lock;
};

static inline uint16_t ctrl_read(mem_addr_t base)
{
	return sys_read16(base + THEJAS32_SPI_CONTROL);
}

static inline void ctrl_write(mem_addr_t base, uint16_t value)
{
	sys_write16(value, base + THEJAS32_SPI_CONTROL);
	barrier_dmem_fence_full();
}

static int wait_for_status(mem_addr_t base, uint8_t mask, uint8_t expect)
{
	const uint32_t max_cycles = k_us_to_cyc_ceil32(CONFIG_SPI_THEJAS32_TIMEOUT_US);
	const uint32_t start = k_cycle_get_32();

	while ((sys_read8(base + THEJAS32_SPI_STATUS) & mask) != expect) {
		if ((k_cycle_get_32() - start) > max_cycles) {
			return -ETIMEDOUT;
		}
	}
	return 0;
}

/* SPI_BAUD_CFD_n names its divisor directly (4, 8, 16, ... 2048), per
 * vega-bsp's libraries/SPI/src/SPI.h enumeration; the exact
 * clock-divider-to-frequency formula is not documented by either source,
 * so this picks the smallest named divisor that does not exceed the
 * requested frequency, erring toward a SLOWER bus than requested rather
 * than a faster, unverified one.
 */
static uint8_t baud_code_for_frequency(uint32_t sys_clock_hz, uint32_t freq_hz)
{
	static const uint16_t divisors[] = {4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048};

	if (freq_hz == 0U) {
		return ARRAY_SIZE(divisors) - 1U;
	}
	for (uint8_t code = 0; code < ARRAY_SIZE(divisors); code++) {
		if ((sys_clock_hz / divisors[code]) <= freq_hz) {
			return code;
		}
	}
	return ARRAY_SIZE(divisors) - 1U;
}

static int spi_thejas32_configure(const struct device *dev, const struct spi_config *spi_cfg)
{
	const struct spi_thejas32_config *config = dev->config;

	if (spi_cfg->slave > 3U) {
		return -ENOTSUP;
	}
	if (SPI_WORD_SIZE_GET(spi_cfg->operation) != 8U) {
		LOG_ERR("only 8-bit words are supported");
		return -ENOTSUP;
	}
	if ((spi_cfg->operation & SPI_OP_MODE_SLAVE) != 0U) {
		return -ENOTSUP;
	}

	/* Mode[1:0] packs CPHA then CPOL the way vega-bsp's SPI_MODEn
	 * constants do (MODE0=0b00 .. MODE3=0b11).
	 */
	uint8_t mode = 0;

	if ((spi_cfg->operation & SPI_MODE_CPHA) != 0U) {
		mode |= BIT(0);
	}
	if ((spi_cfg->operation & SPI_MODE_CPOL) != 0U) {
		mode |= BIT(1);
	}

	const uint8_t lsb_first = (spi_cfg->operation & SPI_TRANSFER_LSB) ? 1U : 0U;
	const uint32_t sys_clock_hz = DT_PROP(DT_NODELABEL(cpu0), clock_frequency);
	const uint8_t baud_code = baud_code_for_frequency(sys_clock_hz, spi_cfg->frequency);

	sys_write8((uint8_t)(baud_code << 4), config->base + THEJAS32_SPI_BAUDRATE);
	barrier_dmem_fence_full();

	const uint16_t ctrl = THEJAS32_SPI_CTRL_PCS(spi_cfg->slave) |
			      THEJAS32_SPI_CTRL_PERIPH(THEJAS32_SPI_FIXED_PERIPHERAL) |
			      THEJAS32_SPI_CTRL_LSB(lsb_first) | THEJAS32_SPI_CTRL_MODE(mode) |
			      THEJAS32_SPI_CTRL_DBITS(THEJAS32_SPI_DBITS_8);

	ctrl_write(config->base, ctrl);

	return 0;
}

static int spi_thejas32_transfer_byte(mem_addr_t base, uint8_t tx, uint8_t *rx)
{
	int rc = wait_for_status(base, THEJAS32_SPI_STATUS_BUSY, 0);

	if (rc != 0) {
		return rc;
	}
	rc = wait_for_status(base, THEJAS32_SPI_STATUS_TXE, THEJAS32_SPI_STATUS_TXE);
	if (rc != 0) {
		return rc;
	}

	sys_write8(tx, base + THEJAS32_SPI_TXDATA);
	barrier_dmem_fence_full();

	rc = wait_for_status(base, THEJAS32_SPI_STATUS_RXC, THEJAS32_SPI_STATUS_RXC);
	if (rc != 0) {
		return rc;
	}
	if (rx != NULL) {
		*rx = sys_read8(base + THEJAS32_SPI_RXDATA);
	}
	return 0;
}

static int spi_thejas32_transceive(const struct device *dev, const struct spi_config *spi_cfg,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs)
{
	const struct spi_thejas32_config *config = dev->config;
	struct spi_thejas32_data *data = dev->data;
	int rc;

	k_mutex_lock(&data->lock, K_FOREVER);

	rc = spi_thejas32_configure(dev, spi_cfg);
	if (rc != 0) {
		k_mutex_unlock(&data->lock);
		return rc;
	}

	/* Hold chip-select asserted across the whole transfer, see file header. */
	ctrl_write(config->base, ctrl_read(config->base) | THEJAS32_SPI_CTRL_CSAAT(1));

	const size_t tx_count = tx_bufs ? tx_bufs->count : 0;
	const size_t rx_count = rx_bufs ? rx_bufs->count : 0;
	size_t tx_idx = 0, tx_off = 0, rx_idx = 0, rx_off = 0;

	for (;;) {
		const struct spi_buf *tb = (tx_idx < tx_count) ? &tx_bufs->buffers[tx_idx] : NULL;
		const struct spi_buf *rb = (rx_idx < rx_count) ? &rx_bufs->buffers[rx_idx] : NULL;

		while (tb != NULL && tx_off >= tb->len) {
			tx_idx++;
			tx_off = 0;
			tb = (tx_idx < tx_count) ? &tx_bufs->buffers[tx_idx] : NULL;
		}
		while (rb != NULL && rx_off >= rb->len) {
			rx_idx++;
			rx_off = 0;
			rb = (rx_idx < rx_count) ? &rx_bufs->buffers[rx_idx] : NULL;
		}
		if (tb == NULL && rb == NULL) {
			break;
		}

		const uint8_t tx_byte = tb ? ((const uint8_t *)tb->buf)[tx_off] : 0U;
		uint8_t rx_byte;

		rc = spi_thejas32_transfer_byte(config->base, tx_byte, &rx_byte);
		if (rc != 0) {
			break;
		}
		if (rb != NULL) {
			((uint8_t *)rb->buf)[rx_off] = rx_byte;
		}
		if (tb != NULL) {
			tx_off++;
		}
		if (rb != NULL) {
			rx_off++;
		}
	}

	ctrl_write(config->base, ctrl_read(config->base) & ~THEJAS32_SPI_CTRL_CSAAT(1));

	k_mutex_unlock(&data->lock);

	return rc;
}

static int spi_thejas32_release(const struct device *dev, const struct spi_config *spi_cfg)
{
	ARG_UNUSED(spi_cfg);
	const struct spi_thejas32_config *config = dev->config;

	ctrl_write(config->base, ctrl_read(config->base) & ~THEJAS32_SPI_CTRL_CSAAT(1));
	return 0;
}

static DEVICE_API(spi, spi_thejas32_driver_api) = {
	.transceive = spi_thejas32_transceive,
	.release = spi_thejas32_release,
};

static int spi_thejas32_init(const struct device *dev)
{
	struct spi_thejas32_data *data = dev->data;

	k_mutex_init(&data->lock);
	return 0;
}

#define THEJAS32_SPI_INIT(n)                                                                      \
	static const struct spi_thejas32_config spi_thejas32_config_##n = {                       \
		.base = DT_INST_REG_ADDR(n),                                                       \
	};                                                                                         \
	static struct spi_thejas32_data spi_thejas32_data_##n;                                    \
	SPI_DEVICE_DT_INST_DEFINE(n, spi_thejas32_init, NULL, &spi_thejas32_data_##n,              \
				  &spi_thejas32_config_##n, POST_KERNEL,                           \
				  CONFIG_SPI_INIT_PRIORITY, &spi_thejas32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(THEJAS32_SPI_INIT)
