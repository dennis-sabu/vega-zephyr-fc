/*
 * Out-of-tree Zephyr I2C driver for the C-DAC THEJAS32 SoC.
 * NOT part of upstream Zephyr -- see zephyr/module/CMakeLists.txt.
 *
 * Register layout and the exact start/write/read/stop bit sequence are
 * taken directly from the native VEGA SDK's own driver
 * (bsp/drivers/i2c/i2c.c, bsp/include/i2c.h, read via its public Doxygen
 * pages at cdac-vega.gitlab.io/vega-sdk) and cross-checked field-for-field
 * against the Arduino core's libraries/Wire/src/Wire.cpp, which uses the
 * same register names with no numeric offsets of its own -- the native
 * SDK's I2C_REG_TYPE struct is what actually supplies those offsets:
 *
 *   offset  field      role
 *   0x00    I2C_CR     control: bit0=START(write) / (len<<2)|bit0=START(read), bit1=STOP
 *   0x01    I2C_SR0    status: bit0=STAS, bit1=STPS, bit2=TxFFF(full), bit3=TxFFE(empty),
 *                       bit4=TxC(complete), bit6=RxFFE(empty), bit7=RxC(ready)
 *   0x02    I2C_SR1    status: bit0=NACK
 *   0x03    I2C_IER    interrupt enable: bit0=global, bit1=tx, bit2=rx
 *   0x04    I2C_TxFF   transmit data (also carries the 7-bit address+R/W byte)
 *   0x05    I2C_RxFF   receive data
 *   0x06    I2C_CHL    clock divider, low byte of CH  (CH  = sys_clk / (2*i2c_clk))
 *   0x07    I2C_CHH    clock divider, high byte of CH
 *   0x08    I2C_CHHL   clock divider, low byte of CHH (CHH = sys_clk / (4*i2c_clk))
 *   0x09    I2C_CHHH   clock divider, high byte of CHH
 *   0x0A    I2C_TXCLR  write 0xFF to clear the TX FIFO
 *
 * The hardware's maximum single read transaction is 16 bytes -- taken
 * directly from the native driver's own bounds check in
 * i2c_start()/i2c_data_read() ("max read length supported is 16"), not
 * inferred from a register bit width.
 *
 * One deliberate deviation between the two reference sources, resolved
 * here: the native SDK's i2c_data_write() only checks TxC/NACK ONCE,
 * after writing an entire buffer, whereas the Arduino core's
 * TwoWire::endTransmission() checks TxC/NACK after EVERY individual
 * byte. This driver follows the Arduino core's per-byte check (see
 * thejas32_i2c_write_byte(), called once per byte) because it detects a
 * NACK immediately instead of after wasting time clocking out bytes a
 * device has already stopped listening to -- both are legitimate vendor
 * implementations of the same hardware; this is this project's own
 * documented choice between them, not a discovered "correct" answer.
 *
 * Every blocking wait below is bounded (CONFIG_I2C_THEJAS32_TIMEOUT_US),
 * unlike the native SDK's own i2c.c and the Arduino core's Wire.cpp,
 * which both use unbounded `while (cond);` loops -- a stuck bus (SDA held
 * low, missing pull-ups) hangs those forever. This driver returns
 * -ETIMEDOUT instead.
 */

#define DT_DRV_COMPAT cdac_thejas32_i2c

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_thejas32, CONFIG_I2C_LOG_LEVEL);

/* Register byte offsets, see file header. */
#define THEJAS32_I2C_CR    0x00
#define THEJAS32_I2C_SR0   0x01
#define THEJAS32_I2C_SR1   0x02
#define THEJAS32_I2C_IER   0x03
#define THEJAS32_I2C_TXFF  0x04
#define THEJAS32_I2C_RXFF  0x05
#define THEJAS32_I2C_CHL   0x06
#define THEJAS32_I2C_CHH   0x07
#define THEJAS32_I2C_CHHL  0x08
#define THEJAS32_I2C_CHHH  0x09
#define THEJAS32_I2C_TXCLR 0x0A

#define THEJAS32_I2C_SR0_STAS  BIT(0)
#define THEJAS32_I2C_SR0_STPS  BIT(1)
#define THEJAS32_I2C_SR0_TXFFF BIT(2)
#define THEJAS32_I2C_SR0_TXFFE BIT(3)
#define THEJAS32_I2C_SR0_TXC   BIT(4)
#define THEJAS32_I2C_SR0_RXFFE BIT(6)
#define THEJAS32_I2C_SR0_RXC   BIT(7)
#define THEJAS32_I2C_SR1_NACK  BIT(0)

#define THEJAS32_I2C_MAX_READ_LEN 16

struct i2c_thejas32_config {
	mem_addr_t base;
};

struct i2c_thejas32_data {
	struct k_mutex bus_lock;
	uint32_t dev_config;
};

static inline uint8_t reg_read(mem_addr_t base, uint32_t offset)
{
	return sys_read8(base + offset);
}

static inline void reg_write(mem_addr_t base, uint32_t offset, uint8_t value)
{
	sys_write8(value, base + offset);
}

/* Spin-waits for (reg & mask) == expect, bounded by
 * CONFIG_I2C_THEJAS32_TIMEOUT_US. Returns 0 on success, -ETIMEDOUT if the
 * bound is reached -- this is the one behavioral difference from the
 * native SDK's own driver, see file header.
 */
static int wait_for(mem_addr_t base, uint32_t offset, uint8_t mask, uint8_t expect)
{
	const uint32_t max_cycles = k_us_to_cyc_ceil32(CONFIG_I2C_THEJAS32_TIMEOUT_US);
	const uint32_t start = k_cycle_get_32();

	while ((reg_read(base, offset) & mask) != expect) {
		if ((k_cycle_get_32() - start) > max_cycles) {
			return -ETIMEDOUT;
		}
	}
	return 0;
}

static int thejas32_i2c_start(mem_addr_t base, uint8_t read_len, bool read)
{
	int rc;

	reg_write(base, THEJAS32_I2C_TXCLR, 0xFF);
	barrier_dmem_fence_full();

	/* Native driver waits for TxFFE and TxC before issuing a new start;
	 * reproduced here rather than assuming the bus is already idle.
	 */
	rc = wait_for(base, THEJAS32_I2C_SR0, THEJAS32_I2C_SR0_TXFFE | THEJAS32_I2C_SR0_TXC,
		      THEJAS32_I2C_SR0_TXFFE | THEJAS32_I2C_SR0_TXC);
	if (rc != 0) {
		return rc;
	}

	if (read) {
		reg_write(base, THEJAS32_I2C_CR, (uint8_t)((read_len << 2) | 0x01));
	} else {
		reg_write(base, THEJAS32_I2C_CR, 0x01);
	}
	barrier_dmem_fence_full();

	return wait_for(base, THEJAS32_I2C_SR0, THEJAS32_I2C_SR0_STAS, THEJAS32_I2C_SR0_STAS);
}

static int thejas32_i2c_stop(mem_addr_t base)
{
	reg_write(base, THEJAS32_I2C_CR, 0x02);
	barrier_dmem_fence_full();
	return wait_for(base, THEJAS32_I2C_SR0, THEJAS32_I2C_SR0_STPS, THEJAS32_I2C_SR0_STPS);
}

/* Writes one byte (address-plus-R/W or payload data -- the hardware does
 * not distinguish them, matching the native driver) and checks for NACK.
 * Returns 1 if NACKed, 0 if ACKed, a negative errno on timeout.
 */
static int thejas32_i2c_write_byte(mem_addr_t base, uint8_t byte)
{
	int rc = wait_for(base, THEJAS32_I2C_SR0, THEJAS32_I2C_SR0_TXFFF, 0);

	if (rc != 0) {
		return rc;
	}
	reg_write(base, THEJAS32_I2C_TXFF, byte);
	barrier_dmem_fence_full();

	rc = wait_for(base, THEJAS32_I2C_SR0, THEJAS32_I2C_SR0_TXC, THEJAS32_I2C_SR0_TXC);
	if (rc != 0) {
		return rc;
	}

	return (reg_read(base, THEJAS32_I2C_SR1) & THEJAS32_I2C_SR1_NACK) ? 1 : 0;
}

static int thejas32_i2c_read_bytes(mem_addr_t base, uint8_t *buf, uint8_t len)
{
	int rc = wait_for(base, THEJAS32_I2C_SR0, THEJAS32_I2C_SR0_RXC, THEJAS32_I2C_SR0_RXC);

	if (rc != 0) {
		return rc;
	}
	for (uint8_t i = 0; i < len; i++) {
		rc = wait_for(base, THEJAS32_I2C_SR0, THEJAS32_I2C_SR0_RXFFE, 0);
		if (rc != 0) {
			return rc;
		}
		buf[i] = reg_read(base, THEJAS32_I2C_RXFF);
	}
	return 0;
}

static int i2c_thejas32_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_thejas32_config *config = dev->config;
	struct i2c_thejas32_data *data = dev->data;
	uint32_t i2c_clock_hz;

	if ((dev_config & I2C_MODE_CONTROLLER) == 0U) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		i2c_clock_hz = 100000U;
		break;
	case I2C_SPEED_FAST:
		i2c_clock_hz = 400000U;
		break;
	default:
		return -ENOTSUP;
	}

	/* CH/CHH derivation reproduced exactly from the native SDK's
	 * i2c_initialize(): CH = sys_clk/(2*i2c_clk), CHH = sys_clk/(4*i2c_clk),
	 * each split into low/high bytes across two registers.
	 */
	const uint32_t sys_clock_hz = DT_PROP(DT_NODELABEL(cpu0), clock_frequency);
	const uint16_t ch = (uint16_t)(sys_clock_hz / (2U * i2c_clock_hz));
	const uint16_t chh = (uint16_t)(sys_clock_hz / (4U * i2c_clock_hz));

	k_mutex_lock(&data->bus_lock, K_FOREVER);

	reg_write(config->base, THEJAS32_I2C_TXCLR, 0xFF);
	barrier_dmem_fence_full();
	reg_write(config->base, THEJAS32_I2C_CHL, (uint8_t)(ch & 0xFF));
	reg_write(config->base, THEJAS32_I2C_CHH, (uint8_t)((ch >> 8) & 0xFF));
	reg_write(config->base, THEJAS32_I2C_CHHL, (uint8_t)(chh & 0xFF));
	reg_write(config->base, THEJAS32_I2C_CHHH, (uint8_t)((chh >> 8) & 0xFF));
	barrier_dmem_fence_full();

	data->dev_config = dev_config;

	k_mutex_unlock(&data->bus_lock);

	return 0;
}

static int i2c_thejas32_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_thejas32_data *data = dev->data;

	if (data->dev_config == 0U) {
		return -EIO;
	}
	*dev_config = data->dev_config;
	return 0;
}

static int i2c_thejas32_transfer_one(mem_addr_t base, struct i2c_msg *msg, uint16_t addr)
{
	const bool read = (msg->flags & I2C_MSG_READ) != 0U;
	int rc;

	if (read && msg->len > THEJAS32_I2C_MAX_READ_LEN) {
		LOG_ERR("read of %u bytes exceeds the 16-byte hardware limit", msg->len);
		return -EINVAL;
	}

	rc = thejas32_i2c_start(base, read ? (uint8_t)msg->len : 0, read);
	if (rc != 0) {
		return rc;
	}

	const uint8_t addr_byte = (uint8_t)((addr << 1) | (read ? 0x01 : 0x00));

	rc = thejas32_i2c_write_byte(base, addr_byte);
	if (rc < 0) {
		return rc;
	}
	if (rc == 1) {
		/* NACK on address: stop the bus before reporting -EIO so the
		 * next transaction does not start from a half-open bus state.
		 */
		(void)thejas32_i2c_stop(base);
		return -EIO;
	}

	if (read) {
		rc = thejas32_i2c_read_bytes(base, msg->buf, (uint8_t)msg->len);
	} else {
		for (uint32_t i = 0; i < msg->len; i++) {
			rc = thejas32_i2c_write_byte(base, msg->buf[i]);
			if (rc != 0) {
				break;
			}
		}
		if (rc == 1) {
			rc = -EIO; /* NACK on data */
		}
	}
	if (rc != 0) {
		(void)thejas32_i2c_stop(base);
		return (rc > 0) ? -EIO : rc;
	}

	if ((msg->flags & I2C_MSG_STOP) != 0U) {
		rc = thejas32_i2c_stop(base);
	}

	return rc;
}

static int i2c_thejas32_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				  uint16_t addr)
{
	const struct i2c_thejas32_config *config = dev->config;
	struct i2c_thejas32_data *data = dev->data;
	int rc = 0;

	if (num_msgs == 0U) {
		return 0;
	}

	k_mutex_lock(&data->bus_lock, K_FOREVER);
	for (uint8_t i = 0; i < num_msgs; i++) {
		rc = i2c_thejas32_transfer_one(config->base, &msgs[i], addr);
		if (rc != 0) {
			break;
		}
	}
	k_mutex_unlock(&data->bus_lock);

	return rc;
}

static DEVICE_API(i2c, i2c_thejas32_driver_api) = {
	.configure = i2c_thejas32_configure,
	.get_config = i2c_thejas32_get_config,
	.transfer = i2c_thejas32_transfer,
};

static int i2c_thejas32_init(const struct device *dev)
{
	struct i2c_thejas32_data *data = dev->data;

	k_mutex_init(&data->bus_lock);

	return i2c_thejas32_configure(dev, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD));
}

#define THEJAS32_I2C_INIT(n)                                                                      \
	static const struct i2c_thejas32_config i2c_thejas32_config_##n = {                       \
		.base = DT_INST_REG_ADDR(n),                                                       \
	};                                                                                         \
	static struct i2c_thejas32_data i2c_thejas32_data_##n;                                    \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_thejas32_init, NULL, &i2c_thejas32_data_##n,              \
				  &i2c_thejas32_config_##n, POST_KERNEL,                           \
				  CONFIG_I2C_INIT_PRIORITY, &i2c_thejas32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(THEJAS32_I2C_INIT)
