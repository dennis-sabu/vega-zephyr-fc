/*
 * sbus.c - SBUS Protocol Parser and UART Driver Implementation
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Peripheral: UART1 @ 0x10000200 (100,000 baud, 8E2)
 */

#include "sbus.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/spinlock.h>
#include <string.h>

LOG_MODULE_REGISTER(sbus, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_EXISTS(DT_ALIAS(sbus_uart))
static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(sbus_uart));
#elif DT_NODE_EXISTS(DT_NODELABEL(uart1))
static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
#else
#error "DeviceTree error: Neither alias 'sbus_uart' nor nodelabel 'uart1' found."
#endif

#define SBUS_RING_BUF_SIZE 256
RING_BUF_DECLARE(s_sbus_rb, SBUS_RING_BUF_SIZE);

static uint8_t s_frame_buf[SBUS_FRAME_LEN];
static uint32_t s_frame_idx = 0;
static sbus_channels_t s_channels;
static bool s_has_valid_frame = false;
static struct k_spinlock s_sbus_lock;

/* ISR callback for UART1 reception */
static void sbus_uart_irq_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
		uint8_t rx_byte;
		int read = uart_fifo_read(dev, &rx_byte, 1);
		if (read > 0) {
			ring_buf_put(&s_sbus_rb, &rx_byte, 1);
		}
	}
}

int sbus_init(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART1 peripheral not ready on THEJAS32!");
		return -ENODEV;
	}

	LOG_INF("Configuring UART1 for SBUS (100000 baud, 8E2)...");

	/* Configure UART line properties: 100000 baud, Even Parity, 2 Stop Bits */
	struct uart_config cfg = {
		.baudrate = 100000,
		.parity = UART_CFG_PARITY_EVEN,
		.stop_bits = UART_CFG_STOP_BITS_2,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
	};

	int ret = uart_configure(uart_dev, &cfg);
	if (ret < 0 && ret != -ENOSYS) {
		LOG_WRN("uart_configure returned %d (relying on DT current-speed)", ret);
	}

	/* Initialize channels to safe defaults */
	memset(&s_channels, 0, sizeof(s_channels));
	for (int i = 0; i < 16; i++) {
		s_channels.raw_channels[i] = SBUS_INPUT_CENTER;
	}
	s_channels.raw_channels[SBUS_CH_THROTTLE] = SBUS_INPUT_MIN;
	s_channels.throttle_norm = 0.0f;
	s_channels.arm_switch = false;
	s_channels.mode_switch = MODE_ANGLE;
	s_channels.frame_lost = true;
	s_channels.failsafe_active = true;

	/* Setup interrupt callback and enable RX interrupt */
	uart_irq_callback_user_data_set(uart_dev, sbus_uart_irq_callback, NULL);
	uart_irq_rx_enable(uart_dev);

	LOG_INF("SBUS receiver initialized successfully on UART1.");
	return 0;
}

static float sbus_scale_stick(uint16_t raw)
{
	int32_t diff = (int32_t)raw - SBUS_INPUT_CENTER;

	if (diff > SBUS_STICK_DEADBAND) {
		float span = (float)(SBUS_INPUT_MAX - SBUS_INPUT_CENTER - SBUS_STICK_DEADBAND);
		float val = (float)(diff - SBUS_STICK_DEADBAND) / span;
		return (val > 1.0f) ? 1.0f : val;
	} else if (diff < -SBUS_STICK_DEADBAND) {
		float span = (float)(SBUS_INPUT_CENTER - SBUS_INPUT_MIN - SBUS_STICK_DEADBAND);
		float val = (float)(diff + SBUS_STICK_DEADBAND) / span;
		return (val < -1.0f) ? -1.0f : val;
	}

	return 0.0f;
}

static float sbus_scale_throttle(uint16_t raw)
{
	if (raw <= SBUS_INPUT_MIN) {
		return 0.0f;
	}
	if (raw >= SBUS_INPUT_MAX) {
		return 1.0f;
	}
	return (float)(raw - SBUS_INPUT_MIN) / (float)(SBUS_INPUT_MAX - SBUS_INPUT_MIN);
}

static void sbus_decode_packet(const uint8_t *b)
{
	uint16_t raw[16];

	/* Decode 16 channels packed into 22 bytes (11 bits each) */
	raw[0]  = (uint16_t)((b[1]       | (b[2]  << 8)) & 0x07FF);
	raw[1]  = (uint16_t)(((b[2] >> 3)| (b[3]  << 5)) & 0x07FF);
	raw[2]  = (uint16_t)(((b[3] >> 6)| (b[4]  << 2) | (b[5] << 10)) & 0x07FF);
	raw[3]  = (uint16_t)(((b[5] >> 1)| (b[6]  << 7)) & 0x07FF);
	raw[4]  = (uint16_t)(((b[6] >> 4)| (b[7]  << 4)) & 0x07FF);
	raw[5]  = (uint16_t)(((b[7] >> 7)| (b[8]  << 1) | (b[9] << 9)) & 0x07FF);
	raw[6]  = (uint16_t)(((b[9] >> 2)| (b[10] << 6)) & 0x07FF);
	raw[7]  = (uint16_t)(((b[10] >> 5)|(b[11] << 3)) & 0x07FF);
	raw[8]  = (uint16_t)((b[12]      | (b[13] << 8)) & 0x07FF);
	raw[9]  = (uint16_t)(((b[13] >> 3)|(b[14] << 5)) & 0x07FF);
	raw[10] = (uint16_t)(((b[14] >> 6)|(b[15] << 2) | (b[16] << 10)) & 0x07FF);
	raw[11] = (uint16_t)(((b[16] >> 1)|(b[17] << 7)) & 0x07FF);
	raw[12] = (uint16_t)(((b[17] >> 4)|(b[18] << 4)) & 0x07FF);
	raw[13] = (uint16_t)(((b[18] >> 7)|(b[19] << 1) | (b[20] << 9)) & 0x07FF);
	raw[14] = (uint16_t)(((b[20] >> 2)|(b[21] << 6)) & 0x07FF);
	raw[15] = (uint16_t)(((b[21] >> 5)|(b[22] << 3)) & 0x07FF);

	uint8_t flags = b[23];
	bool frame_lost = (flags & 0x04) != 0;
	bool failsafe   = (flags & 0x08) != 0;

	k_spinlock_key_t key = k_spin_lock(&s_sbus_lock);

	for (int i = 0; i < 16; i++) {
		s_channels.raw_channels[i] = raw[i];
	}

	s_channels.roll_norm     = sbus_scale_stick(raw[SBUS_CH_ROLL]);
	s_channels.pitch_norm    = sbus_scale_stick(raw[SBUS_CH_PITCH]);
	s_channels.yaw_norm      = sbus_scale_stick(raw[SBUS_CH_YAW]);
	s_channels.throttle_norm = sbus_scale_throttle(raw[SBUS_CH_THROTTLE]);

	s_channels.arm_switch  = (raw[SBUS_CH_ARM] > SBUS_ARM_THRESHOLD);
	s_channels.mode_switch = (raw[SBUS_CH_MODE] > SBUS_MODE_ANGLE_THRESHOLD) ?
				 MODE_ANGLE : MODE_RATE;

	s_channels.frame_lost = frame_lost;
	s_channels.failsafe_active = failsafe;
	s_channels.last_packet_time_ms = (uint32_t)k_uptime_get_32();
	s_has_valid_frame = true;

	k_spin_unlock(&s_sbus_lock, key);
}

void sbus_process(void)
{
	uint8_t byte;

	while (ring_buf_get(&s_sbus_rb, &byte, 1) > 0) {
		if (s_frame_idx == 0) {
			/* Hunt for header byte */
			if (byte == SBUS_HEADER_BYTE) {
				s_frame_buf[0] = byte;
				s_frame_idx = 1;
			}
		} else {
			s_frame_buf[s_frame_idx++] = byte;

			if (s_frame_idx == SBUS_FRAME_LEN) {
				uint8_t end_byte = s_frame_buf[SBUS_FRAME_LEN - 1];

				/* Accept standard 0x00 end byte or Futaba S.BUS2 variations */
				if (end_byte == SBUS_END_BYTE_STANDARD ||
				    (end_byte & 0x0F) == SBUS_END_BYTE_FUTABA2) {
					sbus_decode_packet(s_frame_buf);
				} else {
					LOG_DBG("SBUS frame framing error (end byte 0x%02X)", end_byte);
				}

				s_frame_idx = 0; /* Reset for next frame */
			}
		}
	}
}

bool sbus_get_channels(sbus_channels_t *channels)
{
	if (channels == NULL) {
		return false;
	}

	k_spinlock_key_t key = k_spin_lock(&s_sbus_lock);
	*channels = s_channels;
	bool valid = s_has_valid_frame;
	k_spin_unlock(&s_sbus_lock, key);

	return valid;
}

bool sbus_is_signal_loss(void)
{
	if (!s_has_valid_frame) {
		return true;
	}

	uint32_t now = (uint32_t)k_uptime_get_32();

	k_spinlock_key_t key = k_spin_lock(&s_sbus_lock);
	bool failsafe = s_channels.failsafe_active;
	uint32_t last_time = s_channels.last_packet_time_ms;
	k_spin_unlock(&s_sbus_lock, key);

	if (failsafe) {
		return true;
	}

	if ((now - last_time) > RC_TIMEOUT_MS) {
		return true;
	}

	return false;
}
