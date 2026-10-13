/*
 * telemetry.c - Real-Time Telemetry Stream Implementation over UART2 (FW-18)
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Peripheral: UART2 @ 0x10000300 (115,200 baud, 8N1)
 *
 * Implements:
 * 1. 10 Hz background telemetry thread.
 * 2. Comma-separated ASCII stream ($TELEM,...) with XOR checksum.
 * 3. MSP (MultiWii Serial Protocol) attitude packet transmission.
 * 4. Non-blocking UART transmission over UART2.
 */

#include "telemetry.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/spinlock.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(telemetry, CONFIG_LOG_DEFAULT_LEVEL);

/* Device Tree Node Resolution */
#if DT_NODE_EXISTS(DT_ALIAS(telemetry_uart))
static const struct device *const s_telem_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
#elif DT_NODE_EXISTS(DT_NODELABEL(uart2))
static const struct device *const s_telem_uart = DEVICE_DT_GET(DT_NODELABEL(uart2));
#else
#error "DeviceTree error: Neither alias 'telemetry_uart' nor nodelabel 'uart2' found."
#endif

/* Thread Configuration */
#define TELEMETRY_THREAD_STACK_SIZE   2048
#define TELEMETRY_THREAD_PRIORITY     8    /* Low-priority background worker */
#define TELEMETRY_PERIOD_MS           100  /* 10 Hz loop rate */

/* MSP Identifiers */
#define MSP_HEADER_BYTE0              '$'
#define MSP_HEADER_BYTE1              'M'
#define MSP_HEADER_BYTE2              '<'
#define MSP_ATTITUDE                  108  /* Attitude angles command */

/* Cached Telemetry Payload State */
typedef struct {
	uint32_t timestamp_ms;
	float roll_deg;
	float pitch_deg;
	float yaw_deg;
	float throttle;
	float battery_v;
	bool is_armed;
	uint8_t flight_mode;
	bool failsafe_active;
} telemetry_state_cache_t;

static telemetry_state_cache_t s_cache;
static struct k_spinlock s_cache_lock;
static bool s_is_initialized = false;

/* Thread Memory (defined in main.c) */
K_THREAD_STACK_DECLARE(s_telemetry_stack, TELEMETRY_THREAD_STACK_SIZE);
static struct k_thread s_telemetry_thread_data;

int telemetry_init(void)
{
	if (!device_is_ready(s_telem_uart)) {
		LOG_ERR("UART2 peripheral not ready for Telemetry!");
		return -ENODEV;
	}

	/* Configure UART2: 115,200 baud, 8 data bits, 1 stop bit, no parity */
	struct uart_config cfg = {
		.baudrate  = 115200,
		.parity    = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
	};

	int ret = uart_configure(s_telem_uart, &cfg);
	if (ret < 0 && ret != -ENOSYS) {
		LOG_WRN("uart_configure on UART2 returned %d (using DeviceTree defaults)", ret);
	}

	memset(&s_cache, 0, sizeof(s_cache));
	s_is_initialized = true;

	LOG_INF("Telemetry initialized on UART2 @ 115200 baud (10 Hz stream)");
	return 0;
}

void telemetry_set_state(float roll_deg, float pitch_deg, float yaw_deg,
			 float throttle, float battery_v, bool is_armed,
			 uint8_t flight_mode, bool failsafe_active)
{
	k_spinlock_key_t key = k_spin_lock(&s_cache_lock);
	s_cache.timestamp_ms    = (uint32_t)k_uptime_get_32();
	s_cache.roll_deg        = roll_deg;
	s_cache.pitch_deg       = pitch_deg;
	s_cache.yaw_deg         = yaw_deg;
	s_cache.throttle        = throttle;
	s_cache.battery_v       = battery_v;
	s_cache.is_armed        = is_armed;
	s_cache.flight_mode     = flight_mode;
	s_cache.failsafe_active = failsafe_active;
	k_spin_unlock(&s_cache_lock, key);
}

int telemetry_send_ascii_frame(void)
{
	if (!s_is_initialized) {
		return -EAGAIN;
	}

	telemetry_state_cache_t snap;
	k_spinlock_key_t key = k_spin_lock(&s_cache_lock);
	snap = s_cache;
	k_spin_unlock(&s_cache_lock, key);

	/* Format comma-separated ASCII string without checksum first */
	char payload[128];
	int len = snprintf(payload, sizeof(payload),
			   "TELEM,%u,%.2f,%.2f,%.2f,%.3f,%.2f,%d,%d,%d",
			   snap.timestamp_ms,
			   snap.roll_deg,
			   snap.pitch_deg,
			   snap.yaw_deg,
			   snap.throttle,
			   snap.battery_v,
			   snap.is_armed ? 1 : 0,
			   snap.flight_mode,
			   snap.failsafe_active ? 1 : 0);

	if (len < 0 || len >= (int)sizeof(payload)) {
		return -EINVAL;
	}

	/* Calculate NMEA-style XOR checksum over characters */
	uint8_t cs = 0;
	for (int i = 0; i < len; i++) {
		cs ^= (uint8_t)payload[i];
	}

	/* Complete sentence with header '$' and tail '*<CS>\r\n' */
	char sentence[144];
	int total_len = snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", payload, cs);

	/* Transmit non-blocking over UART2 */
	for (int i = 0; i < total_len; i++) {
		uart_poll_out(s_telem_uart, (unsigned char)sentence[i]);
	}

	return total_len;
}

int telemetry_send_msp_attitude(void)
{
	if (!s_is_initialized) {
		return -EAGAIN;
	}

	telemetry_state_cache_t snap;
	k_spinlock_key_t key = k_spin_lock(&s_cache_lock);
	snap = s_cache;
	k_spin_unlock(&s_cache_lock, key);

	/*
	 * MultiWii Serial Protocol (MSP_ATTITUDE, code 108):
	 * Payload size: 6 bytes
	 * int16_t roll  (deg * 10, e.g. 10.5 deg = 105)
	 * int16_t pitch (deg * 10, e.g. -5.2 deg = -52)
	 * int16_t yaw   (deg, e.g. 180 deg = 180)
	 */
	int16_t roll_i16  = (int16_t)(snap.roll_deg * 10.0f);
	int16_t pitch_i16 = (int16_t)(snap.pitch_deg * 10.0f);
	int16_t yaw_i16   = (int16_t)(snap.yaw_deg);

	uint8_t buf[12];
	buf[0] = MSP_HEADER_BYTE0; /* '$' */
	buf[1] = MSP_HEADER_BYTE1; /* 'M' */
	buf[2] = MSP_HEADER_BYTE2; /* '<' */
	buf[3] = 6;                /* Payload length */
	buf[4] = MSP_ATTITUDE;     /* Command code */

	/* Little-endian 16-bit payload */
	buf[5] = (uint8_t)(roll_i16 & 0xFF);
	buf[6] = (uint8_t)((roll_i16 >> 8) & 0xFF);
	buf[7] = (uint8_t)(pitch_i16 & 0xFF);
	buf[8] = (uint8_t)((pitch_i16 >> 8) & 0xFF);
	buf[9] = (uint8_t)(yaw_i16 & 0xFF);
	buf[10] = (uint8_t)((yaw_i16 >> 8) & 0xFF);

	/* XOR checksum over length, code, and payload bytes */
	uint8_t cs = buf[3] ^ buf[4];
	for (int i = 5; i <= 10; i++) {
		cs ^= buf[i];
	}
	buf[11] = cs;

	/* Transmit binary packet */
	for (int i = 0; i < 12; i++) {
		uart_poll_out(s_telem_uart, buf[i]);
	}

	return 12;
}

void telemetry_step(void)
{
	/* Transmit both human-readable ASCII and standard MSP packet */
	telemetry_send_ascii_frame();
	telemetry_send_msp_attitude();
}

static void telemetry_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("Telemetry streaming thread active (10 Hz)");

	while (1) {
		telemetry_step();
		k_msleep(TELEMETRY_PERIOD_MS);
	}
}

void telemetry_start(void)
{
	k_thread_create(&s_telemetry_thread_data,
			s_telemetry_stack,
			K_THREAD_STACK_SIZEOF(s_telemetry_stack),
			telemetry_thread_entry,
			NULL, NULL, NULL,
			K_PRIO_PREEMPT(TELEMETRY_THREAD_PRIORITY),
			0, K_NO_WAIT);

	k_thread_name_set(&s_telemetry_thread_data, "telemetry_thread");
}
