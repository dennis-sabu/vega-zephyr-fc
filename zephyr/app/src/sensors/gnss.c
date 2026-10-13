/*
 * gnss.c - GNSS NMEA 0183 Protocol Parser Implementation
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Peripheral: UART0 @ 0x10000100
 *
 * Implements:
 * 1. IRQ-driven ring buffer reception on UART0.
 * 2. Strict XOR checksum verification between '$' and '*'.
 * 3. Token extraction for $GPGGA/$GNGGA (Position, Fix, Altitude, Sats)
 *    and $GPRMC/$GNRMC (Ground Speed, Track Course, Validity).
 */

#include "gnss.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/spinlock.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

LOG_MODULE_REGISTER(gnss, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_EXISTS(DT_ALIAS(gnss_uart))
static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(gnss_uart));
#elif DT_NODE_EXISTS(DT_NODELABEL(uart0))
static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
#else
#error "DeviceTree error: No UART device node found for GNSS (uart0)."
#endif

#define GNSS_RING_BUF_SIZE 512
RING_BUF_DECLARE(s_gnss_rb, GNSS_RING_BUF_SIZE);

#define NMEA_LINE_MAX_LEN 128
static char s_line_buf[NMEA_LINE_MAX_LEN];
static uint32_t s_line_idx = 0;

static gnss_data_t s_gnss;
static struct k_spinlock s_gnss_lock;

/* ISR callback for UART0 reception */
static void gnss_uart_irq_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uart_irq_update(dev);
	while (uart_irq_rx_ready(dev)) {
		uint8_t rx_byte;
		int read = uart_fifo_read(dev, &rx_byte, 1);
		if (read > 0) {
			ring_buf_put(&s_gnss_rb, &rx_byte, 1);
		} else {
			break;
		}
	}
}

int gnss_init(uint32_t baudrate)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART0 peripheral not ready on THEJAS32!");
		return -ENODEV;
	}

	if (baudrate == 0) {
		baudrate = 9600; /* Standard NMEA default */
	}

	LOG_INF("Configuring UART0 for GNSS @ %u baud...", baudrate);

	struct uart_config cfg = {
		.baudrate  = baudrate,
		.parity    = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE
	};

	int ret = uart_configure(uart_dev, &cfg);
	if (ret < 0 && ret != -ENOSYS) {
		LOG_WRN("uart_configure for UART0 returned %d (using DT rate)", ret);
	}

	/* Initialize state structure */
	k_spinlock_key_t key = k_spin_lock(&s_gnss_lock);
	memset(&s_gnss, 0, sizeof(s_gnss));
	s_gnss.valid_fix = false;
	k_spin_unlock(&s_gnss_lock, key);

	/* Enable RX interrupt */
	uart_irq_callback_user_data_set(uart_dev, gnss_uart_irq_callback, NULL);
	uart_irq_rx_enable(uart_dev);

	LOG_INF("GNSS receiver initialized on UART0.");
	return 0;
}

/* Custom CSV tokenizer that preserves empty tokens (consecutive commas) */
static int nmea_tokenize(char *str, char **tokens, int max_tokens)
{
	int count = 0;
	tokens[count++] = str;

	while (*str && count < max_tokens) {
		if (*str == ',') {
			*str = '\0';
			tokens[count++] = str + 1;
		}
		str++;
	}

	return count;
}

/* Parse ddmm.mmmm format into decimal degrees */
static double parse_nmea_coord(const char *coord_str, char hemisphere)
{
	if (coord_str == NULL || *coord_str == '\0') {
		return 0.0;
	}

	double raw = strtod(coord_str, NULL);
	int deg = (int)(raw / 100.0);
	double min = raw - ((double)deg * 100.0);
	double dec = (double)deg + (min / 60.0);

	if (hemisphere == 'S' || hemisphere == 'W') {
		dec = -dec;
	}

	return dec;
}

/* Parse $GPGGA / $GNGGA sentence */
static void parse_gga(char **tokens, int num_tokens)
{
	/* Expected tokens:
	 * [0] $xxGGA, [1] Time, [2] Lat, [3] N/S, [4] Lon, [5] E/W,
	 * [6] Fix, [7] Sats, [8] HDOP, [9] Alt, [10] M, [11] Geoid, [12] M
	 */
	if (num_tokens < 10) {
		return;
	}

	uint8_t fix = (uint8_t)atoi(tokens[6]);
	uint8_t sats = (uint8_t)atoi(tokens[7]);
	float hdop = (float)strtod(tokens[8], NULL);
	float alt_m = (float)strtod(tokens[9], NULL);

	double lat = 0.0, lon = 0.0;
	if (tokens[2][0] != '\0' && tokens[3][0] != '\0') {
		lat = parse_nmea_coord(tokens[2], tokens[3][0]);
	}
	if (tokens[4][0] != '\0' && tokens[5][0] != '\0') {
		lon = parse_nmea_coord(tokens[4], tokens[5][0]);
	}

	uint32_t time_utc = (uint32_t)atoi(tokens[1]);
	uint32_t now = (uint32_t)k_uptime_get_32();

	k_spinlock_key_t key = k_spin_lock(&s_gnss_lock);

	s_gnss.time_utc = time_utc;
	s_gnss.fix_quality = fix;
	s_gnss.satellites = sats;
	s_gnss.hdop = hdop;

	if (fix > 0) {
		s_gnss.latitude = lat;
		s_gnss.longitude = lon;
		s_gnss.altitude_msl_m = alt_m;
		s_gnss.last_fix_time_ms = now;
		s_gnss.valid_fix = true;
	} else {
		s_gnss.valid_fix = false;
	}

	s_gnss.sentences_parsed++;
	k_spin_unlock(&s_gnss_lock, key);
}

/* Parse $GPRMC / $GNRMC sentence */
static void parse_rmc(char **tokens, int num_tokens)
{
	/* Expected tokens:
	 * [0] $xxRMC, [1] Time, [2] Status, [3] Lat, [4] N/S,
	 * [5] Lon, [6] E/W, [7] Spd (kts), [8] Crs (deg), [9] Date
	 */
	if (num_tokens < 10) {
		return;
	}

	char status = tokens[2][0];
	float spd_kts = (float)strtod(tokens[7], NULL);
	float spd_mps = spd_kts * 0.5144444f; /* 1 knot = 0.5144444 m/s */
	float crs_deg = (float)strtod(tokens[8], NULL);
	uint32_t date_utc = (uint32_t)atoi(tokens[9]);
	uint32_t now = (uint32_t)k_uptime_get_32();

	k_spinlock_key_t key = k_spin_lock(&s_gnss_lock);

	s_gnss.speed_knots = spd_kts;
	s_gnss.speed_mps = spd_mps;
	s_gnss.course_deg = crs_deg;
	s_gnss.date_utc = date_utc;

	if (status == 'A') {
		s_gnss.valid_fix = true;
		s_gnss.last_fix_time_ms = now;
	}

	s_gnss.sentences_parsed++;
	k_spin_unlock(&s_gnss_lock, key);
}

/* Parse a single verified NMEA sentence */
static void parse_nmea_sentence(char *sentence)
{
	char *tokens[20];
	int num_tokens = nmea_tokenize(sentence, tokens, 20);

	if (num_tokens < 2) {
		return;
	}

	const char *header = tokens[0];

	if (strcmp(header, "$GPGGA") == 0 || strcmp(header, "$GNGGA") == 0) {
		parse_gga(tokens, num_tokens);
	} else if (strcmp(header, "$GPRMC") == 0 || strcmp(header, "$GNRMC") == 0) {
		parse_rmc(tokens, num_tokens);
	}
}

void gnss_process(void)
{
	uint8_t ch;

	while (ring_buf_get(&s_gnss_rb, &ch, 1) > 0) {
		if (ch == '$') {
			/* Start of new NMEA sentence */
			s_line_buf[0] = '$';
			s_line_idx = 1;
		} else if (s_line_idx > 0) {
			if (ch == '\r' || ch == '\n') {
				s_line_buf[s_line_idx] = '\0';

				/* Must have minimum length to contain '$...*hh' */
				if (s_line_idx >= 7) {
					char *star = strchr(s_line_buf, '*');
					if (star != NULL) {
						/* Compute XOR checksum between '$' and '*' */
						uint8_t calculated_cs = 0;
						for (char *p = &s_line_buf[1]; p < star; p++) {
							calculated_cs ^= (uint8_t)*p;
						}

						/* Parse 2 hex characters after '*' */
						uint8_t expected_cs = (uint8_t)strtoul(star + 1, NULL, 16);

						if (calculated_cs == expected_cs) {
							*star = '\0'; /* Terminate before checksum */
							parse_nmea_sentence(s_line_buf);
						} else {
							k_spinlock_key_t key = k_spin_lock(&s_gnss_lock);
							s_gnss.checksum_errors++;
							k_spin_unlock(&s_gnss_lock, key);
						}
					}
				}

				s_line_idx = 0; /* Reset for next sentence */
			} else {
				if (s_line_idx < (NMEA_LINE_MAX_LEN - 1)) {
					s_line_buf[s_line_idx++] = (char)ch;
				} else {
					s_line_idx = 0; /* Buffer overflow protection */
				}
			}
		}
	}
}

bool gnss_get_data(gnss_data_t *data)
{
	if (data == NULL) {
		return false;
	}

	k_spinlock_key_t key = k_spin_lock(&s_gnss_lock);
	*data = s_gnss;
	bool has_fix = s_gnss.valid_fix;
	k_spin_unlock(&s_gnss_lock, key);

	return has_fix;
}

bool gnss_has_fix(void)
{
	uint32_t now = (uint32_t)k_uptime_get_32();

	k_spinlock_key_t key = k_spin_lock(&s_gnss_lock);
	bool valid = s_gnss.valid_fix && (s_gnss.fix_quality > 0) &&
		     ((now - s_gnss.last_fix_time_ms) < 2500);
	k_spin_unlock(&s_gnss_lock, key);

	return valid;
}
