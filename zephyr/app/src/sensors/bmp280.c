/*
 * bmp280.c - Hardware Driver Implementation for Bosch BMP280 Barometer
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 *
 * Strict Constraint:
 * The out-of-tree i2c_thejas32 hardware FIFO has a maximum single burst read
 * limit of 16 bytes. All 24 bytes of PROM calibration parameters are read in
 * two independent 12-byte transactions. Sensor readings are 6 bytes burst.
 */

#include "bmp280.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <math.h>

LOG_MODULE_REGISTER(bmp280, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_EXISTS(DT_ALIAS(baro_i2c))
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_ALIAS(baro_i2c));
#elif DT_NODE_EXISTS(DT_ALIAS(imu_i2c))
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_ALIAS(imu_i2c));
#elif DT_NODE_EXISTS(DT_NODELABEL(i2c0))
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
#else
#error "DeviceTree error: No I2C device node found for BMP280."
#endif

static uint8_t s_active_addr = BMP280_I2C_ADDR_PRIMARY;
static bmp280_calib_param_t s_calib;
static int32_t s_t_fine = 0;
static float s_ground_altitude_m = 0.0f;
static float s_ground_pressure_pa = STANDARD_SEA_LEVEL_PA;
static bool s_ground_calibrated = false;
static uint32_t s_consecutive_errors = 0;
#define MAX_ALLOWED_I2C_ERRORS 5

/* Filtering state */
static float s_prev_filtered_alt = 0.0f;
static float s_climb_rate_filtered = 0.0f;
static uint32_t s_prev_time_us = 0;
static bool s_first_read = true;

/* Helper: Write single register */
static int bmp280_write_reg(uint8_t reg, uint8_t val)
{
	uint8_t tx_buf[2] = {reg, val};
	int ret = i2c_write(i2c_dev, tx_buf, sizeof(tx_buf), s_active_addr);
	if (ret < 0) {
		s_consecutive_errors++;
		return ret;
	}
	s_consecutive_errors = 0;
	return 0;
}

/* Helper: Read single register */
static int bmp280_read_reg(uint8_t reg, uint8_t *val)
{
	int ret = i2c_write_read(i2c_dev, s_active_addr, &reg, 1, val, 1);
	if (ret < 0) {
		s_consecutive_errors++;
		return ret;
	}
	s_consecutive_errors = 0;
	return 0;
}

/* Helper: Burst read <= 16 bytes */
static int bmp280_read_burst(uint8_t start_reg, uint8_t *buf, uint8_t len)
{
	if (len > 16) {
		LOG_ERR("BMP280 read burst %d exceeds THEJAS32 16-byte limit!", len);
		return -EINVAL;
	}

	int ret = i2c_write_read(i2c_dev, s_active_addr, &start_reg, 1, buf, len);
	if (ret < 0) {
		s_consecutive_errors++;
		return ret;
	}
	s_consecutive_errors = 0;
	return 0;
}

int bmp280_init(void)
{
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C peripheral not ready for BMP280!");
		return -ENODEV;
	}

	LOG_INF("Probing BMP280 barometer on I2C0...");

	/* 1. Detect device address (0x76 or 0x77) */
	uint8_t chip_id = 0;
	s_active_addr = BMP280_I2C_ADDR_PRIMARY;
	int ret = bmp280_read_reg(BMP280_REG_ID, &chip_id);

	if (ret < 0 || (chip_id != BMP280_CHIP_ID1 && chip_id != BMP280_CHIP_ID2 &&
			chip_id != BMP280_CHIP_ID3 && chip_id != BME280_CHIP_ID)) {
		s_active_addr = BMP280_I2C_ADDR_SECONDARY;
		ret = bmp280_read_reg(BMP280_REG_ID, &chip_id);
		if (ret < 0 || (chip_id != BMP280_CHIP_ID1 && chip_id != BMP280_CHIP_ID2 &&
				chip_id != BMP280_CHIP_ID3 && chip_id != BME280_CHIP_ID)) {
			LOG_ERR("BMP280 not found at 0x76 or 0x77 (read ID: 0x%02X)", chip_id);
			return -ENODEV;
		}
	}

	LOG_INF("BMP280 confirmed at addr 0x%02X with Chip ID 0x%02X", s_active_addr, chip_id);

	/* 2. Soft Reset */
	bmp280_write_reg(BMP280_REG_RESET, 0xB6);
	k_msleep(15);

	/*
	 * 3. Read 24 bytes of Trimming Parameters from PROM:
	 * Strict constraint: read in two 12-byte bursts (<= 16 bytes limit).
	 */
	uint8_t calib_buf1[12];
	uint8_t calib_buf2[12];

	/* Burst 1: Regs 0x88 to 0x93 (12 bytes) */
	ret = bmp280_read_burst(BMP280_REG_CALIB_00, calib_buf1, 12);
	if (ret < 0) {
		LOG_ERR("Failed to read calibration chunk 1 (err %d)", ret);
		return ret;
	}

	/* Burst 2: Regs 0x94 to 0x9F (12 bytes) */
	ret = bmp280_read_burst(BMP280_REG_CALIB_00 + 12, calib_buf2, 12);
	if (ret < 0) {
		LOG_ERR("Failed to read calibration chunk 2 (err %d)", ret);
		return ret;
	}

	/* Unpack little-endian coefficients */
	s_calib.dig_T1 = (uint16_t)(calib_buf1[0]  | (calib_buf1[1]  << 8));
	s_calib.dig_T2 = (int16_t) (calib_buf1[2]  | (calib_buf1[3]  << 8));
	s_calib.dig_T3 = (int16_t) (calib_buf1[4]  | (calib_buf1[5]  << 8));
	s_calib.dig_P1 = (uint16_t)(calib_buf1[6]  | (calib_buf1[7]  << 8));
	s_calib.dig_P2 = (int16_t) (calib_buf1[8]  | (calib_buf1[9]  << 8));
	s_calib.dig_P3 = (int16_t) (calib_buf1[10] | (calib_buf1[11] << 8));

	s_calib.dig_P4 = (int16_t) (calib_buf2[0]  | (calib_buf2[1]  << 8));
	s_calib.dig_P5 = (int16_t) (calib_buf2[2]  | (calib_buf2[3]  << 8));
	s_calib.dig_P6 = (int16_t) (calib_buf2[4]  | (calib_buf2[5]  << 8));
	s_calib.dig_P7 = (int16_t) (calib_buf2[6]  | (calib_buf2[7]  << 8));
	s_calib.dig_P8 = (int16_t) (calib_buf2[8]  | (calib_buf2[9]  << 8));
	s_calib.dig_P9 = (int16_t) (calib_buf2[10] | (calib_buf2[11] << 8));

	LOG_INF("BMP280 PROM calibration loaded (T1=%u, P1=%u, P9=%d)",
		s_calib.dig_T1, s_calib.dig_P1, s_calib.dig_P9);

	/*
	 * 4. Configure Filter and Standby in CONFIG register (0xF5)
	 * Standby = 0.5 ms (0x00 << 5)
	 * IIR Filter coefficient = 16 (0x04 << 2) -> 0x10
	 * SPI 3-wire disabled (0x00)
	 */
	ret = bmp280_write_reg(BMP280_REG_CONFIG, 0x10);
	if (ret < 0) {
		return ret;
	}

	/*
	 * 5. Configure Oversampling & Mode in CTRL_MEAS register (0xF4)
	 * Temp oversampling = x2 (0x02 << 5) -> 0x40
	 * Press oversampling = x16 (0x05 << 2) -> 0x14
	 * Power Mode = Normal (0x03)
	 * Total = 0x40 | 0x14 | 0x03 = 0x57
	 */
	ret = bmp280_write_reg(BMP280_REG_CTRL_MEAS, 0x57);
	if (ret < 0) {
		return ret;
	}

	k_msleep(20);

	/* 6. Perform initial ground reference baseline reading */
	bmp280_calibrate_ground(30);

	LOG_INF("BMP280 driver initialized successfully.");
	return 0;
}

int bmp280_read(bmp280_data_t *data)
{
	if (data == NULL) {
		return -EINVAL;
	}

	/*
	 * Burst read 6 bytes (0xF7 to 0xFC):
	 * 0xF7: press_msb, 0xF8: press_lsb, 0xF9: press_xlsb
	 * 0xFA: temp_msb,  0xFB: temp_lsb,  0xFC: temp_xlsb
	 */
	uint8_t buf[6];
	int ret = bmp280_read_burst(BMP280_REG_PRESS_MSB, buf, 6);
	if (ret < 0) {
		data->is_healthy = false;
		return ret;
	}

	uint32_t now_us = (uint32_t)k_uptime_get_us();

	int32_t raw_press = (int32_t)((((uint32_t)buf[0]) << 12) |
				      (((uint32_t)buf[1]) << 4) |
				      (((uint32_t)buf[2]) >> 4));

	int32_t raw_temp = (int32_t)((((uint32_t)buf[3]) << 12) |
				     (((uint32_t)buf[4]) << 4) |
				     (((uint32_t)buf[5]) >> 4));

	/* 1. Compensate Temperature (Bosch 32-bit formula) */
	int32_t var1, var2;
	var1 = ((((raw_temp >> 3) - ((int32_t)s_calib.dig_T1 << 1))) *
		((int32_t)s_calib.dig_T2)) >> 11;
	var2 = (((((raw_temp >> 4) - ((int32_t)s_calib.dig_T1)) *
		  ((raw_temp >> 4) - ((int32_t)s_calib.dig_T1))) >> 12) *
		((int32_t)s_calib.dig_T3)) >> 14;

	s_t_fine = var1 + var2;
	int32_t t_centigrade = (s_t_fine * 5 + 128) >> 8;
	data->temperature_c = (float)t_centigrade / 100.0f;

	/* 2. Compensate Pressure (Bosch 64-bit integer formula) */
	int64_t p_var1, p_var2, p;
	p_var1 = ((int64_t)s_t_fine) - 128000;
	p_var2 = p_var1 * p_var1 * (int64_t)s_calib.dig_P6;
	p_var2 = p_var2 + ((p_var1 * (int64_t)s_calib.dig_P5) << 17);
	p_var2 = p_var2 + (((int64_t)s_calib.dig_P4) << 35);
	p_var1 = ((p_var1 * p_var1 * (int64_t)s_calib.dig_P3) >> 8) +
		 ((p_var1 * (int64_t)s_calib.dig_P2) << 12);
	p_var1 = (((((int64_t)1) << 47) + p_var1) * ((int64_t)s_calib.dig_P1)) >> 33;

	if (p_var1 == 0) {
		data->pressure_pa = 0.0f;
	} else {
		p = 1048576 - raw_press;
		p = (((p << 31) - p_var2) * 3125) / p_var1;
		p_var1 = (((int64_t)s_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
		p_var2 = (((int64_t)s_calib.dig_P8) * p) >> 19;
		p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)s_calib.dig_P7) << 4);
		data->pressure_pa = (float)p / 256.0f;
	}

	/* 3. Compute Altitude via Standard Hypsometric Formula */
	if (data->pressure_pa > 1000.0f) {
		/* h = 44330 * (1 - (P / P0)^(1 / 5.255)) */
		float pressure_ratio = data->pressure_pa / STANDARD_SEA_LEVEL_PA;
		data->altitude_abs_m = 44330.0f * (1.0f - powf(pressure_ratio, 0.19029496f));
	} else {
		data->altitude_abs_m = 0.0f;
	}

	/* Compute Relative Altitude above ground */
	if (s_ground_calibrated) {
		data->altitude_rel_m = data->altitude_abs_m - s_ground_altitude_m;
	} else {
		data->altitude_rel_m = 0.0f;
	}

	/* 4. Low-Pass Filter on Relative Altitude (EMA filter alpha = 0.18) */
	if (s_first_read) {
		s_prev_filtered_alt = data->altitude_rel_m;
		s_climb_rate_filtered = 0.0f;
		s_prev_time_us = now_us;
		s_first_read = false;
	}

	float dt_s = (float)(now_us - s_prev_time_us) * 1e-6f;
	if (dt_s <= 0.0f || dt_s > 0.5f) {
		dt_s = 0.02f; /* Fallback to 50 Hz nominal dt */
	}

	const float alpha_alt = 0.18f;
	float alt_filtered = (alpha_alt * data->altitude_rel_m) +
			     ((1.0f - alpha_alt) * s_prev_filtered_alt);
	data->altitude_filtered_m = alt_filtered;

	/* 5. Compute Filtered Vertical Velocity (Variometer) */
	float raw_climb_rate = (alt_filtered - s_prev_filtered_alt) / dt_s;
	const float alpha_climb = 0.15f;
	s_climb_rate_filtered = (alpha_climb * raw_climb_rate) +
				((1.0f - alpha_climb) * s_climb_rate_filtered);
	data->climb_rate_mps = s_climb_rate_filtered;

	s_prev_filtered_alt = alt_filtered;
	s_prev_time_us = now_us;

	data->timestamp_us = now_us;
	data->is_healthy = true;

	return 0;
}

int bmp280_calibrate_ground(uint32_t sample_count)
{
	if (sample_count == 0) {
		sample_count = 30;
	}

	LOG_INF("Calibrating ground altitude baseline (%u samples)...", sample_count);

	float sum_alt = 0.0f;
	float sum_press = 0.0f;
	uint32_t valid = 0;

	for (uint32_t i = 0; i < sample_count; i++) {
		bmp280_data_t d;
		if (bmp280_read(&d) == 0 && d.pressure_pa > 1000.0f) {
			sum_alt += d.altitude_abs_m;
			sum_press += d.pressure_pa;
			valid++;
		}
		k_msleep(10);
	}

	if (valid < (sample_count / 2)) {
		LOG_ERR("Failed to calibrate ground reference pressure!");
		return -EIO;
	}

	s_ground_altitude_m = sum_alt / (float)valid;
	s_ground_pressure_pa = sum_press / (float)valid;
	s_ground_calibrated = true;
	s_first_read = true;

	LOG_INF("Ground calibrated: Ref Alt = %.2f m, Ref Press = %.1f Pa (%.2f hPa)",
		s_ground_altitude_m, s_ground_pressure_pa, s_ground_pressure_pa / 100.0f);

	return 0;
}

void bmp280_set_ground_reference(float ground_pa)
{
	if (ground_pa > 1000.0f) {
		s_ground_pressure_pa = ground_pa;
		s_ground_altitude_m = 44330.0f * (1.0f - powf(ground_pa / STANDARD_SEA_LEVEL_PA, 0.19029496f));
		s_ground_calibrated = true;
	}
}

bool bmp280_is_healthy(void)
{
	return (s_consecutive_errors < MAX_ALLOWED_I2C_ERRORS);
}
