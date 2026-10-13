/*
 * mag.c - Hardware Driver Implementation for QMC5883L / HMC5883L Magnetometer
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 *
 * Implements:
 * 1. Automatic sensor identification on I2C0 (0x0D for QMC, 0x1E for HMC).
 * 2. Hard-iron and soft-iron calibration compensation.
 * 3. Tilt-compensated heading calculation using AHRS Roll and Pitch angles.
 * Strict constraint: Single burst read <= 16 bytes (i2c_thejas32 limitation).
 */

#include "mag.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <math.h>

LOG_MODULE_REGISTER(mag, CONFIG_LOG_DEFAULT_LEVEL);

#define PI_F       3.14159265358979323846f
#define TWO_PI_F   6.28318530717958647692f
#define RAD_TO_DEG (180.0f / PI_F)

#if DT_NODE_EXISTS(DT_ALIAS(mag_i2c))
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_ALIAS(mag_i2c));
#elif DT_NODE_EXISTS(DT_ALIAS(imu_i2c))
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_ALIAS(imu_i2c));
#elif DT_NODE_EXISTS(DT_NODELABEL(i2c0))
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
#else
#error "DeviceTree error: No I2C device node found for Magnetometer."
#endif

static mag_type_t s_mag_type = MAG_TYPE_UNKNOWN;
static uint8_t s_active_addr = 0;
static mag_calib_t s_calib = {
	.offset = {0.0f, 0.0f, 0.0f},
	.scale  = {1.0f, 1.0f, 1.0f},
	.declination_rad = 0.0f,
	.is_calibrated = false
};

static uint32_t s_consecutive_errors = 0;
#define MAX_ALLOWED_I2C_ERRORS 5

/* Helper: Write single register */
static int mag_write_reg(uint8_t reg, uint8_t val)
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
static int mag_read_reg(uint8_t reg, uint8_t *val)
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
static int mag_read_burst(uint8_t start_reg, uint8_t *buf, uint8_t len)
{
	if (len > 16) {
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

int mag_init(void)
{
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C peripheral not ready for Magnetometer!");
		return -ENODEV;
	}

	LOG_INF("Probing for 3-Axis Magnetometer on I2C0...");

	/* 1. Try QMC5883L at 0x0D */
	s_active_addr = QMC5883L_I2C_ADDR;
	uint8_t qmc_id = 0;
	int ret = mag_read_reg(0x0D, &qmc_id);

	if (ret == 0 && (qmc_id == 0xFF || qmc_id == 0x00)) {
		s_mag_type = MAG_TYPE_QMC5883L;
		LOG_INF("Found QMC5883L Magnetometer at 0x0D (ID: 0x%02X)", qmc_id);

		/* Soft Reset */
		mag_write_reg(0x0A, 0x80);
		k_msleep(10);

		/* Set Set/Reset Period register (0x0B -> 0x01) */
		mag_write_reg(0x0B, 0x01);

		/*
		 * Control Register 1 (0x09):
		 * OSR = 512 (0x00 << 6)
		 * RNG = 8G  (0x01 << 4) -> 0x10
		 * ODR = 100 Hz (0x02 << 2) -> 0x08
		 * MODE = Continuous (0x01) -> 0x01
		 * Total: 0x1D
		 */
		ret = mag_write_reg(0x09, 0x1D);
		if (ret < 0) {
			LOG_ERR("Failed to configure QMC5883L control register!");
			return ret;
		}

		LOG_INF("QMC5883L configured: 100 Hz, +/- 8G, Continuous mode.");
		return 0;
	}

	/* 2. Try HMC5883L at 0x1E */
	s_active_addr = HMC5883L_I2C_ADDR;
	uint8_t hmc_ida = 0, hmc_idb = 0, hmc_idc = 0;
	if (mag_read_reg(0x0A, &hmc_ida) == 0 &&
	    mag_read_reg(0x0B, &hmc_idb) == 0 &&
	    mag_read_reg(0x0C, &hmc_idc) == 0) {

		if (hmc_ida == 'H' && hmc_idb == '4' && hmc_idc == '3') {
			s_mag_type = MAG_TYPE_HMC5883L;
			LOG_INF("Found HMC5883L Magnetometer at 0x1E (ID: %c%c%c)",
				hmc_ida, hmc_idb, hmc_idc);

			/* Config A: 8-sample average, 75 Hz (0x78) */
			mag_write_reg(0x00, 0x78);

			/* Config B: Gain +/- 1.3 Gauss (0x20) */
			mag_write_reg(0x01, 0x20);

			/* Mode: Continuous measurement (0x00) */
			ret = mag_write_reg(0x02, 0x00);
			if (ret < 0) {
				LOG_ERR("Failed to configure HMC5883L mode register!");
				return ret;
			}

			LOG_INF("HMC5883L configured: 75 Hz, +/- 1.3G, Continuous mode.");
			return 0;
		}
	}

	LOG_WRN("No supported Magnetometer detected (neither QMC5883L @ 0x0D nor HMC5883L @ 0x1E).");
	s_mag_type = MAG_TYPE_UNKNOWN;
	return -ENODEV;
}

int mag_read(float roll_rad, float pitch_rad, mag_data_t *data)
{
	if (data == NULL) {
		return -EINVAL;
	}

	if (s_mag_type == MAG_TYPE_UNKNOWN) {
		data->is_healthy = false;
		return -ENODEV;
	}

	uint8_t buf[6];
	int16_t raw_x = 0, raw_y = 0, raw_z = 0;

	if (s_mag_type == MAG_TYPE_QMC5883L) {
		/* Burst read 6 bytes starting at 0x00: X_L, X_H, Y_L, Y_H, Z_L, Z_H */
		int ret = mag_read_burst(0x00, buf, 6);
		if (ret < 0) {
			data->is_healthy = false;
			return ret;
		}

		raw_x = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
		raw_y = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
		raw_z = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);

		/* QMC5883L scaling: 3000 LSB / Gauss at 8G range */
		data->raw.x = (float)raw_x / 3000.0f;
		data->raw.y = (float)raw_y / 3000.0f;
		data->raw.z = (float)raw_z / 3000.0f;

	} else if (s_mag_type == MAG_TYPE_HMC5883L) {
		/* Burst read 6 bytes starting at 0x03: X_H, X_L, Z_H, Z_L, Y_H, Y_L */
		int ret = mag_read_burst(0x03, buf, 6);
		if (ret < 0) {
			data->is_healthy = false;
			return ret;
		}

		raw_x = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
		raw_z = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
		raw_y = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

		/* HMC5883L scaling: 1090 LSB / Gauss at 1.3G range */
		data->raw.x = (float)raw_x / 1090.0f;
		data->raw.y = (float)raw_y / 1090.0f;
		data->raw.z = (float)raw_z / 1090.0f;
	}

	data->timestamp_us = (uint32_t)k_uptime_get_us();

	/* 1. Hard-Iron and Soft-Iron Calibration Compensation */
	float mx = (data->raw.x - s_calib.offset.x) * s_calib.scale.x;
	float my = (data->raw.y - s_calib.offset.y) * s_calib.scale.y;
	float mz = (data->raw.z - s_calib.offset.z) * s_calib.scale.z;

	data->cal.x = mx;
	data->cal.y = my;
	data->cal.z = mz;

	/*
	 * 2. Tilt Compensation using AHRS Euler Angles (Roll phi, Pitch theta):
	 * In NED body frame:
	 * X_h = mx * cos(theta) + my * sin(phi) * sin(theta) + mz * cos(phi) * sin(theta)
	 * Y_h = my * cos(phi)   - mz * sin(phi)
	 */
	float cos_phi = cosf(roll_rad);
	float sin_phi = sinf(roll_rad);
	float cos_theta = cosf(pitch_rad);
	float sin_theta = sinf(pitch_rad);

	float x_h = mx * cos_theta + my * sin_phi * sin_theta + mz * cos_phi * sin_theta;
	float y_h = my * cos_phi   - mz * sin_phi;

	/*
	 * 3. Heading Angle Calculation with Declination Adjustment:
	 * Yaw heading = atan2(-Y_h, X_h) + declination
	 */
	float heading = atan2f(-y_h, x_h) + s_calib.declination_rad;

	/* Normalize to [0, 2*PI) */
	while (heading < 0.0f) {
		heading += TWO_PI_F;
	}
	while (heading >= TWO_PI_F) {
		heading -= TWO_PI_F;
	}

	data->heading_rad = heading;
	data->heading_deg = heading * RAD_TO_DEG;
	data->is_healthy = true;

	return 0;
}

void mag_set_calibration(const mag_calib_t *calib)
{
	if (calib != NULL) {
		s_calib = *calib;
		s_calib.is_calibrated = true;
		LOG_INF("Magnetometer calibration updated: Offset=[%.3f, %.3f, %.3f], Scale=[%.3f, %.3f, %.3f]",
			s_calib.offset.x, s_calib.offset.y, s_calib.offset.z,
			s_calib.scale.x, s_calib.scale.y, s_calib.scale.z);
	}
}

void mag_set_declination(float declination_rad)
{
	s_calib.declination_rad = declination_rad;
	LOG_INF("Magnetic declination set to %.2f deg (%.4f rad)",
		declination_rad * RAD_TO_DEG, declination_rad);
}

bool mag_is_healthy(void)
{
	return (s_mag_type != MAG_TYPE_UNKNOWN && s_consecutive_errors < MAX_ALLOWED_I2C_ERRORS);
}

mag_type_t mag_get_type(void)
{
	return s_mag_type;
}
