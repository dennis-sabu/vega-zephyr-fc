/*
 * mpu6050.c - Hardware Driver Implementation for InvenSense MPU6050 6-DOF IMU
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 *
 * Strict Driver Constraint:
 * The out-of-tree i2c_thejas32 hardware FIFO has a maximum single burst read
 * limit of 16 bytes. All transactions here are strictly <= 16 bytes (14 bytes burst).
 */

#include "mpu6050.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

LOG_MODULE_REGISTER(mpu6050, CONFIG_LOG_DEFAULT_LEVEL);

/* Device Tree Node Resolution */
#if DT_NODE_EXISTS(DT_ALIAS(imu_i2c))
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_ALIAS(imu_i2c));
#elif DT_NODE_EXISTS(DT_NODELABEL(i2c0))
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
#else
#error "DeviceTree error: Neither alias 'imu_i2c' nor nodelabel 'i2c0' found."
#endif

/* Calibration storage */
static mpu6050_calib_t s_calib = {
	.accel_offset = {0.0f, 0.0f, 0.0f},
	.gyro_bias = {0.0f, 0.0f, 0.0f},
	.accel_scale = {1.0f, 1.0f, 1.0f},
	.is_calibrated = false
};

static uint32_t s_consecutive_errors = 0;
#define MAX_ALLOWED_I2C_ERRORS 5

/* Helper: Write single register */
static int mpu6050_write_reg(uint8_t reg, uint8_t val)
{
	uint8_t tx_buf[2] = {reg, val};
	int ret = i2c_write(i2c_dev, tx_buf, sizeof(tx_buf), MPU6050_I2C_ADDR);
	if (ret < 0) {
		LOG_ERR("Failed to write reg 0x%02X (err %d)", reg, ret);
		s_consecutive_errors++;
		return ret;
	}
	s_consecutive_errors = 0;
	return 0;
}

/* Helper: Read single register */
static int mpu6050_read_reg(uint8_t reg, uint8_t *val)
{
	int ret = i2c_write_read(i2c_dev, MPU6050_I2C_ADDR, &reg, 1, val, 1);
	if (ret < 0) {
		LOG_ERR("Failed to read reg 0x%02X (err %d)", reg, ret);
		s_consecutive_errors++;
		return ret;
	}
	s_consecutive_errors = 0;
	return 0;
}

int mpu6050_init(void)
{
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C0 peripheral not ready on THEJAS32!");
		return -ENODEV;
	}

	LOG_INF("Initializing MPU6050 on I2C0 (addr 0x%02X)...", MPU6050_I2C_ADDR);

	/* 1. Reset device */
	int ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x80);
	if (ret < 0) {
		return ret;
	}
	k_msleep(100);

	/* 2. Verify Device Identity */
	uint8_t who_am_i = 0;
	ret = mpu6050_read_reg(MPU6050_REG_WHO_AM_I, &who_am_i);
	if (ret < 0 || who_am_i != MPU6050_WHO_AM_I_VAL) {
		LOG_ERR("MPU6050 WHO_AM_I mismatch! Expected 0x%02X, got 0x%02X",
			MPU6050_WHO_AM_I_VAL, who_am_i);
		return -ENODEV;
	}
	LOG_INF("MPU6050 identity verified: 0x%02X", who_am_i);

	/* 3. Wake up and select Gyro X PLL clock source (0x01) */
	ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01);
	if (ret < 0) {
		return ret;
	}
	k_msleep(15);

	/* 4. Configure Digital Low Pass Filter (DLPF = 42 Hz) */
	ret = mpu6050_write_reg(MPU6050_REG_CONFIG, MPU6050_DLPF_42HZ);
	if (ret < 0) {
		return ret;
	}

	/* 5. Set Sample Rate: 1 kHz / (1 + SMPLRT_DIV) = 500 Hz (SMPLRT_DIV = 1) */
	ret = mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, 0x01);
	if (ret < 0) {
		return ret;
	}

	/* 6. Configure Gyroscope Full Scale (+/- 1000 deg/s) */
	ret = mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_1000DPS);
	if (ret < 0) {
		return ret;
	}

	/* 7. Configure Accelerometer Full Scale (+/- 8g) */
	ret = mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_8G);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("MPU6050 configured successfully at 500 Hz DLPF 42 Hz.");
	return 0;
}

int mpu6050_read_raw(mpu6050_raw_t *raw)
{
	if (raw == NULL) {
		return -EINVAL;
	}

	/*
	 * Strict hardware compliance:
	 * Burst read of 14 bytes (0x3B to 0x48).
	 * 14 bytes <= 16 bytes THEJAS32 hardware limit.
	 */
	uint8_t start_reg = MPU6050_REG_ACCEL_XOUT_H;
	uint8_t buffer[MPU6050_BURST_READ_SIZE];

	int ret = i2c_write_read(i2c_dev, MPU6050_I2C_ADDR, &start_reg, 1,
				 buffer, MPU6050_BURST_READ_SIZE);
	if (ret < 0) {
		s_consecutive_errors++;
		return ret;
	}

	s_consecutive_errors = 0;

	raw->accel_x = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
	raw->accel_y = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
	raw->accel_z = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);
	raw->temp    = (int16_t)(((uint16_t)buffer[6] << 8) | buffer[7]);
	raw->gyro_x  = (int16_t)(((uint16_t)buffer[8] << 8) | buffer[9]);
	raw->gyro_y  = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
	raw->gyro_z  = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);

	return 0;
}

int mpu6050_read(mpu6050_data_t *data)
{
	if (data == NULL) {
		return -EINVAL;
	}

	mpu6050_raw_t raw;
	int ret = mpu6050_read_raw(&raw);
	if (ret < 0) {
		return ret;
	}

	data->timestamp_us = (uint32_t)k_uptime_get_us();

	/* Die temperature: T = (raw / 340.0) + 36.53 C */
	data->temperature_c = ((float)raw.temp / 340.0f) + 36.53f;

	if (s_calib.is_calibrated) {
		/* Apply accelerometer offset & scale */
		float ax = ((float)raw.accel_x - s_calib.accel_offset.x) * ACCEL_SCALE_FACTOR_8G;
		float ay = ((float)raw.accel_y - s_calib.accel_offset.y) * ACCEL_SCALE_FACTOR_8G;
		float az = ((float)raw.accel_z - s_calib.accel_offset.z) * ACCEL_SCALE_FACTOR_8G;

		data->accel.x = ax * s_calib.accel_scale.x;
		data->accel.y = ay * s_calib.accel_scale.y;
		data->accel.z = az * s_calib.accel_scale.z;

		/* Apply gyroscope bias subtraction */
		data->gyro.x = ((float)raw.gyro_x - s_calib.gyro_bias.x) * GYRO_SCALE_FACTOR_1000DPS;
		data->gyro.y = ((float)raw.gyro_y - s_calib.gyro_bias.y) * GYRO_SCALE_FACTOR_1000DPS;
		data->gyro.z = ((float)raw.gyro_z - s_calib.gyro_bias.z) * GYRO_SCALE_FACTOR_1000DPS;
	} else {
		/* Uncalibrated raw conversions */
		data->accel.x = (float)raw.accel_x * ACCEL_SCALE_FACTOR_8G;
		data->accel.y = (float)raw.accel_y * ACCEL_SCALE_FACTOR_8G;
		data->accel.z = (float)raw.accel_z * ACCEL_SCALE_FACTOR_8G;

		data->gyro.x = (float)raw.gyro_x * GYRO_SCALE_FACTOR_1000DPS;
		data->gyro.y = (float)raw.gyro_y * GYRO_SCALE_FACTOR_1000DPS;
		data->gyro.z = (float)raw.gyro_z * GYRO_SCALE_FACTOR_1000DPS;
	}

	return 0;
}

int mpu6050_calibrate(mpu6050_calib_t *calib)
{
	LOG_INF("Calibrating MPU6050 on level surface (%d samples)... Keep stationary!",
		IMU_CALIB_SAMPLES);

	int64_t ax_sum = 0, ay_sum = 0, az_sum = 0;
	int64_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

	/* Discard initial 100 samples to let filter settle */
	for (int i = 0; i < 100; i++) {
		mpu6050_raw_t dummy;
		mpu6050_read_raw(&dummy);
		k_msleep(2);
	}

	int valid_samples = 0;
	for (int i = 0; i < IMU_CALIB_SAMPLES; i++) {
		mpu6050_raw_t raw;
		int ret = mpu6050_read_raw(&raw);
		if (ret == 0) {
			ax_sum += raw.accel_x;
			ay_sum += raw.accel_y;
			az_sum += raw.accel_z;
			gx_sum += raw.gyro_x;
			gy_sum += raw.gyro_y;
			gz_sum += raw.gyro_z;
			valid_samples++;
		}
		k_usleep(2000); /* 500 Hz sample spacing */
	}

	if (valid_samples < (IMU_CALIB_SAMPLES * 8 / 10)) {
		LOG_ERR("Calibration failed: insufficient valid samples (%d)", valid_samples);
		return -EIO;
	}

	float ax_mean = (float)ax_sum / (float)valid_samples;
	float ay_mean = (float)ay_sum / (float)valid_samples;
	float az_mean = (float)az_sum / (float)valid_samples;

	/* Gyro zero-rate bias */
	s_calib.gyro_bias.x = (float)gx_sum / (float)valid_samples;
	s_calib.gyro_bias.y = (float)gy_sum / (float)valid_samples;
	s_calib.gyro_bias.z = (float)gz_sum / (float)valid_samples;

	/* Accelerometer level offsets (Z axis experiences +1g = 4096 LSB at 8g range) */
	s_calib.accel_offset.x = ax_mean;
	s_calib.accel_offset.y = ay_mean;
	s_calib.accel_offset.z = az_mean - 4096.0f; /* 4096 LSB = 1g */

	s_calib.accel_scale.x = 1.0f;
	s_calib.accel_scale.y = 1.0f;
	s_calib.accel_scale.z = 1.0f;
	s_calib.is_calibrated = true;

	if (calib != NULL) {
		*calib = s_calib;
	}

	LOG_INF("IMU Calibration complete: Gyro Bias=[%.1f, %.1f, %.1f], Accel Offset=[%.1f, %.1f, %.1f]",
		s_calib.gyro_bias.x, s_calib.gyro_bias.y, s_calib.gyro_bias.z,
		s_calib.accel_offset.x, s_calib.accel_offset.y, s_calib.accel_offset.z);

	return 0;
}

void mpu6050_set_calibration(const mpu6050_calib_t *calib)
{
	if (calib != NULL) {
		s_calib = *calib;
		s_calib.is_calibrated = true;
	}
}

const mpu6050_calib_t* mpu6050_get_calibration(void)
{
	return &s_calib;
}

bool mpu6050_is_healthy(void)
{
	return (s_consecutive_errors < MAX_ALLOWED_I2C_ERRORS);
}
