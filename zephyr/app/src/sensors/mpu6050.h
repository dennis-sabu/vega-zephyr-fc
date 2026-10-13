/*
 * mpu6050.h - Hardware Driver for InvenSense MPU6050 6-DOF IMU
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Strict requirement: Single burst read <= 16 bytes (i2c_thejas32 limitation)
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include "types.h"
#include "config.h"

/* Register Map */
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_INT_PIN_CFG     0x37
#define MPU6050_REG_INT_ENABLE      0x38
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_ACCEL_XOUT_L    0x3C
#define MPU6050_REG_ACCEL_YOUT_H    0x3D
#define MPU6050_REG_ACCEL_YOUT_L    0x3E
#define MPU6050_REG_ACCEL_ZOUT_H    0x3F
#define MPU6050_REG_ACCEL_ZOUT_L    0x40
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_TEMP_OUT_L      0x42
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_GYRO_XOUT_L     0x44
#define MPU6050_REG_GYRO_YOUT_H     0x45
#define MPU6050_REG_GYRO_YOUT_L     0x46
#define MPU6050_REG_GYRO_ZOUT_H     0x47
#define MPU6050_REG_GYRO_ZOUT_L     0x48
#define MPU6050_REG_USER_CTRL       0x6A
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_PWR_MGMT_2      0x6C
#define MPU6050_REG_WHO_AM_I        0x75

/* Expected Identity */
#define MPU6050_WHO_AM_I_VAL        0x68

/**
 * @brief Initialize the MPU6050 sensor on the THEJAS32 I2C0 bus.
 * Configures DLPF (42 Hz), Gyro (+/- 1000 dps), Accel (+/- 8g), and 500 Hz sample rate.
 *
 * @return 0 on success, negative error code on failure.
 */
int mpu6050_init(void);

/**
 * @brief Read 14 bytes of raw sensor data in a single burst <= 16 bytes.
 *
 * @param raw Pointer to mpu6050_raw_t structure to store raw registers.
 * @return 0 on success, negative error code on failure.
 */
int mpu6050_read_raw(mpu6050_raw_t *raw);

/**
 * @brief Read and scale calibrated physical values (rad/s, m/s^2, degrees C).
 *
 * @param data Pointer to mpu6050_data_t output structure.
 * @return 0 on success, negative error code on failure.
 */
int mpu6050_read(mpu6050_data_t *data);

/**
 * @brief Perform static on-ground zero-bias calibration (1000 samples).
 *
 * @param calib Pointer to calibration structure to store computed offsets.
 * @return 0 on success, negative error code on failure.
 */
int mpu6050_calibrate(mpu6050_calib_t *calib);

/**
 * @brief Set the active calibration parameters.
 *
 * @param calib Pointer to pre-computed calibration structure.
 */
void mpu6050_set_calibration(const mpu6050_calib_t *calib);

/**
 * @brief Get the currently active calibration parameters.
 *
 * @return Pointer to current calibration structure.
 */
const mpu6050_calib_t* mpu6050_get_calibration(void);

/**
 * @brief Check health status of IMU communication and integrity.
 *
 * @return true if sensor is healthy and responding, false otherwise.
 */
bool mpu6050_is_healthy(void);

#endif /* MPU6050_H_ */
