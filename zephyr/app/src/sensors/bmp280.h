/*
 * bmp280.h - Hardware Driver for Bosch BMP280 Digital Barometric Pressure Sensor
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Strict requirement: Single burst read <= 16 bytes (i2c_thejas32 limitation)
 */

#ifndef BMP280_H_
#define BMP280_H_

#include <stdbool.h>
#include <stdint.h>
#include "types.h"

/* I2C Address Options */
#define BMP280_I2C_ADDR_PRIMARY     0x76    /* SDO connected to GND */
#define BMP280_I2C_ADDR_SECONDARY   0x77    /* SDO connected to VDD */

/* Register Addresses */
#define BMP280_REG_CALIB_00         0x88    /* Start of 24-byte calibration data */
#define BMP280_REG_ID               0xD0    /* Chip ID register */
#define BMP280_REG_RESET            0xE0    /* Soft reset register */
#define BMP280_REG_STATUS           0xF3    /* Status register */
#define BMP280_REG_CTRL_MEAS        0xF4    /* Control measurement register */
#define BMP280_REG_CONFIG           0xF5    /* Configuration register */
#define BMP280_REG_PRESS_MSB        0xF7    /* Pressure MSB */
#define BMP280_REG_PRESS_LSB        0xF8    /* Pressure LSB */
#define BMP280_REG_PRESS_XLSB       0xF9    /* Pressure XLSB */
#define BMP280_REG_TEMP_MSB         0xFA    /* Temperature MSB */
#define BMP280_REG_TEMP_LSB         0xFB    /* Temperature LSB */
#define BMP280_REG_TEMP_XLSB        0xFC    /* Temperature XLSB */

/* Expected Chip IDs */
#define BMP280_CHIP_ID1             0x58
#define BMP280_CHIP_ID2             0x56
#define BMP280_CHIP_ID3             0x57
#define BME280_CHIP_ID              0x60

/* Standard Atmospheric Constants */
#define STANDARD_SEA_LEVEL_PA       101325.0f /* 1013.25 hPa in Pascals */

/* Trimming Calibration Parameters */
typedef struct {
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
} bmp280_calib_param_t;

/* Barometer Measurement Data */
typedef struct {
	float pressure_pa;          /* Pressure in Pascals */
	float temperature_c;        /* Temperature in degrees Celsius */
	float altitude_abs_m;       /* Absolute altitude above sea level in meters */
	float altitude_rel_m;       /* Relative altitude above ground takeoff in meters */
	float altitude_filtered_m;  /* Low-pass filtered relative altitude */
	float climb_rate_mps;       /* Vertical velocity in m/s (variometer) */
	uint32_t timestamp_us;      /* Measurement timestamp */
	bool is_healthy;            /* Sensor communication health flag */
} bmp280_data_t;

/**
 * @brief Initialize the BMP280 barometric pressure sensor on I2C0.
 * Auto-detects primary (0x76) or secondary (0x77) address, reads PROM
 * calibration coefficients in two <= 16-byte bursts, and sets oversampling.
 *
 * @return 0 on success, negative error code on failure.
 */
int bmp280_init(void);

/**
 * @brief Read temperature and pressure, calculate compensated values,
 * compute hypsometric altitude, and apply low-pass filtering.
 *
 * @param data Pointer to output bmp280_data_t structure.
 * @return 0 on success, negative error code on failure.
 */
int bmp280_read(bmp280_data_t *data);

/**
 * @brief Calibrate ground reference altitude by averaging stationary samples.
 *
 * @param sample_count Number of baseline samples to average (e.g. 50).
 * @return 0 on success, negative error code on failure.
 */
int bmp280_calibrate_ground(uint32_t sample_count);

/**
 * @brief Set ground reference pressure in Pascals.
 *
 * @param ground_pa Reference ground pressure.
 */
void bmp280_set_ground_reference(float ground_pa);

/**
 * @brief Check if BMP280 communication is healthy.
 *
 * @return true if sensor is communicating properly, false otherwise.
 */
bool bmp280_is_healthy(void);

#endif /* BMP280_H_ */
