/*
 * mag.h - Hardware Driver for 3-Axis Magnetometer (QMC5883L / HMC5883L)
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Strict requirement: Single burst read <= 16 bytes (i2c_thejas32 limitation)
 */

#ifndef MAG_H_
#define MAG_H_

#include <stdbool.h>
#include <stdint.h>
#include "types.h"

/* Supported I2C Addresses */
#define QMC5883L_I2C_ADDR           0x0D    /* Primary address for modern QMC modules */
#define HMC5883L_I2C_ADDR           0x1E    /* Standard Honeywell HMC5883L address */

/* Sensor Type Identifier */
typedef enum {
	MAG_TYPE_UNKNOWN = 0,
	MAG_TYPE_QMC5883L = 1,
	MAG_TYPE_HMC5883L = 2
} mag_type_t;

/* Hard-Iron and Soft-Iron Calibration Parameters */
typedef struct {
	vector3f_t offset;          /* Hard-iron bias (subtracted from raw) */
	vector3f_t scale;           /* Soft-iron scale multipliers */
	float declination_rad;      /* Local magnetic declination angle in radians */
	bool is_calibrated;
} mag_calib_t;

/* Magnetometer Processed Output */
typedef struct {
	vector3f_t raw;             /* Raw uncalibrated sensor vectors (Gauss) */
	vector3f_t cal;             /* Hard-iron / soft-iron calibrated vectors */
	float heading_rad;          /* Tilt-compensated heading in radians [-pi, +pi] */
	float heading_deg;          /* Tilt-compensated heading in degrees [0, 360) */
	uint32_t timestamp_us;      /* Acquisition timestamp */
	bool is_healthy;            /* Sensor communication health flag */
} mag_data_t;

/**
 * @brief Initialize the magnetometer sensor on I2C0.
 * Auto-probes for QMC5883L (@ 0x0D) and HMC5883L (@ 0x1E), configures
 * continuous measurement mode, and initializes calibration parameters.
 *
 * @return 0 on success, negative error code on failure.
 */
int mag_init(void);

/**
 * @brief Read magnetic vectors and compute tilt-compensated heading.
 *
 * @param roll_rad Current airframe roll angle in radians from AHRS.
 * @param pitch_rad Current airframe pitch angle in radians from AHRS.
 * @param data Pointer to output mag_data_t structure.
 * @return 0 on success, negative error code on failure.
 */
int mag_read(float roll_rad, float pitch_rad, mag_data_t *data);

/**
 * @brief Set hard-iron offsets and soft-iron scale factors.
 *
 * @param calib Pointer to mag_calib_t configuration structure.
 */
void mag_set_calibration(const mag_calib_t *calib);

/**
 * @brief Set local magnetic declination angle in radians or degrees.
 *
 * @param declination_rad Magnetic declination angle in radians.
 */
void mag_set_declination(float declination_rad);

/**
 * @brief Check if magnetometer communication is healthy.
 *
 * @return true if healthy and responsive, false otherwise.
 */
bool mag_is_healthy(void);

/**
 * @brief Get the identified magnetometer sensor type.
 *
 * @return mag_type_t (QMC5883L, HMC5883L, or UNKNOWN).
 */
mag_type_t mag_get_type(void);

#endif /* MAG_H_ */
