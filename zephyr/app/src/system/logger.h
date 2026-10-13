/*
 * logger.h - High-Speed Flight Blackbox Data Logger via FatFS and SPI SD Card
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Peripheral: SPI0 @ 0x10000600 (SDHC-SPI Mode)
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <stdbool.h>
#include <stdint.h>
#include "types.h"
#include "config.h"

#define BLACKBOX_SYNC_MAGIC           0xB5

/*
 * Compact 44-byte binary Blackbox log frame.
 * All angles and rates are scaled to int16_t to minimize SPI write bandwidth
 * while preserving 0.01 degree / 0.1 dps resolution.
 */
typedef struct __attribute__((packed)) {
	uint8_t  sync_byte;           /* 0xB5 frame alignment marker */
	uint32_t timestamp_us;        /* Microsecond acquisition timestamp */

	/* Target Setpoints */
	int16_t  target_roll_cdeg;    /* Target Roll in centidegrees (deg * 100, +/- 3000) */
	int16_t  target_pitch_cdeg;   /* Target Pitch in centidegrees (deg * 100, +/- 3000) */
	int16_t  target_yaw_dps10;    /* Target Yaw rate in 0.1 deg/s (+/- 1800) */
	uint16_t target_throttle;     /* Normalized throttle scaled [0..1000] */

	/* Estimated Attitude & Angular Rates */
	int16_t  actual_roll_cdeg;    /* Estimated Roll in centidegrees (deg * 100) */
	int16_t  actual_pitch_cdeg;   /* Estimated Pitch in centidegrees (deg * 100) */
	int16_t  actual_yaw_cdeg;     /* Estimated Yaw in centidegrees (deg * 100) */
	int16_t  gyro_x_dps10;        /* Gyro X rate in 0.1 deg/s */
	int16_t  gyro_y_dps10;        /* Gyro Y rate in 0.1 deg/s */
	int16_t  gyro_z_dps10;        /* Gyro Z rate in 0.1 deg/s */

	/* Inner Rate PID Output Torques */
	int16_t  pid_out_roll;        /* Roll torque output scaled [-1000..+1000] */
	int16_t  pid_out_pitch;       /* Pitch torque output scaled [-1000..+1000] */
	int16_t  pid_out_yaw;         /* Yaw torque output scaled [-1000..+1000] */

	/* Actuator Outputs */
	uint16_t motor_pulse_us[4];   /* Motors 1-4 PWM pulse width in microseconds (1000..2000) */

	/* System Health & State */
	uint16_t battery_mv;          /* Battery pack voltage in millivolts */
	uint8_t  flight_state;        /* Flight state enum (DISARMED, ARMED, FAILSAFE) */
	uint8_t  flight_mode;         /* Flight mode enum (RATE, ANGLE, ALT_HOLD) */
	uint8_t  failsafe_flags;      /* Active failsafe reason mask */
} blackbox_frame_t;

/**
 * @brief Initialize the SD card filesystem, mount FatFS at /SD:, create
 * a new sequentially numbered log file, and spawn the sd_writer_thread.
 *
 * @return 0 on success, negative error code on failure.
 */
int logger_init(void);

/**
 * @brief Push a binary frame into the lock-free logging ring buffer.
 * Non-blocking: returns immediately if ring buffer is full (drops frame
 * rather than delaying the real-time flight control loop).
 *
 * @param frame Pointer to populated blackbox_frame_t structure.
 */
void api_log_push(const blackbox_frame_t *frame);

/**
 * @brief Push a binary frame into the lock-free logging ring buffer.
 *
 * @param frame Pointer to populated blackbox_frame_t structure.
 * @return 0 on success, negative error code if ring buffer is full.
 */
int logger_push_frame(const blackbox_frame_t *frame);

/**
 * @brief Helper to pack flight parameters into a blackbox frame and push to logger.
 *
 * @param timestamp_us Current uptime in microseconds.
 * @param sp Pointer to active flight setpoints.
 * @param att Pointer to estimated attitude angles (radians).
 * @param gyro Pointer to body angular rates (rad/s).
 * @param pid_roll Normalized PID roll effort [-1.0, +1.0].
 * @param pid_pitch Normalized PID pitch effort [-1.0, +1.0].
 * @param pid_yaw Normalized PID yaw effort [-1.0, +1.0].
 * @param motors Pointer to active motor PWM outputs.
 * @param battery_v Current battery voltage in Volts.
 * @param state Current flight state.
 * @param mode Current flight mode.
 * @param failsafe Active failsafe mask.
 */
void logger_log_flight_data(uint32_t timestamp_us,
			    const flight_setpoints_t *sp,
			    const euler_angles_t *att,
			    const vector3f_t *gyro,
			    float pid_roll, float pid_pitch, float pid_yaw,
			    const motor_output_t *motors,
			    float battery_v,
			    uint8_t state, uint8_t mode, uint8_t failsafe);

/**
 * @brief Force flush and sync all buffered data to the physical SD card.
 */
void logger_sync(void);

/**
 * @brief Gracefully close the active log file on vehicle disarm or shutdown.
 */
void logger_close(void);

/**
 * @brief Check if SD card is mounted and blackbox logging is active.
 *
 * @return true if logger is actively recording, false otherwise.
 */
bool logger_is_active(void);

/**
 * @brief Get total number of frames successfully written to SD card.
 *
 * @return Total written frames count.
 */
uint32_t logger_get_written_frames(void);

/**
 * @brief Get total number of frames dropped due to buffer overrun.
 *
 * @return Total dropped frames count.
 */
uint32_t logger_get_dropped_frames(void);

#endif /* LOGGER_H_ */
