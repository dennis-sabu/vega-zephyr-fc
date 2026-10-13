/*
 * types.h - Primary Data Type Definitions for VEGA ARIES v3 S500 Flight Controller
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Zephyr RTOS Application
 */

#ifndef TYPES_H_
#define TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/time_units.h>

static inline int64_t k_uptime_get_us(void)
{
	return k_ticks_to_us_floor64(k_uptime_ticks());
}

/* -------------------------------------------------------------------------- */
/* Mathematical Vector and Attitude Representations                           */
/* -------------------------------------------------------------------------- */

typedef struct {
	float x;
	float y;
	float z;
} vector3f_t;

typedef struct {
	float w;
	float x;
	float y;
	float z;
} quaternion_t;

typedef struct {
	float roll;   /* Radians or degrees (context defined) */
	float pitch;  /* Radians or degrees */
	float yaw;    /* Radians or degrees */
} euler_angles_t;

/* -------------------------------------------------------------------------- */
/* System States and Flight Modes                                             */
/* -------------------------------------------------------------------------- */

typedef enum {
	STATE_DISARMED = 0,
	STATE_CALIBRATING = 1,
	STATE_ARMED = 2,
	STATE_FAILSAFE = 3,
	STATE_EMERGENCY_STOP = 4
} flight_state_t;

typedef enum {
	MODE_RATE     = 0,   /* Acrobatic mode: direct rate gyro PID control */
	MODE_ANGLE    = 1,   /* Stabilize mode: outer angle PID + inner rate PID */
	MODE_ALT_HOLD = 2    /* Altitude hold mode: cascaded altitude PID controls collective throttle */
} flight_mode_t;

typedef enum {
	BATTERY_OK = 0,
	BATTERY_WARNING = 1,
	BATTERY_CRITICAL = 2
} battery_status_t;

typedef enum {
	FAILSAFE_REASON_NONE           = 0,
	FAILSAFE_REASON_RC_LOST        = (1 << 0),
	FAILSAFE_REASON_LOW_BATTERY    = (1 << 1),
	FAILSAFE_REASON_EXCESSIVE_TILT = (1 << 2),
	FAILSAFE_REASON_IMU_FAILURE    = (1 << 3),
	FAILSAFE_REASON_MANUAL_ABORT   = (1 << 4)
} failsafe_reason_t;

/* -------------------------------------------------------------------------- */
/* Sensor Data Structures                                                     */
/* -------------------------------------------------------------------------- */

typedef struct {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temp;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} mpu6050_raw_t;

typedef struct {
	vector3f_t accel_offset;
	vector3f_t gyro_bias;
	vector3f_t accel_scale;
	bool is_calibrated;
} mpu6050_calib_t;

typedef struct {
	vector3f_t accel;       /* Accelerations in m/s^2 (Body frame) */
	vector3f_t gyro;        /* Angular rates in rad/s (Body frame) */
	float temperature_c;    /* Die temperature in degrees Celsius */
	uint32_t timestamp_us;  /* Acquisition time in microseconds */
} mpu6050_data_t;

/* -------------------------------------------------------------------------- */
/* Remote Control (RC) / SBUS Data Structures                                 */
/* -------------------------------------------------------------------------- */

typedef struct {
	uint16_t raw_channels[16];  /* 11-bit raw channels (172..1811) */
	float roll_norm;            /* Scaled [-1.0f, +1.0f] */
	float pitch_norm;           /* Scaled [-1.0f, +1.0f] */
	float yaw_norm;             /* Scaled [-1.0f, +1.0f] */
	float throttle_norm;        /* Scaled [0.0f, +1.0f] */
	bool arm_switch;            /* True = Armed requested */
	flight_mode_t mode_switch;  /* MODE_ANGLE, MODE_RATE, or MODE_ALT_HOLD */
	bool frame_lost;            /* SBUS frame lost flag */
	bool failsafe_active;       /* SBUS hardware failsafe flag */
	uint32_t last_packet_time_ms;
} sbus_channels_t;
typedef sbus_channels_t rc_channels_t;

/* Thread-safe normalized flight setpoints exchanged via mutex */
typedef struct {
	float target_roll_deg;       /* [-30.0, +30.0] in Angle mode */
	float target_pitch_deg;      /* [-30.0, +30.0] in Angle mode */
	float target_yaw_rate_dps;   /* [-180.0, +180.0] deg/s */
	float throttle;              /* [0.0, 1.0] */
	bool arm_switch;
	flight_mode_t flight_mode;
	bool signal_valid;
	uint32_t timestamp_ms;
} flight_setpoints_t;

/* -------------------------------------------------------------------------- */
/* PID Controller Data Structures                                             */
/* -------------------------------------------------------------------------- */

typedef struct {
	float kp;
	float ki;
	float kd;
	float integrator;
	float prev_measurement;
	float prev_error;
	float prev_derivative;
	float out_min;
	float out_max;
	float int_min;
	float int_max;
	float d_beta;               /* Filter factor beta in D[k] = beta*D[k-1] + (1-beta)*D_raw */
	bool derivative_on_measurement;
} pid_controller_t;

/* -------------------------------------------------------------------------- */
/* Actuator & Mixer Data Structures                                           */
/* -------------------------------------------------------------------------- */

typedef struct {
	float norm[4];              /* Normalized demands [0.0f, 1.0f] for Motors 1-4 */
	uint32_t pulse_us[4];       /* Microsecond pulse widths [1000..2000] (or 900 if disarmed) */
} motor_output_t;

/* -------------------------------------------------------------------------- */
/* Battery & Power Management                                                 */
/* -------------------------------------------------------------------------- */

typedef struct {
	float voltage;              /* Pack voltage in Volts */
	float cell_voltage;         /* Average voltage per cell in Volts */
	uint8_t cell_count;         /* 3 for 3S, 4 for 4S */
	battery_status_t status;
	uint16_t raw_adc;
} battery_info_t;

/* -------------------------------------------------------------------------- */
/* Safety & Failsafe Management                                               */
/* -------------------------------------------------------------------------- */

typedef struct {
	bool active;
	uint32_t reason_mask;
	uint32_t trigger_timestamp_ms;
	float descent_throttle;
} failsafe_state_t;

/* -------------------------------------------------------------------------- */
/* Telemetry Stream Structure                                                 */
/* -------------------------------------------------------------------------- */

typedef struct __attribute__((packed)) {
	uint32_t timestamp_ms;
	float roll_deg;
	float pitch_deg;
	float yaw_deg;
	float roll_rate_dps;
	float pitch_rate_dps;
	float yaw_rate_dps;
	float throttle_pct;
	uint16_t motor_pulse_us[4];
	float battery_voltage;
	uint8_t flight_state;
	uint8_t flight_mode;
	uint8_t failsafe_reason;
	float altitude_m;
	float climb_rate_mps;
	float mag_heading_deg;
	double latitude;
	double longitude;
	uint8_t gps_sats;
	uint8_t gps_fix;
} telemetry_packet_t;

#endif /* TYPES_H_ */
