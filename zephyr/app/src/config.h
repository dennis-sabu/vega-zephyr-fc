/*
 * config.h - System Configuration, Pinouts, Limits, and Flight Dynamics Constants
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Airframe: S500 Quadcopter (~2.5 kg AUW, Quad-X Layout)
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/* Hardware Memory & RTOS Thread Configuration                                */
/* ========================================================================== */

#define FLIGHT_THREAD_STACK_SIZE       4096
#define ALTITUDE_THREAD_STACK_SIZE     2048
#define RC_THREAD_STACK_SIZE           2048
#define TELEMETRY_THREAD_STACK_SIZE    2048

/* Thread Priorities (Preemptive) */
#define FLIGHT_THREAD_PRIORITY         1    /* K_PRIO_PREEMPT(1) - 250 Hz flight loop */
#define ALTITUDE_THREAD_PRIORITY       2    /* K_PRIO_PREEMPT(2) - 50 Hz altitude loop */
#define RC_THREAD_PRIORITY             3    /* K_PRIO_PREEMPT(3) - 100 Hz SBUS rx */
#define TELEMETRY_THREAD_PRIORITY      6    /* K_PRIO_PREEMPT(6) - 10 Hz housekeeping */

/* Flight Loop Execution Frequency (250 Hz / 4 ms) */
#define FLIGHT_LOOP_FREQ_HZ            250
#define FLIGHT_LOOP_PERIOD_US          (1000000 / FLIGHT_LOOP_FREQ_HZ) /* 4000 us */
#define FLIGHT_LOOP_DT_S               (1.0f / (float)FLIGHT_LOOP_FREQ_HZ) /* 0.004 s */

/* Altitude Control Loop Frequency (50 Hz / 20 ms) */
#define ALTITUDE_LOOP_FREQ_HZ          50
#define ALTITUDE_LOOP_PERIOD_MS        (1000 / ALTITUDE_LOOP_FREQ_HZ)  /* 20 ms */
#define ALTITUDE_LOOP_DT_S             (1.0f / (float)ALTITUDE_LOOP_FREQ_HZ) /* 0.02 s */

/* Background Subsystem Rates */
#define RC_THREAD_PERIOD_MS            10   /* 10 ms / 100 Hz */
#define TELEMETRY_RATE_HZ              10   /* 10 Hz */
#define TELEMETRY_PERIOD_MS            (1000 / TELEMETRY_RATE_HZ)      /* 100 ms */
#define BATTERY_RATE_HZ                10   /* 10 Hz */
#define BATTERY_PERIOD_MS              (1000 / BATTERY_RATE_HZ)        /* 100 ms */

/* ========================================================================== */
/* MPU6050 Inertial Measurement Unit & I2C0 Bus                               */
/* ========================================================================== */

#define MPU6050_I2C_ADDR               0x68

/* Strict C-DAC THEJAS32 i2c_thejas32 hardware limit: max burst read is 16 bytes */
#define THEJAS32_I2C_MAX_BURST_LEN     16
#define MPU6050_BURST_READ_SIZE        14   /* Accel(6) + Temp(2) + Gyro(6) = 14 bytes */

/* Full Scale Range Configurations */
#define MPU6050_ACCEL_FS_8G            0x10 /* +/- 8g range (4096 LSB/g) */
#define MPU6050_GYRO_FS_1000DPS        0x10 /* +/- 1000 deg/s (32.8 LSB/(deg/s)) */
#define MPU6050_DLPF_42HZ              0x03 /* DLPF 42 Hz bandwidth */

/* Physical Unit Conversion Constants */
#define GRAVITY_MSS                    9.80665f
#define ACCEL_SCALE_FACTOR_8G          (GRAVITY_MSS / 4096.0f)           /* m/s^2 per LSB */
#define GYRO_SCALE_FACTOR_1000DPS      (0.017453292519943295f / 32.8f)  /* rad/s per LSB */
#define GYRO_SCALE_FACTOR_DEG          (1.0f / 32.8f)                   /* deg/s per LSB */

/* IMU Calibration Parameters (Startup stationary bias) */
#define IMU_CALIB_SAMPLES              500

/* ========================================================================== */
/* Attitude Filter (Complementary Filter) Parameters                          */
/* ========================================================================== */

#define COMPLEMENTARY_ALPHA            0.980f

/* ========================================================================== */
/* Barometer (BMP280) & Magnetometer (QMC5883L/HMC5883L) Parameters           */
/* ========================================================================== */

#define BMP280_POLL_RATE_HZ            50
#define MAG_POLL_RATE_HZ               50
#define DEFAULT_MAG_DECLINATION_DEG    0.5f  /* Configurable local magnetic declination */

/* ========================================================================== */
/* GNSS Receiver (UART0 @ 0x10000100) Parameters                              */
/* ========================================================================== */

#define GNSS_BAUD_RATE                 9600  /* Standard NMEA baud rate */

/* ========================================================================== */
/* PWM0 & Motor ESC Output (Quad-X S500 Airframe)                             */
/* ========================================================================== */

#define THEJAS32_PWM0_BASE_ADDR        0x10400000
#define THEJAS32_PWM_GLOBAL_ENABLE_REG 0x80     /* Global enable register offset */
#define THEJAS32_PWM_ENABLE_BIT        (1 << 0)

/* 400 Hz Standard Multirotor ESC Timing */
#define PWM_FREQ_HZ                    400
#define PWM_PERIOD_NS                  2500000  /* 2.5 ms = 2,500,000 ns */
#define PWM_PERIOD_US                  2500

/* ESC Pulse Width Specifications (Microseconds) */
#define PWM_PULSE_DISARMED_US          900      /* Forced when disarmed */
#define PWM_PULSE_MIN_US               1000     /* 1000 us idle minimum */
#define PWM_PULSE_ARMED_IDLE_US        1100     /* Minimum spin when armed */
#define PWM_PULSE_MAX_US               2000     /* 2000 us maximum throttle */

/* Motor Hardware Channel Assignments */
#define MOTOR_CHANNEL_FR               0        /* Front-Right (CCW) -> Motor 1 -> Ch 0 */
#define MOTOR_CHANNEL_RR               1        /* Rear-Right  (CW)  -> Motor 2 -> Ch 1 */
#define MOTOR_CHANNEL_RL               2        /* Rear-Left   (CCW) -> Motor 3 -> Ch 2 */
#define MOTOR_CHANNEL_FL               3        /* Front-Left  (CW)  -> Motor 4 -> Ch 3 */

/* ========================================================================== */
/* Cascaded Dual-PID Controller Tuning (S500 Multirotor ~2.5 kg AUW)          */
/* ========================================================================== */

/* Inner Rate Loop */
#define ROLL_RATE_KP                   0.150f
#define ROLL_RATE_KI                   0.100f
#define ROLL_RATE_KD                   0.0040f

#define PITCH_RATE_KP                  0.150f
#define PITCH_RATE_KI                  0.100f
#define PITCH_RATE_KD                  0.0040f

#define YAW_RATE_KP                    0.250f
#define YAW_RATE_KI                    0.050f
#define YAW_RATE_KD                    0.0000f

/* Rate Loop Output and Integrator Clamping */
#define RATE_INTEGRATOR_MAX            0.25f
#define RATE_PID_OUTPUT_MAX            0.80f
#define PID_D_TERM_LPF_CUTOFF_HZ       30.0f

/* Outer Angle Loop */
#define ROLL_ANGLE_KP                  4.50f
#define PITCH_ANGLE_KP                 4.50f

/* Flight Envelope Limits */
#define MAX_ROLL_PITCH_ANGLE_DEG       30.0f    /* +/- 30.0 degrees */
#define MAX_ROLL_PITCH_ANGLE_RAD       0.5235988f
#define MAX_ROLL_PITCH_RATE_RAD_S      3.490659f /* 200.0 deg/s */
#define MAX_YAW_RATE_DPS               180.0f   /* +/- 180.0 deg/s */
#define MAX_YAW_RATE_RAD_S             3.14159265f

/* ========================================================================== */
/* RC Receiver & SBUS Protocol (UART1 @ 100,000 Baud, 8E2)                    */
/* ========================================================================== */

#define SBUS_FRAME_LEN                 25
#define SBUS_HEADER_BYTE               0x0F
#define SBUS_END_BYTE_STANDARD         0x00
#define SBUS_END_BYTE_FUTABA2          0x04

#define SBUS_CH_ROLL                   0
#define SBUS_CH_PITCH                  1
#define SBUS_CH_THROTTLE               2
#define SBUS_CH_YAW                    3
#define SBUS_CH_ARM                    4        /* AUX1: Arm / Disarm switch */
#define SBUS_CH_MODE                   5        /* AUX2: Flight Mode 3-pos switch */

#define SBUS_INPUT_MIN                 172
#define SBUS_INPUT_CENTER              992
#define SBUS_INPUT_MAX                 1811
#define SBUS_STICK_DEADBAND            15
#define SBUS_ARM_THRESHOLD             1400

/* Mode Switch Thresholds (3-position switch on AUX2) */
#define SBUS_MODE_ANGLE_MIN_THRESH     1300     /* 1300 to 1700 selects Angle Mode */
#define SBUS_MODE_ALT_HOLD_THRESH      1700     /* > 1700 selects Altitude Hold Mode */

#define RC_TIMEOUT_MS                  500

/* ========================================================================== */
/* Battery Monitoring & ADC0                                                  */
/* ========================================================================== */

#define BATTERY_ADC_CHANNEL            0
#define ADC_REF_VOLTAGE_V              3.3f
#define ADC_MAX_VALUE                  4095.0f

#define BATTERY_R1_OHMS                10000.0f
#define BATTERY_R2_OHMS                2200.0f
#define BATTERY_VOLTAGE_DIVIDER_RATIO  ((BATTERY_R1_OHMS + BATTERY_R2_OHMS) / BATTERY_R2_OHMS)
#define BATTERY_EMA_ALPHA              0.10f

#define LIPO_4S_CELLS                  4
#define LIPO_4S_FULL_V                 16.8f
#define LIPO_4S_WARN_V                 14.4f
#define LIPO_4S_CRIT_V                 13.2f

#define LIPO_3S_CELLS                  3
#define LIPO_3S_FULL_V                 12.6f
#define LIPO_3S_WARN_V                 10.8f
#define LIPO_3S_CRIT_V                 9.9f

#define LIPO_CELL_DETECT_THRESHOLD_V   13.0f

/* ========================================================================== */
/* Safety, Failsafe & Arming Checks                                           */
/* ========================================================================== */

#define ARMING_MAX_TILT_RAD            0.174533f /* 10.0 degrees */
#define CRASH_TILT_LIMIT_DEG           75.0f
#define CRASH_TILT_LIMIT_RAD           1.3089969f
#define FAILSAFE_DESCENT_THROTTLE      0.30f
#define FAILSAFE_DESCENT_TIME_MS       5000
#define BATTERY_CRITICAL_TIMEOUT_MS    2000

#endif /* CONFIG_H_ */
