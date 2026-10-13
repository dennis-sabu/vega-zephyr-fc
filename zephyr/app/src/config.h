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
#define RC_THREAD_STACK_SIZE           2048
#define TELEMETRY_THREAD_STACK_SIZE    2048

#define FLIGHT_THREAD_PRIORITY         1    /* Highest priority real-time loop */
#define RC_THREAD_PRIORITY             2    /* SBUS reception & channel decoding */
#define TELEMETRY_THREAD_PRIORITY      5    /* Housekeeping & telemetry downlink */

/* Flight Loop Execution Frequency */
#define FLIGHT_LOOP_FREQ_HZ            500
#define FLIGHT_LOOP_PERIOD_US          (1000000 / FLIGHT_LOOP_FREQ_HZ) /* 2000 us */
#define FLIGHT_LOOP_DT_S               (1.0f / (float)FLIGHT_LOOP_FREQ_HZ)

/* Background Subsystem Rates */
#define TELEMETRY_RATE_HZ              20
#define TELEMETRY_PERIOD_MS            (1000 / TELEMETRY_RATE_HZ)      /* 50 ms */
#define BATTERY_RATE_HZ                10
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

/* IMU Calibration Parameters */
#define IMU_CALIB_SAMPLES              1000

/* ========================================================================== */
/* AHRS (Madgwick Orientation Filter) Parameters                              */
/* ========================================================================== */

#define MADGWICK_BETA                  0.045f   /* Filter gain for accelerometer fusion */

/* ========================================================================== */
/* PWM0 & Motor ESC Output (Quad-X S500 Airframe)                             */
/* ========================================================================== */

/* THEJAS32 PWM Peripheral Memory Map */
#define THEJAS32_PWM0_BASE_ADDR        0x10400000
#define THEJAS32_PWM_GLOBAL_ENABLE_REG 0x80     /* Global enable register offset */
#define THEJAS32_PWM_ENABLE_BIT        (1 << 0)

/* 400 Hz Standard Multirotor ESC Timing */
#define PWM_FREQ_HZ                    400
#define PWM_PERIOD_NS                  2500000  /* 2.5 ms = 2,500,000 ns */
#define PWM_PERIOD_US                  2500

/* ESC Pulse Width Specifications (Microseconds) */
#define PWM_PULSE_MIN_US               1000     /* Motors disarmed / zero throttle */
#define PWM_PULSE_ARMED_IDLE_US        1120     /* Minimum spin when armed (~12%) */
#define PWM_PULSE_MAX_US               2000     /* 100% full throttle */

/* Motor Hardware Channel Assignments */
#define MOTOR_CHANNEL_FR               0        /* Front-Right (CCW) -> Ch 0 */
#define MOTOR_CHANNEL_RR               1        /* Rear-Right  (CW)  -> Ch 1 */
#define MOTOR_CHANNEL_RL               2        /* Rear-Left   (CCW) -> Ch 2 */
#define MOTOR_CHANNEL_FL               3        /* Front-Left  (CW)  -> Ch 3 */

/* ========================================================================== */
/* Cascaded PID Controller Tuning (S500 Multirotor ~2.5 kg AUW)               */
/* ========================================================================== */

/* Inner Rate Loop (Gyro Angular Rates -> Normalized Torque Demand [-1.0, 1.0]) */
#define ROLL_RATE_KP                   0.145f
#define ROLL_RATE_KI                   0.095f
#define ROLL_RATE_KD                   0.0038f

#define PITCH_RATE_KP                  0.145f
#define PITCH_RATE_KI                  0.095f
#define PITCH_RATE_KD                  0.0038f

#define YAW_RATE_KP                    0.240f
#define YAW_RATE_KI                    0.050f
#define YAW_RATE_KD                    0.0000f

/* Rate Loop Output and Integrator Clamping */
#define RATE_INTEGRATOR_MAX            0.25f    /* Max integrator contribution */
#define RATE_PID_OUTPUT_MAX            0.80f    /* Max angular control demand */

/* Derivative Low-Pass Filter (Cutoff at 30 Hz to attenuate motor vibrations) */
#define PID_D_TERM_LPF_CUTOFF_HZ       30.0f

/* Outer Angle Loop (Tilt Angle Error -> Target Angular Rate in rad/s) */
#define ROLL_ANGLE_KP                  4.50f
#define PITCH_ANGLE_KP                 4.50f

/* Flight Envelope Limits */
#define MAX_ROLL_PITCH_ANGLE_RAD       0.785398f /* 45.0 degrees */
#define MAX_ROLL_PITCH_RATE_RAD_S      3.490659f /* 200.0 deg/s */
#define MAX_YAW_RATE_RAD_S             2.617994f /* 150.0 deg/s */

/* ========================================================================== */
/* RC Receiver & SBUS Protocol (UART1 @ 100,000 Baud, 8E2)                    */
/* ========================================================================== */

#define SBUS_FRAME_LEN                 25
#define SBUS_HEADER_BYTE               0x0F
#define SBUS_END_BYTE_STANDARD         0x00
#define SBUS_END_BYTE_FUTABA2          0x04

/* Channel Indices (0-indexed) */
#define SBUS_CH_ROLL                   0        /* Aileron */
#define SBUS_CH_PITCH                  1        /* Elevator */
#define SBUS_CH_THROTTLE               2        /* Throttle */
#define SBUS_CH_YAW                    3        /* Rudder */
#define SBUS_CH_ARM                    4        /* AUX1: 2-position or 3-position switch */
#define SBUS_CH_MODE                   5        /* AUX2: Flight mode switch */

/* SBUS Signal Calibration */
#define SBUS_INPUT_MIN                 172
#define SBUS_INPUT_CENTER              992
#define SBUS_INPUT_MAX                 1811
#define SBUS_STICK_DEADBAND            15       /* Raw counts around center */
#define SBUS_ARM_THRESHOLD             1400     /* Value above 1400 triggers armed */
#define SBUS_MODE_ANGLE_THRESHOLD      1400     /* Value above 1400 selects Angle Mode */

#define RC_TIMEOUT_MS                  500      /* Failsafe triggered if no frame for 500 ms */

/* ========================================================================== */
/* Battery Monitoring & ADC0                                                  */
/* ========================================================================== */

#define BATTERY_ADC_CHANNEL            0
#define ADC_REF_VOLTAGE_V              3.3f
#define ADC_MAX_VALUE                  4095.0f  /* 12-bit ADC */
#define BATTERY_VOLTAGE_DIVIDER_RATIO  11.0f    /* (R1 + R2) / R2, e.g. 10k + 1k divider */
#define BATTERY_EMA_ALPHA              0.08f    /* Exponential moving average filter */

/* 4S LiPo Thresholds (14.8V Nominal) */
#define LIPO_4S_CELLS                  4
#define LIPO_4S_FULL_V                 16.8f
#define LIPO_4S_WARN_V                 14.4f    /* 3.6V per cell */
#define LIPO_4S_CRIT_V                 13.6f    /* 3.4V per cell */

/* 3S LiPo Thresholds (11.1V Nominal) */
#define LIPO_3S_CELLS                  3
#define LIPO_3S_FULL_V                 12.6f
#define LIPO_3S_WARN_V                 10.8f    /* 3.6V per cell */
#define LIPO_3S_CRIT_V                 10.2f    /* 3.4V per cell */

/* Auto-detect 3S vs 4S voltage split */
#define LIPO_CELL_DETECT_THRESHOLD_V   13.2f

/* ========================================================================== */
/* Safety, Failsafe & Arming Checks                                           */
/* ========================================================================== */

#define ARMING_MAX_TILT_RAD            0.174533f /* 10.0 degrees maximum tilt to arm */
#define CRASH_TILT_LIMIT_RAD           1.134464f /* 65.0 degrees triggers emergency cut */
#define FAILSAFE_DESCENT_THROTTLE      0.32f     /* 32% throttle for gentle descent */
#define FAILSAFE_DESCENT_TIME_MS       5000      /* Descend for 5 seconds then disarm */
#define BATTERY_CRITICAL_TIMEOUT_MS    3000      /* 3 seconds sustained low voltage */

#endif /* CONFIG_H_ */
