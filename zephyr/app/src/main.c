/*
 * main.c - Production Flight Controller Core & RTOS Scheduling Executive
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V 32-bit @ 100 MHz)
 * Airframe: S500 Quadcopter (~2.5 kg AUW, Quad-X Layout)
 *
 * System Architecture & Threading Model:
 * - 256 KB On-Chip SRAM (Static stacks and buffers, zero dynamic allocation)
 * - Thread 1: flight_control_thread (250 Hz / 4 ms, K_PRIO_PREEMPT(1), Stack 4 KB)
 * - Thread 2: altitude_control_thread (50 Hz / 20 ms, K_PRIO_PREEMPT(2), Stack 2 KB)
 * - Thread 3: rc_receiver_thread (100 Hz / 10 ms, K_PRIO_PREEMPT(3), Stack 2 KB)
 * - Thread 4: housekeeping_telemetry_thread (10 Hz / 100 ms, K_PRIO_PREEMPT(6), Stack 2 KB)
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/spinlock.h>
#include <stdio.h>
#include <math.h>

#include "config.h"
#include "types.h"
#include "mpu6050.h"
#include "bmp280.h"
#include "mag.h"
#include "gnss.h"
#include "ahrs.h"
#include "pid.h"
#include "mixer.h"
#include "altitude.h"
#include "sbus.h"
#include "battery.h"
#include "failsafe.h"
#include "logger.h"
#include "esc_pwm.h"
#include "telemetry.h"

LOG_MODULE_REGISTER(flight_core, CONFIG_LOG_DEFAULT_LEVEL);

/* -------------------------------------------------------------------------- */
/* Static Stack & Thread Allocations (Strict 10 KB Total Stack Budget)        */
/* -------------------------------------------------------------------------- */

K_THREAD_STACK_DEFINE(s_flight_stack, FLIGHT_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(s_altitude_stack, ALTITUDE_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(s_rc_stack, RC_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(s_telemetry_stack, TELEMETRY_THREAD_STACK_SIZE);

static struct k_thread s_flight_thread_data;
static struct k_thread s_altitude_thread_data;
static struct k_thread s_rc_thread_data;
static struct k_thread s_telemetry_thread_data;

/* -------------------------------------------------------------------------- */
/* Global Flight State & Controller Instances                                 */
/* -------------------------------------------------------------------------- */

static volatile flight_state_t s_flight_state = STATE_CALIBRATING;
static volatile flight_mode_t s_flight_mode = MODE_ANGLE;

/* Inner Loop (Rate) PID Controllers */
static pid_controller_t s_pid_roll_rate;
static pid_controller_t s_pid_pitch_rate;
static pid_controller_t s_pid_yaw_rate;

/* Latest System States for Cross-Thread Exchange */
static flight_setpoints_t s_current_setpoints;
static sbus_channels_t s_current_rc_raw;
static euler_angles_t s_current_att_rad;
static euler_angles_t s_current_att_deg;
static vector3f_t s_current_gyro_rad_s;
static motor_output_t s_current_motors;
static battery_info_t s_current_battery;
static bmp280_data_t s_current_baro;
static mag_data_t s_current_mag;
static gnss_data_t s_current_gnss;
static float s_alt_hold_throttle = 0.0f;
static uint32_t s_flight_loop_max_jitter_us = 0;
static struct k_spinlock s_state_lock;

/* Telemetry UART Device Resolution */
#if DT_NODE_EXISTS(DT_ALIAS(telemetry_uart))
static const struct device *const s_telem_uart = DEVICE_DT_GET(DT_ALIAS(telemetry_uart));
#elif DT_NODE_EXISTS(DT_NODELABEL(uart2))
static const struct device *const s_telem_uart = DEVICE_DT_GET(DT_NODELABEL(uart2));
#else
#error "DeviceTree error: Neither alias 'telemetry_uart' nor nodelabel 'uart2' found."
#endif

/* -------------------------------------------------------------------------- */
/* Helper Functions                                                           */
/* -------------------------------------------------------------------------- */

static void reset_all_pids(void)
{
	pid_reset(&s_pid_roll_rate);
	pid_reset(&s_pid_pitch_rate);
	pid_reset(&s_pid_yaw_rate);
}

static void send_telemetry_packet(const telemetry_packet_t *pkt, uint32_t jitter_us)
{
	if (!device_is_ready(s_telem_uart) || pkt == NULL) {
		return;
	}

	char buf[256];
	int len = snprintf(buf, sizeof(buf),
		"$VEGA,%u,%.1f,%.1f,%.1f,%.2f,%u,%u,%u,%u,%.2f,%u,%u,%u,%.2f,%.2f,%.1f,%.6f,%.6f,%u,%u,%u\r\n",
		pkt->timestamp_ms,
		pkt->roll_deg, pkt->pitch_deg, pkt->yaw_deg,
		pkt->throttle_pct,
		pkt->motor_pulse_us[0], pkt->motor_pulse_us[1],
		pkt->motor_pulse_us[2], pkt->motor_pulse_us[3],
		pkt->battery_voltage,
		pkt->flight_state,
		pkt->flight_mode,
		pkt->failsafe_reason,
		pkt->altitude_m,
		pkt->climb_rate_mps,
		pkt->mag_heading_deg,
		pkt->latitude,
		pkt->longitude,
		pkt->gps_sats,
		pkt->gps_fix,
		jitter_us);

	if (len > 0) {
		for (int i = 0; i < len; i++) {
			uart_poll_out(s_telem_uart, (unsigned char)buf[i]);
		}
	}
}

/* -------------------------------------------------------------------------- */
/* Thread 1: Flight Control Thread (Priority: K_PRIO_PREEMPT(1), 250 Hz)       */
/* -------------------------------------------------------------------------- */

static void flight_control_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("Flight control thread active @ %d Hz (period %d us).",
		FLIGHT_LOOP_FREQ_HZ, FLIGHT_LOOP_PERIOD_US);

	int64_t next_cycle_us = k_uptime_get_us();

	while (1) {
		int64_t cycle_start_us = k_uptime_get_us();

		/* 1. Acquire IMU Data (14-byte burst read under 16-byte limit) */
		mpu6050_data_t imu_data;
		int imu_err = mpu6050_read(&imu_data);

		if (imu_err == 0) {
			/* 2. Compute Attitude via Complementary Filter */
			ahrs_update(&imu_data.gyro, &imu_data.accel, FLIGHT_LOOP_DT_S);
			ahrs_get_euler_angles(&s_current_att_rad);
			ahrs_get_euler_degrees(&s_current_att_deg);
			s_current_gyro_rad_s = imu_data.gyro;
		} else {
			LOG_WRN("IMU burst read error: %d", imu_err);
		}

		/* 3. Retrieve Thread-Safe Flight Setpoints from RC receiver */
		flight_setpoints_t sp;
		bool sp_valid = sbus_get_setpoints(&sp);

		k_spinlock_key_t key = k_spin_lock(&s_state_lock);
		if (sp_valid) {
			s_current_setpoints = sp;
			s_flight_mode = sp.flight_mode;
		}
		flight_state_t current_state = s_flight_state;
		flight_mode_t current_mode = s_flight_mode;
		float alt_throttle = s_alt_hold_throttle;
		k_spin_unlock(&s_state_lock, key);

		/* 4. Cascaded Dual-PID Control Law Execution */
		float u_roll = 0.0f;
		float u_pitch = 0.0f;
		float u_yaw = 0.0f;
		float throttle_demand = 0.0f;
		bool motors_armed = false;

		if (current_state == STATE_ARMED) {
			motors_armed = true;

			/* Throttle Source: Altitude Hold PID vs Manual RC Throttle */
			if (current_mode == MODE_ALT_HOLD) {
				throttle_demand = alt_throttle;
			} else {
				throttle_demand = sp.throttle;
			}

			float target_roll_rate_rad_s = 0.0f;
			float target_pitch_rate_rad_s = 0.0f;
			float target_yaw_rate_rad_s = sp.target_yaw_rate_dps * (3.14159265f / 180.0f);

			if (current_mode == MODE_ANGLE || current_mode == MODE_ALT_HOLD) {
				/* Outer Angle Loop: Angle Error -> Target Angular Rate (rad/s) */
				float target_roll_angle_rad = sp.target_roll_deg * (3.14159265f / 180.0f);
				float target_pitch_angle_rad = sp.target_pitch_deg * (3.14159265f / 180.0f);

				float roll_err = target_roll_angle_rad - s_current_att_rad.roll;
				float pitch_err = target_pitch_angle_rad - s_current_att_rad.pitch;

				target_roll_rate_rad_s = roll_err * ROLL_ANGLE_KP;
				target_pitch_rate_rad_s = pitch_err * PITCH_ANGLE_KP;

				/* Clamp target rates to safe airframe envelope */
				if (target_roll_rate_rad_s > MAX_ROLL_PITCH_RATE_RAD_S) {
					target_roll_rate_rad_s = MAX_ROLL_PITCH_RATE_RAD_S;
				} else if (target_roll_rate_rad_s < -MAX_ROLL_PITCH_RATE_RAD_S) {
					target_roll_rate_rad_s = -MAX_ROLL_PITCH_RATE_RAD_S;
				}

				if (target_pitch_rate_rad_s > MAX_ROLL_PITCH_RATE_RAD_S) {
					target_pitch_rate_rad_s = MAX_ROLL_PITCH_RATE_RAD_S;
				} else if (target_pitch_rate_rad_s < -MAX_ROLL_PITCH_RATE_RAD_S) {
					target_pitch_rate_rad_s = -MAX_ROLL_PITCH_RATE_RAD_S;
				}
			} else {
				/* Rate / Acrobatic Mode: Direct stick-to-rate mapping */
				target_roll_rate_rad_s = (sp.target_roll_deg / MAX_ROLL_PITCH_ANGLE_DEG) *
							 MAX_ROLL_PITCH_RATE_RAD_S;
				target_pitch_rate_rad_s = (sp.target_pitch_deg / MAX_ROLL_PITCH_ANGLE_DEG) *
							  MAX_ROLL_PITCH_RATE_RAD_S;
			}

			/* Inner Rate Loop: Compares desired rates against gyro measurements */
			u_roll = pid_update(&s_pid_roll_rate, target_roll_rate_rad_s,
					    s_current_gyro_rad_s.x, FLIGHT_LOOP_DT_S);
			u_pitch = pid_update(&s_pid_pitch_rate, target_pitch_rate_rad_s,
					     s_current_gyro_rad_s.y, FLIGHT_LOOP_DT_S);
			u_yaw = pid_update(&s_pid_yaw_rate, target_yaw_rate_rad_s,
					   s_current_gyro_rad_s.z, FLIGHT_LOOP_DT_S);

		} else if (current_state == STATE_FAILSAFE) {
			/* Failsafe Landing: Auto-level roll/pitch and ramp descent throttle */
			motors_armed = true;
			throttle_demand = failsafe_get_throttle();

			float target_roll_rate = (0.0f - s_current_att_rad.roll) * ROLL_ANGLE_KP;
			float target_pitch_rate = (0.0f - s_current_att_rad.pitch) * PITCH_ANGLE_KP;
			float target_yaw_rate = 0.0f;

			u_roll = pid_update(&s_pid_roll_rate, target_roll_rate,
					    s_current_gyro_rad_s.x, FLIGHT_LOOP_DT_S);
			u_pitch = pid_update(&s_pid_pitch_rate, target_pitch_rate,
					     s_current_gyro_rad_s.y, FLIGHT_LOOP_DT_S);
			u_yaw = pid_update(&s_pid_yaw_rate, target_yaw_rate,
					   s_current_gyro_rad_s.z, FLIGHT_LOOP_DT_S);

		} else {
			/* Disarmed or Emergency Cutoff */
			motors_armed = false;
			throttle_demand = 0.0f;
			reset_all_pids();
		}

		/* 5. Quad-X Motor Mixing Matrix */
		motor_output_t motors;
		mixer_compute(throttle_demand, u_roll, u_pitch, u_yaw, motors_armed, &motors);

		/* 6. Hardware Actuation via PWM0 (Channels 0-3 @ 400 Hz) */
		mixer_apply_output(&motors);

		key = k_spin_lock(&s_state_lock);
		s_current_motors = motors;
		k_spin_unlock(&s_state_lock, key);

		/* 7. Push Binary Frame to Blackbox Logger (Lock-Free & Non-Blocking) */
		logger_log_flight_data((uint32_t)cycle_start_us, &sp,
				       &s_current_att_rad, &s_current_gyro_rad_s,
				       u_roll, u_pitch, u_yaw,
				       &motors, s_current_battery.voltage,
				       (uint8_t)current_state, (uint8_t)current_mode,
				       0);

		/* 8. Loop Jitter Tracking and Deterministic Timing */
		int64_t cycle_elapsed_us = k_uptime_get_us() - cycle_start_us;
		uint32_t jitter_us = (cycle_elapsed_us > FLIGHT_LOOP_PERIOD_US) ?
				     (uint32_t)(cycle_elapsed_us - FLIGHT_LOOP_PERIOD_US) : 0;
		if (jitter_us > s_flight_loop_max_jitter_us) {
			s_flight_loop_max_jitter_us = jitter_us;
		}

		next_cycle_us += FLIGHT_LOOP_PERIOD_US;
		int64_t current_time_us = k_uptime_get_us();
		int64_t sleep_us = next_cycle_us - current_time_us;

		if (sleep_us > 0) {
			k_usleep((int32_t)sleep_us);
		} else {
			next_cycle_us = current_time_us;
		}
	}
}

/* -------------------------------------------------------------------------- */
/* Thread 2: Altitude Control & Navigation Thread (Priority: K_PRIO_PREEMPT(2), 50 Hz) */
/* -------------------------------------------------------------------------- */

static void altitude_control_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("Altitude & navigation control thread active @ 50 Hz.");

	while (1) {
		/* 1. Acquire BMP280 Barometer Data */
		bmp280_data_t baro;
		bmp280_read(&baro);

		/* 2. Acquire Magnetometer Data with AHRS Tilt Compensation */
		euler_angles_t att;
		flight_mode_t mode;
		float stick_throttle;

		k_spinlock_key_t key = k_spin_lock(&s_state_lock);
		att = s_current_att_rad;
		mode = s_flight_mode;
		stick_throttle = s_current_setpoints.throttle;
		k_spin_unlock(&s_state_lock, key);

		mag_data_t mag;
		mag_read(att.roll, att.pitch, &mag);

		/* 3. Run Cascaded Altitude Controller */
		bool is_alt_hold = (mode == MODE_ALT_HOLD);
		float computed_alt_throttle = altitude_controller_update(
			stick_throttle,
			baro.altitude_filtered_m,
			baro.climb_rate_mps,
			is_alt_hold,
			ALTITUDE_LOOP_DT_S
		);

		key = k_spin_lock(&s_state_lock);
		s_current_baro = baro;
		s_current_mag = mag;
		s_alt_hold_throttle = computed_alt_throttle;
		k_spin_unlock(&s_state_lock, key);

		k_msleep(ALTITUDE_LOOP_PERIOD_MS);
	}
}

/* -------------------------------------------------------------------------- */
/* Thread 3: RC Receiver & GNSS Thread (Priority: K_PRIO_PREEMPT(3), 100 Hz)   */
/* -------------------------------------------------------------------------- */

static void rc_receiver_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("RC receiver thread active (100 Hz).");

	while (1) {
		/* 1. Process SBUS Byte Stream on UART1 */
		sbus_process();

		sbus_channels_t raw_rc;
		if (sbus_get_channels(&raw_rc)) {
			k_spinlock_key_t key = k_spin_lock(&s_state_lock);
			s_current_rc_raw = raw_rc;
			k_spin_unlock(&s_state_lock, key);
		}

		/* 2. Process GNSS NMEA Byte Stream on UART0 */
		gnss_process();

		k_msleep(RC_THREAD_PERIOD_MS);
	}
}

/* -------------------------------------------------------------------------- */
/* Thread 4: Housekeeping & Telemetry Thread (Priority: K_PRIO_PREEMPT(6), 10 Hz) */
/* -------------------------------------------------------------------------- */

static void housekeeping_telemetry_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("Housekeeping & telemetry thread active (10 Hz).");

	while (1) {
		/* 1. Sample ADC0 and Update Battery Voltage */
		battery_update();

		battery_info_t bat;
		battery_get_info(&bat);

		bool bat_crit = battery_is_critical();
		bool imu_healthy = mpu6050_is_healthy();

		/* 2. Query Latest GNSS Data */
		gnss_data_t gnss;
		gnss_get_data(&gnss);

		/* 3. Run Failsafe Supervisor State Machine */
		sbus_channels_t rc;
		sbus_get_channels(&rc);

		k_spinlock_key_t key = k_spin_lock(&s_state_lock);
		flight_state_t cur_state = s_flight_state;
		euler_angles_t att_rad = s_current_att_rad;
		k_spin_unlock(&s_state_lock, key);

		flight_state_t next_state = failsafe_update(cur_state, &rc, &att_rad,
							    imu_healthy, bat_crit,
							    TELEMETRY_PERIOD_MS);

		key = k_spin_lock(&s_state_lock);
		s_flight_state = next_state;
		s_current_battery = bat;
		s_current_gnss = gnss;
		motor_output_t motors = s_current_motors;
		euler_angles_t att_deg = s_current_att_deg;
		vector3f_t gyro_rate = s_current_gyro_rad_s;
		flight_mode_t mode = s_flight_mode;
		bmp280_data_t baro = s_current_baro;
		mag_data_t mag = s_current_mag;
		uint32_t jitter = s_flight_loop_max_jitter_us;
		k_spin_unlock(&s_state_lock, key);

		/* 4. Assemble and Transmit Downlink Telemetry Packet over UART2 */
		failsafe_state_t fs_state;
		failsafe_get_state(&fs_state);

		telemetry_packet_t pkt;
		pkt.timestamp_ms = (uint32_t)k_uptime_get_32();
		pkt.roll_deg = att_deg.roll;
		pkt.pitch_deg = att_deg.pitch;
		pkt.yaw_deg = att_deg.yaw;
		pkt.roll_rate_dps = gyro_rate.x * (180.0f / 3.14159f);
		pkt.pitch_rate_dps = gyro_rate.y * (180.0f / 3.14159f);
		pkt.yaw_rate_dps = gyro_rate.z * (180.0f / 3.14159f);
		pkt.throttle_pct = rc.throttle_norm * 100.0f;
		for (int i = 0; i < 4; i++) {
			pkt.motor_pulse_us[i] = (uint16_t)motors.pulse_us[i];
		}
		pkt.battery_voltage = bat.voltage;
		pkt.flight_state = (uint8_t)next_state;
		pkt.flight_mode = (uint8_t)mode;
		pkt.failsafe_reason = (uint8_t)fs_state.reason_mask;
		pkt.altitude_m = baro.altitude_filtered_m;
		pkt.climb_rate_mps = baro.climb_rate_mps;
		pkt.mag_heading_deg = mag.heading_deg;
		pkt.latitude = gnss.latitude;
		pkt.longitude = gnss.longitude;
		pkt.gps_sats = gnss.satellites;
		pkt.gps_fix = gnss.fix_quality;

		send_telemetry_packet(&pkt, jitter);
		telemetry_set_state(pkt.roll_deg, pkt.pitch_deg, pkt.yaw_deg,
				    rc.throttle_norm, pkt.battery_voltage,
				    (next_state == STATE_ARMED), (uint8_t)mode,
				    fs_state.active);

		k_msleep(TELEMETRY_PERIOD_MS);
	}
}

/* -------------------------------------------------------------------------- */
/* Main System Initialization                                                 */
/* -------------------------------------------------------------------------- */

int main(void)
{
	LOG_INF("=====================================================");
	LOG_INF(" VEGA ARIES v3.0 S500 Flight Controller Initializing ");
	LOG_INF(" SoC: C-DAC THEJAS32 / VEGA ET1031 RISC-V @ 100 MHz  ");
	LOG_INF(" Nav Sensors: MPU6050 + BMP280 + QMC/HMC5883L + GNSS ");
	LOG_INF("=====================================================");

	/* 1. Initialize Subsystem Peripherals */
	int ret = sbus_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize SBUS RC on UART1 (err %d)", ret);
	}

	ret = esc_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize ESC PWM0 (err %d)", ret);
	}

	ret = mixer_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize Motor PWM0 (err %d)", ret);
	}

	ret = battery_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize Battery ADC0 (err %d)", ret);
	}

	ret = telemetry_init();
	if (ret < 0) {
		LOG_WRN("Failed to initialize Telemetry UART2 (err %d)", ret);
	}

	failsafe_init();

	ret = mpu6050_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize MPU6050 on I2C0 (err %d)!", ret);
	}

	/* Phase 2 Sensor Initializations */
	ret = bmp280_init();
	if (ret < 0) {
		LOG_WRN("BMP280 Barometer not detected on I2C0 (err %d)", ret);
	}

	ret = mag_init();
	if (ret < 0) {
		LOG_WRN("Magnetometer not detected on I2C0 (err %d)", ret);
	}

	ret = gnss_init(GNSS_BAUD_RATE);
	if (ret < 0) {
		LOG_WRN("GNSS Receiver not initialized on UART0 (err %d)", ret);
	}

	/* Phase 3 Altitude Controller Initialization */
	altitude_controller_init();

	/* Phase 5 MicroSD Blackbox Logger Initialization */
	ret = logger_init();
	if (ret < 0) {
		LOG_WRN("Blackbox logger not initialized on SPI0 (err %d)", ret);
	}

	/* 2. Initialize Inner Rate PID Controllers */
	pid_init(&s_pid_roll_rate, ROLL_RATE_KP, ROLL_RATE_KI, ROLL_RATE_KD,
		 -RATE_PID_OUTPUT_MAX, RATE_PID_OUTPUT_MAX,
		 -RATE_INTEGRATOR_MAX, RATE_INTEGRATOR_MAX,
		 PID_D_TERM_LPF_CUTOFF_HZ, FLIGHT_LOOP_DT_S, true);

	pid_init(&s_pid_pitch_rate, PITCH_RATE_KP, PITCH_RATE_KI, PITCH_RATE_KD,
		 -RATE_PID_OUTPUT_MAX, RATE_PID_OUTPUT_MAX,
		 -RATE_INTEGRATOR_MAX, RATE_INTEGRATOR_MAX,
		 PID_D_TERM_LPF_CUTOFF_HZ, FLIGHT_LOOP_DT_S, true);

	pid_init(&s_pid_yaw_rate, YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD,
		 -RATE_PID_OUTPUT_MAX, RATE_PID_OUTPUT_MAX,
		 -RATE_INTEGRATOR_MAX, RATE_INTEGRATOR_MAX,
		 PID_D_TERM_LPF_CUTOFF_HZ, FLIGHT_LOOP_DT_S, true);

	/* 3. Perform 500-Sample Stationary Bias Calibration */
	LOG_INF("Calibrating IMU sensors (500 samples)... Aircraft must remain still!");
	s_flight_state = STATE_CALIBRATING;
	mpu6050_calib_t calib;
	ret = mpu6050_calibrate(&calib);
	if (ret < 0) {
		LOG_ERR("IMU Calibration failed! Halting in emergency state.");
		s_flight_state = STATE_EMERGENCY_STOP;
	} else {
		ahrs_init(FLIGHT_LOOP_FREQ_HZ, COMPLEMENTARY_ALPHA);
		s_flight_state = STATE_DISARMED;
		LOG_INF("IMU Calibration complete. Controller DISARMED & STANDBY.");
	}

	/* 4. Spawn Real-Time Preemptive RTOS Threads */
	k_thread_create(&s_flight_thread_data, s_flight_stack,
			K_THREAD_STACK_SIZEOF(s_flight_stack),
			flight_control_thread, NULL, NULL, NULL,
			K_PRIO_PREEMPT(FLIGHT_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&s_flight_thread_data, "flight_ctrl");

	k_thread_create(&s_altitude_thread_data, s_altitude_stack,
			K_THREAD_STACK_SIZEOF(s_altitude_stack),
			altitude_control_thread, NULL, NULL, NULL,
			K_PRIO_PREEMPT(ALTITUDE_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&s_altitude_thread_data, "alt_ctrl");

	k_thread_create(&s_rc_thread_data, s_rc_stack,
			K_THREAD_STACK_SIZEOF(s_rc_stack),
			rc_receiver_thread, NULL, NULL, NULL,
			K_PRIO_PREEMPT(RC_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&s_rc_thread_data, "rc_rx");

	k_thread_create(&s_telemetry_thread_data, s_telemetry_stack,
			K_THREAD_STACK_SIZEOF(s_telemetry_stack),
			housekeeping_telemetry_thread, NULL, NULL, NULL,
			K_PRIO_PREEMPT(TELEMETRY_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&s_telemetry_thread_data, "housekeeping");

	LOG_INF("All RTOS flight threads active (Flight 250Hz, Alt 50Hz, RC 100Hz, Telem 10Hz).");
	return 0;
}
