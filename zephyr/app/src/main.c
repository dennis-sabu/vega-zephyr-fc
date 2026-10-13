/*
 * main.c - Production Flight Controller Core & RTOS Scheduling Executive
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V 32-bit @ 100 MHz)
 * Airframe: S500 Quadcopter (~2.5 kg AUW, Quad-X Layout)
 *
 * System Architecture:
 * - 256 KB On-Chip SRAM (Static stacks and buffers, zero dynamic allocation)
 * - Thread 1: flight_control_thread (500 Hz, Priority 1, Stack 4 KB)
 * - Thread 2: rc_rx_thread (100 Hz, Priority 2, Stack 2 KB)
 * - Thread 3: telemetry_and_housekeeping_thread (20 Hz, Priority 5, Stack 2 KB)
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
#include "ahrs.h"
#include "pid.h"
#include "mixer.h"
#include "sbus.h"
#include "battery.h"
#include "failsafe.h"

LOG_MODULE_REGISTER(flight_core, CONFIG_LOG_DEFAULT_LEVEL);

/* -------------------------------------------------------------------------- */
/* Static Stack & Thread Allocations (Strict 8 KB Total Stack Budget)         */
/* -------------------------------------------------------------------------- */

K_THREAD_STACK_DEFINE(s_flight_stack, FLIGHT_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(s_rc_stack, RC_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(s_telemetry_stack, TELEMETRY_THREAD_STACK_SIZE);

static struct k_thread s_flight_thread_data;
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
static sbus_channels_t s_current_rc;
static euler_angles_t s_current_att_rad;
static euler_angles_t s_current_att_deg;
static vector3f_t s_current_gyro_rad_s;
static motor_output_t s_current_motors;
static battery_info_t s_current_battery;
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

static void send_telemetry_packet(const telemetry_packet_t *pkt)
{
	if (!device_is_ready(s_telem_uart) || pkt == NULL) {
		return;
	}

	char buf[128];
	int len = snprintf(buf, sizeof(buf),
		"$VEGA,%u,%.1f,%.1f,%.1f,%.2f,%u,%u,%u,%u,%.2f,%u,%u,%u\r\n",
		pkt->timestamp_ms,
		pkt->roll_deg, pkt->pitch_deg, pkt->yaw_deg,
		pkt->throttle_pct,
		pkt->motor_pulse_us[0], pkt->motor_pulse_us[1],
		pkt->motor_pulse_us[2], pkt->motor_pulse_us[3],
		pkt->battery_voltage,
		pkt->flight_state,
		pkt->flight_mode,
		pkt->failsafe_reason);

	if (len > 0) {
		for (int i = 0; i < len; i++) {
			uart_poll_out(s_telem_uart, (unsigned char)buf[i]);
		}
	}
}

/* -------------------------------------------------------------------------- */
/* Thread 1: High-Rate Flight Control Loop (500 Hz / 2000 us)                */
/* -------------------------------------------------------------------------- */

static void flight_control_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("Flight control loop thread active @ %d Hz.", FLIGHT_LOOP_FREQ_HZ);

	int64_t next_cycle_us = k_uptime_get_us();

	while (1) {
		int64_t cycle_start_us = k_uptime_get_us();

		/* 1. Acquire IMU Data (14-byte burst read under 16-byte limit) */
		mpu6050_data_t imu_data;
		int imu_err = mpu6050_read(&imu_data);

		if (imu_err == 0) {
			/* 2. Run AHRS Madgwick Filter */
			ahrs_update(&imu_data.gyro, &imu_data.accel, FLIGHT_LOOP_DT_S);
			ahrs_get_euler_angles(&s_current_att_rad);
			ahrs_get_euler_degrees(&s_current_att_deg);

			s_current_gyro_rad_s = imu_data.gyro;
		} else {
			LOG_WRN("Flight loop: IMU read error %d", imu_err);
		}

		/* 3. Snapshot current RC input */
		sbus_channels_t rc;
		sbus_get_channels(&rc);

		k_spinlock_key_t key = k_spin_lock(&s_state_lock);
		s_current_rc = rc;
		flight_state_t current_state = s_flight_state;
		s_flight_mode = rc.mode_switch;
		flight_mode_t current_mode = s_flight_mode;
		k_spin_unlock(&s_state_lock, key);

		/* 4. Control Law Execution */
		float u_roll = 0.0f;
		float u_pitch = 0.0f;
		float u_yaw = 0.0f;
		float throttle_demand = 0.0f;
		bool motors_armed = false;

		if (current_state == STATE_ARMED) {
			motors_armed = true;
			throttle_demand = rc.throttle_norm;

			float target_roll_rate = 0.0f;
			float target_pitch_rate = 0.0f;
			float target_yaw_rate = rc.yaw_norm * MAX_YAW_RATE_RAD_S;

			if (current_mode == MODE_ANGLE) {
				/* Outer Angle Loop: Angle Error -> Target Rate (rad/s) */
				float target_roll_angle = rc.roll_norm * MAX_ROLL_PITCH_ANGLE_RAD;
				float target_pitch_angle = rc.pitch_norm * MAX_ROLL_PITCH_ANGLE_RAD;

				float roll_angle_err = target_roll_angle - s_current_att_rad.roll;
				float pitch_angle_err = target_pitch_angle - s_current_att_rad.pitch;

				target_roll_rate = roll_angle_err * ROLL_ANGLE_KP;
				target_pitch_rate = pitch_angle_err * PITCH_ANGLE_KP;

				/* Clamp target rates to safe maximums */
				if (target_roll_rate > MAX_ROLL_PITCH_RATE_RAD_S) {
					target_roll_rate = MAX_ROLL_PITCH_RATE_RAD_S;
				} else if (target_roll_rate < -MAX_ROLL_PITCH_RATE_RAD_S) {
					target_roll_rate = -MAX_ROLL_PITCH_RATE_RAD_S;
				}

				if (target_pitch_rate > MAX_ROLL_PITCH_RATE_RAD_S) {
					target_pitch_rate = MAX_ROLL_PITCH_RATE_RAD_S;
				} else if (target_pitch_rate < -MAX_ROLL_PITCH_RATE_RAD_S) {
					target_pitch_rate = -MAX_ROLL_PITCH_RATE_RAD_S;
				}
			} else {
				/* Rate / Acro Mode: Direct stick-to-rate mapping */
				target_roll_rate = rc.roll_norm * MAX_ROLL_PITCH_RATE_RAD_S;
				target_pitch_rate = rc.pitch_norm * MAX_ROLL_PITCH_RATE_RAD_S;
			}

			/* Inner Rate Loop (PID) */
			u_roll = pid_update(&s_pid_roll_rate, target_roll_rate,
					    s_current_gyro_rad_s.x, FLIGHT_LOOP_DT_S);
			u_pitch = pid_update(&s_pid_pitch_rate, target_pitch_rate,
					     s_current_gyro_rad_s.y, FLIGHT_LOOP_DT_S);
			u_yaw = pid_update(&s_pid_yaw_rate, target_yaw_rate,
					   s_current_gyro_rad_s.z, FLIGHT_LOOP_DT_S);

		} else if (current_state == STATE_FAILSAFE) {
			/* Failsafe Landing: Level flight + gradual descent */
			motors_armed = true;
			throttle_demand = failsafe_get_throttle();

			/* Target zero tilt */
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
			/* Disarmed or Emergency Cut */
			motors_armed = false;
			throttle_demand = 0.0f;
			reset_all_pids();
		}

		/* 5. Motor Mixing & Dynamic Desaturation */
		motor_output_t motors;
		mixer_compute(throttle_demand, u_roll, u_pitch, u_yaw, motors_armed, &motors);

		/* 6. Hardware Actuation via PWM0 */
		mixer_apply_output(&motors);

		key = k_spin_lock(&s_state_lock);
		s_current_motors = motors;
		k_spin_unlock(&s_state_lock, key);

		/* 7. Precise Loop Sleep to maintain exact 500 Hz execution */
		next_cycle_us += FLIGHT_LOOP_PERIOD_US;
		int64_t current_time_us = k_uptime_get_us();
		int64_t sleep_us = next_cycle_us - current_time_us;

		if (sleep_us > 0) {
			k_usleep((int32_t)sleep_us);
		} else {
			/* Loop overrun compensation */
			next_cycle_us = current_time_us;
		}
	}
}

/* -------------------------------------------------------------------------- */
/* Thread 2: RC Receiver & SBUS Processing Task (100 Hz / 10 ms)             */
/* -------------------------------------------------------------------------- */

static void rc_rx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("RC receiver thread active.");

	while (1) {
		sbus_process();
		k_msleep(10);
	}
}

/* -------------------------------------------------------------------------- */
/* Thread 3: Telemetry, Battery & Failsafe Supervisor (20 Hz / 50 ms)        */
/* -------------------------------------------------------------------------- */

static void telemetry_and_housekeeping_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("Telemetry and housekeeping thread active @ %d Hz.", TELEMETRY_RATE_HZ);

	uint32_t battery_tick_count = 0;

	while (1) {
		/* 1. Periodic Battery Update (10 Hz) */
		if (++battery_tick_count >= 2) {
			battery_update();
			battery_tick_count = 0;
		}

		battery_info_t bat;
		battery_get_info(&bat);

		bool bat_crit = battery_is_critical();
		bool imu_healthy = mpu6050_is_healthy();

		/* 2. Run Failsafe Supervisor State Machine */
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
		motor_output_t motors = s_current_motors;
		euler_angles_t att_deg = s_current_att_deg;
		vector3f_t gyro_rate = s_current_gyro_rad_s;
		flight_mode_t mode = s_flight_mode;
		k_spin_unlock(&s_state_lock, key);

		/* 3. Assemble and Transmit Downlink Telemetry Packet */
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

		send_telemetry_packet(&pkt);

		k_msleep(TELEMETRY_PERIOD_MS);
	}
}

/* -------------------------------------------------------------------------- */
/* Main Initialization Function                                               */
/* -------------------------------------------------------------------------- */

int main(void)
{
	LOG_INF("=====================================================");
	LOG_INF(" VEGA ARIES v3.0 Flight Controller Starting...       ");
	LOG_INF(" SoC: C-DAC THEJAS32 / VEGA ET1031 RISC-V @ 100 MHz  ");
	LOG_INF(" Target: S500 Quadcopter (Quad-X, AUW ~2.5 kg)       ");
	LOG_INF("=====================================================");

	/* 1. Initialize Subsystem Peripherals */
	int ret = sbus_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize SBUS RC on UART1 (err %d)", ret);
	}

	ret = mixer_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize Motor PWM0 (err %d)", ret);
	}

	ret = battery_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize Battery ADC0 (err %d)", ret);
	}

	failsafe_init();

	ret = mpu6050_init();
	if (ret < 0) {
		LOG_ERR("Failed to initialize MPU6050 on I2C0 (err %d)!", ret);
	}

	/* 2. Initialize PID Controllers */
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

	/* 3. On-Ground Sensor Zero-Bias Calibration */
	LOG_INF("Calibrating IMU sensors... Aircraft must remain stationary!");
	s_flight_state = STATE_CALIBRATING;
	mpu6050_calib_t calib;
	ret = mpu6050_calibrate(&calib);
	if (ret < 0) {
		LOG_ERR("IMU Calibration failed! Emergency halt.");
		s_flight_state = STATE_EMERGENCY_STOP;
	} else {
		ahrs_init(FLIGHT_LOOP_FREQ_HZ, MADGWICK_BETA);
		s_flight_state = STATE_DISARMED;
		LOG_INF("System calibration complete. Flight controller DISARMED & READY.");
	}

	/* 4. Spawn Real-Time RTOS Threads */
	k_thread_create(&s_flight_thread_data, s_flight_stack,
			K_THREAD_STACK_SIZEOF(s_flight_stack),
			flight_control_thread, NULL, NULL, NULL,
			FLIGHT_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&s_flight_thread_data, "flight_ctrl");

	k_thread_create(&s_rc_thread_data, s_rc_stack,
			K_THREAD_STACK_SIZEOF(s_rc_stack),
			rc_rx_thread, NULL, NULL, NULL,
			RC_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&s_rc_thread_data, "rc_sbus");

	k_thread_create(&s_telemetry_thread_data, s_telemetry_stack,
			K_THREAD_STACK_SIZEOF(s_telemetry_stack),
			telemetry_and_housekeeping_thread, NULL, NULL, NULL,
			TELEMETRY_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&s_telemetry_thread_data, "telemetry");

	LOG_INF("All threads spawned successfully. RTOS kernel executing.");
	return 0;
}
