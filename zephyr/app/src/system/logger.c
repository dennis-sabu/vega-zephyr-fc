/*
 * logger.c - High-Speed Flight Blackbox Data Logger Implementation
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Peripheral: SPI0 @ 0x10000600 (MicroSD FatFS)
 *
 * Implements:
 * 1. Non-blocking lock-free ring buffer for 250 Hz flight loop.
 * 2. Preemptive background SD card writer thread (K_PRIO_PREEMPT(7)).
 * 3. Dynamic filename creation (/SD:/LOG_XXXX.BFL).
 * 4. Periodic 1-second fs_sync() to survive sudden crash/power loss.
 */

#include "logger.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <zephyr/sys/ring_buffer.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(blackbox, CONFIG_LOG_DEFAULT_LEVEL);

#define SD_WRITER_THREAD_STACK_SIZE   3072
#define SD_WRITER_THREAD_PRIORITY     7    /* Low-priority background writer */

#define BLACKBOX_RING_BUF_FRAMES      64   /* Holds 64 frames (> 50 minimum required) */
#define BLACKBOX_RING_BUF_BYTES       (sizeof(blackbox_frame_t) * BLACKBOX_RING_BUF_FRAMES)
#define SD_WRITE_CHUNK_BYTES          512  /* Standard 512-byte sector-aligned buffer */

RING_BUF_DECLARE(s_blackbox_rb, BLACKBOX_RING_BUF_BYTES);

K_THREAD_STACK_DEFINE(s_sd_writer_stack, SD_WRITER_THREAD_STACK_SIZE);
static struct k_thread s_sd_writer_thread_data;
static struct k_sem s_log_sem;

/* File System Structures */
static FATFS s_fat_fs;
static struct fs_mount_t s_mp = {
	.type = FS_FATFS,
	.fs_data = &s_fat_fs,
	.mnt_point = "/SD:",
};

static struct fs_file_t s_log_file;
static char s_active_filepath[32];
static bool s_is_mounted = false;
static bool s_file_open = false;
static uint32_t s_written_frames = 0;
static uint32_t s_dropped_frames = 0;
static uint32_t s_last_sync_time_ms = 0;

/* Worker Thread Prototype */
static void sd_writer_thread(void *p1, void *p2, void *p3);

int logger_init(void)
{
	LOG_INF("Initializing MicroSD Blackbox logger on SPI0...");

	k_sem_init(&s_log_sem, 0, 100);

	/* 1. Mount FatFS File System */
	int ret = fs_mount(&s_mp);
	if (ret < 0) {
		LOG_WRN("Failed to mount MicroSD FatFS at /SD: (err %d). Running without Blackbox.", ret);
		s_is_mounted = false;
		return ret;
	}

	s_is_mounted = true;
	LOG_INF("MicroSD Card mounted successfully at /SD:");

	/* 2. Dynamically Find Next Available Log Filename (LOG_0001.BFL .. LOG_9999.BFL) */
	struct fs_dirent entry;
	int file_num = 1;

	for (; file_num <= 9999; file_num++) {
		snprintf(s_active_filepath, sizeof(s_active_filepath), "/SD:/LOG_%04d.BFL", file_num);
		if (fs_stat(s_active_filepath, &entry) != 0) {
			/* File does not exist, use this filename */
			break;
		}
	}

	LOG_INF("Opening new Blackbox flight log: %s", s_active_filepath);

	fs_file_t_init(&s_log_file);
	ret = fs_open(&s_log_file, s_active_filepath, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
	if (ret < 0) {
		LOG_ERR("Failed to create log file %s (err %d)", s_active_filepath, ret);
		s_file_open = false;
		return ret;
	}

	s_file_open = true;

	/* Write 32-byte ASCII File Identification Header */
	const char file_header[32] = "VEGA_ARIES_V3_BLACKBOX_LOG_V1\r\n";
	fs_write(&s_log_file, file_header, 32);
	fs_sync(&s_log_file);

	/* 3. Spawn Preemptive SD Writer Thread */
	k_thread_create(&s_sd_writer_thread_data, s_sd_writer_stack,
			K_THREAD_STACK_SIZEOF(s_sd_writer_stack),
			sd_writer_thread, NULL, NULL, NULL,
			K_PRIO_PREEMPT(SD_WRITER_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&s_sd_writer_thread_data, "sd_writer");

	LOG_INF("Blackbox logging active. Writer thread running at Priority %d.",
		SD_WRITER_THREAD_PRIORITY);

	return 0;
}

int logger_push_frame(const blackbox_frame_t *frame)
{
	if (frame == NULL || !s_file_open) {
		return -EINVAL;
	}

	if (ring_buf_space_get(&s_blackbox_rb) < sizeof(blackbox_frame_t)) {
		s_dropped_frames++;
		return -ENOSPC;
	}

	ring_buf_put(&s_blackbox_rb, (const uint8_t *)frame, sizeof(blackbox_frame_t));
	k_sem_give(&s_log_sem);
	return 0;
}

void api_log_push(const blackbox_frame_t *frame)
{
	logger_push_frame(frame);
}

void logger_log_flight_data(uint32_t timestamp_us,
			    const flight_setpoints_t *sp,
			    const euler_angles_t *att,
			    const vector3f_t *gyro,
			    float pid_roll, float pid_pitch, float pid_yaw,
			    const motor_output_t *motors,
			    float battery_v,
			    uint8_t state, uint8_t mode, uint8_t failsafe)
{
	if (!s_file_open) {
		return;
	}

	blackbox_frame_t frame;
	frame.sync_byte = BLACKBOX_SYNC_MAGIC;
	frame.timestamp_us = timestamp_us;

	/* Target Setpoints */
	frame.target_roll_cdeg  = (int16_t)(sp->target_roll_deg * 100.0f);
	frame.target_pitch_cdeg = (int16_t)(sp->target_pitch_deg * 100.0f);
	frame.target_yaw_dps10  = (int16_t)(sp->target_yaw_rate_dps * 10.0f);
	frame.target_throttle   = (uint16_t)(sp->throttle * 1000.0f);

	/* Measured Attitude & Rates */
	frame.actual_roll_cdeg  = (int16_t)(att->roll * (18000.0f / 3.14159265f));
	frame.actual_pitch_cdeg = (int16_t)(att->pitch * (18000.0f / 3.14159265f));
	frame.actual_yaw_cdeg   = (int16_t)(att->yaw * (18000.0f / 3.14159265f));
	frame.gyro_x_dps10      = (int16_t)(gyro->x * (1800.0f / 3.14159265f));
	frame.gyro_y_dps10      = (int16_t)(gyro->y * (1800.0f / 3.14159265f));
	frame.gyro_z_dps10      = (int16_t)(gyro->z * (1800.0f / 3.14159265f));

	/* PID Demands */
	frame.pid_out_roll      = (int16_t)(pid_roll * 1000.0f);
	frame.pid_out_pitch     = (int16_t)(pid_pitch * 1000.0f);
	frame.pid_out_yaw       = (int16_t)(pid_yaw * 1000.0f);

	/* Actuator Motors */
	frame.motor_pulse_us[0] = (uint16_t)motors->pulse_us[0];
	frame.motor_pulse_us[1] = (uint16_t)motors->pulse_us[1];
	frame.motor_pulse_us[2] = (uint16_t)motors->pulse_us[2];
	frame.motor_pulse_us[3] = (uint16_t)motors->pulse_us[3];

	/* Health & Safety */
	frame.battery_mv        = (uint16_t)(battery_v * 1000.0f);
	frame.flight_state      = state;
	frame.flight_mode       = mode;
	frame.failsafe_flags    = failsafe;

	api_log_push(&frame);
}

static void sd_writer_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint8_t write_buffer[SD_WRITE_CHUNK_BYTES];
	s_last_sync_time_ms = (uint32_t)k_uptime_get_32();

	while (1) {
		/* Sleep until frames are placed into ring buffer, or 50 ms timeout */
		k_sem_take(&s_log_sem, K_MSEC(50));

		if (!s_file_open) {
			k_msleep(100);
			continue;
		}

		/* Drain ring buffer in 512-byte chunks */
		while (ring_buf_is_empty(&s_blackbox_rb) == 0) {
			uint32_t bytes_to_read = ring_buf_get(&s_blackbox_rb,
							      write_buffer,
							      sizeof(write_buffer));

			if (bytes_to_read > 0) {
				ssize_t written = fs_write(&s_log_file, write_buffer, bytes_to_read);
				if (written > 0) {
					s_written_frames += (uint32_t)written / sizeof(blackbox_frame_t);
				} else {
					LOG_ERR("SD write error: %d", (int)written);
				}
			}
		}

		/* Periodic 1-Second fs_sync() to preserve FAT clusters against crash power cuts */
		uint32_t now_ms = (uint32_t)k_uptime_get_32();
		if ((now_ms - s_last_sync_time_ms) >= 1000) {
			fs_sync(&s_log_file);
			s_last_sync_time_ms = now_ms;
		}
	}
}

void logger_sync(void)
{
	if (s_file_open) {
		fs_sync(&s_log_file);
		s_last_sync_time_ms = (uint32_t)k_uptime_get_32();
	}
}

void logger_close(void)
{
	if (s_file_open) {
		logger_sync();
		fs_close(&s_log_file);
		s_file_open = false;
		LOG_INF("Blackbox log file closed. Total frames: %u, Dropped: %u",
			s_written_frames, s_dropped_frames);
	}
}

bool logger_is_active(void)
{
	return s_file_open;
}

uint32_t logger_get_written_frames(void)
{
	return s_written_frames;
}

uint32_t logger_get_dropped_frames(void)
{
	return s_dropped_frames;
}
