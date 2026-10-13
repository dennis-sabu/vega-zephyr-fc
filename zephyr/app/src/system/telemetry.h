/*
 * telemetry.h - Real-Time Telemetry Stream over UART2 (FW-18)
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Peripheral: UART2 @ 0x10000300 (115,200 baud, 8N1)
 *
 * Implements:
 * - Low-priority 10 Hz telemetry streaming thread.
 * - Non-blocking comma-separated ASCII sentence with XOR checksum.
 * - MultiWii Serial Protocol (MSP) binary packet support.
 * - Thread-safe state payload updating.
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <stdint.h>
#include <stdbool.h>
#include "types.h"

/**
 * @brief Initialize UART2 peripheral for 115,200 baud 8N1 telemetry streaming.
 *
 * @return 0 on success, negative error code on failure.
 */
int telemetry_init(void);

/**
 * @brief Thread-safely update telemetry flight data cache.
 *
 * @param roll_deg Current estimated roll angle in degrees.
 * @param pitch_deg Current estimated pitch angle in degrees.
 * @param yaw_deg Current estimated yaw heading in degrees.
 * @param throttle Normalized throttle demand [0.0f, 1.0f].
 * @param battery_v Current battery voltage in Volts.
 * @param is_armed Current arming status.
 * @param flight_mode Active flight mode (0=Angle, 1=Rate, 2=AltHold).
 * @param failsafe_active True if failsafe is engaged.
 */
void telemetry_set_state(float roll_deg, float pitch_deg, float yaw_deg,
			 float throttle, float battery_v, bool is_armed,
			 uint8_t flight_mode, bool failsafe_active);

/**
 * @brief Transmit a formatted ASCII telemetry frame over UART2.
 * Format: $TELEM,<time_ms>,<roll>,<pitch>,<yaw>,<thr>,<vbat>,<arm>,<mode>,<fs>*<CS>\r\n
 *
 * @return Number of characters transmitted.
 */
int telemetry_send_ascii_frame(void);

/**
 * @brief Transmit a MultiWii Serial Protocol (MSP_ATTITUDE) binary frame over UART2.
 *
 * @return Number of bytes transmitted.
 */
int telemetry_send_msp_attitude(void);

/**
 * @brief Single iteration step of the 10 Hz telemetry worker.
 */
void telemetry_step(void);

/**
 * @brief Spawn and start the background 10 Hz telemetry thread.
 */
void telemetry_start(void);

#endif /* TELEMETRY_H_ */
