/*
 * sbus.h - SBUS RC Protocol Decoder and UART Driver
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * UART1 @ 0x10000200 (100,000 baud, 8E2)
 */

#ifndef SBUS_H_
#define SBUS_H_

#include "types.h"
#include "config.h"

/**
 * @brief Initialize UART1 peripheral for SBUS reception with IRQ-driven ring buffer.
 *
 * @return 0 on success, negative error code on failure.
 */
int sbus_init(void);

/**
 * @brief Process bytes accumulated in ring buffer, parse frames, and update channels.
 * Called periodically from the rc_receiver_thread (10 ms / 100 Hz).
 */
void sbus_process(void);

/**
 * @brief Retrieve raw and normalized SBUS channel data.
 *
 * @param channels Pointer to output sbus_channels_t structure.
 * @return true if valid frame has been received, false otherwise.
 */
bool sbus_get_channels(sbus_channels_t *channels);

/**
 * @brief Thread-safe retrieval of normalized flight setpoints using Zephyr mutex.
 * - Throttle: 0.0 to 1.0
 * - Roll / Pitch: -30.0 deg to +30.0 deg (Angle mode)
 * - Yaw Rate: -180.0 deg/s to +180.0 deg/s
 *
 * @param setpoints Pointer to output flight_setpoints_t structure.
 * @return true if setpoints are valid and RC link is alive; false otherwise.
 */
bool sbus_get_setpoints(flight_setpoints_t *setpoints);

/**
 * @brief Check if RC signal has been lost (timeout > 500 ms or failsafe flag).
 *
 * @return true if signal is lost, false if active and healthy.
 */
bool sbus_is_signal_loss(void);

#endif /* SBUS_H_ */
