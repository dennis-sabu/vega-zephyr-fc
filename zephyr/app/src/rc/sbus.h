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
 * Called periodically from the RC task thread.
 */
void sbus_process(void);

/**
 * @brief Retrieve the most recent decoded SBUS channels.
 *
 * @param channels Pointer to output sbus_channels_t structure.
 * @return true if valid frame has been decoded, false otherwise.
 */
bool sbus_get_channels(sbus_channels_t *channels);

/**
 * @brief Check if RC signal has been lost (timeout > 500 ms or failsafe flag).
 *
 * @return true if signal is lost, false if active and healthy.
 */
bool sbus_is_signal_loss(void);

#endif /* SBUS_H_ */
