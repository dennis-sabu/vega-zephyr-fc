/*
 * battery.h - LiPo Battery Voltage Monitor and ADC0 Driver
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Peripheral: ADC0 @ 0x10001000 (Channel 0)
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include "types.h"
#include "config.h"

/**
 * @brief Initialize ADC0 channel 0 and detect connected LiPo pack (3S vs 4S).
 *
 * @return 0 on success, negative error code on failure.
 */
int battery_init(void);

/**
 * @brief Read ADC0 Channel 0, compute voltage based on R1=10k / R2=2.2k divider,
 * apply exponential moving average filter, and update battery status.
 * Called periodically at 10 Hz.
 *
 * @return 0 on success, negative error code on failure.
 */
int battery_update(void);

/**
 * @brief Retrieve current battery status and voltages.
 *
 * @param info Pointer to output battery_info_t structure.
 */
void battery_get_info(battery_info_t *info);

/**
 * @brief Check if battery is at critical low-voltage threshold (< 3.3V per cell).
 *
 * @return true if critical voltage reached, false otherwise.
 */
bool battery_is_critical(void);

/**
 * @brief Get the current filtered pack voltage in Volts.
 *
 * @return Filtered pack voltage.
 */
float battery_get_voltage(void);

#endif /* BATTERY_H_ */
