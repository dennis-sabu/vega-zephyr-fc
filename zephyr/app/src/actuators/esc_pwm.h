/*
 * esc_pwm.h - Hardware Driver for Electronic Speed Controller (ESC) PWM Outputs
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Peripheral: PWM0 @ 0x10400000 (Channels 0-3, 400 Hz ESC Update Rate)
 *
 * Implements:
 * - FW-15: Standard 400 Hz ESC update rate (Period: 2,500,000 ns).
 * - Output pulse mapping: 1000 us (Idle) to 2000 us (Max).
 * - Global enable register assert (offset 0x80 on THEJAS32 PWM controller).
 * - Safety default: 900 us pulse width when disarmed.
 */

#ifndef ESC_PWM_H_
#define ESC_PWM_H_

#include <stdint.h>
#include <stdbool.h>

/* THEJAS32 PWM0 Memory Mapped Register Definitions */
#define THEJAS32_PWM0_BASE_ADDR         0x10400000
#define THEJAS32_PWM_GLOBAL_CTRL_OFFSET 0x80
#define THEJAS32_PWM_GLOBAL_ENABLE_BIT  (1U << 0)

/* Timing and Pulse Width Specifications */
#define ESC_PWM_FREQUENCY_HZ            400
#define ESC_PWM_PERIOD_NS               2500000U /* 2.5 ms = 400 Hz */
#define ESC_PULSE_MIN_US                1000U    /* 1000 us = Idle spin */
#define ESC_PULSE_MAX_US                2000U    /* 2000 us = Full throttle */
#define ESC_PULSE_DISARM_US             900U     /* 900 us = Forced disarm cutoff */

/* Channel Count */
#define ESC_NUM_CHANNELS                4

/**
 * @brief Initialize the PWM0 peripheral and configure 400 Hz ESC channels 0-3.
 * Asserts the THEJAS32 global enable register at offset 0x80 and forces all
 * channels to the disarmed 900 us pulse width.
 *
 * @return 0 on success, negative error code on failure.
 */
int esc_init(void);

/**
 * @brief Write microsecond pulse widths directly to ESC channels 1-4.
 * If disarmed, pulse widths are overridden to 900 us.
 * If armed, pulse widths are strictly clamped between 1000 us and 2000 us.
 *
 * @param m1 Front-Right motor pulse width in microseconds (Channel 0).
 * @param m2 Rear-Right motor pulse width in microseconds (Channel 1).
 * @param m3 Rear-Left motor pulse width in microseconds (Channel 2).
 * @param m4 Front-Left motor pulse width in microseconds (Channel 3).
 * @return 0 on success, negative error code on failure.
 */
int esc_write_pulses(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

/**
 * @brief Set the global arming state for the ESC actuator driver.
 *
 * @param armed true to allow active pulse generation (1000-2000 us);
 *              false to clamp all channels to disarm state (900 us).
 */
void esc_set_armed(bool armed);

/**
 * @brief Check if ESC output driver is currently armed.
 *
 * @return true if armed, false if disarmed.
 */
bool esc_is_armed(void);

/**
 * @brief Immediately force all ESC channels to 900 us and set disarmed state.
 */
void esc_disarm(void);

#endif /* ESC_PWM_H_ */
