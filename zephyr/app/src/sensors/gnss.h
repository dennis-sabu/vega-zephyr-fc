/*
 * gnss.h - GNSS Receiver NMEA 0183 Protocol Parser and UART Driver
 * Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
 * Peripheral: UART0 @ 0x10000100 (9600 / 115200 baud)
 */

#ifndef GNSS_H_
#define GNSS_H_

#include <stdbool.h>
#include <stdint.h>
#include "types.h"

/* Primary GNSS State Container */
typedef struct {
	double latitude;            /* Decimal degrees (-90.0 to +90.0) */
	double longitude;           /* Decimal degrees (-180.0 to +180.0) */
	float altitude_msl_m;       /* Altitude above Mean Sea Level in meters */
	float speed_knots;          /* Ground speed in knots */
	float speed_mps;            /* Ground speed in meters/second */
	float course_deg;           /* Course over ground in degrees (0.0 to 360.0) */
	uint8_t fix_quality;        /* 0 = Invalid, 1 = GPS Fix, 2 = DGPS, etc. */
	uint8_t satellites;         /* Number of active tracking satellites */
	float hdop;                 /* Horizontal Dilution of Precision */
	uint32_t time_utc;          /* UTC time formatted as hhmmss */
	uint32_t date_utc;          /* UTC date formatted as ddmmyy */
	uint32_t last_fix_time_ms;  /* System uptime timestamp of last valid fix */
	bool valid_fix;             /* True if 3D position fix is valid */
	uint32_t sentences_parsed;  /* Total valid NMEA sentences processed */
	uint32_t checksum_errors;   /* Total checksum mismatch errors rejected */
} gnss_data_t;

/**
 * @brief Initialize UART0 peripheral and setup ring buffer IRQ for GNSS NMEA reception.
 *
 * @param baudrate Desired baud rate (typically 9600 or 115200).
 * @return 0 on success, negative error code on failure.
 */
int gnss_init(uint32_t baudrate);

/**
 * @brief Process incoming bytes from the UART0 ring buffer, parse NMEA sentences,
 * verify XOR checksums, and update internal GNSS state.
 * Called periodically from telemetry or housekeeping thread.
 */
void gnss_process(void);

/**
 * @brief Retrieve the latest decoded GNSS position, velocity, and fix state.
 *
 * @param data Pointer to output gnss_data_t structure.
 * @return true if valid 3D GPS fix is available; false otherwise.
 */
bool gnss_get_data(gnss_data_t *data);

/**
 * @brief Check if GNSS receiver currently maintains a valid navigational fix.
 *
 * @return true if fix_quality > 0 and fix is fresh (< 2000 ms), false otherwise.
 */
bool gnss_has_fix(void);

#endif /* GNSS_H_ */
