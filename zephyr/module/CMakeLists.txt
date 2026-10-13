# THEJAS32 peripheral drivers not (yet) upstream in Zephyr.
#
# Upstream Zephyr's merged cdac/aries_v3 board port (zephyrproject-rtos/zephyr
# PR #114202) provides GPIO, UART (via the existing ns16550 driver), and the
# custom interrupt controller only -- see that PR and
# boards/cdac/aries_v3/aries_v3.yaml's `supported:` list (gpio, serial).
# I2C, SPI, PWM, and ADC drivers for this SoC do not exist upstream.
#
# This module adds them as a standard Zephyr out-of-tree module (the same
# pattern as zephyr/samples/application_development/out_of_tree_driver),
# so they can be dropped into any west workspace via ZEPHYR_EXTRA_MODULES
# without patching the Zephyr tree itself.
#
# Register-level behavior in drivers/*/*.c is cited against two
# independently-read sources for every protocol step:
#   1. The official vega-bsp Arduino core (cores/arduino/pwm.*,
#      libraries/{SPI,Wire}/src/*) -- register field layouts.
#   2. The official native VEGA SDK (cdac-vega.gitlab.io/vega-sdk Doxygen
#      pages for bsp/drivers/{i2c,spi,pwmc,adc}/*.c) -- the exact bit
#      sequences, which the Arduino core's own source does not fully show
#      (several of its register-field macros have no definition reachable
#      from the Arduino core repository alone).
# Both sources agree on every bit position checked; see each driver's
# header comment for the specific function/line each constant comes from.

add_subdirectory(drivers)
