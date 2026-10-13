# CMakeLists.txt - Build system configuration for VEGA ARIES v3 Flight Controller
# Target: C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V @ 100 MHz)
# Zephyr RTOS Application

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(aries_v3_flight_controller LANGUAGES C)

# Application include paths
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/actuators
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system
)

# Application source files
target_sources(app PRIVATE
    src/main.c
    src/actuators/esc_pwm.c
    src/sensors/mpu6050.c
    src/sensors/bmp280.c
    src/sensors/mag.c
    src/sensors/gnss.c
    src/control/ahrs.c
    src/control/pid.c
    src/control/mixer.c
    src/control/altitude.c
    src/rc/sbus.c
    src/system/battery.c
    src/system/failsafe.c
    src/system/telemetry.c
    src/system/logger.c
)
