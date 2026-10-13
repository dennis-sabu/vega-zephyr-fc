# VEGA ARIES v3.0 Flight Controller for S500 Multirotor

[![Architecture](https://img.shields.io/badge/Architecture-RISC--V%2032--bit-red.svg)](https://riscv.org/)
[![SoC](https://img.shields.io/badge/SoC-C--DAC%20THEJAS32%20%40%20100MHz-orange.svg)](https://vegaprocessors.in/)
[![RTOS](https://img.shields.io/badge/RTOS-Zephyr%20RTOS%20v3.5.0-blue.svg)](https://zephyrproject.org/)
[![Airframe](https://img.shields.io/badge/Airframe-S500%20Quad--X%20(~2.5kg)-green.svg)]()
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

A production-grade, open-source flight controller firmware and hardware specification designed for the **VEGA ARIES v3.0 Development Board**, powered by the indigenous **C-DAC THEJAS32 SoC** (VEGA ET1031 RISC-V 32-bit core running at 100 MHz). Built on **Zephyr RTOS**, this flight controller delivers deterministic real-time attitude estimation, dual-cascaded PID stabilization, autonomous altitude hold, and blackbox data logging on an S500 multirotor airframe.

---

## Technical Highlights

- **Processor:** C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V 32-bit @ 100 MHz).
- **Strict Memory Budget:** 256 KB On-Chip SRAM (Zero dynamic memory allocation in control loops; static thread stacks and lock-free ring buffers).
- **Real-Time Multithreading (Zephyr RTOS):**
  - **Thread 1 (250 Hz / 4 ms, Prio 1):** Primary flight control loop (MPU6050 IMU acquisition, Complementary AHRS, Dual-Cascaded Rate/Angle PID, Quad-X Mixer).
  - **Thread 2 (50 Hz / 20 ms, Prio 2):** Cascaded altitude hold controller (BMP280 Barometer, hypsometric altitude, EMA variometer).
  - **Thread 3 (100 Hz / 10 ms, Prio 3):** SBUS RC receiver parser (UART1 @ 100,000 baud 8E2, 25-byte frame unpacker, mutex-protected setpoints).
  - **Thread 4 (10 Hz / 100 ms, Prio 6):** Housekeeping, battery voltage monitor (ADC0), failsafe supervisor, and live telemetry streamer (UART2 @ 115,200 baud).
  - **Thread 5 (Background, Prio 7):** Preemptive Blackbox data logger (FatFS MicroSD card via SPI0 @ 25 MHz, 64-frame lock-free ring buffer, periodic 1s sync).
- **I2C Bus Optimization:** Hardware-level compliance with THEJAS32 16-byte burst read constraint (BMP280 24-byte calibration split into two 12-byte burst reads; MPU6050 14-byte single burst read).
- **Comprehensive Failsafe Matrix:**
  - 500 ms RC loss timeout $\rightarrow$ Level attitude auto-hold with 5-second smooth throttle ramp down to zero, followed by disarm.
  - Low battery cutoff ($< 3.3\text{V/cell}$ sustained for $> 2.0\text{ s}$) $\rightarrow$ Controlled descent.
  - Crash detection ($> 75^\circ$ roll or pitch tilt) $\rightarrow$ Instantaneous motor cutoff ($900\ \mu\text{s}$) in $< 4\text{ ms}$.

---

## Hardware Interconnect & Wiring Architecture

```
[3S/4S LiPo Battery] ====> [External 120A PDB Board]
                                 |
                                 +---> [ESC 1 (30A)] =====> Motor 1 (Front-Right, CCW)
                                 +---> [ESC 2 (30A)] =====> Motor 2 (Rear-Right, CW)
                                 +---> [ESC 3 (30A)] =====> Motor 3 (Rear-Left, CCW)
                                 +---> [ESC 4 (30A)] =====> Motor 4 (Front-Left, CW)
                                 |
              +------------------+ (VBAT_RAW + GND Single 20 AWG Pair)
              |
              v
+----------------------------------------------------------------------------------+
|                   VEGA ARIES v3.0 FLIGHT CONTROLLER (THEJAS32)                   |
|                                                                                  |
|  [Power Section]                                                                 |
|   - TVS Surge Clamp (SMBJ20A)                                                    |
|   - 5V Buck (TI TPS54302) --------> Offboard 5V Ports (+5V_SYS @ 3A)             |
|   - 3.3V Ultra-Low Noise LDO -----> SoC & Sensor Buses (+3V3_SYS @ 500mA)        |
|   - Voltage Divider (10k/2.2k) ---> ADC0 Channel 0 (0.0V - 3.03V Max)            |
|                                                                                  |
|  [Actuator Ports]                                                                |
|   - PWM0_OUT0 (220 Ohm series) ---> ESC 1 Signal (400 Hz, 1000-2000 us)          |
|   - PWM0_OUT1 (220 Ohm series) ---> ESC 2 Signal (400 Hz, 1000-2000 us)          |
|   - PWM0_OUT2 (220 Ohm series) ---> ESC 3 Signal (400 Hz, 1000-2000 us)          |
|   - PWM0_OUT3 (220 Ohm series) ---> ESC 4 Signal (400 Hz, 1000-2000 us)          |
|                                                                                  |
|  [Sensors & Comms Ports]                                                         |
|   - I2C0 (400 kHz, 2.2k Pullup) --> Onboard MPU6050 (0x68), BMP280 (0x76)        |
|   - I2C0 Port (4-Pin JST-GH) -----> External Mast Compass (QMC5883L @ 0x0D)      |
|   - UART0 Port (4-Pin JST-GH) ----> External GNSS Module (NMEA @ 9600 baud)      |
|   - UART1 Port (74LVC1G04 Invert)-> SBUS Radio Receiver (100k baud, 8E2)         |
|   - UART2 Port (4-Pin JST-GH) ----> 915 MHz Telemetry Transceiver (115.2k baud)  |
|   - SPI0 Slot (SDHC-SPI Mode) ----> MicroSD Card Socket (Blackbox Logs)          |
+----------------------------------------------------------------------------------+
```

---

## Repository Structure

```
.
├── HARDWARE_SPEC_AND_BENCH_TESTING.md    # Master Hardware Specification & Bench Bring-Up Guide
├── FLIGHT_TEST_PROGRAM_AND_TUNING.md     # Flight Test Program (F1-F6) & Cascaded PID Tuning Guide
├── PCB_PRODUCTION_AND_MANUFACTURING.md   # Final PCB Manufacturing (P1-P5) & Production Release Package
├── zephyr/
│   └── app/
│       ├── CMakeLists.txt                # Zephyr Application CMake Build Configuration
│       ├── prj.conf                      # Zephyr Kconfig Options (RTOS, FatFS, Drivers)
│       ├── boards/
│       │   └── aries_v3.overlay          # DeviceTree Overlay for THEJAS32 Peripheral Mapping
│       └── src/
│           ├── main.c                    # Real-Time Scheduling Executive & 5 Preemptive Threads
│           ├── config.h                  # Loop Rates, Memory Budgets, PID Gains, Failsafe Limits
│           ├── types.h                   # Primary Data Structures, Vectors, Enums
│           ├── actuators/
│           │   ├── esc_pwm.h             # FW-15: ESC PWM Output Driver (pwm0 @ 0x10400000)
│           │   └── esc_pwm.c
│           ├── sensors/
│           │   ├── mpu6050.h             # InvenSense MPU6050 6-DOF IMU Driver (I2C0)
│           │   ├── mpu6050.c
│           │   ├── bmp280.h              # FW-06: Bosch BMP280 Barometer Driver (I2C0)
│           │   ├── bmp280.c
│           │   ├── mag.h                 # FW-07: QMC5883L / HMC5883L Magnetometer Driver (I2C0)
│           │   ├── mag.c
│           │   ├── gnss.h                # FW-08: NMEA 0183 Protocol Parser & UART0 Driver
│           │   └── gnss.c
│           ├── control/
│           │   ├── ahrs.h                # Discrete Complementary Attitude Estimator
│           │   ├── ahrs.c
│           │   ├── pid.h                 # Discrete Filtered Derivative PID Controller
│           │   ├── pid.c
│           │   ├── mixer.h               # Quad-X Motor Mixer & Pulse Normalizer
│           │   ├── mixer.c
│           │   ├── altitude.h            # FW-12/13: Cascaded Altitude Hold Controller (50 Hz)
│           │   └── altitude.c
│           ├── rc/
│           │   ├── sbus.h                # FW-16: SBUS RC Input Driver & Frame Unpacker (UART1)
│           │   └── sbus.c
│           └── system/
│               ├── battery.h             # FW-17: LiPo Battery Monitor & EMA Filter (ADC0)
│               ├── battery.c
│               ├── failsafe.h            # FW-20: Safety Supervisor & Failsafe State Machine
│               ├── failsafe.c
│               ├── telemetry.h           # FW-18: 10 Hz Telemetry Streamer over UART2
│               ├── telemetry.c
│               ├── logger.h              # FW-19: Blackbox FatFS MicroSD Logger (SPI0)
│               └── logger.c
```

---

## Quick Start & Firmware Build Guide

### Prerequisites
1. **Zephyr SDK:** Install Zephyr SDK (v0.16.0 or later) with the `riscv64-zephyr-elf` cross-compiler.
2. **West Tool:** Install the West build meta-tool:
   ```bash
   pip install west
   ```
3. **OpenOCD:** Install OpenOCD with FTDI driver support for the THEJAS32 JTAG adapter.

### Building the Firmware
Clone the repository and initialize the build environment:

```bash
# Navigate to the workspace
cd c:/dev/vega

# Build the flight controller image for VEGA ARIES v3
west build -b aries_v3 zephyr/app -- -DEXTRA_DTC_OVERLAY_FILE="boards/aries_v3.overlay"
```

The compiled binary will be located at `build/zephyr/zephyr.bin`.

### Flashing via OpenOCD JTAG
Connect your Olimex ARM-USB-OCD-H or FT2232D JTAG debugger to the ARIES v3 JTAG header:

```bash
# Flash the image to THEJAS32 on-chip flash
west flash
```

Alternatively, invoke OpenOCD directly:

```bash
openocd -f interface/ftdi/jtag.cfg -f target/thejas32.cfg \
    -c "init" \
    -c "reset halt" \
    -c "flash write_image erase build/zephyr/zephyr.bin 0x10000000" \
    -c "reset run" \
    -c "shutdown"
```

---

## Flight Modes & Control

| Mode | Indicator | Primary Control Loops | Stick Behavior |
| :--- | :--- | :--- | :--- |
| **`MODE_RATE`** | Acro Mode | 250 Hz Inner Rate PID ($K_p, K_i, K_d$) | Roll/Pitch sticks directly command angular rates (deg/s). Vehicle holds attitude when stick is released. |
| **`MODE_ANGLE`** | Stabilize Mode | 50 Hz Outer Angle $K_p \rightarrow$ 250 Hz Inner Rate PID | Roll/Pitch sticks command tilt angles ($\pm 30^\circ$). Vehicle self-levels when sticks are centered. |
| **`MODE_ALT_HOLD`**| Altitude Hold | 50 Hz Position Z $\rightarrow$ Velocity Z $\rightarrow$ Collective Throttle | Throttle center deadband ($\pm 8\%$, $[0.42, 0.58]$) holds altitude within $\pm 0.3\text{ m}$. Sticks command $\pm 1.0\text{ m/s}$ vertical climb/descent. |

---

## Documentation Links

- 🛠️ **[Hardware Architecture & Bench Testing Specification](file:///c:/dev/vega/HARDWARE_SPEC_AND_BENCH_TESTING.md)**: Master pinout matrix, passive component sizing, power conditioning, 4-layer PCB layout rules, and unpowered smoke test protocol.
- ✈️ **[Flight Test Program (F1–F6) & PID Tuning Guide](file:///c:/dev/vega/FLIGHT_TEST_PROGRAM_AND_TUNING.md)**: Gated maiden test protocol (F1–F4), cascaded PID field tuning sequence, autonomous navigation verification (F6), and Python Blackbox log decoder.
- 🏭 **[PCB Manufacturing & Production Release Package (P1–P5)](file:///c:/dev/vega/PCB_PRODUCTION_AND_MANUFACTURING.md)**: Design freeze sign-off checklist, Gerber/CPL/BOM export guide for JLCPCB/PCBWay, automated factory test fixture script, and open-source release hierarchy.

---

## License

This project is released under the **Apache License 2.0**. See [`LICENSE`](LICENSE) for details. Developed for research, aerospace education, and autonomous multirotor applications on RISC-V architectures.
