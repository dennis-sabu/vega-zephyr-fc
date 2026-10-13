# VEGA ARIES v3.0 Flight Controller: Hardware Architecture, PCB Design & Bring-Up Specification

**Target Platform:** C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V 32-bit Core @ 100 MHz)  
**Airframe:** S500 Quadcopter (~2.5 kg AUW, Quad-X Configuration, 120A Peak Propulsion Current)  
**Document Revision:** 1.0 (Hardware Translation for Production Zephyr Firmware)

---

## 1. Schematic Pinout & Component Mapping

### 1.1 Master Electrical Interface Matrix

| Peripheral | SoC Physical Pin / Pad | Target Device | Signal Name | Electrical Characteristics & Passive Network |
| :--- | :--- | :--- | :--- | :--- |
| **Power Input** | XT60 / Main Bus | External LiPo | `VBAT_RAW` | 3S–4S LiPo (9.9V – 16.8V). Must pass through **TVS clamp** (`SMBJ20A`). |
| **Power Reg** | Internal Rail | Buck Regulator | `+5V_SYS` | **TI TPS54302** (or MP2315) Synchronous Buck: 12V $\rightarrow$ 5.0V @ 3A. |
| **Power Reg** | Internal Rail | Low-Noise LDO | `+3V3_SYS` | **TI TPS7A2033** Ultra-Low Noise LDO: 5.0V $\rightarrow$ 3.3V @ 500mA. |
| **ADC0** | `ADC0_CH0` (`0x10001000`) | Battery Sense | `VBAT_SENSE` | Divider: $R_1=10\text{ k}\Omega$, $R_2=2.2\text{ k}\Omega$ ($1\%$), $C_f=100\text{ nF}$, Clamp Zener $3.3\text{V}$. |
| **I2C0 SCL** | `GPIO0_2` / `I2C0_SCL` | Sensors Bus | `I2C0_SCL` | 3.3V Open-Drain, **$2.2\text{ k}\Omega$ pull-up to `+3V3_SYS`**, Fast-Mode 400 kHz. |
| **I2C0 SDA** | `GPIO0_3` / `I2C0_SDA` | Sensors Bus | `I2C0_SDA` | 3.3V Open-Drain, **$2.2\text{ k}\Omega$ pull-up to `+3V3_SYS`**, Fast-Mode 400 kHz. |
| **PWM0 Ch 0** | `PWM0_OUT0` (`0x10400000`)| ESC 1 (FR, CCW) | `MOTOR1_PWM` | 3.3V Push-Pull, **$220\ \Omega$ series resistor**, 400 Hz (1000–2000 µs). |
| **PWM0 Ch 1** | `PWM0_OUT1` (`0x10400000`)| ESC 2 (RR, CW)  | `MOTOR2_PWM` | 3.3V Push-Pull, **$220\ \Omega$ series resistor**, 400 Hz (1000–2000 µs). |
| **PWM0 Ch 2** | `PWM0_OUT2` (`0x10400000`)| ESC 3 (RL, CCW) | `MOTOR3_PWM` | 3.3V Push-Pull, **$220\ \Omega$ series resistor**, 400 Hz (1000–2000 µs). |
| **PWM0 Ch 3** | `PWM0_OUT3` (`0x10400000`)| ESC 4 (FL, CW)  | `MOTOR4_PWM` | 3.3V Push-Pull, **$220\ \Omega$ series resistor**, 400 Hz (1000–2000 µs). |
| **UART0 RX** | `UART0_RXD` (`0x10000100`)| GNSS NMEA TX    | `GNSS_RX`   | 3.3V CMOS Level (Level-shifted from 5V GNSS module), 9600 / 115200 baud. |
| **UART0 TX** | `UART0_TXD` (`0x10000100`)| GNSS NMEA RX    | `GNSS_TX`   | 3.3V CMOS Level, inline $100\ \Omega$ series resistor. |
| **UART1 RX** | `UART1_RXD` (`0x10000200`)| SBUS Receiver   | `SBUS_IN`   | 100,000 baud, 8E2. Dedicated hardware inverter (e.g., **74LVC1G04**). |
| **UART2 TX** | `UART2_TXD` (`0x10000300`)| Telemetry Stream| `TELEM_TX`  | 3.3V CMOS Level, 115,200 baud, 4-pin JST-GH connector. |
| **UART2 RX** | `UART2_RXD` (`0x10000300`)| Telemetry Ground| `TELEM_RX`  | 3.3V CMOS Level, 115,200 baud, 4-pin JST-GH connector. |
| **SPI0 SCK** | `SPI0_SCK`  (`0x10000600`)| MicroSD Card    | `SD_SCK`    | 25 MHz SPI Clock, **$33\ \Omega$ series damping resistor**. |
| **SPI0 MOSI**| `SPI0_MOSI` (`0x10000600`)| MicroSD Card    | `SD_MOSI`   | 3.3V Push-Pull, **$33\ \Omega$ series damping resistor**. |
| **SPI0 MISO**| `SPI0_MISO` (`0x10000600`)| MicroSD Card    | `SD_MISO`   | 3.3V Input, **$47\text{ k}\Omega$ pull-up to `+3V3_SYS`**. |
| **SPI0 CS**  | `GPIO1_10`                | MicroSD Card    | `SD_CS`     | 3.3V Active-Low Chip Select, **$10\text{ k}\Omega$ pull-up to `+3V3_SYS`**. |

---

### 1.2 Power Supply & Conditioning Architecture

```
               +-------------------+
  3S/4S LiPo   |  TVS Diode        |
  [9.9V-16.8V] |  (SMBJ20A)        |
  ======+======+========+==========+===================================> High-Current PDB
        |               |                                                (120A Peak to 4x ESCs)
        |         +-----+-----+
        |         | LC Filter |
        |         | (10uH+22uF|
        |         +-----+-----+
        |               |
        |      +--------+------------------+
        |      | 5V Buck (TPS54302)        |
        |      | 12V -> 5.0V @ 3A          |
        |      +--------+------------------+
        |               |
        |               +-----------------------> +5V_SYS Rail (GNSS, RC Receiver, Telemetry)
        |               |
        |      +--------+------------------+
        |      | 3.3V LDO (TPS7A2033)      |
        |      | 5.0V -> 3.3V @ 500mA      |
        |      +--------+------------------+
        |               |
        |               +-----------------------> +3V3_SYS Rail (THEJAS32 SoC, MPU6050, BMP280, SD)
        |
        |      +---------------------------+
        +----->| Scaled Voltage Divider    |
               | R1=10k, R2=2.2k (1%)      |----> ADC0_CH0 (0.0V - 3.03V Max)
               +---------------------------+
```

1. **Transient Surge Protection (TVS):**
   - Connect an **SMBJ20A** (unidirectional, 600W peak pulse, breakdown voltage $V_{BR}=22.2\text{ V}$) across `VBAT_RAW` and `GND` at the board entry connector to absorb motor back-EMF spikes during aggressive decelerations.
2. **5V System Buck Regulator (TI TPS54302):**
   - Input: 8V – 28V; Output: $5.0\text{ V} \pm 1.5\%$ @ 3A continuous.
   - Output ripple: $< 20\text{ mV}_{p-p}$ using a $4.7\ \mu\text{H}$ shielded inductor (Würth 74438356047) and $2\times 22\ \mu\text{F}$ X7R ceramic output capacitors.
   - Dedicated exclusively to off-board peripheral modules (GPS/Galileo, SBUS Receiver, 500mW 433/915 MHz Telemetry Transceiver).
3. **3.3V Sensor & SoC LDO (TI TPS7A2033):**
   - Input: Driven from the clean `+5V_SYS` rail (NOT directly from the noisy battery rail).
   - High Power Supply Rejection Ratio (PSRR: 95 dB @ 1 kHz) suppresses remaining buck regulator switching noise.
   - Powers the THEJAS32 SoC, external SPI flash, MPU6050, BMP280, and MicroSD card.

---

### 1.3 Voltage Divider & Analog Conditioning Circuit

To match `BATTERY_VOLTAGE_DIVIDER_RATIO = 5.545455` defined in firmware:

$$V_{\text{ADC}} = V_{\text{BAT}} \cdot \frac{R_2}{R_1 + R_2} = V_{\text{BAT}} \cdot \frac{2.2\text{ k}\Omega}{10\text{ k}\Omega + 2.2\text{ k}\Omega} = \frac{V_{\text{BAT}}}{5.545455}$$

- **Component Selection:**
  - $R_1$: $10.0\text{ k}\Omega \pm 0.1\%$, 0603 Thin-Film (Low temp-drift: $\le 25\text{ ppm/}^\circ\text{C}$).
  - $R_2$: $2.20\text{ k}\Omega \pm 0.1\%$, 0603 Thin-Film.
  - $C_f$: $100\text{ nF}$ X7R in parallel with $R_2$ forming an anti-aliasing low-pass filter ($f_c \approx 880\text{ Hz}$).
  - $D_{\text{clamp}}$: **BAT54S** dual Schottky clamp diode to `+3V3_SYS` and `GND` to prevent overvoltage injection into the THEJAS32 ADC core under fault conditions.

---

### 1.4 I2C0 Bus Pull-Up Network

- The shared I2C0 bus connects three sensor nodes: **MPU6050** (`0x68`), **BMP280** (`0x76`), and **QMC5883L** (`0x0D`).
- Total estimated bus capacitance: $C_{\text{bus}} \approx 30\text{ pF}$ (traces) $+ 3 \times 10\text{ pF}$ (pins) $= 60\text{ pF}$.
- For 400 kHz Fast-Mode, rise time requirement is $t_r \le 300\text{ ns}$:

$$R_p \le \frac{t_r}{0.8473 \cdot C_{\text{bus}}} = \frac{300\times 10^{-9}}{0.8473 \cdot 60\times 10^{-12}} \approx 5.9\text{ k}\Omega$$

- Minimum pull-up (drive limit $3\text{ mA}$ at $V_{OL} = 0.4\text{ V}$):

$$R_p \ge \frac{3.3\text{ V} - 0.4\text{ V}}{3\text{ mA}} \approx 966\ \Omega$$

- **Selected Value:** Install **$2.2\text{ k}\Omega \pm 1\%$ 0402 pull-up resistors** connected to `+3V3_SYS` placed physically adjacent to the SoC pins. Do **not** use the internal sensor breakout module pull-ups (cut or remove on-module resistors to prevent parallel impedance degradation below $1\text{ k}\Omega$).

---

### 1.5 Actuator PWM Protection Circuitry

- ESC signal lines run long parallel traces across carbon-fiber arms to 30A switching inverters.
- Place a **$220\ \Omega \pm 5\%$ 0402 resistor in series** with each PWM output (`PWM0_OUT0` .. `PWM0_OUT3`) within 5 mm of the THEJAS32 SoC output pins.
- Add an ultra-low capacitance TVS array (**Semtech RClamp0504F**) to ground to protect SoC I/O cells against ESD during field handling and high-voltage motor ground-bounce transients.

---

## 2. Strict PCB Layout & Routing Guidelines

### 2.1 4-Layer Stackup Recommendation

Use an engineered symmetrical 4-layer stackup with controlled dielectric thickness:

```
[Layer 1: Top Signal / Components]  - 1 oz Cu (35 um)  --> High-speed SPI, I2C, ADC, Sensors
---------------------------------  Dielectric: Prepreg 2116 (0.10 mm / 4.0 mil)
[Layer 2: Ground Plane (GND)]       - 1 oz Cu (35 um)  --> Solid, unbroken reference ground plane
---------------------------------  Dielectric: FR4 Core (1.20 mm / 47.2 mil)
[Layer 3: Power Distribution]       - 1 oz Cu (35 um)  --> Split Planes: +5V_SYS, +3V3_SYS
---------------------------------  Dielectric: Prepreg 2116 (0.10 mm / 4.0 mil)
[Layer 4: Bottom Signal]            - 1 oz Cu (35 um)  --> PWM to ESCs, UARTs, Debug LEDs
```

> [!CRITICAL]
> **Total Ground Plane Integrity (Layer 2):**
> Layer 2 **MUST REMAIN AN UNBROKEN COPPER PLANE**. Absolutely no signal traces are permitted to route through Layer 2. Any void or slot cut in Layer 2 by via clusters forces return currents to take loop detours, generating fatal electromagnetic emissions and ground bounce that will lock up the I2C0 bus during motor throttle transitions.

---

### 2.2 Noise Isolation & 120A Propulsion Ground Decoupling

1. **Galvanic Separation of Motor Power:**
   - **DO NOT route the 120A motor current through the flight controller PCB.**
   - All high-current traces and power leads must remain strictly on an isolated external Power Distribution Board (PDB) or 4-in-1 ESC plate mounted below the flight controller.
2. **Ground Loop Prevention (Star Ground):**
   - Connect the ESC signal ground wires directly to the FC ESC connector grounds.
   - Connect the FC primary ground to the battery ground via a **single, designated 20 AWG wire** at the battery voltage divider ground pad.
3. **Switching Regulator Isolation:**
   - Place the TPS54302 Buck converter in a dedicated corner of the PCB, as far as physically possible from the MPU6050 and ADC0.
   - Keep the high-di/dt switching loop (input capacitor $\rightarrow$ high-side FET $\rightarrow$ catch diode $\rightarrow$ input capacitor GND) under **$25\text{ mm}^2$ in total loop area**.
4. **ADC Route Shielding:**
   - Route `VBAT_SENSE` as a short, shielded analog trace on Layer 1 over the solid Layer 2 GND plane.
   - Surround `VBAT_SENSE` with ground fill guard traces; do not route parallel to any PWM, clock, or switching lines.

---

### 2.3 Sensor Physical Placement Rules

#### A. Inertial Measurement Unit (MPU6050)
- **Positioning:** Mount at the **exact geometric center of the PCB**, which must align with the physical Center of Gravity (CG) of the assembled S500 quadcopter within $\pm 2.0\text{ mm}$.
- **Mechanical Isolation:**
  - Route a mechanical milling slot (C-shaped router cutout, width $1.0\text{ mm}$) around the IMU section of the PCB, leaving a small bridge tab to attenuate high-frequency acoustic vibrations originating from the motor arms.
  - Mount the flight controller stack to the S500 frame using **M3 silicone vibration-damping bobbins** (durometer 30A–40A).
- **Thermal Isolation:** Do not place high-dissipation components (buck inductor, linear regulators) within $15\text{ mm}$ of the MPU6050 to prevent thermal gradient drift on the gyroscope zero-rate offset.

#### B. Magnetometer / Compass (QMC5883L)
- **Magnetic Interference Physics:**
  A 120A propulsion current flowing in a battery cable at distance $r = 50\text{ mm}$ generates a magnetic field:

$$B = \frac{\mu_0 \cdot I}{2\pi \cdot r} = \frac{(4\pi \times 10^{-7}) \cdot 120}{2\pi \cdot 0.05} = 480\ \mu\text{T}$$

  The Earth's horizontal magnetic field is typically only **$20\ \mu\text{T} \text{ to } 50\ \mu\text{T}$**. An onboard compass will suffer complete magnetic saturation and permanent loss of yaw reference during flight.
- **Strict Rule:**
  - **Do NOT locate the primary compass on the main flight controller PCB.**
  - Route I2C0 signals to an **external GPS/Compass puck mounted on a $12\text{ cm}$ elevated carbon-fiber mast** located above the plane of the motors and battery harness.
  - If placed onboard for development only, it must sit at the absolute leading edge of the board with a minimum $60\text{ mm}$ clearance from the main battery pads and any iron/ferrous mounting screws (use brass, nylon, or titanium hardware exclusively).

#### C. Barometer (BMP280)
- **Light & Turbulence Shielding:**
  - The silicon diaphragm of the BMP280 is photo-sensitive; direct sunlight exposure through the airhole creates instantaneous altitude transients of $> 5\text{ meters}$.
  - Place a piece of **black, open-cell acoustic foam** (thickness $3\text{ mm}$) directly over the BMP280 metal lid.
  - Encapsulate the sensor in a 3D-printed enclosure or protective shroud to shield it from dynamic aerodynamic pressure (propeller downwash turbulence) while maintaining static ambient pressure exposure.

---

## 3. Hardware Bring-Up Protocol (Smoke Test)

Follow this rigorous, sequential procedure before ever installing propellers or attaching a flight battery.

```
+---------------------------------------------------------------------------------------------------+
|                                  HARDWARE BRING-UP CHECKLIST                                      |
+---------------------------------------------------------------------------------------------------+
  [Step 1: Cold Impedance Test]           [Step 2: Current-Limited Power]
  - Check 3.3V to GND: > 500 Ohm          - Set Bench Supply: 12.0V, 100mA clamp
  - Check 5.0V to GND: > 1000 Ohm         - Connect to VBAT pads
  - Check VBAT to GND: > 10 kOhm          - Verify current draw is < 45mA
                |                                       |
                v                                       v
  [Step 3: Voltage Rail Inspection]       [Step 4: JTAG & Peripheral Audit]
  - 5V Rail: Verify 4.95V - 5.05V         - Connect JTAG (OpenOCD / RISC-V GDB)
  - 3.3V Rail: Verify 3.28V - 3.32V       - Flash Zephyr firmware image
  - Ripple: Confirm < 25mV on scope       - Run I2C scan: confirm 0x68, 0x76, 0x0D
+---------------------------------------------------------------------------------------------------+
```

### Step 1: Unpowered Cold Continuity & Impedance Testing
1. Disconnect all external devices (cables, sensors, JTAG programmers).
2. Set digital multimeter to Resistance ($\Omega$) mode. Measure resistance to GND:
   - **`VBAT_RAW` to `GND`:** Must read $> 10\text{ k}\Omega$ (rising as decoupling caps charge). If $< 100\ \Omega$, check TVS diode orientation and electrolytic capacitors.
   - **`+5V_SYS` to `GND`:** Must read $> 1\text{ k}\Omega$. If near $0\ \Omega$, inspect buck regulator output caps and inductor solder bridges.
   - **`+3V3_SYS` to `GND`:** Must read $> 500\ \Omega$. If $< 50\ \Omega$, check THEJAS32 SoC orientation and sensor bypass capacitors.
3. Test continuity between `+5V_SYS` and `+3V3_SYS`: Must read **OPEN CIRCUIT**. A short here will destroy the 3.3V THEJAS32 core immediately upon power-up.

---

### Step 2: First Power-On via Current-Limited Bench Supply
1. **DO NOT CONNECT A LIPO BATTERY.** LiPo batteries can deliver $> 100\text{ A}$ into a short, vaporizing traces and chips instantly.
2. Configure a laboratory DC bench power supply:
   - **Voltage:** Set to **$12.0\text{ V}$ DC** (simulating a nominal 3S pack).
   - **Current Limit:** Clamp strictly at **$100\text{ mA}$**.
3. Connect the bench supply leads to the `VBAT_RAW` and `GND` pads.
4. Switch ON the supply and immediately observe the current indicator:
   - **Expected current:** **$25\text{ mA} \text{ to } 45\text{ mA}$** (quiescent regulator and un-programmed SoC draw).
   - **If current hits the $100\text{ mA}$ clamp:** Switch OFF in $< 1\text{ second}$. Inspect the board with a thermal camera or isopropyl alcohol evaporation test to locate the short.

---

### Step 3: Rail Voltage & Noise Verification
Using an oscilloscope (20 MHz bandwidth limit, AC coupling for ripple):
1. **Measure `+5V_SYS` Test Point:**
   - DC Voltage: $4.95\text{ V} \le V_{\text{out}} \le 5.08\text{ V}$.
   - Ripple: Peak-to-peak switching ripple must be $< 30\text{ mV}_{p-p}$.
2. **Measure `+3V3_SYS` Test Point:**
   - DC Voltage: $3.27\text{ V} \le V_{\text{out}} \le 3.33\text{ V}$.
   - Ripple: High-frequency noise must be $< 10\text{ mV}_{p-p}$.
3. **Measure ADC Voltage Divider Tap (`VBAT_SENSE`):**
   - At $12.0\text{ V}$ input: $V_{\text{sense}} = \frac{12.0\text{ V}}{5.545455} = 2.164\text{ V} \pm 0.03\text{ V}$.
   - Confirm BAT54S protection diodes are not conducting or loading the node.

---

### Step 4: Firmware Flashing & I2C Device Discovery
1. Connect the RISC-V JTAG adapter (Olimex ARM-USB-OCD-H or FT2232D) to the THEJAS32 JTAG header.
2. Launch OpenOCD and verify core target detection:
   ```bash
   openocd -f interface/ftdi/jtag.cfg -f target/thejas32.cfg
   ```
3. Flash the compiled Zephyr flight controller binary:
   ```bash
   west flash
   ```
4. Attach a USB-UART adapter to UART2 (`TELEM_TX` / `TELEM_RX` @ 115,200 baud).
5. Verify clean boot logs in terminal:
   - `mpu6050: MPU6050 identity verified: 0x68`
   - `bmp280: BMP280 confirmed at addr 0x76 with Chip ID 0x58`
   - `mag: Found QMC5883L Magnetometer at 0x0D`
   - `gnss: GNSS receiver initialized on UART0`
   - `blackbox: MicroSD Card mounted successfully at /SD:`

---

## 4. Restrained Bench Test Protocol (Pre-Flight Safety)

> [!WARNING]
> **MANDATORY SAFETY INSTRUCTION:**
> **ALL PROPELLERS MUST BE COMPLETELY REMOVED FROM ALL FOUR MOTORS BEFORE COMMENCING BENCH TESTING.** No exceptions are permitted under any circumstances. Restraining an aircraft with propellers installed can lead to blade detachment, motor burnout, or severe personal injury.

```
       Front
  M4 (CW)   M1 (CCW)       Quad-X S500 Layout
     \         /           Motor 1: Front-Right, Counter-Clockwise (CCW)
      \       /            Motor 2: Rear-Right,  Clockwise (CW)
       [ ARIES ]           Motor 3: Rear-Left,   Counter-Clockwise (CCW)
      /       \            Motor 4: Front-Left,  Clockwise (CW)
     /         \
  M3 (CCW)  M2 (CW)
       Rear
```

### 4.1 Sensor Orientation & Attitude Verification

Connect the flight controller to the telemetry monitor (UART2 @ 115,200 baud or ground control station) and observe the real-time attitude feedback:

1. **Roll Axis Verification:**
   - Tilt the aircraft **right wing down (Starboard tilt)**:
     - Output telemetry `roll_deg` must show a **positive increase ($+0^\circ \rightarrow +30^\circ$)**.
   - Tilt the aircraft **left wing down (Port tilt)**:
     - Output telemetry `roll_deg` must show a **negative decrease ($-0^\circ \rightarrow -30^\circ$)**.
2. **Pitch Axis Verification:**
   - Tilt the aircraft **nose up**:
     - Output telemetry `pitch_deg` must show a **positive increase ($+0^\circ \rightarrow +30^\circ$)**.
   - Tilt the aircraft **nose down**:
     - Output telemetry `pitch_deg` must show a **negative decrease ($-0^\circ \rightarrow -30^\circ$)**.
3. **Yaw / Compass Heading Verification:**
   - Rotate the nose **clockwise (turning right / towards East)**:
     - Output telemetry `yaw_deg` must show a **positive clockwise rotation ($0^\circ \rightarrow 90^\circ \rightarrow 180^\circ$)**.

---

### 4.2 Actuator Channel Mapping & Rotation Direction Test

With propellers removed and ESCs connected to the 12V bench supply:

1. **Verify Disarmed PWM State:**
   - With transmitter disarmed (AUX1 switch low), measure PWM channel outputs with an oscilloscope or servo tester:
   - Channels 0, 1, 2, 3 must strictly read **$900\ \mu\text{s} \pm 2\ \mu\text{s}$ at 400 Hz**. ESCs should emit initial ready beeps but must remain completely stationary.
2. **Arming Transition:**
   - Lower throttle stick to zero ($< 5\%$). Keep airframe level ($< 10^\circ$ tilt).
   - Flip AUX1 switch to **ARMED**:
   - Verify all four channels step to the armed idle setpoint: **$1100\ \mu\text{s} \pm 5\ \mu\text{s}$**.
   - Motors 1–4 must start spinning smoothly at idle RPM.
3. **Physical Rotation Direction Audit:**
   - Visually check or use a paper card gently touching the bell to verify mechanical direction:
     - **Motor 1 (Front-Right):** Must spin **Counter-Clockwise (CCW)**.
     - **Motor 2 (Rear-Right):** Must spin **Clockwise (CW)**.
     - **Motor 3 (Rear-Left):** Must spin **Counter-Clockwise (CCW)**.
     - **Motor 4 (Front-Left):** Must spin **Clockwise (CW)**.
   - *Correction:* If any motor spins backward, swap any two of the three bullet-connector phase wires between the ESC and motor.
4. **Attitude Control Authority (Hand Tilt Test):**
   - Advance throttle stick to 30% (motors spinning at moderate RPM).
   - Physically tilt the quadcopter **forward (nose down)**:
     - Flight loop mixer equations demand: Front motors (M1, M4) must immediately **increase RPM**, while rear motors (M2, M3) must **decrease RPM** to counter the disturbance.
   - Physically tilt the quadcopter **right (right wing down)**:
     - Right motors (M1, M2) must immediately **increase RPM**, while left motors (M3, M4) must **decrease RPM**.

---

### 4.3 Safety-Critical Failsafe Matrix Validation

Execute these tests on the bench to verify that safety interlocks trigger cleanly:

| Test Scenario | Action Performed | Expected Firmware Response | Pass/Fail Criteria |
| :--- | :--- | :--- | :--- |
| **1. RC Link Loss** | With motors spinning at 30% throttle, **turn off the handheld RC transmitter**. | After **500 ms**, the system flags `FAILSAFE_REASON_RC_LOST`, sets level attitude, ramps throttle down over 5 seconds, and **disarms motors to 900 µs**. | **PASS:** All motors stop spinning within 5.5 seconds of transmitter power-down. |
| **2. Crash Inversion** | With motors spinning at 30% throttle, **invert the board beyond $75^\circ$ tilt**. | System flags `FAILSAFE_REASON_EXCESSIVE_TILT` and triggers `STATE_EMERGENCY_STOP`. | **PASS:** Motors immediately cut to 0 throttle / 900 µs in $< 4\text{ ms}$ (one flight loop cycle). |
| **3. Critical Battery** | Lower bench supply voltage to **$9.8\text{ V}$** (simulating depleted 3S pack $< 3.3\text{V/cell}$). | After **2.0 seconds** of sustained low voltage, system triggers `FAILSAFE_REASON_LOW_BATTERY` and engages forced landing ramp. | **PASS:** Controlled descent throttle engages; motors disarm upon timer expiry. |
| **4. Arming Interlocks** | Tilt the quadcopter to $25^\circ$ or advance throttle stick to 50%; attempt to flip the ARM switch. | Safety supervisor blocks arming; logs: `Arming blocked: Excessive tilt angle` or `Throttle stick not at zero`. | **PASS:** Motors do NOT spin; system remains in `STATE_DISARMED`. |

---

## 5. Summary Wiring Harness Diagram

```
[3S/4S LiPo] ====> [External 120A PDB Board]
                         |
                         +---> [ESC 1 (30A)] =====> Motor 1 (Front-Right, CCW)
                         +---> [ESC 2 (30A)] =====> Motor 2 (Rear-Right, CW)
                         +---> [ESC 3 (30A)] =====> Motor 3 (Rear-Left, CCW)
                         +---> [ESC 4 (30A)] =====> Motor 4 (Front-Left, CW)
                         |
      +------------------+ (VBAT_RAW + GND Single Pair)
      |
      v
+----------------------------------------------------------------------------------+
|                   VEGA ARIES v3.0 FLIGHT CONTROLLER (THEJAS32)                   |
|                                                                                  |
|  [Power Section]                                                                 |
|   - TVS Clamp (SMBJ20A)                                                          |
|   - 5V Buck (TPS54302) ----------> Offboard Ports (+5V_SYS)                      |
|   - 3.3V LDO (TPS7A2033) --------> SoC & Sensor Buses (+3V3_SYS)                 |
|   - Divider (10k/2.2k) ----------> ADC0 Channel 0 (Voltage Sense)                |
|                                                                                  |
|  [Actuator Ports]                                                                |
|   - PWM0_OUT0 (220 Ohm series) --> ESC 1 Signal                                  |
|   - PWM0_OUT1 (220 Ohm series) --> ESC 2 Signal                                  |
|   - PWM0_OUT2 (220 Ohm series) --> ESC 3 Signal                                  |
|   - PWM0_OUT3 (220 Ohm series) --> ESC 4 Signal                                  |
|                                                                                  |
|  [Sensors & Comms Ports]                                                         |
|   - I2C0 (400kHz, 2.2k Pullup) --> Onboard MPU6050, BMP280                       |
|   - I2C0 Port (JST-GH) ----------> External Mast Compass (QMC5883L)              |
|   - UART0 Port (JST-GH) ---------> External GNSS Module (NMEA @ 9600)            |
|   - UART1 Port (74LVC1G04 Invert)-> SBUS Radio Receiver (100k baud)              |
|   - UART2 Port (JST-GH) ---------> 915MHz Telemetry Transceiver (115.2k)         |
|   - SPI0 Slot (SDHC-SPI Mode) ---> MicroSD Card Socket (Blackbox Logs)           |
+----------------------------------------------------------------------------------+
```
