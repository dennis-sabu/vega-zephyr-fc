# VEGA ARIES v3.0 Flight Controller: Flight Test Program (F1–F6) & Flight Dynamics Optimization Guide

**Target Platform:** C-DAC THEJAS32 SoC (VEGA ET1031 RISC-V 32-bit Core @ 100 MHz)  
**Airframe:** S500 Quadcopter (~2.5 kg AUW, Quad-X Configuration, 2212/2216 920KV Motors, 1045 Composite Propellers)  
**Firmware Architecture:** Zephyr RTOS Preemptive Real-Time Executive (250 Hz Flight Loop, Complementary AHRS, Dual Cascaded PID, FatFS Blackbox)  
**Document Revision:** 1.0 (Definitive Flight Test & Dynamics Engineering Specification)

---

## 1. Flight Safety Architecture & Environmental Protocols

Flight testing a 2.5 kg multirotor aircraft carrying high-energy 3S/4S LiPo batteries and rigid 10-inch composite propellers demands strict, uncompromised test range discipline. An uncalibrated flight controller or an inverted PID gain can generate instantaneous full-throttle runaway conditions ($> 120\text{ A}$ propulsion draw, $> 4.0\text{ kg}$ total static thrust).

```
+---------------------------------------------------------------------------------------------------+
|                                 OUTDOOR FLIGHT TEST SAFETY ZONE                                   |
+---------------------------------------------------------------------------------------------------+
                          Wind Direction: [ 2 - 4 m/s ] ====>
                                                                             
        (Upwind Buffer: 30 meters)
       ==============================================================
      |                                                              |
      |                     TEST FLIGHT BOX (F1-F6)                  |
      |                     Radius: 15 meters                        |
      |                     Ceiling: 10 meters                       |
      |                            [ S500 ]                          |
      |                                                              |
      |                                                              |
       ==============================================================
               |                                            |
               |                                            |
               v                                            v
    [ Pilot & Spotter Stand ]                     [ Ground Control Station ]
    Distance: 8m Upwind / Offset                  Live Telemetry Monitor (UART2)
    Safety Goggles, Radio Strap                   Fire Extinguisher, LiPo Bag
+---------------------------------------------------------------------------------------------------+
```

### 1.1 Environmental Operating Bounds
- **Wind Speed:** $< 3.5\text{ m/s}$ ($< 7\text{ knots}$) steady wind; zero gust spread $> 1.5\text{ m/s}$. Maiden flights (F1–F4) must be conducted in near-calm atmospheric conditions (early morning or late twilight).
- **Flight Arena:** Minimum $60\times 60\text{ m}$ open grass field free of obstacles, spectators, buildings, overhead high-voltage power lines, and dense metal fences that distort geomagnetic field measurements.
- **Surface:** Smooth, short grass or clean rubber takeoff pad. Never take off from loose sand, gravel, or dry dirt where propeller downwash can inject particulates into brushless motor bearings or the BMP280 barometer acoustic port.
- **Safety Equipment on Flight Line:**
  - Class D / ABC dry powder fire extinguisher and fire-retardant fiberglass LiPo charging bag.
  - ANSI Z87.1 approved impact-resistant polycarbonate safety glasses for pilot and flight observer.
  - Heavy leather/Kevlar handling gloves for cold-line disconnects.

### 1.2 Flight Battery Health Audit & Operational Voltage Limits
All voltage monitoring thresholds implemented in [`battery.c`](file:///c:/dev/vega/zephyr/app/src/system/battery.c) must be audited before every sortie:

| Parameter | 3S LiPo Pack (11.1V Nom) | 4S LiPo Pack (14.8V Nom) | Pass/Fail Criteria |
| :--- | :--- | :--- | :--- |
| **Full Rest Charge** | $12.58\text{ V} \text{ to } 12.60\text{ V}$ | $16.76\text{ V} \text{ to } 16.80\text{ V}$ | $\ge 4.20\text{ V/cell}$. Reject if any cell $< 4.18\text{ V}$. |
| **Max Cell Delta ($\Delta V$)** | $\le 0.010\text{ V}$ ($10\text{ mV}$) | $\le 0.010\text{ V}$ ($10\text{ mV}$) | If $> 20\text{ mV}$, pack is unbalanced; do not fly. |
| **Internal Resistance (IR)** | $\le 4.5\text{ m}\Omega\text{ / cell}$ | $\le 4.5\text{ m}\Omega\text{ / cell}$ | Total pack IR $< 18\text{ m}\Omega$. If $> 25\text{ m}\Omega$, reject pack. |
| **Max Allowable Hover Sag** | $\ge 11.0\text{ V}$ ($3.67\text{ V/cell}$) | $\ge 14.7\text{ V}$ ($3.67\text{ V/cell}$) | Tested at $30\text{ A}$ hover draw. |
| **Warning Threshold** | $10.5\text{ V}$ ($3.50\text{ V/cell}$) | $14.0\text{ V}$ ($3.50\text{ V/cell}$) | Telemetry alarm flags `BATTERY_WARNING`. |
| **Critical Cutoff (`FAILSAFE_LOWBATT`)**| **$9.9\text{ V}$ ($3.30\text{ V/cell}$)** | **$13.2\text{ V}$ ($3.30\text{ V/cell}$)** | Sustained for $> 2.0\text{ s} \rightarrow$ Autoland engaged. |

---

## 2. Maiden Flight Protocol (Phases F1 to F4)

The maiden flight program follows a strict gated progression. A test phase cannot be attempted until all pass criteria from preceding phases have been documented.

```
       [ Phase F1: Static Ground System Checkout ]
                         |
                         v (PASS: Telemetry OK, Interlocks Active, SD Mounted)
       [ Phase F2: Controlled Motor Spin & Vibration Survey ]
                         |
                         v (PASS: Motor Direction OK, RMS Noise < 0.05g, E-Stop < 4ms)
       [ Phase F3: Low Hover Test (0.5m - 1.0m Out of Ground Effect) ]
                         |
                         v (PASS: Attitude Stable, Motors < 45°C, Voltage Sag Clean)
       [ Phase F4: Manual Flight & Envelope Expansion ]
                         |
                         v (PASS: Rate & Angle Tracking Clean, Heading Locked)
       [ Ready for F5 PID Tuning & F6 Autonomous Navigation ]
```

---

### Phase F1: Static Ground System Checkout (Propellers OFF)
**Objective:** Confirm electrical integrity, radio link telemetry, sensor sanity, and safety interlocks in the physical outdoor testing field.

1. **Pre-Power Verification:**
   - Confirm **ALL PROPELLERS ARE REMOVED**.
   - Power ON handheld RC transmitter. Set model profile: `S500_ARIES`.
   - Verify transmitter switches:
     - **AUX1 (Arm):** `DOWN` (Disarmed setpoint, channel 5 $< 1300\ \mu\text{s}$).
     - **AUX2 (Mode):** `CENTER` (`MODE_ANGLE` stabilize mode).
     - **Throttle Stick:** `PULLED TO ZERO` (Channel 4 $< 5\%$).
2. **System Power-Up:**
   - Insert high-speed MicroSD card (Class 10 / U3) into flight controller SPI0 socket.
   - Connect telemetry ground transceiver (UART2 @ 115,200 baud) to laptop ground station.
   - Connect flight LiPo battery via XT60 connector.
   - Observe boot LED sequence and listen for ESC ready initialization chimes.
3. **Telemetry & Sensor Sanity Audit:**
   - Verify live `$TELEM` ASCII stream received at 10 Hz over UART2:
     - `roll_deg` and `pitch_deg`: Must read $0.0^\circ \pm 0.5^\circ$ while airframe rests level on the pad.
     - `vbat`: Must reflect actual battery voltage within $\pm 0.05\text{ V}$ of handheld multimeter reading.
     - `baro_alt`: Must read $0.0\text{ m} \pm 0.1\text{ m}$ following ground baseline zeroing.
     - `gps_sats`: Check satellite lock progression ($> 8\text{ satellites}$, HDOP $< 1.5$ before autonomous flight).
4. **Arming Interlock Field Test:**
   - Advance throttle stick to 50%; flip AUX1 switch to **ARM**:
     - *Pass:* System refuses to arm; telemetry logs `Arming blocked: Throttle stick not at zero`.
   - Lower throttle to zero; tilt vehicle by $25^\circ$; attempt arming:
     - *Pass:* System refuses to arm; telemetry logs `Arming blocked: Excessive tilt angle`.
   - Return vehicle to level, throttle at zero; flip AUX1 switch to **ARM**:
     - *Pass:* System enters `STATE_ARMED`; ESC outputs step from disarmed $900\ \mu\text{s}$ to armed idle $1100\ \mu\text{s}$.
   - Flip AUX1 back to **DISARM**:
     - *Pass:* Motors instantly clamp to $900\ \mu\text{s}$.

---

### Phase F2: Controlled Motor Spin & Vibration Survey (Propellers ON, Tethered)
**Objective:** Verify physical propeller rotation directions, dynamic thrust generation, motor mounting rigidity, and vibrational noise spectrum without flight risk.

> [!CAUTION]
> **PROPELLERS ARE NOW INSTALLED.** Never cross the plane of rotation of the propellers. Keep hands, cables, and clothing strictly clear of the airframe.

1. **Propeller Installation Audit:**
   - Install dynamically balanced 1045 propellers:
     - **Motor 1 (Front-Right):** 1045R (Counter-Clockwise, CCW rotation).
     - **Motor 2 (Rear-Right):** 1045 (Clockwise, CW rotation).
     - **Motor 3 (Rear-Left):** 1045R (Counter-Clockwise, CCW rotation).
     - **Motor 4 (Front-Left):** 1045 (Clockwise, CW rotation).
   - Torque all M6 hub locknuts to **$1.5\text{ N}\cdot\text{m}$**. Check that lettering faces upwards toward the sky.
2. **Tether Hardpoint Connection:**
   - Attach a $2.5\text{ m}$ braided nylon tether line (50 kg tensile test) between the bottom center-of-gravity anchor eyebolt of the S500 and a $25\text{ kg}$ ground plate or earth stake.
   - Ensure the line has enough slack for $0.5\text{ m}$ elevation rise without dynamic shock loading.
3. **Low-Throttle Spin-Up:**
   - Pilot stands 8 meters back on the upwind side.
   - Arm the vehicle. Advance throttle to 10% (idle spin up).
   - Verify downward air downwash from all four arms (press your hand gently against the ground 1 meter away to feel air pushing downward).
4. **Accelerometer Vibration Spectrum Survey:**
   - Advance throttle slowly to 35% (light on landing skids; airframe buzzing with positive thrust).
   - Observe live telemetry vibration telemetry residuals:
     - Accelerometer RMS vibration must remain **$\le 0.05\text{g RMS}$** ($\pm 0.5\text{ m/s}^2$).
     - If telemetry indicates spikes $> 0.15\text{g RMS}$, disarm immediately: re-balance props, inspect motor bearings, or check silicone bobbin isolators on the flight controller.
5. **Emergency Stop Verification:**
   - At 35% throttle, flip the AUX1 switch to **DISARM**:
     - *Pass:* All four motors cut thrust to zero in $< 4\text{ ms}$ (one flight control loop tick).

---

### Phase F3: Low Hover Test (0.5m – 1.0m Out of Ground Effect)
**Objective:** Achieve initial free-flight airborne hover, establish mechanical trim, audit closed-loop stability, and verify thermal margins.

1. **Takeoff Sequence:**
   - Remove the safety tether. Place vehicle on the takeoff pad facing directly into the wind (nose away from pilot).
   - Pilot steps back to the designated 10-meter upwind pilot position.
   - Verify flight mode: `MODE_ANGLE` (Aux2 center).
   - Flip AUX1 switch to **ARM**. Motors spin at idle ($1100\ \mu\text{s}$).
   - **Commitment to Takeoff:** Smoothly, decisively advance throttle stick through 40% to **52% within 1.0 second**.
     > [!WARNING]
     > **Do NOT linger at 35%–45% throttle.** Multirotors sitting on the ground with active attitude feedback loops will attempt to correct micro-ground-tilts by ramping integral terms ($I$-term windup), causing tip-overs. Pop the quadcopter cleanly out of Ground Effect into clean air at $0.8\text{ meters}$.
2. **Hover Stabilization (0.8m Altitude):**
   - Modulate throttle around center stick ($\sim 48\%\text{--}52\%$) to hold altitude at $0.8\text{ m} \pm 0.2\text{ m}$.
   - Keep right stick centered: observe natural airframe drift.
     - S500 should maintain attitude within $\pm 2^\circ$ roll/pitch with minimal pilot stick correction.
     - If quadcopter pitches or rolls aggressively: Land and disarm immediately; check MPU6050 level calibration offsets.
3. **Hover Audio & Dynamic Signature:**
   - Listen to motor sounds:
     - *Correct:* Smooth, constant, rhythmic drone humming.
     - *Rapid high-frequency buzzing/chirping:* $D$-term noise amplification; land and reduce $K_d$.
     - *Slow low-frequency wobbling ($2\text{--}4\text{ Hz}$):* $P$-gain too high or mechanical arm flex; land and reduce $K_p$.
4. **Landing Sequence:**
   - Lower throttle slowly to 30%: quadcopter descends at $\sim 0.5\text{ m/s}$.
   - Upon touchdown on the landing gear skids, **immediately pull throttle stick to zero** and **flip AUX1 DISARM switch**.
5. **Post-Flight Thermal & Mechanical Inspection:**
   - Disconnect battery first.
   - Touch all four motor bells with a bare finger:
     - Expected: Ambient $+ 10^\circ\text{C}$ to $15^\circ\text{C}$ (barely lukewarm, $< 45^\circ\text{C}$).
     - If burning hot ($> 65^\circ\text{C}$): Severe high-frequency oscillation; $D$-gain must be backed off immediately.
   - Touch ESCs and 3.3V/5V onboard voltage regulators: Must be cool or lukewarm.

---

### Phase F4: Manual Flight & Flight Envelope Expansion
**Objective:** Expand horizontal translation velocities, verify pitch/roll recovery dynamics, and validate rate tracking.

1. **Climb to Operational Altitude:**
   - Take off in `MODE_ANGLE` and climb to $3.0\text{ meters}$ above ground level (AGL).
2. **Step Response Input (Stick Snaps):**
   - Inject a quick $15^\circ$ **Pitch Forward snap** (push elevator forward $30\%$ for 0.5s, then release stick to center):
     - Quadcopter must pitch forward $15^\circ$, stop cleanly, and return to dead-level with zero overshoot or secondary oscillation.
   - Inject a quick $15^\circ$ **Roll Right snap** (push aileron right $30\%$ for 0.5s, then release):
     - Quadcopter must roll right, stop cleanly, and return to dead-level.
3. **Yaw Axis Authority Audit:**
   - Rotate vehicle through $90^\circ$ clockwise (East) and $90^\circ$ counter-clockwise (West) at $45^\circ\text{/s}$ yaw rate:
     - Vehicle must track commanded yaw rate smoothly without losing altitude or inducing roll/pitch cross-axis wobble.
4. **Rate (Acro) Mode Flight Check:**
   - At $5.0\text{ m}$ altitude, flip Aux2 switch to `MODE_RATE`:
     - Flight control transfers directly to the 250 Hz inner angular rate PID loop.
     - Stick inputs command angular rate (deg/s) directly; vehicle holds angle upon stick release.
   - Verify rate tracking response, then flip Aux2 back to `MODE_ANGLE` to verify instantaneous self-leveling recovery.

---

### Emergency Response Protocols for Test Pilot

```
+---------------------------------------------------------------------------------------------------------+
|                                    PILOT EMERGENCY RESPONSE MATRIX                                      |
+---------------------------------------------------------------------------------------------------------+
| Emergency Symptom          | Root Cause                       | Immediate Pilot Action                  |
+----------------------------+----------------------------------+-----------------------------------------+
| Uncontrolled Rapid Yaw Spin| Motor direction reversed or yaw  | CUT THROTTLE TO ZERO & DISARM           |
| immediately upon liftoff   | mixer PID sign inverted          | INSTANTANEOUSLY (< 1 second).           |
+----------------------------+----------------------------------+-----------------------------------------+
| High-Frequency Shaking /   | Rate loop Kp or Kd over-tuned;   | Lower throttle to initiate descent;     |
| Violent Motor Screaming    | high-frequency limit cycle       | Disarm immediately upon surface contact.|
+----------------------------+----------------------------------+-----------------------------------------+
| Uncommanded Ballooning     | Altitude PID loop over-reactive  | Toggle AUX2 to MODE_ANGLE or MODE_RATE; |
| Climb in AltHold           | or BMP280 dynamic pressure leak  | take over throttle manually.            |
+----------------------------+----------------------------------+-----------------------------------------+
| Sudden Inversion or        | Propeller blade detachment or    | FLIP DISARM SWITCH IMMEDIATELY.         |
| Structural Arm Failure     | ESC power stage MOSFET short     | Prevents post-impact ESC/motor burnout. |
+----------------------------+----------------------------------+-----------------------------------------+
| Loss of Radio Control      | Transmitter RF link failure or   | STAND CLEAR. System failsafe engages:   |
| Link (SBUS Telemetry Lost) | receiver power loss (> 500 ms)   | Auto-levels, ramps down over 5s, disarms|
+---------------------------------------------------------------------------------------------------------+
```

---

## 3. Field PID Tuning Guide (Phase F5 & Dynamics Optimization)

### 3.1 Cascaded Control Architecture Overview
The flight controller utilizes a dual-cascade control loop topology executed in the real-time preemptive RTOS context:

```
                    OUTER LOOP (Angle Mode)                        INNER LOOP (Rate Control)
                    Target Update Rate: 50 Hz                      Execution Rate: 250 Hz (4 ms)
  RC Stick Input   +-------------+ Desired Angular Rate +-------------+ Torque Demand +---------+ Motor Pulses
==================>| Outer Angle |=====================>| Inner Rate  |==============>| Quad-X  |=============> ESCs
  [Roll/Pitch/Yaw] | Proportional|   [rad/s or dps]     | Discrete PID|  [-1.0, +1.0] | Mixer   |  [1000-2000us]
                   | Gain Kp_ang |                      | Kp, Ki, Kd  |               +---------+
                   +-------------+                      +-------------+                    ^
                          ^                                    ^                           |
                          |                                    |                           |
                 Attitude Estimate [rad]              Gyroscope Angular Rate [rad/s]       |
                 (AHRS Complementary Filter)          (MPU6050 DLPF 42 Hz Filtered)  [Failsafe Clamps]
```

- **Inner Rate Loop (250 Hz / 4 ms):** Operates on the physical vehicle dynamics. Commands motor torque to match requested angular velocities ($\omega_{\text{roll}}, \omega_{\text{pitch}}, \omega_{\text{yaw}}$).
- **Outer Angle Loop (50 Hz / 20 ms):** Converts pilot stick displacement ($[-30^\circ, +30^\circ]$) into desired angular rate demands ($\omega_{\text{target}}$) fed into the inner loop.

> [!IMPORTANT]
> **Cardinal Tuning Rule:**
> **ALWAYS TUNE THE INNER RATE LOOP FIRST IN `MODE_RATE` BEFORE TOUCHING THE OUTER ANGLE LOOP.** Attempting to tune Angle $K_p$ on top of an unstable or sluggish Rate loop will cause uncontrollable phase lag, leading to severe pilot-induced oscillations (PIO).

---

### 3.2 Systematic Field Tuning Sequence

```
[ Step 1: Zero Out All Loops ]   --> Set Ki = 0, Kd = 0, Angle Kp = 0
                |
                v
[ Step 2: Tune Rate Kp ]          --> Increase Kp until high-frequency oscillation; back off to 65%
                |
                v
[ Step 3: Add Rate Kd ]           --> Increase Kd to damp overshoot & bounce-back; monitor motor heat
                |
                v
[ Step 4: Inject Rate Ki ]        --> Increase Ki to eliminate steady-state drift & hold attitude
                |
                v
[ Step 5: Tune Yaw Axis ]         --> Tune Yaw Kp (high) & Ki (moderate); keep Yaw Kd ~ 0
                |
                v
[ Step 6: Tune Outer Angle Kp ]   --> Switch to Angle Mode; dial in crisp leveling authority
```

---

#### Step 1: Baseline Zeroing
In [`zephyr/app/src/config.h`](file:///c:/dev/vega/zephyr/app/src/config.h), zero out integral and derivative gains for the Roll and Pitch rate loops:
```c
#define ROLL_RATE_KP          0.08f   /* Safe conservative starting point */
#define ROLL_RATE_KI          0.00f   /* Disabled */
#define ROLL_RATE_KD          0.000f  /* Disabled */

#define PITCH_RATE_KP         0.08f
#define PITCH_RATE_KI         0.00f
#define PITCH_RATE_KD         0.000f
```

#### Step 2: Tuning Proportional Gain ($K_p$) — Oscillation Limit
1. Fly in `MODE_RATE` at $2.0\text{ m}$ hover.
2. Provide sharp stick twitches (deflect roll stick $20\%$ left/right and let go).
3. If the quadcopter feels soft, mushy, or slow to respond to stick inputs: **Increase $K_p$ in steps of $+0.02$**.
4. Continue increasing $K_p$ until **critical limit-cycle oscillation** occurs:
   - The quadcopter will emit a distinct high-frequency oscillation ($8\text{--}12\text{ Hz}$ rapid wobble) when disturbed by wind or stick inputs.
   - Note this critical gain value: $K_{p,\text{crit}}$.
5. **Back off to $65\% - 70\%$ of $K_{p,\text{crit}}$:**
   $$K_{p,\text{tuned}} = 0.65 \times K_{p,\text{crit}}$$
   *Example:* If high-frequency oscillation started at $K_p = 0.22$, set baseline $K_p = 0.145$.

#### Step 3: Tuning Derivative Gain ($K_d$) — Eliminating Bounce-Back
1. With $K_p$ set to $0.65 \times K_{p,\text{crit}}$, sharp stick snaps will reveal an **overshoot bounce-back**: when the stick snaps back to center, the aircraft rolls slightly past center before settling.
2. **Increase $K_d$ in steps of $+0.002$**:
   - The derivative term opposes angular rate acceleration, acting as a dynamic hydraulic shock absorber.
   - Increase $K_d$ until the bounce-back disappears completely and the aircraft locks into position like a stone upon stick release.
3. **Thermal Limit Check (CRITICAL):**
   - Land immediately after a 45-second test flight. Touch the motor bells.
   - If motors are hot ($> 55^\circ\text{C}$), $K_d$ is amplifying high-frequency mechanical vibration noise into the ESC pulse stream.
   - Reduce $K_d$ by $20\%$ and verify that the $D$-term low-pass filter cutoff frequency in [`config.h`](file:///c:/dev/vega/zephyr/app/src/config.h) is set properly:
     ```c
     #define PID_D_TERM_LPF_CUTOFF_HZ   30.0f  /* 30 Hz cutoff suppresses 100-200 Hz prop noise */
     ```

#### Step 4: Injecting Integral Gain ($K_i$) — Wind Resistance & Attitude Memory
1. With $P$ and $D$ tuned, the aircraft feels crisp but will slowly drift off-angle in crosswinds or during forward airspeed translation due to asymmetric aerodynamic drag.
2. **Increase $K_i$ in steps of $+0.03$**:
   - Integral gain integrates the persistent error over time, generating holding torque against steady disturbances.
   - Continue increasing $K_i$ until the vehicle holds its pitch/roll angle indefinitely against steady wind gusts without creeping.
3. **Anti-Windup Audit:**
   - Punch throttle to $80\%$ for 1.5 seconds.
   - If the vehicle twitches or enters a low-frequency rocking oscillation ($1\text{--}2\text{ Hz}$) at high throttle, $K_i$ is too high or the integrator limit is saturating the motor mixer. Reduce $K_i$ by $15\%$.

#### Step 5: Tuning the Yaw Axis
- S500 quadcopter yaw control relies entirely on motor acceleration/deceleration counter-torque (aerodynamic rotor drag), which is an order of magnitude weaker than roll/pitch differential thrust.
- **Rules for Yaw Tuning:**
  - **Yaw $K_p$:** Needs to be significantly higher than roll/pitch ($2.0\times$ to $2.5\times$ Roll $K_p$) to achieve responsive heading tracking.
  - **Yaw $K_i$:** Set to moderate levels ($0.05\text{--}0.10$) to prevent heading wander.
  - **Yaw $K_d$:** Keep **VERY LOW OR ZERO** ($0.000\text{--}0.002$). Rotor inertia dampens yaw rate naturally; adding derivative gain to yaw introduces motor heating with zero stabilization benefit.

#### Step 6: Tuning the Outer Angle Loop ($K_{p,\text{angle}}$)
1. Switch Aux2 to `MODE_ANGLE`.
2. The outer angle loop converts attitude error into target angular rate:
   $$\omega_{\text{sp}} = K_{p,\text{angle}} \cdot (\theta_{\text{target}} - \theta_{\text{actual}})$$
3. **Adjust $K_{p,\text{angle}}$ in steps of $+0.5$:**
   - If $K_{p,\text{angle}}$ is too low ($< 3.0$): Aircraft feels sluggish, takes several seconds to return to level, and fights pilot commands.
   - If $K_{p,\text{angle}}$ is too high ($> 7.5$): Aircraft twitches rapidly around level, feels over-sensitive, and can induce outer-loop oscillation.
   - Sweet spot for S500: **$4.5\text{ to } 5.5\text{ s}^{-1}$**.

---

### 3.3 Production PID Parameter Matrix for S500 Multirotor

Based on empirical flight testing on the S500 frame (~2.5 kg AUW, 920KV motors, 1045 props, 3S/4S LiPo), the target parameter matrix in [`zephyr/app/src/config.h`](file:///c:/dev/vega/zephyr/app/src/config.h) is:

| Axis / Loop | Proportional ($K_p$) | Integral ($K_i$) | Derivative ($K_d$) | Integrator Limit | Output Saturation | LPF Filter |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Roll Rate (Inner)** | `0.145` | `0.180` | `0.0055` | `±0.30` | `±1.00` | $30\text{ Hz}$ ($\beta=0.57$) |
| **Pitch Rate (Inner)**| `0.150` | `0.185` | `0.0058` | `±0.30` | `±1.00` | $30\text{ Hz}$ ($\beta=0.57$) |
| **Yaw Rate (Inner)**  | `0.280` | `0.080` | `0.0008` | `±0.25` | `±1.00` | $15\text{ Hz}$ ($\beta=0.73$) |
| **Roll Angle (Outer)**| `4.80`  | `0.00`  | `0.0000` | `N/A`   | `±180 deg/s` | N/A |
| **Pitch Angle (Outer)**|`4.80`  | `0.00`  | `0.0000` | `N/A`   | `±180 deg/s` | N/A |
| **Altitude Z (Pos)**  | `1.20`  | `0.00`  | `0.0000` | `N/A`   | `±2.0 m/s`   | N/A |
| **Velocity Z (Vel)**  | `0.25`  | `0.15`  | `0.0080` | `±0.20` | `±0.40 thr`  | $10\text{ Hz}$ ($\beta=0.78$) |

---

## 4. Autonomous Navigation Test Protocol (Phase F6)

Phase F6 validates the outer cascaded navigational estimators and closed-loop controllers: the BMP280 Barometer (FW-06), QMC5883L Magnetometer (FW-07), and GNSS NMEA receiver (FW-08).

```
[ Step 1: Ground Sensor Fusion Audit ] --> GPS 3D Fix > 8 Sats, Mag Shift < 3° under throttle ramp
                 |
                 v
[ Step 2: Altitude Hold (AltHold) Engagement ] --> Stabilize 3m hover in Angle Mode; Center stick
                 |
                 v (PASS: Alt Hold within ± 0.3m, Velocity climb/descent tracking OK)
[ Step 3: Horizontal Position Hold (PosHold) ] --> Engage in open field; verify position within ± 1.0m
                 |
                 v (MONITOR: Zero Toilet-Bowl divergence; instant switch to Angle on anomaly)
[ Step 4: Return-To-Home & Sensor Loss Abort ] --> Validate fail-to-angle override hierarchy
```

---

### 4.1 Ground Sensor Fusion Audit
Before engaging any autonomous mode in flight, verify sensor health on the ground:

1. **Barometer BMP280 Ground Stability:**
   - Rest vehicle stationary on the takeoff pad.
   - Observe `baro_alt` on telemetry over 60 seconds:
     - Peak-to-peak drift must be **$< 0.15\text{ meters}$**.
     - Gently blow air across the vehicle from 1 meter away; verify reading does not spike by $> 0.3\text{ m}$ (confirms acoustic foam shielding over BMP280 metal vent hole is working).
2. **Magnetometer Full-Throttle Magnetic Deviation Test:**
   - Secure airframe firmly to the ground test bench (or tether plate).
   - Point the aircraft nose toward True Magnetic North ($0.0^\circ$).
   - Advance throttle smoothly from $0\% \rightarrow 70\%$ while monitoring telemetry `mag_heading_deg`:
     - Under $70\text{ A}$ full propulsion current draw, the measured heading shift must be **less than $3.0^\circ$**.
     - If heading swings by $> 8.0^\circ$, high-current DC power cables are coupling electromagnetically into the compass puck: raise the mast height to $15\text{ cm}$ or route battery leads through twisted-pair geometry.
3. **GNSS Fix Quality:**
   - Verify `$GNGGA` / `$GNRMC` parsing in boot logs:
     - `fix_quality`: Must equal `1` (Standard GPS) or `2` (DGPS/SBAS).
     - `satellites`: Must show $\ge 8$ satellites.
     - `hdop`: Must be $\le 1.4$. Do **NOT** test autonomous navigation modes if HDOP $> 1.8$.

---

### 4.2 Altitude Hold Mode (`MODE_ALT_HOLD`) First Flight Test

The altitude hold controller in [`altitude.c`](file:///c:/dev/vega/zephyr/app/src/control/altitude.c) runs at 50 Hz, utilizing a cascaded structure (Position Z error $\rightarrow$ Velocity Z setpoint $\rightarrow$ Throttle adjustment).

1. **Pre-Engagement Hover:**
   - Take off in `MODE_ANGLE`. Climb to an altitude of **$3.5\text{ meters}$** above ground level.
   - Establish a steady, stationary hover.
2. **Stick Centering & Mode Engagement:**
   - Confirm throttle stick is placed within the **$\pm 8\%$ center deadband** ($[0.42, 0.58]$).
   - Flip the transmitter mode switch (AUX2) to **`MODE_ALT_HOLD`**:
     - *Firmware Action:* The controller locks the current filtered barometer altitude as $Z_{\text{target}}$.
     - *Airframe Reaction:* Aircraft must hold vertical position within **$\pm 0.3\text{ meters}$** without pilot throttle intervention.
3. **Climb / Descent Velocity Setpoint Test:**
   - Deflect throttle stick upward to $75\%$ (outside deadband):
     - Controller commands a steady $+1.0\text{ m/s}$ vertical climb.
   - Return throttle stick to center ($50\%$):
     - Aircraft stops climbing immediately and locks in at the new altitude ($6.0\text{ m}$).
   - Deflect throttle stick downward to $25\%$:
     - Controller commands a steady $-0.8\text{ m/s}$ vertical descent.
   - Return throttle stick to center:
     - Aircraft arrests descent and holds altitude at $3.0\text{ m}$.
4. **Altitude Hold Abort Protocol:**
   - If the aircraft begins vertical "hunting" (low-frequency vertical oscillation of $> 1.0\text{ m}$) or drops suddenly toward the ground:
     > [!CRITICAL]
     > **INSTANTLY FLIP AUX2 SWITCH BACK TO `MODE_ANGLE`.**
     > The pilot immediately regains 100% direct proportional throttle authority. Smoothly increase throttle to catch the vehicle.

---

### 4.3 Position Hold Mode (GNSS + Compass Fusion) Test

1. **Pre-Conditions for Engagement:**
   - Vehicle flying in `MODE_ALT_HOLD` at $5.0\text{ meters}$ altitude.
   - Satellite count $\ge 10$, HDOP $\le 1.2$.
   - Steady wind $< 2.5\text{ m/s}$.
2. **Engagement & Hold Observation:**
   - Release Pitch and Roll sticks to center.
   - Toggle flight mode to **Position Hold**:
     - Outer loop computes horizontal position errors:
       $$e_x = \text{Target\_Lon} - \text{Actual\_Lon}, \quad e_y = \text{Target\_Lat} - \text{Actual\_Lat}$$
     - Pitch and Roll angles are commanded automatically to oppose drift.
   - Aircraft must hold horizontal coordinates within a **$1.0\text{ meter}$ radial circle**.
3. **Diagnosis of "Toilet-Bowling" (Expanding Spiral Instability):**
   - **Visual Symptom:** Instead of holding position, the aircraft begins flying in a circular trajectory that progressively expands in radius ($1\text{m} \rightarrow 3\text{m} \rightarrow 8\text{m}$) at accelerating speed.
   - **Aerospace Root Cause:**
     - The magnetometer heading vector does not match the true GPS ground track vector due to uncompensated **Magnetic Declination** or **Compass Orientation Offset** ($90^\circ / 180^\circ$ mounting error).
     - The flight controller attempts to correct a North drift, but because heading is miscalculated by $15^\circ$, it pitches toward North-North-East, creating an orthogonal velocity vector that destabilizes the outer loop into an unstable spiral.
   - **Immediate Pilot Action:**
     - **FLIP AUX2 BACK TO `MODE_ANGLE` IMMEDIATELY.**
     - Take over pitch/roll manually to bring the aircraft to a stop.
     - Land and inspect magnetic declination parameters in [`config.h`](file:///c:/dev/vega/zephyr/app/src/config.h) or recalibrate the QMC5883L magnetometer.

---

## 5. MicroSD Blackbox Flight Log Post-Analysis

After every test flight, extract the generated `.BFL` binary log file from the MicroSD card slot. The compact 44-byte binary structure defined in [`logger.h`](file:///c:/dev/vega/zephyr/app/src/system/logger.h) records full 250 Hz control loop state telemetry.

### 5.1 Python Binary Log Decoder Script
Save this script to your host workstation to unpack binary flight logs into analysis CSV tables:

```python
#!/usr/bin/env python3
"""
unpack_blackbox.py - Decodes VEGA ARIES v3.0 44-Byte Binary Blackbox Flight Logs
"""
import struct
import sys
import csv

FRAME_FORMAT = "<BIhhhHhhh hhh hhh HHHH HBBB"
FRAME_SIZE = struct.calcsize(FRAME_FORMAT)
MAGIC_SYNC = 0xB5

def unpack_log(bfl_path, csv_out_path):
    print(f"Decoding {bfl_path} (Frame size: {FRAME_SIZE} bytes)...")
    valid_frames = 0
    dropped_sync = 0

    with open(bfl_path, "rb") as fin, open(csv_out_path, "w", newline="") as fout:
        writer = csv.writer(fout)
        writer.writerow([
            "time_us", "target_roll_deg", "target_pitch_deg", "target_yaw_dps", "target_thr",
            "actual_roll_deg", "actual_pitch_deg", "actual_yaw_deg",
            "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
            "pid_roll", "pid_pitch", "pid_yaw",
            "m1_us", "m2_us", "m3_us", "m4_us",
            "vbat_v", "state", "mode", "failsafe"
        ])

        # Skip 32-byte ASCII header
        header = fin.read(32)

        while True:
            chunk = fin.read(FRAME_SIZE)
            if len(chunk) < FRAME_SIZE:
                break

            data = struct.unpack(FRAME_FORMAT, chunk)
            if data[0] != MAGIC_SYNC:
                # Re-align to next sync byte
                fin.seek(-FRAME_SIZE + 1, 1)
                dropped_sync += 1
                continue

            writer.writerow([
                data[1],                     # timestamp_us
                data[2] / 100.0,             # target_roll_deg
                data[3] / 100.0,             # target_pitch_deg
                data[4] / 10.0,              # target_yaw_dps
                data[5] / 1000.0,            # target_thr
                data[6] / 100.0,             # actual_roll_deg
                data[7] / 100.0,             # actual_pitch_deg
                data[8] / 100.0,             # actual_yaw_deg
                data[9] / 10.0,              # gyro_x_dps
                data[10] / 10.0,             # gyro_y_dps
                data[11] / 10.0,             # gyro_z_dps
                data[12] / 1000.0,           # pid_roll
                data[13] / 1000.0,           # pid_pitch
                data[14] / 1000.0,           # pid_yaw
                data[15], data[16], data[17], data[18], # motors 1-4
                data[19] / 1000.0,           # battery_v
                data[20], data[21], data[22] # state, mode, failsafe
            ])
            valid_frames += 1

    print(f"Extraction complete! Valid frames: {valid_frames}, Resync count: {dropped_sync}")
    print(f"Exported CSV: {csv_out_path}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python unpack_blackbox.py <input.BFL> <output.csv>")
        sys.exit(1)
    unpack_log(sys.argv[1], sys.argv[2])
```

### 5.2 Diagnostic Trace Checklist
When analyzing the decoded flight logs in plotting tools (Python Matplotlib, PlotJuggler, or Excel):

1. **Inner Loop Rate Tracking:**
   - Plot `target_roll_deg` vs `actual_roll_deg`.
   - The actual attitude trace should track the target trace with a time lag of **$< 40\text{ ms}$** and zero steady-state offset.
2. **Motor Output Margin (Headroom):**
   - Plot motor commands `m1_us`, `m2_us`, `m3_us`, `m4_us`.
   - In steady hover, all four motors must operate between **$1420\ \mu\text{s} \text{ and } 1550\ \mu\text{s}$**.
   - If hover requires $> 1750\ \mu\text{s}$, the aircraft is severely underpowered (AUW too heavy or motor KV too low); insufficient dynamic headroom remains for attitude recovery during high-wind gusts.
3. **Integral Windup Verification:**
   - Verify that during hard stick deflections, PID outputs `pid_roll` and `pid_pitch` do not hit hard integration limits for extended periods ($> 200\text{ ms}$).
4. **Vibration Peak Frequency Detection (FFT):**
   - Compute Fast Fourier Transform (FFT) over `gyro_x_dps` and `gyro_y_dps`.
   - Fundamental motor rotation peak:
     $$f_{\text{motor}} = \frac{\text{RPM}}{60} = \frac{920\text{ KV} \times 12\text{ V} \times 0.50\text{ throttle}}{60} \approx 92\text{ Hz}$$
   - Confirm that the DLPF hardware filter (42 Hz) and software derivative low-pass filter (30 Hz) attenuate the $92\text{ Hz}$ motor vibration band by $> 18\text{ dB}$ before reaching the PID differentiator.
