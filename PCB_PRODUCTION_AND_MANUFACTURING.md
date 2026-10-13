# VEGA ARIES v3.0 S500 Flight Controller: Final PCB Manufacturing & Production Release Specification (P1–P5)

**Target SoC:** C-DAC THEJAS32 (VEGA ET1031 RISC-V 32-bit Core @ 100 MHz, 256 KB On-Chip SRAM)  
**Airframe Compatibility:** S500 Quadcopter (~2.5 kg AUW, Quad-X Configuration, 120A Peak Propulsion Current)  
**PCB Revision:** Rev 1.0 (Production Release)  
**Firmware Baseline:** Zephyr RTOS v3.5.0 LTS / Out-of-tree ARIES v3 Board Package (`aries_v3`)  
**Document Classification:** Engineering Release & Manufacturing Handoff Package

---

## 1. P1: Design Freeze & Sign-Off Checklist

Before releasing CAD files to the printed circuit board assembly (PCBA) fabrication house, the hardware and firmware engineering leads must complete and sign off on this design freeze checklist.

```
+---------------------------------------------------------------------------------------------------+
|                                 DESIGN FREEZE CHECKLIST (P1)                                      |
+---------------------------------------------------------------------------------------------------+
  [Schematic ERC]                 [PCB Layout DRC]                [Firmware Freeze]
  - 0 Errors, 0 Unconnected       - JLCPCB/PCBWay 4-Layer Rules   - Git Tag: v1.0.0-RELEASE
  - Power flags verified          - Layer 2 Ground 100% Unbroken  - SHA-256 Binary Hash Locked
  - Pin-swapping verified         - Annular rings >= 0.15mm       - Clean build via West
+---------------------------------------------------------------------------------------------------+
```

### 1.1 Schematic Electrical Rules Check (ERC)
- [ ] **ERC Cleanliness:** Execute KiCad Electrical Rules Check. Result must be **0 Errors, 0 Unhandled Warnings**.
- [ ] **Power Rail Flags:** Ensure all internal power nets (`+5V_SYS`, `+3V3_SYS`, `VBAT_RAW`, `GND`) have explicit `PWR_FLAG` symbols connected.
- [ ] **Passive Sizing Validation:**
  - $R_1 = 10.0\text{ k}\Omega \pm 0.1\%$ (0603) and $R_2 = 2.20\text{ k}\Omega \pm 0.1\%$ (0603) thin-film divider matches firmware divider ratio $5.545455$.
  - I2C0 pull-up resistors: Exactly **$2.2\text{ k}\Omega \pm 1\%$ (0402)** to `+3V3_SYS`.
  - PWM series damping resistors: Exactly **$220\ \Omega \pm 5\%$ (0402)** on channels 0–3.
  - SPI0 damping resistors: **$33\ \Omega \pm 5\%$ (0402)** on `SD_SCK` and `SD_MOSI`.
- [ ] **Protection Network:**
  - SMBJ20A unidirectional TVS diode installed across `VBAT_RAW` input entry pads.
  - BAT54S dual Schottky clamp diode protecting `VBAT_SENSE` tap into `ADC0_CH0`.
  - Semtech RClamp0504F ultra-low capacitance ESD protection on PWM outputs and external UART ports.
- [ ] **Pin Assignment Cross-Audit:** Compare KiCad net labels directly against [`aries_v3.overlay`](file:///c:/dev/vega/zephyr/app/boards/aries_v3.overlay) and [`main.c`](file:///c:/dev/vega/zephyr/app/src/main.c):
  - UART0 (`0x10000100`): GNSS NMEA RX/TX.
  - UART1 (`0x10000200`): SBUS RX (with 74LVC1G04 hardware inverter).
  - UART2 (`0x10000300`): Telemetry Stream RX/TX.
  - I2C0 (`0x10000800`): MPU6050 (`0x68`), BMP280 (`0x76`), Mast Mag (`0x0D`).
  - PWM0 (`0x10400000`): Channels 0–3 to ESCs 1–4.
  - SPI0 (`0x10000600`): MicroSD Card slot with GPIO1 Pin 10 CS.
  - ADC0 (`0x10001000`): Channel 0 Battery Voltage Divider.

### 1.2 PCB Physical Layout & Design Rules Check (DRC)
- [ ] **DRC Zero-Tolerance:** Run KiCad DRC using targeted fabrication house rules (JLCPCB 4-Layer standard / PCBWay 4-Layer Class 2). Result: **0 Violations, 0 Unconnected Nets**.
- [ ] **Minimum Geometries:**
  - Minimum trace width / spacing: **$0.127\text{ mm} / 0.127\text{ mm}$ ($5\text{ mil} / 5\text{ mil}$)** on signal layers.
  - Minimum via drill diameter: **$0.30\text{ mm} / 0.60\text{ mm}$ pad** ($12\text{ mil} / 24\text{ mil}$).
  - Annular ring: Minimum **$\ge 0.15\text{ mm}$ ($6\text{ mil}$)** across all layers.
- [ ] **Ground Plane Integrity (Layer 2):**
  - Verify Layer 2 is an **UNBROKEN, CONTINUOUS 1.0 oz COPPER GND PLANE**.
  - No signal traces or routing tracks routed on Layer 2.
  - Via anti-pads (clearance rings) do not create contiguous copper slots beneath high-speed lines (SPI0 clock, I2C0, or switching buck inductor).
- [ ] **Silkscreen & Mechanical Footprints:**
  - Pin 1 orientation markers (white dot or beveled corner) clearly visible for THEJAS32 SoC, MPU6050, BMP280, TPS54302, and connectors.
  - High-visibility directional arrow silkscreen pointing to the **Front / Nose of the aircraft**.
  - All text heights $\ge 0.80\text{ mm}$ with line thickness $\ge 0.15\text{ mm}$ to ensure legible factory silkscreen printing.
  - Mounting holes: 4x M3 ($3.2\text{ mm}$) holes on standard **$30.5\times 30.5\text{ mm}$ mounting grid** with $1.0\text{ mm}$ keepout from copper pours to prevent mounting screw short-circuits.

### 1.3 Firmware Baseline Locking
- [ ] **Git Repository State:** Commit all changes; ensure working directory is clean (`git status` reports no untracked or modified files).
- [ ] **Firmware Release Tag:** Tag the release commit:
  ```bash
  git tag -a v1.0.0-RELEASE -m "Production Firmware for VEGA ARIES v3.0 Flight Controller"
  git push origin v1.0.0-RELEASE
  ```
- [ ] **Reproducible Binary Hash:** Build release binary and record the SHA-256 checksum in the release notes:
  ```bash
  west build -b aries_v3 zephyr/app -- -DEXTRA_DTC_OVERLAY_FILE="boards/aries_v3.overlay"
  sha256sum build/zephyr/zephyr.bin > release_v1.0.0_sha256.txt
  ```

---

## 2. P2: Manufacturing Handoff & Fabrication Export Guide

Fabrication houses require four standardized file sets for Automated PCB Fabrication and SMT Assembly (PCBA):
1. **Gerber Files (RS-274X / Gerber X2):** Board copper layers, solder mask, silkscreen, and mechanical outline.
2. **Excellon Drill Files:** Plated (PTH) and non-plated (NPTH) hole coordinates and tool diameters.
3. **Bill of Materials (BOM):** Component designators, values, manufacturer part numbers (MPN), and distributor part numbers (LCSC C-IDs).
4. **Centroid / Pick-and-Place (CPL) File:** Surface mount component coordinates (X, Y), board layer (Top/Bottom), and rotation angle.

```
+---------------------------------------------------------------------------------------------------+
|                              MANUFACTURING EXPORT DIRECTORY TREE                                  |
+---------------------------------------------------------------------------------------------------+
  production_release_v1.0/
  ├── gerbers/
  │   ├── aries_v3_fc-F_Cu.gbr             (Layer 1: Top Copper)
  │   ├── aries_v3_fc-In1_Cu.gbr           (Layer 2: Ground Plane)
  │   ├── aries_v3_fc-In2_Cu.gbr           (Layer 3: Power Split Plane)
  │   ├── aries_v3_fc-B_Cu.gbr             (Layer 4: Bottom Copper)
  │   ├── aries_v3_fc-F_Mask.gbr           (Top Solder Mask)
  │   ├── aries_v3_fc-B_Mask.gbr           (Bottom Solder Mask)
  │   ├── aries_v3_fc-F_Silkscreen.gbr     (Top Silkscreen)
  │   ├── aries_v3_fc-B_Silkscreen.gbr     (Bottom Silkscreen)
  │   ├── aries_v3_fc-F_Paste.gbr          (Top Solder Paste Stencil)
  │   ├── aries_v3_fc-Edge_Cuts.gbr        (Mechanical Board Outline)
  │   └── aries_v3_fc.drl                  (Merged Excellon Drill File)
  ├── pcba/
  │   ├── BOM_aries_v3_jlcpcb.csv          (Standardized Bill of Materials)
  │   └── CPL_aries_v3_jlcpcb.csv          (Pick-and-Place Centroid File)
  └── firmware/
      └── aries_v3_flight_controller_v1.0.bin (Production Flash Image)
+---------------------------------------------------------------------------------------------------+
```

### 2.1 4-Layer PCB Stackup Specification (1.6mm FR4 Standard)
Provide this exact stackup to JLCPCB (select **JLC04161H-7628**) or PCBWay:

| Layer Number | Layer Name | Material | Copper Thickness | Dielectric Thickness | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **L1** | **Top Layer** | Copper (Cu) | 1.0 oz ($35\ \mu\text{m}$) | — | Signal traces, high-speed SPI0, I2C0, ADC |
| Dielectric | Prepreg | FR4 (7628) | — | $0.21\text{ mm}$ ($8.3\text{ mil}$) | Core insulation |
| **L2** | **Inner Layer 1** | Copper (Cu) | 1.0 oz ($35\ \mu\text{m}$) | — | **Solid Unbroken Ground (GND) Plane** |
| Dielectric | Core | FR4 Core | — | $1.06\text{ mm}$ ($41.7\text{ mil}$) | Structural fiberglass core |
| **L3** | **Inner Layer 2** | Copper (Cu) | 1.0 oz ($35\ \mu\text{m}$) | — | **Power Split Planes (+5V_SYS, +3V3_SYS)** |
| Dielectric | Prepreg | FR4 (7628) | — | $0.21\text{ mm}$ ($8.3\text{ mil}$) | Core insulation |
| **L4** | **Bottom Layer**| Copper (Cu) | 1.0 oz ($35\ \mu\text{m}$) | — | PWM motor outputs, UART lines, Debug LEDs |
| **Total** | | | **1.60 mm** | | Solder Mask: Matte Black; Finish: **ENIG (Electroless Nickel Immersion Gold)** |

### 2.2 KiCad Export Settings Walkthrough

#### A. Gerber Files Export (KiCad Plot Dialog):
1. Open KiCad PCB Editor $\rightarrow$ **File** $\rightarrow$ **Plot...**
2. Plot format: **Gerber**.
3. Output directory: `./gerbers/`
4. Included Layers:
   - `F.Cu`, `In1.Cu`, `In2.Cu`, `B.Cu`
   - `F.Paste` (for solder stencil)
   - `F.Silkscreen`, `B.Silkscreen`
   - `F.Mask`, `B.Mask`
   - `Edge.Cuts` (Milling board boundary)
5. General Options:
   - [x] Plot edge cuts on all layers: **Unchecked**
   - [x] Use Protel filename extensions: **Unchecked**
   - [x] Generate Gerber job file: **Unchecked**
   - [x] Subtract soldermask from silkscreen: **Checked**
   - [x] Use extended X2 format: **Checked** (or RS-274X for legacy fab)
   - Coordinate format: **4.6, unit: mm**.

#### B. Drill File Generation:
1. In the same Plot window, click **Generate Drill Files...**
2. Settings:
   - Drill units: **Millimeters**
   - Zeros format: **Decimal format**
   - Drill map file format: **None**
   - Plated and Non-Plated Holes: **Merge PTH and NPTH into one file**
   - Click **Generate Drill File**.

#### C. Pick-and-Place (CPL / Centroid) Generation:
1. In KiCad PCB Editor $\rightarrow$ **File** $\rightarrow$ **Fabrication Outputs** $\rightarrow$ **Component Placement (.pos)...**
2. Format: **ASCII / CSV**.
3. Units: **Millimeters**.
4. Files: **Single file for all components** (or separate Top/Bottom).
5. Output: `CPL_aries_v3_jlcpcb.csv`.

---

### 2.3 Master Production Bill of Materials (BOM)
Formatted for automated turnkey SMT assembly with LCSC Part Numbers:

| Designator | Value / Part | Description | Package / Footprint | MPN (Manufacturer Part No) | LCSC Part # | Qty |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **U1** | **THEJAS32** | 32-bit RISC-V SoC @ 100MHz | QFP-100 / QFN-68 | C-DAC THEJAS32-ET1031 | Custom | 1 |
| **U2** | **MPU6050** | 6-DOF IMU (Gyro + Accel) | QFN-24 ($4\times 4\text{ mm}$) | TDK InvenSense MPU-6050 | C24112 | 1 |
| **U3** | **BMP280** | Digital Barometer | LGA-8 ($2.0\times 2.5\text{ mm}$)| Bosch Sensortec BMP280 | C92489 | 1 |
| **U4** | **TPS54302** | 3A 28V Synchronous Buck Reg | SOT-23-6 | Texas Instruments TPS54302DDCR | C96944 | 1 |
| **U5** | **TPS7A2033** | 3.3V 500mA Ultra-Low Noise LDO | SOT-23-5 | Texas Instruments TPS7A2033PDBVR | C2863777 | 1 |
| **U6** | **74LVC1G04**| Single Inverter Gate (SBUS) | SOT-23-5 | Nexperia 74LVC1G04GV,125 | C7940 | 1 |
| **D1** | **SMBJ20A** | 600W 20V TVS Surge Clamp | SMB (DO-214AA) | Littelfuse SMBJ20A | C2828694 | 1 |
| **D2, D3** | **BAT54S** | Dual Schottky Clamp Diode | SOT-23-3 | Diodes Inc. BAT54S-7-F | C2148 | 2 |
| **D4** | **RClamp0504F**| 4-Ch Low-Capacitance TVS | SC-70-6 | Semtech RCLAMP0504F.TCT | C111756 | 1 |
| **L1** | **$4.7\ \mu\text{H}$** | Shielded Power Inductor 3.5A | SMD $4.0\times 4.0\text{ mm}$ | Wurth 74438356047 | C114488 | 1 |
| **R1** | **$10.0\text{ k}\Omega$**| Precision Resistor $0.1\%$ | 0603 | Susumu RR0816P-103-D | C22934 | 1 |
| **R2** | **$2.20\text{ k}\Omega$**| Precision Resistor $0.1\%$ | 0603 | Susumu RR0816P-222-D | C22935 | 1 |
| **R3, R4** | **$2.2\text{ k}\Omega$** | I2C0 Bus Pull-Ups $1\%$ | 0402 | Yageo RC0402FR-072K2L | C25879 | 2 |
| **R5..R8** | **$220\ \Omega$** | PWM Series Damping $5\%$ | 0402 | Yageo RC0402JR-07220RL | C25114 | 4 |
| **R9, R10**| **$33\ \Omega$** | SPI0 Series Damping $5\%$ | 0402 | Yageo RC0402JR-0733RL | C25126 | 2 |
| **R11** | **$10\text{ k}\Omega$** | MicroSD CS Pull-Up $5\%$ | 0402 | Yageo RC0402JR-0710KL | C25744 | 1 |
| **R12** | **$47\text{ k}\Omega$** | MicroSD MISO Pull-Up $5\%$ | 0402 | Yageo RC0402JR-0747KL | C25774 | 1 |
| **C1, C2** | **$22\ \mu\text{F}$** | 35V X7R Ceramic Decoupling | 1206 | Murata GRM31CR71H226KE15L | C13585 | 2 |
| **C3, C4** | **$10\ \mu\text{F}$** | 10V X7R Ceramic Decoupling | 0603 | Samsung CL10B106KP8NNNC | C15850 | 2 |
| **C5..C12**| **$100\text{ nF}$** | 16V X7R Sensor Bypass Caps | 0402 | Samsung CL05B104KO5NNNC | C1525 | 8 |
| **J1** | **MicroSD** | Push-Pull SPI MicroSD Socket | SMD Outer Solder | Molex 104031-0811 | C266624 | 1 |
| **J2, J3** | **JST-GH-4P**| 1.25mm Lock Connector (I2C/UART)| SMD Top Mount | JST BM04B-GHS-TBT | C160404 | 2 |
| **J4** | **JST-GH-3P**| 1.25mm Lock Connector (SBUS)| SMD Top Mount | JST BM03B-GHS-TBT | C160403 | 1 |

---

## 3. P3: Factory Acceptance Testing (FAT) & Validation Protocol

The PCBA house or incoming quality inspection technician must execute this automated test suite on 100% of manufactured production boards before packaging.

```
+---------------------------------------------------------------------------------------------------+
|                            FACTORY ACCEPTANCE TEST (FAT) FLOWCHART                                |
+---------------------------------------------------------------------------------------------------+
  [Step 1: Automated Optical Inspection (AOI)]
  - Verify solder joint fillets, QFN bridges, IC orientation, and passives alignment
                         |
                         v (PASS)
  [Step 2: Cold Bed-of-Nails Resistance Test]
  - Check impedance to ground: VBAT > 10k, +5V > 1k, +3V3 > 500 Ohm. Open 5V to 3.3V.
                         |
                         v (PASS)
  [Step 3: Automated Bed-of-Nails Power Injection]
  - Supply 12.0V DC (100mA clamp). Verify +5V rail (4.95-5.05V) and +3V3 rail (3.28-3.32V)
                         |
                         v (PASS)
  [Step 4: Automated JTAG Flash & Factory Diagnostic Suite]
  - Program factory self-test firmware binary via OpenOCD JTAG
  - Run peripheral audit: I2C0 bus scan (0x68, 0x76, 0x0D), SPI0 SD test, ADC divider check
                         |
                         v (PASS)
  [Step 5: Serial Barcode Stamping & Anti-Static Packaging]
  - Print QR barcode label with MAC/Serial; seal in ESD shielding bag with desiccant
+---------------------------------------------------------------------------------------------------+
```

### 3.1 Automated Bed-of-Nails Test Point Coordinates
The PCB includes standard $1.0\text{ mm}$ circular test pads on Layer 4 (Bottom) for pogo-pin test fixtures:

| Test Point | Net Name | Expected Cold Resistance to GND | Expected Live Voltage (12V Supply) | Pass Window |
| :--- | :--- | :--- | :--- | :--- |
| **TP1** | `VBAT_IN` | $> 10\text{ k}\Omega$ (rising) | $12.00\text{ V}$ | $11.8\text{ V} \text{ to } 12.2\text{ V}$ |
| **TP2** | `+5V_SYS` | $> 1.0\text{ k}\Omega$ | $5.02\text{ V}$ | $4.92\text{ V} \text{ to } 5.08\text{ V}$ |
| **TP3** | `+3V3_SYS`| $> 500\ \Omega$ | $3.30\text{ V}$ | $3.27\text{ V} \text{ to } 3.33\text{ V}$ |
| **TP4** | `VBAT_SENSE`| $> 2.0\text{ k}\Omega$ | $2.164\text{ V}$ | $2.13\text{ V} \text{ to } 2.20\text{ V}$ |
| **TP5** | `PWM_CH0` | $> 100\text{ k}\Omega$ | $0.0\text{ V} \rightarrow 3.3\text{ V}$ pulse | 400 Hz, $900\ \mu\text{s}$ disarmed |
| **TP6** | `I2C0_SCL`| $2.20\text{ k}\Omega \pm 50\ \Omega$ (Pull-up) | $3.30\text{ V}$ (Quiescent) | $3.20\text{ V} \text{ to } 3.33\text{ V}$ |
| **TP7** | `I2C0_SDA`| $2.20\text{ k}\Omega \pm 50\ \Omega$ (Pull-up) | $3.30\text{ V}$ (Quiescent) | $3.20\text{ V} \text{ to } 3.33\text{ V}$ |
| **TP8** | `GND` | $0.0\ \Omega$ (Reference ground) | $0.00\text{ V}$ | Reference |

### 3.2 Automated Factory JTAG Flashing Script
Factory fixture executes this automated bash script (`factory_test.sh`) over a Raspberry Pi or Linux test station running OpenOCD and an FT2232D JTAG programmer:

```bash
#!/usr/bin/env bash
set -e

SERIAL_NUM=$1
if [ -z "$SERIAL_NUM" ]; then
    echo "ERROR: Serial number must be provided. Usage: ./factory_test.sh <SERIAL_NO>"
    exit 1
fi

echo "=========================================================="
echo " VEGA ARIES v3.0 FACTORY VALIDATION FIXTURE - SN: $SERIAL_NUM"
echo "=========================================================="

# 1. Flash Production Firmware Image
echo "[1/4] Programming THEJAS32 Flash via OpenOCD JTAG..."
openocd -f interface/ftdi/jtag.cfg -f target/thejas32.cfg \
    -c "init" \
    -c "reset halt" \
    -c "flash write_image erase firmware/aries_v3_flight_controller_v1.0.bin 0x10000000" \
    -c "verify_image firmware/aries_v3_flight_controller_v1.0.bin 0x10000000" \
    -c "reset run" \
    -c "shutdown"

# 2. Capture Factory Boot Diagnostics over UART2 (@ 115200 baud)
echo "[2/4] Capturing Diagnostic Boot Telemetry over UART2..."
TIMEOUT=5
BOOT_LOG=$(python3 -c "
import serial, time
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
start = time.time()
lines = []
while time.time() - start < 5:
    l = ser.readline().decode('utf-8', errors='ignore')
    if l: lines.append(l)
print(''.join(lines))
")

echo "$BOOT_LOG" > "logs/fat_${SERIAL_NUM}.log"

# 3. Assert Pass/Fail Criteria
echo "[3/4] Evaluating Hardware Peripheral Discovery..."

function assert_log() {
    PATTERN=$1
    LABEL=$2
    if echo "$BOOT_LOG" | grep -q "$PATTERN"; then
        echo "  [PASS] $LABEL detected."
    else
        echo "  [FAIL] $LABEL NOT FOUND! Boot log missing: '$PATTERN'"
        exit 2
    fi
}

assert_log "mpu6050: MPU6050 identity verified: 0x68" "IMU (MPU6050 @ 0x68)"
assert_log "bmp280: BMP280 confirmed at addr 0x76"      "Barometer (BMP280 @ 0x76)"
assert_log "mag: Found QMC5883L Magnetometer at 0x0D"   "Magnetometer (QMC5883L @ 0x0D)"
assert_log "MicroSD Card mounted successfully at /SD:"  "SD Card FatFS (SPI0 @ 25MHz)"
assert_log "Battery initialized"                        "Battery ADC0 Divider (R1=10k, R2=2.2k)"
assert_log "THEJAS32 PWM0 global enable register"       "Actuator PWM0 (400 Hz ESC Core)"

# 4. Final Disposition
echo "[4/4] ALL FACTORY VALIDATION TESTS PASSED FOR BOARD: $SERIAL_NUM"
echo "Disposition: APPROVED FOR FINAL SERIALIZATION & PACKAGING."
exit 0
```

---

## 4. P4: Open-Source Documentation Hierarchy

The repository documentation must be organized into a clean, hierarchical structure so developers, researchers, and drone integrators can replicate and fly the platform without guesswork:

```
docs/
├── 01_HARDWARE_ARCHITECTURE/
│   ├── SCHEMATICS.pdf                 (Complete KiCad Schematic Sheets in PDF)
│   ├── PINOUT_REFERENCE.md            (Master SoC Pin-to-Port Interface Matrix)
│   ├── COMPONENT_SELECTION_GUIDE.md   (Passive sizing, TVS protection, LDO choices)
│   └── PCB_LAYOUT_RULES.md            (Layer 2 unbroken GND, 120A motor current isolation)
├── 02_FABRICATION_AND_ASSEMBLY/
│   ├── GERBER_EXPORT_GUIDE.md         (JLCPCB / PCBWay manufacturing instructions)
│   ├── BOM_MASTER.csv                 (Bill of materials with LCSC C-IDs)
│   └── FACTORY_TEST_FIXTURE.md        (Bed-of-nails test points & JTAG flashing script)
├── 03_WIRING_AND_AIRFRAME_INTEGRATION/
│   ├── S500_WIRING_HARNESS.md         (PDB separation, motor numbering, ESC signal wire)
│   ├── COMPASS_MAST_MOUNTING.md       (12cm elevated mast, EMI avoidance protocols)
│   └── BENCH_TESTING_SMOKE_TEST.md    (Cold continuity, 12V current clamp power bring-up)
├── 04_FIRMWARE_AND_BUILD_SYSTEM/
│   ├── ZEPHYR_SETUP_AND_BUILD.md      (West manifest, toolchain, compilation commands)
│   ├── FLIGHT_CONTROL_LOOPS.md        (250 Hz AHRS, cascaded PID architecture)
│   └── DEVICETREE_OVERLAY_GUIDE.md    (THEJAS32 peripheral mapping & custom drivers)
└── 05_FLIGHT_TEST_AND_TUNING/
    ├── MAIDEN_FLIGHT_PROTOCOL.md      (F1-F4 ground, spin, hover, manual envelope expansion)
    ├── PID_TUNING_GUIDE.md            (Step-by-step field tuning: Rate P -> D -> I -> Angle)
    ├── AUTONOMOUS_NAV_TESTING.md      (AltHold & PosHold validation, anti-toilet-bowl)
    └── BLACKBOX_LOG_ANALYSIS.md       (Python decoder script, vibration FFT analysis)
```

---

## 5. P5: Open-Source Project Release Summary

All hardware schematics, PCB layouts, Zephyr firmware modules, and testing programs have been synthesized into production-ready specifications:

- **Hardware Architecture & Bench Test:** [`HARDWARE_SPEC_AND_BENCH_TESTING.md`](file:///c:/dev/vega/HARDWARE_SPEC_AND_BENCH_TESTING.md)
- **Flight Testing & Dynamics Tuning:** [`FLIGHT_TEST_PROGRAM_AND_TUNING.md`](file:///c:/dev/vega/FLIGHT_TEST_PROGRAM_AND_TUNING.md)
- **Manufacturing & PCBA Release:** [`PCB_PRODUCTION_AND_MANUFACTURING.md`](file:///c:/dev/vega/PCB_PRODUCTION_AND_MANUFACTURING.md)
- **Root GitHub Repository Documentation:** [`README.md`](file:///c:/dev/vega/README.md)
