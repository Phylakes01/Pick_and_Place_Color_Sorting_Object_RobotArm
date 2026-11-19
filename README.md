# Pick and Place Color-Sorting Object Robot Arm

An automated **6-Servo Pick-and-Place Color-Sorting Robotic Arm** controlled by an **Arduino UNO R3**. The system detects when an object enters the pickup zone using an **HC-SR04 ultrasonic sensor**, performs full-spectrum **RGB + HSV** (Hue `0°–360°`, Saturation `0–100%`) 9-color classification (**Red**, **Orange**, **Yellow**, **Green**, **Cyan**, **Blue**, **Purple**, **Magenta**, and **White**) using a **TCS34725 I2C RGB color sensor**, and smoothly drives a **6-servo robotic arm** (base rotation, shoulder, elbow, wrist pitch, wrist roll, and gripper claw) using **calibrated joint poses & smooth 1°/step interpolation** to sort each color family into its matching bucket.

## 3D Model Preview

![3D Model 360 Preview](3d_Model_Preview.gif)

*360° animated preview of [`3d Model.stl`](3d%20Model.stl) (parametric OpenSCAD source: [`3d Model.scad`](3d%20Model.scad)).*

## Circuit Schematic

![Circuit Schematic](Schematics.png)

### Power & Grounding Architecture
As shown in the Fritzing wiring diagram ([`Schematics.png`](Schematics.png)):
- **External Power Supply**: A dedicated external power supply connected via a DC barrel jack adapter powers the breadboard `+`/`-` rails, supplying high current directly to the **6 servo motors** without overloading the Arduino's onboard voltage regulator.
- **Common Ground**: The Arduino UNO R3 `GND` pin is connected to the breadboard negative rail so the Arduino, external servo power supply, and sensors share a common ground reference.
- **Sensor Power**: The **HC-SR04** (`VCC`) and **TCS34725** (`VIN`) are powered from the Arduino UNO `5V` pin and share the common ground.

---

## Hardware Components & Pinout

| Component | Module / Function | Arduino UNO Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Microcontroller** | Arduino UNO R3 | — | Single-file controller at `9600` baud Serial |
| **Ultrasonic Sensor** | HC-SR04 `Echo` | `D7` | Object presence threshold: `<= 10.0 cm`, 3-sample median + `250 ms` settle check (`30 ms` timeout) |
| **Ultrasonic Sensor** | HC-SR04 `Trig` | `D8` | `10 µs` trigger pulse |
| **RGB Color Sensor** | TCS34725 `SDA` (I2C) | `A4` | Hardware I2C address `0x29`, auto-increment `0x20` read; full-spectrum RGB + HSV (Hue `0°–360°`, Saturation `0–100%`) |
| **RGB Color Sensor** | TCS34725 `SCL` (I2C) | `A5` | `50 ms` integration time, `4x` analog gain, 3-sample average; 9-color classification (`Red`, `Orange`, `Yellow`, `Green`, `Cyan`, `Blue`, `Purple`, `Magenta`, `White`) |
| **Servo 1** | Base Rotation (`servos[0]`) | `D9` | Swings arm between pickup (`140°`–`145°`) and sorting buckets (`15°`–`100°`) |
| **Servo 2** | Shoulder (`servos[1]`) | `D5` | Vertical arm elevation and reach |
| **Servo 3** | Elbow (`servos[2]`) | `D10` | Forearm extension (`0°` transit/standby, `15°` pickup dip) |
| **Servo 4** | Wrist Pitch (`servos[3]`) | `D6` | End-effector pitch angle |
| **Servo 5** | Wrist Roll (`servos[4]`) | `D11` | End-effector roll orientation (`35°` default, `20°` right bucket) |
| **Servo 6** | Gripper Claw (`servos[5]`) | `D3` | `35°` Open (`GRIP_FULL_OPEN`) / `180°` Closed (`GRIP_FULL_CLOSE`) |

---

## Calibrated 6-Servo Poses

All poses follow the joint order `{ Base (D9), Shoulder (D5), Elbow (D10), Wrist Pitch (D6), Wrist Roll (D11), Gripper (D3) }` in degrees:

| Pose Name | Target / Role | Base (`D9`) | Shoulder (`D5`) | Elbow (`D10`) | Wrist Pitch (`D6`) | Wrist Roll (`D11`) | Gripper (`D3`) | Array Literal |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| `BASE_POSE` | Base Hover / Standby | `145` | `145` | `0` | `100` | `35` | `180` | `{ 145, 145, 0, 100, 35, 180 }` |
| `PICKUP_POSE` | Pickup Location Grab Pose | `140` | `120` | `15` | `75` | `35` | `35` | `{ 140, 120, 15, 75, 35, 35 }` |
| `MID_BUCKET_POSE` | Mid Bucket → **Blue / Purple / White** | `55` | `145` | `0` | `100` | `35` | `180` | `{ 55, 145, 0, 100, 35, 180 }` |
| `LEFT_BUCKET_POSE` | Left Bucket → **Green / Yellow / Cyan** | `100` | `125` | `0` | `55` | `35` | `180` | `{ 100, 125, 0, 55, 35, 180 }` |
| `RIGHT_BUCKET_POSE` | Right Bucket → **Red / Orange / Magenta** | `15` | `135` | `0` | `70` | `20` | `180` | `{ 15, 135, 0, 70, 20, 180 }` |

- **High-Clearance Upper-Arm Transit Posture**: `{ Shoulder, Elbow, Wrist Pitch, Wrist Roll } = { 145, 0, 100, 35 }` — lifted prior to any `Base (D9)` rotation so the gripper clears the pickup housing and sorting bins.

---

## 3D Model & Physical Layout

The complete mechanical layout and printable parts are provided in [`3d Model.stl`](3d%20Model.stl) and its parametric OpenSCAD source [`3d Model.scad`](3d%20Model.scad) (static render: [`3d Model.png`](3d%20Model.png)).

### Robotic Arm Link Dimensions
- **Base height (`L1`)**: `64.50 mm` (surface to Joint 2 pivot)
- **Upper arm (`L2`)**: `105.00 mm` (Joint 2 to Joint 3)
- **Forearm (`L3`)**: `98.50 mm` (Joint 3 to Joint 4)
- **Wrist + Gripper (`L4`)**: `115.00 mm` (Joint 4 to end-effector tip)
- **Wrist Mounting Offset**: `45.00°`

### Board & Station Coordinates (OpenSCAD Reference)
- **Base Board**: `360 x 280 x 3.175 mm` (`X ∈ [-180, 180]`, `Y ∈ [-140, 140]`)
- **Arm Base Origin**: `(-15, 20, 3.175)` mm
- **Arduino UNO R3 Mount**: `(-132, -6.6, 3.175)` mm (behind the arm base)
- **Angled Sensor Mount**: `X = 108 mm`, angled at `-42°` overlooking the pickup window (`TCS34725` at `[108, -1, 40.675]`, `HC-SR04` at `[108, 31, 40.675]`)
- **Pickup Basket**: `(75, 15, 3.175)` mm with sensor window (`74 x 74 x 36 mm`) + inverted stand at `(148, 15, 3.175)` mm → **`PICKUP_POSE`** (`Base = 140°`)
- **Front-Row Sorting Baskets** (OpenSCAD `Y = -85 mm`, `36 mm` height):
  - **Green Bin** (Left, Black — Green / Yellow / Cyan Family): Board `(-115, -85, 3.175)` mm → **`LEFT_BUCKET_POSE`** (`Base = 100°`)
  - **Blue Bin** (Middle, RoyalBlue — Blue / Purple / White Family): Board `(-35, -85, 3.175)` mm → **`MID_BUCKET_POSE`** (`Base = 55°`)
  - **Red Bin** (Right, WhiteSmoke — Red / Orange / Magenta Family): Board `(45, -85, 3.175)` mm → **`RIGHT_BUCKET_POSE`** (`Base = 15°`)

### Reference Tested Object Specifications
- **Blue object**: `32 g`, `3.1 cm x 3.4 cm x 5.1 cm`
- **Green object**: `29 g`, `3.1 cm x 3.4 cm x 4.6 cm`
- **Red object**: `31 g`, `3.1 cm x 3.4 cm x 4.9 cm`

---

## How It Works & Trajectory Sequence

The firmware in [`Pick_and_Place_Color_Sorting_Object_RobotArm.ino`](Pick_and_Place_Color_Sorting_Object_RobotArm.ino) is self-contained and operates in the following sequence:

### 1. Staggered Startup (`setup()`)
- Starts `Serial` at `9600` baud, configures HC-SR04 trigger/echo pins (`D8`/`D7`), and initializes the TCS34725 (`0x29`, `50 ms` integration, `4x` gain).
- Attaches all 6 servos one-by-one with an **`800 ms` delay** between each servo, pre-positioning every joint directly to **`BASE_POSE`** (`{ 145, 145, 0, 100, 35, 180 }`) to prevent power-rail brownouts.

### 2. Object Detection & Color Sensing (`loop()`)
- **Ultrasonic Check (`HC-SR04`)**: Reads a 3-sample median distance (`readStableDistanceCm()`). When an object is detected within `<= 10.0 cm`, the controller pauses **`250 ms`** for the object to settle and re-confirms `<= 10.0 cm`.
- **Full-Spectrum RGB + HSV Color Classification (`TCS34725`)**: Performs 3-sample averaged I2C register reads (`readColorsAveraged(3)`) using the TCS34725 auto-increment command bit (`0x20`), normalizes `R`/`G`/`B` against Clear (`0–255`), and converts RGB to **HSV** (Hue `0°–360°`, Saturation `0–100%`). `detectColor()` classifies all **9 detectable colors** (`Red`, `Orange`, `Yellow`, `Green`, `Cyan`, `Blue`, `Purple`, `Magenta`, and `White`) or ignores dark/low-reflectance objects (`COLOR_NONE`), prints `R`, `G`, `B`, `C`, `Hue (deg)`, `Saturation (%)`, and the detected color name to the Serial Monitor (`9600` baud), and routes each color family to its calibrated sorting bucket:
  - **Right Bucket (`RIGHT_BUCKET_POSE`, Base `15°` — Red / Warm Family)**: `COLOR_RED`, `COLOR_ORANGE`, `COLOR_MAGENTA`
  - **Left Bucket (`LEFT_BUCKET_POSE`, Base `100°` — Green / Yellow / Cyan Family)**: `COLOR_GREEN`, `COLOR_YELLOW`, `COLOR_CYAN`
  - **Mid Bucket (`MID_BUCKET_POSE`, Base `55°` — Blue / Purple / White Family)**: `COLOR_BLUE`, `COLOR_PURPLE`, `COLOR_WHITE`
  - **Ignored (`COLOR_NONE`)**: Dark (`rawClear < 50`) or low-reflectance gray (`rawClear < 450` and RGB spread `< 6`).

### Full-Spectrum TCS34725 Color Classification & Bucket Routing

| Enum Value | Color Name | Classification Criteria (RGB / HSV / Clear) | Target Bucket Pose | Color Family & Base Angle |
| :--- | :--- | :--- | :--- | :--- |
| `COLOR_RED` | `Red` | `Hue < 18°` or `>= 340°` (RGB spread `>= 6`, `rawClear >= 50`) | `RIGHT_BUCKET_POSE` | **Right Bucket** (`15°` — Red / Warm Family) |
| `COLOR_ORANGE` | `Orange` | `18° <= Hue < 42°` (RGB spread `>= 6`, `rawClear >= 50`) | `RIGHT_BUCKET_POSE` | **Right Bucket** (`15°` — Red / Warm Family) |
| `COLOR_MAGENTA` | `Magenta` | `295° <= Hue < 340°` (RGB spread `>= 6`, `rawClear >= 50`) | `RIGHT_BUCKET_POSE` | **Right Bucket** (`15°` — Red / Warm Family) |
| `COLOR_GREEN` | `Green` | `72° <= Hue < 160°` (RGB spread `>= 6`, `rawClear >= 50`) | `LEFT_BUCKET_POSE` | **Left Bucket** (`100°` — Green / Yellow / Cyan Family) |
| `COLOR_YELLOW` | `Yellow` | `42° <= Hue < 72°` (RGB spread `>= 6`, `rawClear >= 50`) | `LEFT_BUCKET_POSE` | **Left Bucket** (`100°` — Green / Yellow / Cyan Family) |
| `COLOR_CYAN` | `Cyan` | `160° <= Hue < 200°` (RGB spread `>= 6`, `rawClear >= 50`) | `LEFT_BUCKET_POSE` | **Left Bucket** (`100°` — Green / Yellow / Cyan Family) |
| `COLOR_BLUE` | `Blue` | `200° <= Hue < 255°` (RGB spread `>= 6`, `rawClear >= 50`) | `MID_BUCKET_POSE` | **Mid Bucket** (`55°` — Blue / Purple / White Family) |
| `COLOR_PURPLE` | `Purple` | `255° <= Hue < 295°` (RGB spread `>= 6`, `rawClear >= 50`) | `MID_BUCKET_POSE` | **Mid Bucket** (`55°` — Blue / Purple / White Family) |
| `COLOR_WHITE` | `White` | `rawClear >= 450` with RGB spread `< 6` | `MID_BUCKET_POSE` | **Mid Bucket** (`55°` — Blue / Purple / White Family) |
| `COLOR_NONE` | `None` (Ignored) | Dark (`rawClear < 50`) or low-reflectance gray (`rawClear < 450` and spread `< 6`) | *Ignored* | Remains at `BASE_POSE` (waits) |

### 3. 5-Stage Clearance-Safe Pick-and-Place Trajectory (`executePickAndPlace()`)
All joint transitions interpolate smoothly at **`1°` per step** (`20 ms/deg` normal speed, `10 ms/deg` fast speed via `stepDelayMs`), lifting the upper arm to the clearance posture `{ 145, 0, 100, 35 }` before any `Base (D9)` rotation:

1. **Stage 1 — Standby, Open & Dip**: Ensures the arm is at `BASE_POSE` (`{ 145, 145, 0, 100, 35, 180 }`), opens Gripper (`D3`) to `35°` (`PICKUP_POSE[5]`), and moves smoothly via `moveToPoseSmooth(PICKUP_POSE)` to `{ 140, 120, 15, 75, 35, 35 }`.
2. **Stage 2 — Grab & Lift**: Closes Gripper (`D3`) to `180°` (`GRIP_FULL_CLOSE`) to secure the object, then lifts smoothly back to `BASE_POSE` (`{ 145, 145, 0, 100, 35, 180 }`) while holding Gripper closed (`180°`).
3. **Stage 3 — Clearance Transit to Bucket**: Rotates `Base (D9)` at high-clearance posture `{ 145, 0, 100, 35 }` to the target bucket angle (`55°`, `100°`, or `15°`), then moves `{ Shoulder, Elbow, Wrist Pitch, Wrist Roll }` smoothly into the target bucket pose while holding Gripper at `180°`.
4. **Stage 4 — Release**: Opens Gripper (`D3`) to `35°` (`GRIP_FULL_OPEN`) to drop the object into the bucket.
5. **Stage 5 — Retract & Return**: Lifts the upper arm smoothly back to clearance `{ 145, 0, 100, 35 }`, rotates `Base (D9)` back to `145°`, and restores `BASE_POSE` (`{ 145, 145, 0, 100, 35, 180 }`).

---

## Serial Monitor Commands (`9600` Baud)

Open the Arduino IDE **Serial Monitor** at `9600` baud (handled by `handleSerialCommands()`) to view live TCS34725 color telemetry (`R`, `G`, `B`, `C`, `Hue (deg)`, `Saturation (%)`, and the detected color name), test individual poses, run full sorting cycles, or adjust motion behavior:

| Command | Action | Target / Effect |
| :--- | :--- | :--- |
| `'1'` | Direct pose test | Moves smoothly to `BASE_POSE` `{ 145, 145, 0, 100, 35, 180 }` |
| `'2'` | Direct pose test | Moves smoothly to `PICKUP_POSE` `{ 140, 120, 15, 75, 35, 35 }` |
| `'3'` | Direct pose test | Moves smoothly to `MID_BUCKET_POSE` `{ 55, 145, 0, 100, 35, 180 }` |
| `'4'` | Direct pose test | Moves smoothly to `LEFT_BUCKET_POSE` `{ 100, 125, 0, 55, 35, 180 }` |
| `'5'` | Direct pose test | Moves smoothly to `RIGHT_BUCKET_POSE` `{ 15, 135, 0, 70, 20, 180 }` |
| `'m'` / `'b'` | Full pick-and-place test | Runs 5-stage `executePickAndPlace()` to **Mid Bucket** (`MID_BUCKET_POSE` — Blue / Purple / White family) |
| `'l'` / `'g'` | Full pick-and-place test | Runs 5-stage `executePickAndPlace()` to **Left Bucket** (`LEFT_BUCKET_POSE` — Green / Yellow / Cyan family) |
| `'r'` | Full pick-and-place test | Runs 5-stage `executePickAndPlace()` to **Right Bucket** (`RIGHT_BUCKET_POSE` — Red / Orange / Magenta family) |
| `'d'` | Pickup dip test | Dips smoothly to `PICKUP_POSE` and lifts back to `BASE_POSE` |
| `'p'` / `'h'` | Return to standby | Moves smoothly back to `BASE_POSE` (`{ 145, 145, 0, 100, 35, 180 }`) |
| `'s'` | Auto-sort toggle | Pauses or resumes automatic HC-SR04 + TCS34725 sorting |
| `'f'` | Fast speed | Sets smooth interpolation delay to `10 ms/deg` |
| `'n'` | Normal speed | Sets smooth interpolation delay to `20 ms/deg` |

---

## Getting Started

1. Wire the Arduino UNO R3, **6 servos**, external servo power supply, HC-SR04, and TCS34725 according to [`Schematics.png`](Schematics.png) and the pinout table above.
2. Open [`Pick_and_Place_Color_Sorting_Object_RobotArm.ino`](Pick_and_Place_Color_Sorting_Object_RobotArm.ino) in the **Arduino IDE**.
3. No external third-party color sensor library is needed—only standard built-in Arduino libraries (`Wire.h` and `Servo.h`) are used.
4. Select **Arduino Uno** and the appropriate serial port, then click **Upload**.
5. Open the **Serial Monitor** at `9600` baud to view live TCS34725 readings (`R`, `G`, `B`, `C`, `Hue (deg)`, `Saturation (%)`, and the detected color name) and run pose (`'1'`–`'5'`) or cycle (`'m'`/`'b'`, `'l'`/`'g'`, `'r'`, `'d'`, `'p'`/`'h'`, `'s'`, `'f'`, `'n'`) commands.

---

## Repository Files

- [`Pick_and_Place_Color_Sorting_Object_RobotArm.ino`](Pick_and_Place_Color_Sorting_Object_RobotArm.ino) — Complete Arduino UNO R3 6-servo controller sketch
- [`Schematics.png`](Schematics.png) — Fritzing circuit wiring diagram
- [`3d Model.stl`](3d%20Model.stl) — 3D assembly model (base board, arm mounts, sensor mount, pickup & sorting baskets)
- [`3d Model.scad`](3d%20Model.scad) — OpenSCAD parametric 3D source file
- [`3d_Model_Preview.gif`](3d%20Model_Preview.gif) — 360° rotating 3D model preview animation
- [`3d Model.png`](3d%20Model.png) — Static 3D model preview render
- [`README.md`](README.md) — Project documentation, pinout, calibrated poses, trajectory, and Serial commands reference
