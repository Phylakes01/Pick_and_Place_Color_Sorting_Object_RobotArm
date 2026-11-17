/**
 * @file Pick_and_Place_Color_Sorting_Object_RobotArm.ino
 * @brief Single-file controller for the Pick-and-Place Color-Sorting Robotic Arm.
 *
 * System Overview:
 * - Detects whether an object is present in the pickup zone using an HC-SR04
 *   ultrasonic sensor (distance <= 10.0 cm, median-filtered and settled).
 * - Identifies the object's color (Blue, Green, or Red) using a TCS34725 I2C
 *   RGB color sensor (averaged 3-sample scan with Auto-Increment 16-bit I2C reads).
 * - Moves a 4-DOF robotic arm and gripper (5 servos driven by geometric Inverse
 *   Kinematics with microsecond PWM interpolation and smooth return trajectories)
 *   to grasp the object and place it in its matching color bin.
 *
 * 3D Model Physical Layout (OpenSCAD Board Coordinates, in mm):
 * - Base Board     : 360 x 280 x 3.175 mm (X in [-180, 180], Y in [-140, 140])
 * - Arm Base Origin: (-15, 20, 3.175)
 * - Arduino UNO R3 : (-132, -6.6, 3.175) directly behind the arm base (-X side)
 * - Sensor Mount   : X = 108, angled at -42 deg overlooking the pickup window
 *                    (TCS34725 at [108, -1, 40.675], HC-SR04 at [108, 31, 40.675])
 * - Pickup Basket  : (75, 15, 3.175) with sensor window (74 x 74 x 36 mm) +
 *                    inverted stand at (148, 15, 3.175)
 * - Sorting Baskets: Front row at Y = -85, height = 36 mm (top rim ~39.175 mm):
 *                    Left (Black -> Green)       at (-115, -85, 3.175)
 *                    Middle (RoyalBlue -> Blue)  at (-35, -85, 3.175)
 *                    Right (WhiteSmoke -> Red)   at (45, -85, 3.175)
 *
 * Code Organization (logically separated inside this single .ino file):
 * 1. Hardware Pin Assignments, Constants, & Data Structures
 * 2. Ultrasonic Sensor Section (HC-SR04)
 * 3. Color Sensor Section (TCS34725 I2C)
 * 4. Servo, Gripper, & Inverse Kinematics Section
 * 5. Main Arduino Program (setup & loop)
 *
 * Reference Tested Object Specifications:
 * - Blue  object: mass = 32 g, dimensions = 3.1 cm x 3.4 cm x 5.1 cm
 * - Green object: mass = 29 g, dimensions = 3.1 cm x 3.4 cm x 4.6 cm
 * - Red   object: mass = 31 g, dimensions = 3.1 cm x 3.4 cm x 4.9 cm
 */

#include <Wire.h>
#include <Servo.h>
#include <math.h>

// ============================================================================
// 1. HARDWARE PINS, CONSTANTS, & DATA STRUCTURES
// ============================================================================

// --- HC-SR04 Ultrasonic Sensor Pins ---
const uint8_t ULTRASONIC_ECHO_PIN = 7;   // HC-SR04 "Echo" pin -> Digital Pin 7
const uint8_t ULTRASONIC_TRIG_PIN = 8;   // HC-SR04 "Trig" pin -> Digital Pin 8

// --- TCS34725 Color Sensor Pins (Hardware I2C on Arduino UNO) ---
// Power: VIN -> 5V, GND -> GND | LED & INT left unconnected (onboard LED stays ON)
const uint8_t COLOR_SDA_PIN = A4;        // TCS34725 "SDA" -> Analog Pin A4 (Hardware I2C Data)
const uint8_t COLOR_SCL_PIN = A5;        // TCS34725 "SCL" -> Analog Pin A5 (Hardware I2C Clock)

// --- Robotic Arm Servo Pins (signal / yellow wires) ---
const uint8_t SERVO_JOINT1_PIN  = 9;     // Joint 1 (Base rotation)  -> Pin 9
const uint8_t SERVO_JOINT2_PIN  = 5;     // Joint 2 (Shoulder pitch) -> Pin 5
const uint8_t SERVO_JOINT3_PIN  = 10;    // Joint 3 (Elbow pitch)    -> Pin 10
const uint8_t SERVO_JOINT4_PIN  = 6;     // Joint 4 (Wrist pitch)    -> Pin 6
const uint8_t SERVO_GRIPPER_PIN = 3;     // Gripper mechanism servo  -> Pin 3

// --- Ultrasonic & TCS34725 Color Sensor Settings ---
const float DETECTION_DISTANCE_CM   = 10.00f;   // Object pickup threshold (cm)
const unsigned long ECHO_TIMEOUT_US = 30000UL;  // 30 ms timeout (~5.1 m max range)
const unsigned long SETTLE_DELAY_MS = 250UL;    // Wait for object to come to rest in basket

// TCS34725 I2C address, register map, and gain/integration settings
const uint8_t TCS34725_I2C_ADDR    = 0x29;  // Fixed 7-bit I2C address of TCS34725
const uint8_t TCS34725_COMMAND_BIT = 0x80;  // Command bit required for register access
const uint8_t TCS34725_AUTOINC_BIT = 0x20;  // Auto-increment protocol for 16-bit register pairs
const uint8_t TCS34725_REG_ENABLE  = 0x00;  // Power and ADC enable register
const uint8_t TCS34725_REG_ATIME   = 0x01;  // Integration time register
const uint8_t TCS34725_REG_CONTROL = 0x0F;  // Gain control register
const uint8_t TCS34725_REG_CDATAL  = 0x14;  // Clear channel 16-bit data (low byte)
const uint8_t TCS34725_REG_RDATAL  = 0x16;  // Red channel 16-bit data (low byte)
const uint8_t TCS34725_REG_GDATAL  = 0x18;  // Green channel 16-bit data (low byte)
const uint8_t TCS34725_REG_BDATAL  = 0x1A;  // Blue channel 16-bit data (low byte)

const uint8_t TCS34725_ENABLE_PON  = 0x01;  // Power ON internal oscillator
const uint8_t TCS34725_ENABLE_AEN  = 0x02;  // Enable RGBC ADC
const uint8_t TCS34725_ATIME_50MS  = 0xEB;  // 50 ms integration time (21 cycles)
const uint8_t TCS34725_GAIN_4X     = 0x01;  // 4x analog gain

const uint16_t COLOR_MIN_CLEAR     = 50;    // Minimum Clear count to accept reading
const int COLOR_MIN_SPREAD         = 6;     // Minimum (max - min) RGB spread (0..255) to reject neutral gray/white

// --- Robotic Arm Physical Link Dimensions (millimeters) ---
const float PI_VAL           = 3.141592653589793f;
const float RAD_TO_DEG_VAL   = 180.0f / PI_VAL;
const float LINK_L1          = 64.50f;   // Base height from surface to Joint 2 pivot
const float LINK_L2          = 105.00f;  // Upper arm length (Joint 2 to Joint 3)
const float LINK_L3          = 98.50f;   // Forearm length (Joint 3 to Joint 4)
const float LINK_L4          = 115.00f;  // Wrist + gripper length (Joint 4 to tip)
const float WRIST_OFFSET_DEG = 45.00f;   // Mechanical mounting offset on Joint 4

// --- Calibrated Servo PWM Bounds (microseconds) ---
// Joint 1: 209 deg range over 550..2250 us -> 550 + 180 / (209 / (2250 - 550)) = 2014 us
const int SERVO1_MIN_PWM = 550;
const int SERVO1_MAX_PWM = 2014;
// Joints 2 & 3: full 500..2500 us range
const int SERVO2_MIN_PWM = 500;
const int SERVO2_MAX_PWM = 2500;
const int SERVO3_MIN_PWM = 500;
const int SERVO3_MAX_PWM = 2500;
// Joint 4: 164 deg range over 550..2000 us -> 550 + 180 / (164 / (2000 - 550)) = 2141 us
const int SERVO4_MIN_PWM = 550;
const int SERVO4_MAX_PWM = 2141;

// --- Gripper Servo Angles (degrees) ---
const int GRIPPER_OPEN_ANGLE    = 90;   // Open jaws to release or prepare to grab
const int GRIPPER_CLOSE_ANGLE   = 120;  // Closed jaws to hold the object firmly
const int GRIPPER_STANDBY_ANGLE = 150;  // Wide open idle angle while waiting

// --- 3D Model Reference Coordinates & IK Radial Offset Mapping (millimeters) ---
// OpenSCAD arm pedestal origin: (-15, 20) on the 360 x 280 mm board.
// Because calculateInverseKinematics() subtracts LINK_L4 (115 mm) while the angled
// gripper jaws sit ~25 mm ahead of the wrist pivot, IK targets use D_ik = D_board + 90 mm
// along each basket's exact board angle (avoiding overshoot on outer baskets).
const float ARM_BASE_SCAD_X     = -15.00f;  // Arm base X on OpenSCAD board
const float ARM_BASE_SCAD_Y     = 20.00f;   // Arm base Y on OpenSCAD board
const float IK_RADIAL_OFFSET_MM = 90.00f;   // Additive radial offset (L4 115 mm - 25 mm jaw reach)
const float CONTAINER_DROP_Y    = 50.00f;   // Safe release height above 36 mm container + 3.175 mm board (~39.2 mm)

// --- Pick, Travel, & Bin Coordinates (millimeters, derived from 3D model) ---
// Pickup Basket at OpenSCAD (75, 15): dx = +90, dz = +5, D_board = 90.14 mm (theta1 = 3.18 deg)
// D_ik = 90.14 + 90.0 = 180.14 mm -> IK (180.00, 40.00, 10.00)
const float PICKUP_X  = 180.00f;  // Pickup zone X coordinate (D_ik = 180.28 mm)
const float PICKUP_Y  = 40.00f;   // ~39.2 mm container rim / inside basket
const float PICKUP_Z  = 10.00f;   // Pickup zone Z coordinate
const float TRAVEL_Y  = 150.00f;  // Safe lifted Y height while moving between zones
const float TRANSIT_Z = 120.00f;  // Safe transit Z (> L4 = 115 mm so d > 0 across X = 0)

// Front-Row Sorting Baskets at OpenSCAD Y = -85 (dz = +105 mm from arm base):
// Middle RoyalBlue Basket at (-35, -85): dx = -20, dz = +105, D_board = 106.89 mm (theta1 = 100.78 deg)
// D_ik = 106.89 + 90.0 = 196.89 mm -> IK (-36.84, 50.00, 193.41)
const float BLUE_BIN_X  = -36.84f;
const float BLUE_BIN_Y  = CONTAINER_DROP_Y;
const float BLUE_BIN_Z  = 193.41f;

// Left Black Basket (Green target) at (-115, -85): dx = -100, dz = +105, D_board = 145.00 mm (theta1 = 133.60 deg)
// D_ik = 145.00 + 90.0 = 235.00 mm -> IK (-162.07, 50.00, 170.17)
const float GREEN_BIN_X = -162.07f;
const float GREEN_BIN_Y = CONTAINER_DROP_Y;
const float GREEN_BIN_Z = 170.17f;

// Right WhiteSmoke Basket (Red target) at (45, -85): dx = +60, dz = +105, D_board = 120.93 mm (theta1 = 60.26 deg)
// D_ik = 120.93 + 90.0 = 210.93 mm -> IK (104.65, 50.00, 183.15)
const float RED_BIN_X   = 104.65f;
const float RED_BIN_Y   = CONTAINER_DROP_Y;
const float RED_BIN_Z   = 183.15f;

// --- Smooth Motion Interpolation Settings ---
const float STEP_SIZE_MM         = 0.50f;  // Coordinate increment per step (mm)
const unsigned int STEP_DELAY_US = 1500;   // Pause between steps (microseconds) for smooth travel speed

// --- Color Classification Types ---
enum ObjectColor {
  COLOR_NONE = 0,
  COLOR_BLUE,
  COLOR_GREEN,
  COLOR_RED
};

/**
 * Stores one RGBC scan from the TCS34725 I2C color sensor.
 * Raw 16-bit counts increase with light intensity; red/green/blue are
 * normalized against rawClear to a 0..255 scale for lighting-tolerant sorting.
 */
struct ColorReading {
  uint16_t rawClear;  // Raw 16-bit Clear (ambient + reflected light) ADC count
  uint16_t rawRed;    // Raw 16-bit Red ADC count
  uint16_t rawGreen;  // Raw 16-bit Green ADC count
  uint16_t rawBlue;   // Raw 16-bit Blue ADC count
  int red;            // Normalized Red intensity (0..255)
  int green;          // Normalized Green intensity (0..255)
  int blue;           // Normalized Blue intensity (0..255)
};

// --- Servo Objects & Arm Kinematic State ---
Servo servo1;   // Joint 1: Base rotation
Servo servo2;   // Joint 2: Shoulder pitch
Servo servo3;   // Joint 3: Elbow pitch
Servo servo4;   // Joint 4: Wrist pitch
Servo gripper;  // End-effector gripper servo

float xEndEffector = PICKUP_X;  // Current X coordinate of gripper tip (mm)
float yEndEffector = TRAVEL_Y;  // Current Y height of gripper tip (mm)
float zEndEffector = PICKUP_Z;  // Current Z depth of gripper tip (mm)

float theta1 = 0.0f;  // Solved Joint 1 angle (degrees)
float theta2 = 0.0f;  // Solved Joint 2 angle (degrees)
float theta3 = 0.0f;  // Solved Joint 3 angle (degrees)
float theta4 = 0.0f;  // Solved Joint 4 angle (degrees)

bool isAtStandby = false;  // Tracks whether arm is already parked at standby

// Explicit prototypes for functions using custom types
ColorReading readColors();
ColorReading readColorsAveraged(uint8_t sampleCount);
ObjectColor detectColor(const ColorReading& reading);
void printColorReading(const ColorReading& reading);

inline float sq_f(float v) {
  return v * v;
}


// ============================================================================
// 2. ULTRASONIC SENSOR FUNCTIONS (HC-SR04)
// ============================================================================

/**
 * @brief Configures HC-SR04 Trigger (OUTPUT) and Echo (INPUT) pins.
 */
void initUltrasonic() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
}

/**
 * @brief Measures object distance in front of the pickup zone in centimeters.
 *
 * Sends a 10 us HIGH trigger pulse and converts the round-trip echo duration
 * using the speed of sound (~0.034 cm/us divided by 2). Returns 999.0 cm on
 * timeout so a missed echo is not treated as an object close to the sensor.
 *
 * @return Distance to the nearest object in centimeters (999.0 if no echo).
 */
float readDistanceCm() {
  // Ensure a clean LOW pulse before triggering the 10 us ultrasonic burst
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // Measure Echo HIGH duration (round-trip time in microseconds)
  unsigned long durationUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (durationUs == 0) {
    return 999.0f;
  }

  // Convert round-trip time to one-way distance in cm
  return (durationUs * 0.034f) / 2.0f;
}

/**
 * @brief Takes 3 ultrasonic readings and returns the median to reject echo glitches.
 */
float readStableDistanceCm() {
  float a = readDistanceCm();
  delay(15);
  float b = readDistanceCm();
  delay(15);
  float c = readDistanceCm();

  // Sort 3 values to obtain median
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}


// ============================================================================
// 3. COLOR SENSOR FUNCTIONS (TCS34725 I2C)
// ============================================================================

/**
 * @brief Writes a single 8-bit configuration value to a TCS34725 register over I2C.
 * @param reg   Target register address (combined with TCS34725_COMMAND_BIT).
 * @param value Byte value to write.
 */
void writeColorRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TCS34725_I2C_ADDR);
  Wire.write(static_cast<uint8_t>(TCS34725_COMMAND_BIT | reg));
  Wire.write(value);
  Wire.endTransmission();
}

/**
 * @brief Reads a 16-bit little-endian ADC channel value from a TCS34725 register pair.
 * Uses the TCS34725 Auto-Increment command bit (0x20) and repeated start so the
 * consecutive low and high bytes are fetched atomically.
 * @param reg Low-byte register address (e.g., TCS34725_REG_CDATAL).
 * @return 16-bit unsigned sensor count (0 if I2C transfer fails or returns < 2 bytes).
 */
uint16_t readColorRegister16(uint8_t reg) {
  Wire.beginTransmission(TCS34725_I2C_ADDR);
  Wire.write(static_cast<uint8_t>(TCS34725_COMMAND_BIT | TCS34725_AUTOINC_BIT | reg));
  if (Wire.endTransmission(false) != 0) {
    return 0;
  }

  Wire.requestFrom(TCS34725_I2C_ADDR, static_cast<uint8_t>(2));
  if (Wire.available() < 2) {
    return 0;
  }

  uint16_t lowByte = Wire.read();
  uint16_t highByte = Wire.read();
  return (highByte << 8) | lowByte;
}

/**
 * @brief Initializes I2C (A4/A5) and configures the TCS34725 sensor (50 ms, 4x gain).
 */
void initColorSensor() {
  Wire.begin();
  writeColorRegister(TCS34725_REG_ATIME, TCS34725_ATIME_50MS);
  writeColorRegister(TCS34725_REG_CONTROL, TCS34725_GAIN_4X);

  // Power ON internal oscillator first, wait 3 ms, then enable RGBC ADC
  writeColorRegister(TCS34725_REG_ENABLE, TCS34725_ENABLE_PON);
  delay(3);
  writeColorRegister(TCS34725_REG_ENABLE, static_cast<uint8_t>(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));

  // Wait for first 50 ms integration cycle to complete before reading ADC registers
  delay(60);
}

/**
 * @brief Reads raw 16-bit Clear/Red/Green/Blue channels and normalizes RGB to 0..255.
 * @return ColorReading containing raw 16-bit counts and normalized 0..255 RGB values.
 */
ColorReading readColors() {
  ColorReading reading;

  reading.rawClear = readColorRegister16(TCS34725_REG_CDATAL);
  reading.rawRed   = readColorRegister16(TCS34725_REG_RDATAL);
  reading.rawGreen = readColorRegister16(TCS34725_REG_GDATAL);
  reading.rawBlue  = readColorRegister16(TCS34725_REG_BDATAL);

  if (reading.rawClear > 0) {
    reading.red   = constrain(static_cast<int>((static_cast<unsigned long>(reading.rawRed)   * 255UL) / reading.rawClear), 0, 255);
    reading.green = constrain(static_cast<int>((static_cast<unsigned long>(reading.rawGreen) * 255UL) / reading.rawClear), 0, 255);
    reading.blue  = constrain(static_cast<int>((static_cast<unsigned long>(reading.rawBlue)  * 255UL) / reading.rawClear), 0, 255);
  } else {
    reading.red   = 0;
    reading.green = 0;
    reading.blue  = 0;
  }

  // Wait slightly longer than the 50 ms integration period for the next fresh sample
  delay(55);
  return reading;
}

/**
 * @brief Averages multiple RGBC scans for noise-resistant color classification.
 * @param sampleCount Number of scans to average (clamped to >= 1).
 * @return Averaged ColorReading.
 */
ColorReading readColorsAveraged(uint8_t sampleCount) {
  if (sampleCount == 0) {
    sampleCount = 1;
  }

  unsigned long sumClear = 0;
  unsigned long sumRed   = 0;
  unsigned long sumGreen = 0;
  unsigned long sumBlue  = 0;

  for (uint8_t i = 0; i < sampleCount; ++i) {
    ColorReading sample = readColors();
    sumClear += sample.rawClear;
    sumRed   += sample.rawRed;
    sumGreen += sample.rawGreen;
    sumBlue  += sample.rawBlue;
  }

  ColorReading avg;
  avg.rawClear = static_cast<uint16_t>(sumClear / sampleCount);
  avg.rawRed   = static_cast<uint16_t>(sumRed   / sampleCount);
  avg.rawGreen = static_cast<uint16_t>(sumGreen / sampleCount);
  avg.rawBlue  = static_cast<uint16_t>(sumBlue  / sampleCount);

  if (avg.rawClear > 0) {
    avg.red   = constrain(static_cast<int>((static_cast<unsigned long>(avg.rawRed)   * 255UL) / avg.rawClear), 0, 255);
    avg.green = constrain(static_cast<int>((static_cast<unsigned long>(avg.rawGreen) * 255UL) / avg.rawClear), 0, 255);
    avg.blue  = constrain(static_cast<int>((static_cast<unsigned long>(avg.rawBlue)  * 255UL) / avg.rawClear), 0, 255);
  } else {
    avg.red   = 0;
    avg.green = 0;
    avg.blue  = 0;
  }

  return avg;
}

/**
 * @brief Classifies an object's color from normalized 0..255 RGB channels.
 * Rejects dark/disconnected readings (rawClear < COLOR_MIN_CLEAR) and neutral white/gray
 * surfaces where channel contrast (max - min) < COLOR_MIN_SPREAD, then selects the
 * highest RGB channel.
 * @param reading ColorReading obtained from readColors() or readColorsAveraged().
 * @return ObjectColor (COLOR_BLUE, COLOR_GREEN, COLOR_RED, or COLOR_NONE).
 */
ObjectColor detectColor(const ColorReading& reading) {
  if (reading.rawClear < COLOR_MIN_CLEAR) {
    return COLOR_NONE;  // Too dark or sensor not responding
  }

  int maxCh = max(reading.red, max(reading.green, reading.blue));
  int minCh = min(reading.red, min(reading.green, reading.blue));
  if ((maxCh - minCh) < COLOR_MIN_SPREAD) {
    return COLOR_NONE;  // Neutral / gray / white background
  }

  if (reading.blue >= reading.red && reading.blue >= reading.green) {
    return COLOR_BLUE;
  }
  if (reading.green >= reading.red && reading.green >= reading.blue) {
    return COLOR_GREEN;
  }
  return COLOR_RED;
}

/**
 * @brief Prints normalized RGB values and Clear count to Serial for calibration.
 * @param reading ColorReading whose values will be printed.
 */
void printColorReading(const ColorReading& reading) {
  Serial.print("R= ");
  Serial.print(reading.red);
  Serial.print("  G= ");
  Serial.print(reading.green);
  Serial.print("  B= ");
  Serial.print(reading.blue);
  Serial.print("  C= ");
  Serial.println(reading.rawClear);
}


// ============================================================================
// 4. SERVO, GRIPPER, & INVERSE KINEMATICS FUNCTIONS
// ============================================================================

/**
 * @brief Maps a floating-point angle [0..180 deg] to calibrated microseconds for sub-degree accuracy.
 */
int angleToPwmUs(float angleDeg, int minPwm, int maxPwm) {
  float clamped = constrain(angleDeg, 0.0f, 180.0f);
  return static_cast<int>(minPwm + ((clamped / 180.0f) * static_cast<float>(maxPwm - minPwm)) + 0.5f);
}

/**
 * @brief Computes joint angles (theta1..theta4) from (xEndEffector, yEndEffector, zEndEffector).
 *
 * Solves geometric 4-DOF Inverse Kinematics (originally named Rumus_IK).
 * Uses sq_f() instead of pow(..., 2) and atan2() for negative-coordinate safety on AVR.
 * Keeps link L4 horizontal while calculating planar shoulder, elbow, and wrist angles.
 */
void calculateInverseKinematics() {
  // 1. Horizontal radial distance D in X-Z plane and Base rotation angle theta1
  float D = sqrt(sq_f(xEndEffector) + sq_f(zEndEffector));
  theta1 = atan2(zEndEffector, xEndEffector) * RAD_TO_DEG_VAL;
  if (theta1 < 0.0f) {
    theta1 = 0.0f;
  }

  // 2. Effective planar reach (d, yOffset) from Joint 2 pivot to Joint 4 wrist pivot
  float d = D - LINK_L4;
  float yOffset = yEndEffector - LINK_L1;
  float R = sqrt(sq_f(d) + sq_f(yOffset));

  // Guard against division by zero if wrist pivot coincides with shoulder pivot
  if (R <= 0.0001f) {
    return;
  }

  // 3. Triangle angles via Law of Cosines (constrained to [-1, 1] to avoid NaN)
  // alpha1: angle of vector R relative to horizontal; alpha2: shoulder triangle offset
  float cosAlpha1 = constrain(d / R, -1.0f, 1.0f);
  float cosAlpha2 = constrain((sq_f(LINK_L2) + sq_f(R) - sq_f(LINK_L3)) / (2.0f * LINK_L2 * R), -1.0f, 1.0f);
  float cosTheta3 = constrain((sq_f(LINK_L2) + sq_f(LINK_L3) - sq_f(R)) / (2.0f * LINK_L2 * LINK_L3), -1.0f, 1.0f);

  float alpha1 = acos(cosAlpha1) * RAD_TO_DEG_VAL;
  float alpha2 = acos(cosAlpha2) * RAD_TO_DEG_VAL;
  theta3 = acos(cosTheta3) * RAD_TO_DEG_VAL;

  // 4. Shoulder (theta2) and Wrist (theta4) angles depend on whether Y is above/below L1
  if (yEndEffector >= LINK_L1) {
    theta2 = alpha1 + alpha2;
    theta4 = 180.00f - ((180.00f - (alpha2 + theta3)) - alpha1);
  } else {
    theta2 = alpha2 - alpha1;
    theta4 = 180.00f - ((180.00f - (alpha2 + theta3)) + alpha1);
  }
}

/**
 * @brief Sends the computed joint angles to the 4 arm servos in microseconds (with -45 deg wrist offset).
 */
void applyJointAngles() {
  servo1.writeMicroseconds(angleToPwmUs(theta1, SERVO1_MIN_PWM, SERVO1_MAX_PWM));
  servo2.writeMicroseconds(angleToPwmUs(theta2, SERVO2_MIN_PWM, SERVO2_MAX_PWM));
  servo3.writeMicroseconds(angleToPwmUs(theta3, SERVO3_MIN_PWM, SERVO3_MAX_PWM));
  servo4.writeMicroseconds(angleToPwmUs(theta4 - WRIST_OFFSET_DEG, SERVO4_MIN_PWM, SERVO4_MAX_PWM));
}

/**
 * @brief Pre-loads standby pulses and attaches all 5 servos with calibrated PWM limits.
 * Writing standby pulses before attach() avoids the default 90-degree startup jerk.
 */
void initServos() {
  xEndEffector = PICKUP_X;
  yEndEffector = TRAVEL_Y;
  zEndEffector = PICKUP_Z;
  calculateInverseKinematics();

  applyJointAngles();
  gripper.write(GRIPPER_STANDBY_ANGLE);

  servo1.attach(SERVO_JOINT1_PIN, SERVO1_MIN_PWM, SERVO1_MAX_PWM);
  servo2.attach(SERVO_JOINT2_PIN, SERVO2_MIN_PWM, SERVO2_MAX_PWM);
  servo3.attach(SERVO_JOINT3_PIN, SERVO3_MIN_PWM, SERVO3_MAX_PWM);
  servo4.attach(SERVO_JOINT4_PIN, SERVO4_MIN_PWM, SERVO4_MAX_PWM);
  gripper.attach(SERVO_GRIPPER_PIN);

  // Re-apply with attached min/max bounds active
  applyJointAngles();
  gripper.write(GRIPPER_STANDBY_ANGLE);
  isAtStandby = true;
}

/**
 * @brief Smoothly moves a single coordinate axis in STEP_SIZE_MM increments to targetValue.
 * @param axisCoord   Reference to xEndEffector, yEndEffector, or zEndEffector.
 * @param targetValue Destination coordinate in millimeters.
 */
void moveAxisSmooth(float& axisCoord, float targetValue) {
  if (axisCoord < targetValue) {
    // Sweep upward/forward in smooth steps
    for (; axisCoord < targetValue; axisCoord += STEP_SIZE_MM) {
      calculateInverseKinematics();
      applyJointAngles();
      delayMicroseconds(STEP_DELAY_US);
    }
  } else if (axisCoord > targetValue) {
    // Sweep downward/backward in smooth steps
    for (; axisCoord > targetValue; axisCoord -= STEP_SIZE_MM) {
      calculateInverseKinematics();
      applyJointAngles();
      delayMicroseconds(STEP_DELAY_US);
    }
  }
  // Snap cleanly to exact target and apply final joint angles
  axisCoord = targetValue;
  calculateInverseKinematics();
  applyJointAngles();
}

/**
 * @brief Smoothly returns the arm to its standby waiting position if not already parked there.
 */
void moveToStandby() {
  if (isAtStandby) {
    return;
  }

  gripper.write(GRIPPER_STANDBY_ANGLE);
  delay(150);

  moveAxisSmooth(yEndEffector, TRAVEL_Y);
  moveAxisSmooth(zEndEffector, TRANSIT_Z);
  moveAxisSmooth(xEndEffector, PICKUP_X);
  moveAxisSmooth(zEndEffector, PICKUP_Z);

  isAtStandby = true;
}

/**
 * @brief Runs the full smooth pick-and-place trajectory and returns smoothly to standby.
 * @param dropX Target bin X coordinate in mm.
 * @param dropY Target bin drop height in mm.
 * @param dropZ Target bin Z coordinate in mm.
 */
void pickAndPlace(float dropX, float dropY, float dropZ) {
  isAtStandby = false;

  // 1. Ensure arm is smoothly at hover point above pickup zone with gripper open
  gripper.write(GRIPPER_OPEN_ANGLE);
  moveAxisSmooth(yEndEffector, TRAVEL_Y);
  if (fabs(xEndEffector - PICKUP_X) > 0.01f || fabs(zEndEffector - PICKUP_Z) > 0.01f) {
    moveAxisSmooth(zEndEffector, TRANSIT_Z);
    moveAxisSmooth(xEndEffector, PICKUP_X);
    moveAxisSmooth(zEndEffector, PICKUP_Z);
  }
  delay(150);

  // 2. Lower down, grip object, and lift back to travel height
  moveAxisSmooth(yEndEffector, PICKUP_Y);
  delay(300);
  gripper.write(GRIPPER_CLOSE_ANGLE);
  delay(300);
  moveAxisSmooth(yEndEffector, TRAVEL_Y);

  // 3. Travel smoothly over to the destination color bin
  moveAxisSmooth(zEndEffector, TRANSIT_Z);
  moveAxisSmooth(xEndEffector, dropX);
  moveAxisSmooth(zEndEffector, dropZ);

  // 4. Lower into bin, release object, and retract upward
  moveAxisSmooth(yEndEffector, dropY);
  delay(150);
  gripper.write(GRIPPER_OPEN_ANGLE);
  delay(150);
  moveAxisSmooth(yEndEffector, TRAVEL_Y);

  // 5. Smoothly return to standby hover above pickup zone (no sudden snap)
  moveAxisSmooth(zEndEffector, TRANSIT_Z);
  moveAxisSmooth(xEndEffector, PICKUP_X);
  moveAxisSmooth(zEndEffector, PICKUP_Z);
  gripper.write(GRIPPER_STANDBY_ANGLE);
  isAtStandby = true;
  delay(150);
}


// ============================================================================
// 5. MAIN ARDUINO PROGRAM (SETUP & LOOP)
// ============================================================================

/**
 * @brief Initializes Serial Monitor and all hardware subsystems on startup.
 */
void setup() {
  Serial.begin(9600);
  initUltrasonic();
  initColorSensor();
  initServos();
}

/**
 * @brief Continuously scans distance & color, then parks at standby or sorts the object.
 */
void loop() {
  // 1. Check whether an object is within pickup range (<= 10 cm, median filtered)
  float distanceCm = readStableDistanceCm();
  // Serial.print("Distance = ");
  // Serial.println(distanceCm);

  // 2. No object nearby -> park smoothly at standby
  if (distanceCm > DETECTION_DISTANCE_CM) {
    moveToStandby();
    delay(50);
    return;
  }

  // 3. Wait briefly for object to settle in pickup basket, then re-confirm
  delay(SETTLE_DELAY_MS);
  distanceCm = readStableDistanceCm();
  if (distanceCm > DETECTION_DISTANCE_CM) {
    moveToStandby();
    return;
  }

  // 4. Scan averaged RGB values from TCS34725 color sensor and sort into matching bin
  ColorReading rgb = readColorsAveraged(3);
  printColorReading(rgb);

  switch (detectColor(rgb)) {
    case COLOR_BLUE:
      pickAndPlace(BLUE_BIN_X, BLUE_BIN_Y, BLUE_BIN_Z);
      break;

    case COLOR_GREEN:
      pickAndPlace(GREEN_BIN_X, GREEN_BIN_Y, GREEN_BIN_Z);
      break;

    case COLOR_RED:
      pickAndPlace(RED_BIN_X, RED_BIN_Y, RED_BIN_Z);
      break;

    case COLOR_NONE:
    default:
      break;
  }
}
