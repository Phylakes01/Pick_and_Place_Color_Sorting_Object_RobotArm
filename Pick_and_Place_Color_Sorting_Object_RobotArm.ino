/**
 * @file Pick_and_Place_Color_Sorting_Object_RobotArm.ino
 * @brief 6-Servo Pick-and-Place Color-Sorting Robotic Arm controller for Arduino UNO.
 *
 * Verified Physical Setup (from hardware photos, sweep video, & Base_And_Gripper_Sweep_Test.ino):
 * - 6 Servos on Pins { 9, 5, 10, 6, 11, 3 }:
 *   - Pin 9  (servos[0]): Base Rotation (15 deg to 150 deg, 45 deg spacing per container)
 *   - Pin 5  (servos[1]): Lower Shoulder (held at calibrated 150 deg)
 *   - Pin 10 (servos[2]): Upper Shoulder / Elbow (held at calibrated 25 deg)
 *   - Pin 6  (servos[3]): Wrist Pitch (held at calibrated 35 deg)
 *   - Pin 11 (servos[4]): Wrist Roll (held at calibrated 35 deg)
 *   - Pin 3  (servos[5]): Gripper Claw (35 = open, 140 = close, 105 = standby)
 *
 * Verified Container Base Angles (45 deg equal spacing):
 * - Far Sorting Bin (Green Bin)    : Base = 15 deg
 * - Middle Sorting Bin (Blue Bin)  : Base = 60 deg
 * - Near Sorting Bin (Red Bin)     : Base = 105 deg
 * - Side Pickup Container          : Base = 150 deg
 *
 * System Sequence:
 * 1. Detects object in pickup zone with HC-SR04 ultrasonic sensor (<= 10.0 cm).
 * 2. Waits briefly for object to settle, then analyzes color with TCS34725 I2C sensor.
 * 3. Smoothly rotates Base (Pin 9) to Pickup (150 deg), opens/closes Gripper (Pin 3),
 *    rotates to the matching Color Bin (15 / 60 / 105 deg), drops the object, and
 *    smoothly returns to Home Pose (60 deg).
 *
 * Serial Monitor Commands (9600 baud):
 * - 'g' : Test Pick & Place to Green Bin (15 deg)
 * - 'b' : Test Pick & Place to Blue Bin (60 deg)
 * - 'r' : Test Pick & Place to Red Bin (105 deg)
 * - 'h' : Return smoothly to Home Pose (60 deg)
 * - 's' : Pause / Resume automatic ultrasonic + color sorting
 * - 'f' : Fast base rotation (10 ms per degree)
 * - 'n' : Normal base rotation (20 ms per degree)
 */

#include <Wire.h>
#include <Servo.h>

// ============================================================================
// 1. HARDWARE PINS, VIDEO-VERIFIED ANGLES, & SENSOR CONSTANTS
// ============================================================================

// --- 6 Robotic Arm Servo Pins ---
const uint8_t NUM_SERVOS = 6;
const uint8_t servoPins[NUM_SERVOS] = {
  9,   // [0] Base Rotation (Pin 9)
  5,   // [1] Lower Shoulder (Pin 5)
  10,  // [2] Upper Shoulder / Elbow (Pin 10)
  6,   // [3] Wrist Pitch (Pin 6)
  11,  // [4] Wrist Roll (Pin 11)
  3    // [5] Gripper Claw (Pin 3)
};

// --- Exact Tested Home / Stand Pose: { Base, Servo2, Servo3, Servo4, Servo5, Gripper } ---
const int STAND_POSE[NUM_SERVOS] = { 60, 150, 25, 35, 35, 105 };

// --- Exact Container Base Angles (degrees, 45 deg spacing verified from video) ---
const int GREEN_BIN_BASE_ANGLE = 15;   // Far front sorting bin (Green)
const int BLUE_BIN_BASE_ANGLE  = 60;   // Middle front sorting bin (Blue)
const int RED_BIN_BASE_ANGLE   = 105;  // Near front sorting bin (Red)
const int PICKUP_BASE_ANGLE    = 150;  // Side pickup container

// --- Gripper Open / Close / Standby Angles (Pin 3) ---
const int GRIP_FULL_OPEN  = 35;   // Wide open claw
const int GRIP_FULL_CLOSE = 140;  // Closed claw holding object
const int GRIP_STANDBY    = 105;  // Idle standby angle

// --- Base Rotation Speed & Auto-Sort Toggle ---
int stepDelayMs = 20;   // 20 ms per degree (normal), 10 ms per degree (fast)
bool isRunning = true;  // Automatic ultrasonic + color sorting active

// --- HC-SR04 Ultrasonic Sensor Pins & Settings ---
const uint8_t ULTRASONIC_ECHO_PIN   = 7;        // HC-SR04 Echo -> Pin 7
const uint8_t ULTRASONIC_TRIG_PIN   = 8;        // HC-SR04 Trig -> Pin 8
const float DETECTION_DISTANCE_CM   = 10.00f;   // Object detection threshold (cm)
const unsigned long ECHO_TIMEOUT_US = 30000UL;  // 30 ms echo timeout
const unsigned long SETTLE_DELAY_MS = 250UL;    // Wait for object to settle before scanning

// --- TCS34725 Color Sensor Pins & Register Map (Hardware I2C: A4 SDA, A5 SCL) ---
const uint8_t COLOR_SDA_PIN        = A4;
const uint8_t COLOR_SCL_PIN        = A5;
const uint8_t TCS34725_I2C_ADDR    = 0x29;
const uint8_t TCS34725_COMMAND_BIT = 0x80;
const uint8_t TCS34725_AUTOINC_BIT = 0x20;
const uint8_t TCS34725_REG_ENABLE  = 0x00;
const uint8_t TCS34725_REG_ATIME   = 0x01;
const uint8_t TCS34725_REG_CONTROL = 0x0F;
const uint8_t TCS34725_REG_CDATAL  = 0x14;
const uint8_t TCS34725_REG_RDATAL  = 0x16;
const uint8_t TCS34725_REG_GDATAL  = 0x18;
const uint8_t TCS34725_REG_BDATAL  = 0x1A;

const uint8_t TCS34725_ENABLE_PON  = 0x01;
const uint8_t TCS34725_ENABLE_AEN  = 0x02;
const uint8_t TCS34725_ATIME_50MS  = 0xEB;
const uint8_t TCS34725_GAIN_4X     = 0x01;

const uint16_t COLOR_MIN_CLEAR     = 50;
const int COLOR_MIN_SPREAD         = 6;

// --- Color Classification Types ---
enum ObjectColor {
  COLOR_NONE = 0,
  COLOR_BLUE,
  COLOR_GREEN,
  COLOR_RED
};

struct ColorReading {
  uint16_t rawClear;
  uint16_t rawRed;
  uint16_t rawGreen;
  uint16_t rawBlue;
  int red;
  int green;
  int blue;
};

// --- Servo Objects & State ---
Servo servos[NUM_SERVOS];
int currentBase = STAND_POSE[0];

// Explicit prototypes for functions using custom types
ColorReading readColors();
ColorReading readColorsAveraged(uint8_t sampleCount);
ObjectColor detectColor(const ColorReading& reading);
void printColorReading(const ColorReading& reading);


// ============================================================================
// 2. ULTRASONIC SENSOR FUNCTIONS (HC-SR04)
// ============================================================================

void initUltrasonic() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
}

float readDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long durationUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (durationUs == 0) {
    return 999.0f;
  }
  return (durationUs * 0.034f) / 2.0f;
}

float readStableDistanceCm() {
  float a = readDistanceCm();
  delay(15);
  float b = readDistanceCm();
  delay(15);
  float c = readDistanceCm();

  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}


// ============================================================================
// 3. COLOR SENSOR FUNCTIONS (TCS34725 I2C)
// ============================================================================

void writeColorRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TCS34725_I2C_ADDR);
  Wire.write(static_cast<uint8_t>(TCS34725_COMMAND_BIT | reg));
  Wire.write(value);
  Wire.endTransmission();
}

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

void initColorSensor() {
  Wire.begin();
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000, true);
#endif

  writeColorRegister(TCS34725_REG_ATIME, TCS34725_ATIME_50MS);
  writeColorRegister(TCS34725_REG_CONTROL, TCS34725_GAIN_4X);

  writeColorRegister(TCS34725_REG_ENABLE, TCS34725_ENABLE_PON);
  delay(3);
  writeColorRegister(TCS34725_REG_ENABLE, static_cast<uint8_t>(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));

  delay(60);
}

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

  delay(55);
  return reading;
}

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

ObjectColor detectColor(const ColorReading& reading) {
  if (reading.rawClear < COLOR_MIN_CLEAR) {
    return COLOR_NONE;
  }

  int maxCh = max(reading.red, max(reading.green, reading.blue));
  int minCh = min(reading.red, min(reading.green, reading.blue));
  if ((maxCh - minCh) < COLOR_MIN_SPREAD) {
    return COLOR_NONE;
  }

  if (reading.blue >= reading.red && reading.blue >= reading.green) {
    return COLOR_BLUE;
  }
  if (reading.green >= reading.red && reading.green >= reading.blue) {
    return COLOR_GREEN;
  }
  return COLOR_RED;
}

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
// 4. 6-SERVO MOTION & PICK-AND-PLACE SEQUENCE
// ============================================================================

/**
 * @brief Smoothly rotates the Base Servo (Pin 9) 1 degree at a time to targetAngle.
 */
void moveBaseSmooth(int targetAngle) {
  targetAngle = constrain(targetAngle, 0, 180);
  if (currentBase < targetAngle) {
    for (int a = currentBase; a <= targetAngle; a++) {
      servos[0].write(a);
      delay(stepDelayMs);
    }
  } else if (currentBase > targetAngle) {
    for (int a = currentBase; a >= targetAngle; a--) {
      servos[0].write(a);
      delay(stepDelayMs);
    }
  }
  currentBase = targetAngle;
}

/**
 * @brief Ensures upper arm joints (Servos 2..5) remain at the verified STAND_POSE angles.
 */
void holdStandArmPose() {
  for (uint8_t i = 1; i < 5; i++) {
    servos[i].write(STAND_POSE[i]);
  }
}

/**
 * @brief Runs a smooth Pick-and-Place cycle while keeping Servos 2..5 at STAND_POSE:
 *        Pickup (150 deg) -> Target Bin (15 / 60 / 105 deg) -> Home (60 deg).
 */
void executePickAndPlace(int targetBinBaseAngle, const char* colorName) {
  Serial.print(">>> Sorting ");
  Serial.print(colorName);
  Serial.print(" object -> Bin Base Angle = ");
  Serial.println(targetBinBaseAngle);

  holdStandArmPose();

  // 1. Rotate Base smoothly to Side Pickup Container (150 deg) and open Gripper
  servos[5].write(GRIP_FULL_OPEN);
  moveBaseSmooth(PICKUP_BASE_ANGLE);
  delay(400);

  // 2. Close Gripper (Pin 3) to grab the object from the side pickup container
  servos[5].write(GRIP_FULL_CLOSE);
  delay(400);

  // 3. Rotate Base smoothly to the designated Color Bin (15, 60, or 105 deg)
  moveBaseSmooth(targetBinBaseAngle);
  delay(300);

  // 4. Open Gripper (Pin 3) to release the object into the bin
  servos[5].write(GRIP_FULL_OPEN);
  delay(400);

  // 5. Return Base smoothly to Home Pose (60 deg) and set Gripper to standby
  moveBaseSmooth(STAND_POSE[0]);
  servos[5].write(GRIP_STANDBY);
  delay(200);

  Serial.println(">>> Cycle complete! Returned to Home Pose (60 deg).");
}


// ============================================================================
// 5. SERIAL MONITOR TEST COMMANDS & MAIN ARDUINO PROGRAM
// ============================================================================

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'g' || c == 'G') {
      executePickAndPlace(GREEN_BIN_BASE_ANGLE, "GREEN (Manual Test)");
    } else if (c == 'b' || c == 'B') {
      executePickAndPlace(BLUE_BIN_BASE_ANGLE, "BLUE (Manual Test)");
    } else if (c == 'r' || c == 'R') {
      executePickAndPlace(RED_BIN_BASE_ANGLE, "RED (Manual Test)");
    } else if (c == 'h' || c == 'H') {
      Serial.println(">>> Returning smoothly to Home Pose (60 deg)...");
      holdStandArmPose();
      moveBaseSmooth(STAND_POSE[0]);
      servos[5].write(GRIP_STANDBY);
    } else if (c == 's' || c == 'S') {
      isRunning = !isRunning;
      Serial.println(isRunning ? ">>> Auto Sorting: ON" : ">>> Auto Sorting: PAUSED");
    } else if (c == 'f' || c == 'F') {
      stepDelayMs = 10;
      Serial.println(">>> Speed: FAST (10 ms/deg)");
    } else if (c == 'n' || c == 'N') {
      stepDelayMs = 20;
      Serial.println(">>> Speed: NORMAL (20 ms/deg)");
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Pinapatayo ang 6-Servo Arm (STAND_POSE)...");

  initUltrasonic();
  initColorSensor();

  // Paisa-isang i-attach ang lahat ng 6 servos (800 ms pagitan iwas brownout)
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].write(STAND_POSE[i]);
    servos[i].attach(servoPins[i]);
    delay(800);
  }
  currentBase = STAND_POSE[0];

  Serial.println("Ready! Angles: Green=15, Blue=60, Red=105, Pickup=150");
  Serial.println("Commands: 'g'=Green(15), 'b'=Blue(60), 'r'=Red(105), 'h'=Home(60), 's'=Pause/Resume, 'f'=Fast, 'n'=Normal");
}

void loop() {
  handleSerialCommands();

  if (!isRunning) {
    delay(50);
    return;
  }

  // 1. Scan HC-SR04 ultrasonic distance (median of 3 readings)
  float distanceCm = readStableDistanceCm();
  if (distanceCm > DETECTION_DISTANCE_CM) {
    delay(50);
    return;
  }

  // 2. Wait briefly for object to settle, then re-confirm distance
  delay(SETTLE_DELAY_MS);
  distanceCm = readStableDistanceCm();
  if (distanceCm > DETECTION_DISTANCE_CM) {
    return;
  }

  // 3. Analyze object color with TCS34725 and sort into designated bin
  ColorReading rgb = readColorsAveraged(3);
  printColorReading(rgb);

  switch (detectColor(rgb)) {
    case COLOR_GREEN:
      executePickAndPlace(GREEN_BIN_BASE_ANGLE, "GREEN");
      break;

    case COLOR_BLUE:
      executePickAndPlace(BLUE_BIN_BASE_ANGLE, "BLUE");
      break;

    case COLOR_RED:
      executePickAndPlace(RED_BIN_BASE_ANGLE, "RED");
      break;

    case COLOR_NONE:
    default:
      Serial.println(">>> Color not recognized (neutral/dark) - waiting...");
      delay(200);
      break;
  }
}
