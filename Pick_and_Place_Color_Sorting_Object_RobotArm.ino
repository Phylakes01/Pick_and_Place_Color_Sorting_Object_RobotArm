/**
 * @file Pick_and_Place_Color_Sorting_Object_RobotArm.ino
 * @brief 6-Servo Pick-and-Place Color-Sorting Robotic Arm controller for Arduino UNO.
 *
 * Servo Pin Mapping:
 * - Pin 9  (servos[0]): Base Rotation
 * - Pin 5  (servos[1]): Shoulder
 * - Pin 10 (servos[2]): Elbow
 * - Pin 6  (servos[3]): Wrist Pitch
 * - Pin 11 (servos[4]): Wrist Roll
 * - Pin 3  (servos[5]): Gripper Claw (35 = open, 180 = closed)
 *
 * Calibrated Station Poses { Base, Shoulder, Elbow, WristPitch, WristRoll, Gripper }:
 * - PICKUP_POSE       : { 145, 145, 0, 100, 35, 180 }  (Side Pickup Container)
 * - LEFT_BUCKET_POSE  : { 100, 125, 0,  55, 35, 180 }  (Left Bucket  -> Green)
 * - MID_BUCKET_POSE   : {  55, 145, 0, 100, 35, 180 }  (Mid Bucket   -> Blue)
 * - RIGHT_BUCKET_POSE : {  15, 135, 0,  70, 20, 180 }  (Right Bucket -> Red)
 *
 * Serial Monitor Commands (9600 baud):
 * - 'm' or 'b' : Test Pick & Place -> Mid Bucket / Blue (Base 55 deg)
 * - 'l' or 'g' : Test Pick & Place -> Left Bucket / Green (Base 100 deg)
 * - 'r'        : Test Pick & Place -> Right Bucket / Red (Base 15 deg)
 * - 'p' or 'h' : Return smoothly to Pickup / Base Pose (Base 145 deg, Gripper Open)
 * - 's'        : Pause / Resume automatic ultrasonic + color sorting
 * - 'f'        : Fast servo speed (10 ms per degree)
 * - 'n'        : Normal servo speed (20 ms per degree)
 */

#include <Wire.h>
#include <Servo.h>

// ============================================================================
// 1. HARDWARE PINS, CALIBRATED POSES, & SENSOR CONSTANTS
// ============================================================================

// --- 6 Robotic Arm Servo Pins ---
const uint8_t NUM_SERVOS = 6;
const uint8_t servoPins[NUM_SERVOS] = {
  9,   // [0] Base Rotation (Pin 9)
  5,   // [1] Shoulder (Pin 5)
  10,  // [2] Elbow (Pin 10)
  6,   // [3] Wrist Pitch (Pin 6)
  11,  // [4] Wrist Roll (Pin 11)
  3    // [5] Gripper Claw (Pin 3)
};

// --- Exact Calibrated 6-Servo Poses: { Base, Shoulder, Elbow, WristPitch, WristRoll, Gripper } ---
const int PICKUP_POSE[NUM_SERVOS]       = { 145, 145, 0, 100, 35, 180 };  // Side Pickup Container
const int MID_BUCKET_POSE[NUM_SERVOS]   = {  55, 145, 0, 100, 35, 180 };  // Mid Bucket  (Blue)
const int LEFT_BUCKET_POSE[NUM_SERVOS]  = { 100, 125, 0,  55, 35, 180 };  // Left Bucket (Green)
const int RIGHT_BUCKET_POSE[NUM_SERVOS] = {  15, 135, 0,  70, 20, 180 };  // Right Bucket (Red)

// --- High-Clearance Travel Upper-Arm Posture (used while Base rotates between stations) ---
const int CLEARANCE_SHOULDER    = 145;
const int CLEARANCE_ELBOW       = 0;
const int CLEARANCE_WRIST_PITCH = 100;
const int CLEARANCE_WRIST_ROLL  = 35;

// --- Gripper Angles (Pin 3) ---
const int GRIP_FULL_OPEN  = 35;   // Open claw to release or prepare to grab
const int GRIP_FULL_CLOSE = 180;  // Closed claw holding object (matches calibrated poses)
const int GRIP_STANDBY    = 35;   // Open at pickup standby so arm is ready to grab

// --- Motion Speed & Auto-Sort Toggle ---
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

// --- Servo Objects & Live Tracked Angles ---
Servo servos[NUM_SERVOS];
int currentAngles[NUM_SERVOS] = { 145, 145, 0, 100, 35, GRIP_STANDBY };

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
// 4. 6-SERVO MOTION & CALIBRATED PICK-AND-PLACE SEQUENCE
// ============================================================================

/**
 * @brief Smoothly rotates one servo (index 0..5) 1 degree per step to targetAngle.
 */
void moveJointSmooth(uint8_t idx, int targetAngle) {
  if (idx >= NUM_SERVOS) {
    return;
  }

  targetAngle = constrain(targetAngle, 0, 180);
  while (currentAngles[idx] != targetAngle) {
    if (currentAngles[idx] < targetAngle) {
      currentAngles[idx]++;
    } else {
      currentAngles[idx]--;
    }
    servos[idx].write(currentAngles[idx]);
    delay(stepDelayMs);
  }
}

/**
 * @brief Smoothly moves upper arm joints (Shoulder, Elbow, WristPitch, WristRoll)
 *        1 degree per tick while keeping Base and Gripper unchanged.
 */
void moveUpperArmSmooth(int targetS2, int targetS3, int targetS4, int targetS5) {
  int targets[4] = {
    constrain(targetS2, 0, 180),
    constrain(targetS3, 0, 180),
    constrain(targetS4, 0, 180),
    constrain(targetS5, 0, 180)
  };

  bool moving = true;
  while (moving) {
    moving = false;
    for (uint8_t k = 0; k < 4; k++) {
      uint8_t idx = k + 1;  // servos[1..4]
      if (currentAngles[idx] < targets[k]) {
        currentAngles[idx]++;
        servos[idx].write(currentAngles[idx]);
        moving = true;
      } else if (currentAngles[idx] > targets[k]) {
        currentAngles[idx]--;
        servos[idx].write(currentAngles[idx]);
        moving = true;
      }
    }
    if (moving) {
      delay(stepDelayMs);
    }
  }
}

/**
 * @brief Smoothly moves to targetPose[0..4] safely via the high-clearance travel posture:
 *        1) Lift upper arm to clearance posture (145, 0, 100, 35)
 *        2) Rotate Base (Pin 9) to targetPose[0]
 *        3) Lower/tilt upper arm to targetPose[1..4]
 */
void moveToPoseSmooth(const int targetPose[NUM_SERVOS]) {
  moveUpperArmSmooth(CLEARANCE_SHOULDER, CLEARANCE_ELBOW, CLEARANCE_WRIST_PITCH, CLEARANCE_WRIST_ROLL);
  moveJointSmooth(0, targetPose[0]);
  moveUpperArmSmooth(targetPose[1], targetPose[2], targetPose[3], targetPose[4]);
}

/**
 * @brief Sets the gripper claw (Pin 3) and waits briefly for jaw travel.
 */
void setGripper(int angle, unsigned long settleMs = 400) {
  angle = constrain(angle, 0, 180);
  currentAngles[5] = angle;
  servos[5].write(angle);
  delay(settleMs);
}

/**
 * @brief Runs a complete calibrated Pick-and-Place cycle:
 *        Pickup Pose (145) -> Target Bucket Pose (55 / 100 / 15) -> Pickup Standby Pose (145).
 */
void executePickAndPlace(const int targetBucketPose[NUM_SERVOS], const char* bucketName) {
  Serial.print(">>> Sorting -> ");
  Serial.print(bucketName);
  Serial.print(" (Base = ");
  Serial.print(targetBucketPose[0]);
  Serial.println(" deg)");

  // 1. Ensure arm is at Pickup Pose (145, 145, 0, 100, 35) with Gripper OPEN (35)
  moveToPoseSmooth(PICKUP_POSE);
  setGripper(GRIP_FULL_OPEN, 300);

  // 2. Close Gripper (180 deg) to grab the object
  setGripper(GRIP_FULL_CLOSE, 400);

  // 3. Move smoothly to Target Bucket Pose (lift -> rotate base -> dip to bucket pose)
  moveToPoseSmooth(targetBucketPose);
  delay(300);

  // 4. Open Gripper (35 deg) to release the object into the bucket
  setGripper(GRIP_FULL_OPEN, 400);

  // 5. Retract upper arm to clearance posture, then smoothly return to Pickup Standby Pose
  moveUpperArmSmooth(CLEARANCE_SHOULDER, CLEARANCE_ELBOW, CLEARANCE_WRIST_PITCH, CLEARANCE_WRIST_ROLL);
  moveToPoseSmooth(PICKUP_POSE);
  setGripper(GRIP_STANDBY, 200);

  Serial.println(">>> Cycle complete! Returned to Pickup Standby Pose (Base 145, Gripper Open).");
}


// ============================================================================
// 5. SERIAL MONITOR TEST COMMANDS & MAIN ARDUINO PROGRAM
// ============================================================================

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'm' || c == 'M' || c == 'b' || c == 'B') {
      executePickAndPlace(MID_BUCKET_POSE, "MID BUCKET / BLUE (Manual Test)");
    } else if (c == 'l' || c == 'L' || c == 'g' || c == 'G') {
      executePickAndPlace(LEFT_BUCKET_POSE, "LEFT BUCKET / GREEN (Manual Test)");
    } else if (c == 'r' || c == 'R') {
      executePickAndPlace(RIGHT_BUCKET_POSE, "RIGHT BUCKET / RED (Manual Test)");
    } else if (c == 'p' || c == 'P' || c == 'h' || c == 'H') {
      Serial.println(">>> Returning smoothly to Pickup / Base Pose (145 deg, Gripper Open)...");
      moveToPoseSmooth(PICKUP_POSE);
      setGripper(GRIP_STANDBY, 200);
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
  Serial.println("Starting 6-Servo Arm at calibrated Pickup / Base Pose...");

  initUltrasonic();
  initColorSensor();

  // Attach all 6 servos one-by-one (800 ms gap to prevent power brownout)
  // Gripper starts OPEN (GRIP_STANDBY = 35 deg) ready to receive an object
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    int initAngle = (i == 5) ? GRIP_STANDBY : PICKUP_POSE[i];
    currentAngles[i] = initAngle;
    servos[i].write(initAngle);
    servos[i].attach(servoPins[i]);
    delay(800);
  }

  Serial.println("Ready at Pickup Pose {145, 145, 0, 100, 35, Open=35}!");
  Serial.println("Commands: 'm'/'b'=Mid/Blue(55), 'l'/'g'=Left/Green(100), 'r'=Right/Red(15), 'p'/'h'=Pickup(145), 's'=Pause, 'f'=Fast, 'n'=Normal");
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

  // 2. Wait briefly for object to settle in pickup zone, then re-confirm distance
  delay(SETTLE_DELAY_MS);
  distanceCm = readStableDistanceCm();
  if (distanceCm > DETECTION_DISTANCE_CM) {
    return;
  }

  // 3. Analyze object color with TCS34725 and sort into calibrated bucket pose
  ColorReading rgb = readColorsAveraged(3);
  printColorReading(rgb);

  switch (detectColor(rgb)) {
    case COLOR_BLUE:
      executePickAndPlace(MID_BUCKET_POSE, "BLUE -> MID BUCKET");
      break;

    case COLOR_GREEN:
      executePickAndPlace(LEFT_BUCKET_POSE, "GREEN -> LEFT BUCKET");
      break;

    case COLOR_RED:
      executePickAndPlace(RIGHT_BUCKET_POSE, "RED -> RIGHT BUCKET");
      break;

    case COLOR_NONE:
    default:
      Serial.println(">>> Color not recognized (neutral/dark) - waiting...");
      delay(200);
      break;
  }
}
