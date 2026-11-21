/**
 * @file Pick_and_Place_Color_Sorting_Object_RobotArm.ino
 * @brief 6-Servo Pick-and-Place Color-Sorting Robotic Arm controller for Arduino UNO.
 *
 * Servo Pin Mapping (MG996R PWM Limits: 600 - 2400 us):
 * - Pin 9  (servos[0]): Base Rotation
 * - Pin 5  (servos[1]): Shoulder
 * - Pin 10 (servos[2]): Elbow
 * - Pin 6  (servos[3]): Wrist Pitch
 * - Pin 11 (servos[4]): Wrist Roll
 * - Pin 3  (servos[5]): Gripper Claw (20 = open, 180 = closed)
 *
 * Exact Calibrated 6-Servo Poses { Base, Shoulder, Elbow, WristPitch, WristRoll, Gripper }:
 * - BASE_POSE         : { 160, 175, 25, 65, 55, 180 }  (Base Angle / Elevated Standby Hover)
 * - PICKUP_POSE       : { 140, 120, 40, 45, 40,  20 }  (Pickup Location Pose)
 * - MID_BUCKET_POSE   : {  60, 150, 25, 60, 35, 180 }  (Mid Bucket   -> White Only)
 * - LEFT_BUCKET_POSE  : { 100, 150, 30, 35, 35, 180 }  (Left Bucket  -> Black)
 * - RIGHT_BUCKET_POSE : {  15, 135,  5, 60, 35, 180 }  (Right Bucket -> All Colors: Red, Green, Blue, etc.)
 *
 * Adafruit_TCS34725 Color Sensor (50ms Integration + 4X Gain, 1-Sample 50ms Fast Read,
 * IR Offset Removal, Gentle Warm-LED Compensation, and HSV Hue/Sat Telemetry):
 * - Right Bucket (Base  15 deg) : ALL COLORS (RED, GREEN, BLUE, COLOR)
 * - Left Bucket  (Base 100 deg) : BLACK
 * - Mid Bucket   (Base  60 deg) : WHITE ONLY
 * - Ignored                     : WALANG OBJECT, NONE (unmatched)
 *
 * Serial Monitor Commands (9600 baud):
 * - '1'..'6'         : Select live adjuster servo 1..6 (Base..Gripper Claw)
 * - '+' / '-'        : Move selected servo +5 / -5 deg smoothly (2 ms/deg)
 * - 's1'..'s6 <ang>' : Move servo 1..6 directly to <ang> deg (e.g. "s2 105")
 * - 'c'              : Close gripper (if open) or read & print TCS34725 color (if closed)
 * - 'v'              : Read & print TCS34725 color sample anytime
 * - 'o' / 'x'        : Open (20 deg) / Close (180 deg) gripper smoothly
 * - 't'              : Toggle auto gripper open/close loop (500 ms non-blocking)
 * - 'p'              : Print CURRENT ANGLES summary block
 * - 'p1'..'p5'       : Move smoothly to BASE / PICKUP / MID / LEFT / RIGHT calibrated pose
 * - 'm'              : Test Pick & Place -> Mid Bucket / White Only (Base 60 deg)
 * - 'l' or 'k'       : Test Pick & Place -> Left Bucket / Black (Base 100 deg)
 * - 'r', 'g', or 'b' : Test Pick & Place -> Right Bucket / All Colors (Base 15 deg)
 * - 'd'              : Test dipping down to PICKUP_POSE (Base 140 deg) and back to BASE_POSE
 * - 'h'              : Return smoothly to BASE_POSE (Base 160 deg, Gripper 180 deg)
 * - 'w'              : Calibrate RGB white-balance gains & WHITE C reference against a WHITE object
 * - 'z'              : Calibrate BLACK threshold against a BLACK object in front of sensor
 * - 's' (alone)/'a'  : Pause / Resume automatic ultrasonic + color sorting
 * - 'f' / 'n'        : Servo speed Max Turbo (1 ms/deg) / Max (2 ms/deg)
 * - '?'              : Print organized Help Menu & current angles
 */

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Servo.h>
#include <avr/pgmspace.h>

// ============================================================================
// 1. HARDWARE PINS, CALIBRATED POSES, & SENSOR CONSTANTS
// ============================================================================

// --- 6 Robotic Arm Servo Pins & MG996R PWM Limits ---
const uint8_t NUM_SERVOS = 6;
const int MG996R_MIN_US = 600;
const int MG996R_MAX_US = 2400;

const uint8_t servoPins[NUM_SERVOS] = {
  9,   // [0] Base Rotation (Pin 9)
  5,   // [1] Shoulder (Pin 5)
  10,  // [2] Elbow (Pin 10)
  6,   // [3] Wrist Pitch (Pin 6)
  11,  // [4] Wrist Roll (Pin 11)
  3    // [5] Gripper Claw (Pin 3)
};

const char* const servoNames[NUM_SERVOS] = {
  "1: Base (Pin 9)",
  "2: Shoulder (Pin 5)",
  "3: Elbow (Pin 10)",
  "4: Wrist Pitch (Pin 6)",
  "5: Wrist Roll (Pin 11)",
  "6: Gripper Claw (Pin 3)"
};

// --- Exact Calibrated 6-Servo Poses: { Base, Shoulder, Elbow, WristPitch, WristRoll, Gripper } ---
const int BASE_POSE[NUM_SERVOS] = { 160, 175, 25, 65, 55, 180 };         // Base Angle / Elevated Standby Hover
const int PICKUP_POSE[NUM_SERVOS] = { 140, 120, 40, 45, 40, 20 };        // Pickup Location Pose
const int MID_BUCKET_POSE[NUM_SERVOS] = { 60, 150, 25, 60, 35, 180 };    // Mid Bucket   -> White Only
const int LEFT_BUCKET_POSE[NUM_SERVOS] = { 100, 150, 30, 35, 35, 180 };  // Left Bucket  -> Black
const int RIGHT_BUCKET_POSE[NUM_SERVOS] = { 15, 135, 5, 60, 35, 180 };   // Right Bucket -> All Colors (Red, Green, Blue, etc.)

// --- High-Clearance Travel Upper-Arm Posture (used while Base rotates between stations) ---
const int CLEARANCE_SHOULDER = 175;
const int CLEARANCE_ELBOW = 25;
const int CLEARANCE_WRIST_PITCH = 65;
const int CLEARANCE_WRIST_ROLL = 55;

// --- Gripper Angles (Pin 3) & Live Angle Adjuster State ---
const int GRIP_FULL_OPEN = 20;    // Open claw for picking and releasing
const int GRIP_FULL_CLOSE = 180;  // Closed claw (matches BASE_POSE[5] and calibrated poses)

const int GRIP_OPEN_ANGLE = GRIP_FULL_OPEN;
const int GRIP_CLOSE_ANGLE = GRIP_FULL_CLOSE;
const unsigned long GRIP_INTERVAL_MS = 500UL;

uint8_t selectedServo = 0;
bool gripperLoopRunning = false;
bool gripperClosed = true;
unsigned long lastGripStepTime = 0;

// --- Motion Speed & Auto-Sort Toggle ---
int stepDelayMs = 2;    // 2 ms per degree (MG996R max speed default), 1 ms per degree (max turbo)
bool isRunning = true;  // Automatic ultrasonic + color sorting active

// --- HC-SR04 Ultrasonic Sensor Pins & Settings ---
const int ULTRASONIC_TRIG_PIN = 8;  // HC-SR04 Trig -> Pin 8
const int ULTRASONIC_ECHO_PIN = 7;  // HC-SR04 Echo -> Pin 7

// Detection Range (in cm)
// Bahagyang pinalawak pero mas mababa pa rin sa 7.1 cm opposite wall
const float MIN_RANGE = 1.5f;
const float MAX_RANGE = 6.2f;

// Stability filter (ilang sunod-sunod na detect bago mag-trigger)
const int REQUIRED_HITS = 2;
int hitCount = 0;

const unsigned long ECHO_TIMEOUT_US = 15000UL;   // 15 ms echo timeout
const unsigned long PRE_READ_SETTLE_MS = 40UL;   // Fast settle delay after hits before 1st color read
const unsigned long SETTLE_DELAY_MS = 25UL;      // Fast settle delay before confirmation scan

// --- TCS34725 Color Sensor (Hardware I2C: SDA=A4, SCL=A5, 50ms Integration + 4X Gain) ---
const uint8_t COLOR_SDA_PIN = A4;
const uint8_t COLOR_SCL_PIN = A5;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool colorSensorReady = false;

// I-adjust ang mga value na ito base sa lumalabas na C= sa Serial Monitor
// Tuned for actual TCS34725 readings (steady WHITE C ~ 1766, BLACK/dark C <= ~850)
const int NO_OBJECT_MAX_C = 42;
int whiteRefC = 1766;       // Reference Clear value for WHITE object (updated by 'w')
int blackRefC = -1;         // Optional measured Clear value for BLACK object (updated by 'z', -1 = auto)
int blackSolidMaxC = 40;    // Pitch-black ceiling (only zero-signal pitch-dark bypasses saturation)
int blackMaxC = 850;        // Max C for achromatic BLACK (~48% of whiteRefC)
int whiteMinC = 1020;       // Min C for WHITE (~58% of whiteRefC)
int whiteBrightMinC = 1320; // Bright White threshold (~75% of whiteRefC)
const int WHITE_MAX_SAT_PCT = 26;
const int WHITE_BRIGHT_MAX_SAT_PCT = 30;
const int WHITE_MAX_TOPGAP_PCT = 17;
const int DARK_COLOR_MIN_SAT_PCT = 32;
const int COLOR_MIN_SAT_PCT = 22;

// Calibrated post-IR white-balance gains for this TCS34725 (measured at White C ~ 1766)
float rGain = 0.89f;
float gGain = 0.84f;
float bGain = 1.45f;
float whiteChRef = 520.0f;
float blackChRef = 0.0f;

int Red = 0;
int Green = 0;
int Blue = 0;
int LastClear = 0;
int LastHue = 0;
int LastSat = 0;

// --- Color Classification Types ---
enum ObjectColor {
  COLOR_NO_OBJECT = 0,  // WALANG OBJECT
  COLOR_NONE,           // NONE
  COLOR_WHITE,          // WHITE      -> Mid Bucket (Base 60 deg)
  COLOR_BLACK,          // BLACK      -> Left Bucket (Base 100 deg)
  COLOR_RED,            // RED        -> Right Bucket (Base 15 deg)
  COLOR_GREEN,          // GREEN      -> Right Bucket (Base 15 deg)
  COLOR_BLUE,           // BLUE       -> Right Bucket (Base 15 deg)
  COLOR_OTHER           // ALL COLORS -> Right Bucket (Base 15 deg)
};

// --- PROGMEM Shade & Recommended TCS34725 RGB Range Category Tables ---
struct ShadeRef {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  char name[20];
};

struct RgbRangeCategory {
  uint8_t rMin, rMax;
  uint8_t gMin, gMax;
  uint8_t bMin, bMax;
  char name[14];
};

const uint8_t NUM_WHITE_SHADES = 30;
const ShadeRef WHITE_SHADES[NUM_WHITE_SHADES] PROGMEM = {
  { 255, 255, 255, "Pure White" },
  { 254, 254, 254, "Bright White" },
  { 252, 252, 252, "Paper White" },
  { 255, 250, 250, "Snow White" },
  { 255, 253, 245, "Warm White" },
  { 248, 250, 255, "Cool White" },
  { 250, 250, 247, "Natural White" },
  { 253, 253, 253, "Soft White" },
  { 255, 255, 240, "Ivory" },
  { 255, 254, 238, "Ivory White" },
  { 250, 235, 215, "Antique White" },
  { 255, 250, 240, "Floral White" },
  { 248, 248, 255, "Ghost White" },
  { 245, 245, 245, "White Smoke" },
  { 255, 254, 249, "Milk White" },
  { 248, 246, 240, "Pearl White" },
  { 255, 253, 208, "Cream White" },
  { 250, 249, 246, "Off-White" },
  { 250, 240, 230, "Linen White" },
  { 240, 234, 216, "Eggshell" },
  { 242, 240, 230, "Alabaster" },
  { 244, 244, 244, "Chalk White" },
  { 229, 228, 226, "Platinum White" },
  { 240, 240, 240, "Light Gray White" },
  { 245, 245, 220, "Beige White" },
  { 240, 255, 255, "Azure White" },
  { 245, 255, 250, "Mint White" },
  { 240, 248, 255, "Blue-Tinted White" },
  { 255, 251, 234, "Yellow-Tinted White" },
  { 255, 252, 242, "Warm Paper White" }
};

const uint8_t NUM_BLACK_SHADES = 24;
const ShadeRef BLACK_SHADES[NUM_BLACK_SHADES] PROGMEM = {
  { 0, 0, 0, "Pure Black" },
  { 1, 1, 1, "Pitch Black" },
  { 5, 5, 5, "Absolute Black" },
  { 10, 10, 10, "Jet Black" },
  { 16, 16, 16, "Deep Black" },
  { 12, 12, 12, "Rich Black" },
  { 25, 25, 25, "Soft Black" },
  { 28, 28, 28, "Matte Black" },
  { 34, 34, 34, "Coal Black" },
  { 27, 27, 27, "Raven Black" },
  { 53, 56, 57, "Onyx Black" },
  { 54, 69, 79, "Charcoal Black" },
  { 56, 56, 56, "Graphite Black" },
  { 41, 41, 41, "Ebony" },
  { 16, 12, 8, "Smoky Black" },
  { 59, 60, 54, "Black Olive" },
  { 43, 43, 43, "Black Gray" },
  { 51, 51, 51, "Dark Charcoal" },
  { 70, 70, 70, "Grayish Black" },
  { 33, 26, 22, "Brownish Black" },
  { 16, 24, 32, "Bluish Black" },
  { 16, 20, 14, "Greenish Black" },
  { 26, 20, 16, "Warm Black" },
  { 17, 24, 32, "Cool Black" }
};

const uint8_t NUM_RGB_CATEGORIES = 15;
const RgbRangeCategory RGB_RANGE_CATEGORIES[NUM_RGB_CATEGORIES] PROGMEM = {
  { 250, 255, 250, 255, 250, 255, "Pure White" },
  { 240, 255, 240, 255, 240, 255, "Bright White" },
  { 235, 255, 235, 255, 235, 255, "Paper White" },
  { 240, 255, 235, 255, 235, 255, "Snow White" },
  { 230, 255, 235, 255, 245, 255, "Cool White" },
  { 240, 255, 240, 255, 225, 250, "Warm White" },
  { 230, 255, 230, 255, 225, 250, "Natural White" },
  { 240, 255, 240, 255, 210, 245, "Ivory" },
  { 240, 255, 235, 255, 190, 230, "Cream White" },
  { 225, 250, 225, 250, 220, 250, "Off-White" },
  { 180, 220, 180, 220, 180, 220, "Light Gray" },
  { 100, 150, 100, 150, 100, 150, "Gray" },
  { 50, 90, 50, 90, 50, 90, "Dark Gray" },
  { 30, 70, 30, 80, 30, 90, "Charcoal" },
  { 0, 30, 0, 30, 0, 30, "Black" }
};

// --- Servo Objects & Live Tracked Angles ---
Servo servos[NUM_SERVOS];
int currentAngles[NUM_SERVOS] = { 160, 175, 25, 65, 55, 180 };

// Explicit prototypes for color sensor & live adjuster functions
void initColorSensor();
bool ensureColorSensorReady();
void sampleColorAverages(float& r, float& g, float& b, float& c);
float computeIrOffset(float r, float g, float b, float c);
void printHexByte(int val);
void printProgmemStr(const char* progmemStr);
int8_t matchRgbCategoryIndex(int r, int g, int b);
uint8_t findNearestShadeIndex(int r, int g, int b, const ShadeRef* table, uint8_t count);
void printColorTelemetryPrefix(const __FlashStringHelper* label);
void printColorTelemetry(const __FlashStringHelper* label);
void printAchromaticTelemetry(const __FlashStringHelper* label, int8_t catIdx, const ShadeRef* table, uint8_t count);
ObjectColor reportColorSensorOffline();
ObjectColor readAndDetectColor(bool objectDetected = false);
void updateColorThresholds();
bool sampleCalibrationReference(float& r, float& g, float& b, float& c);
void calibrateWhiteBalance();
void calibrateBlackReference();
void moveServoSmooth(uint8_t idx, int targetAngle);
void printAllAngles();
void printHelpMenu();
void toggleAutoSorting();
bool isObjectInRangeWithRetry();
uint8_t getTargetBucket(ObjectColor color);


// ============================================================================
// 2. ULTRASONIC SENSOR FUNCTIONS (HC-SR04)
// ============================================================================

void initUltrasonic() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
}

float readDistanceCM() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ECHO_TIMEOUT_US);

  if (duration == 0) {
    return -1.0f;
  }

  return (duration * 0.0343f) / 2.0f;
}

bool isObjectInRangeWithRetry() {
  float d = readDistanceCM();
  if (d >= MIN_RANGE && d <= MAX_RANGE) {
    return true;
  }
  delay(15);
  d = readDistanceCM();
  return (d >= MIN_RANGE && d <= MAX_RANGE);
}


// ============================================================================
// 3. COLOR SENSOR FUNCTIONS (Adafruit_TCS34725 I2C)
// ============================================================================

void initColorSensor() {
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_ARCH_AVR)
  Wire.setWireTimeout(25000, true);  // 25 ms I2C timeout with auto-reset on bus lockup
#endif
  if (!tcs.begin()) {
    colorSensorReady = false;
    isRunning = false;  // Pause auto-sorting while TCS34725 is disconnected
    Serial.println(F("WARNING: Hindi mahanap ang TCS34725! I-check ang wiring (SDA=A4, SCL=A5)."));
    Serial.println(F(">>> Running in DIAGNOSTIC MODE (Auto-Sorting PAUSED; Serial & Servo controls active)."));
    Serial.println(F(">>> Reconnect TCS34725 and press 'v', 'c', 'w', or 'a' to retry sensor init."));
    return;
  }
  colorSensorReady = true;
  tcs.setInterrupt(false);  // Siguraduhing naka-ON ang onboard white LED
  delay(100);
  Serial.println(F(">>> TCS34725 Color Sensor: OK (SDA=A4, SCL=A5)"));
}

/**
 * @brief Ensures the TCS34725 sensor is online, attempting re-initialization if needed.
 */
bool ensureColorSensorReady() {
  if (colorSensorReady) {
    return true;
  }
  if (tcs.begin()) {
#if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_ARCH_AVR)
    Wire.clearWireTimeoutFlag();
    Wire.setWireTimeout(25000, true);
#endif
    colorSensorReady = true;
    tcs.setInterrupt(false);
    delay(100);
    Serial.println(F(">>> TCS34725 Color Sensor reconnected and initialized!"));
    return true;
  }
  return false;
}

void sampleColorAverages(float& r, float& g, float& b, float& c) {
  if (!ensureColorSensorReady()) {
    r = 0.0f;
    g = 0.0f;
    b = 0.0f;
    c = 0.0f;
    return;
  }

  uint16_t r_raw = 0, g_raw = 0, b_raw = 0, c_raw = 0;

#if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_ARCH_AVR)
  Wire.clearWireTimeoutFlag();
#endif

  // Single 50 ms hardware-integrated reading for instant response (Black & Colors)
  tcs.getRawData(&r_raw, &g_raw, &b_raw, &c_raw);

#if defined(WIRE_HAS_TIMEOUT) || defined(ARDUINO_ARCH_AVR)
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    colorSensorReady = false;
    r = 0.0f;
    g = 0.0f;
    b = 0.0f;
    c = 0.0f;
    return;
  }
#endif

  r = (float)r_raw;
  g = (float)g_raw;
  b = (float)b_raw;
  c = (float)c_raw;
}

float computeIrOffset(float r, float g, float b, float c) {
  return (r + g + b > c) ? (r + g + b - c) / 2.0f : 0.0f;
}

void printHexByte(int val) {
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  uint8_t b = (uint8_t)constrain(val, 0, 255);
  Serial.print(HEX_DIGITS[(b >> 4) & 0x0F]);
  Serial.print(HEX_DIGITS[b & 0x0F]);
}

void printProgmemStr(const char* progmemStr) {
  char ch;
  while ((ch = (char)pgm_read_byte(progmemStr++)) != '\0') {
    Serial.print(ch);
  }
}

int8_t matchRgbCategoryIndex(int r, int g, int b) {
  RgbRangeCategory cat;
  for (uint8_t i = 0; i < NUM_RGB_CATEGORIES; i++) {
    memcpy_P(&cat, &RGB_RANGE_CATEGORIES[i], sizeof(cat));
    if (r >= cat.rMin && r <= cat.rMax && g >= cat.gMin && g <= cat.gMax && b >= cat.bMin && b <= cat.bMax) {
      return (int8_t)i;
    }
  }
  return -1;
}

uint8_t findNearestShadeIndex(int r, int g, int b, const ShadeRef* table, uint8_t count) {
  uint8_t bestIdx = 0;
  long bestDistSq = 0x7FFFFFFFL;
  ShadeRef sh;
  for (uint8_t i = 0; i < count; i++) {
    memcpy_P(&sh, &table[i], sizeof(sh));
    long dr = (long)r - sh.r;
    long dg = (long)g - sh.g;
    long db = (long)b - sh.b;
    long distSq = dr * dr + dg * dg + db * db;
    if (distSq < bestDistSq) {
      bestDistSq = distSq;
      bestIdx = i;
    }
  }
  return bestIdx;
}

void printColorTelemetryPrefix(const __FlashStringHelper* label) {
  if (Red == 255 && Green == 255 && Blue == 255) {
    Serial.print(F("rgb(255, 255, 255) (hex #FFFFFF)"));
  } else if (Red == 0 && Green == 0 && Blue == 0) {
    Serial.print(F("rgb(0, 0, 0) (#000000)"));
  } else {
    Serial.print(F("rgb("));
    Serial.print(Red);
    Serial.print(F(", "));
    Serial.print(Green);
    Serial.print(F(", "));
    Serial.print(Blue);
    Serial.print(F(") (hex #"));
    printHexByte(Red);
    printHexByte(Green);
    printHexByte(Blue);
    Serial.print(F(")"));
  }

  Serial.print(F(" | Hue: "));
  Serial.print(LastHue);
  Serial.print(F(" deg | Sat: "));
  Serial.print(LastSat);
  Serial.print(F("% | C: "));
  Serial.print(LastClear);
  Serial.print(F(" -> "));
  Serial.print(label);
}

void printColorTelemetry(const __FlashStringHelper* label) {
  printColorTelemetryPrefix(label);
  Serial.println();
}

void printAchromaticTelemetry(const __FlashStringHelper* label, int8_t catIdx, const ShadeRef* table, uint8_t count) {
  printColorTelemetryPrefix(label);
  Serial.print(F(" | Cat: "));
  if (catIdx >= 0 && catIdx < (int8_t)NUM_RGB_CATEGORIES) {
    printProgmemStr(RGB_RANGE_CATEGORIES[catIdx].name);
  } else {
    Serial.print(F("Custom"));
  }
  Serial.print(F(" | Shade: "));
  uint8_t shadeIdx = findNearestShadeIndex(Red, Green, Blue, table, count);
  printProgmemStr(table[shadeIdx].name);
  Serial.println();
}

ObjectColor reportColorSensorOffline() {
  Red = 0;
  Green = 0;
  Blue = 0;
  LastClear = 0;
  LastHue = 0;
  LastSat = 0;
  Serial.println(F("[TCS34725 OFFLINE] Check SDA=A4, SCL=A5 wiring -> WALANG OBJECT"));
  return COLOR_NO_OBJECT;
}

ObjectColor readAndDetectColor(bool objectDetected) {
  if (!ensureColorSensorReady()) {
    return reportColorSensorOffline();
  }

  // 1) Check ultrasonic range when objectDetected is not already confirmed
  if (!objectDetected) {
    float dist = readDistanceCM();
    objectDetected = (dist >= MIN_RANGE && dist <= MAX_RANGE);
  }

  // 2) Sample 3-reading average and compute IR offset
  float r, g, b, c;
  sampleColorAverages(r, g, b, c);
  if (!colorSensorReady) {
    return reportColorSensorOffline();
  }
  LastClear = (int)c;
  float ir = computeIrOffset(r, g, b, c);

  // 3) Remove IR offset and apply gentle warm-LED gains
  float r_bal = max(0.0f, r - ir) * rGain;
  float g_bal = max(0.0f, g - ir) * gGain;
  float b_bal = max(0.0f, b - ir) * bGain;

  // 4) Channel spread & order statistics
  float maxCh = max(r_bal, max(g_bal, b_bal));
  float minCh = min(r_bal, min(g_bal, b_bal));
  float midCh = (r_bal + g_bal + b_bal) - maxCh - minCh;
  float delta = maxCh - minCh;

  // 5) HSV Saturation (%) & top-channel dominance gap (%)
  int satPct = (maxCh > 1.0f)
                 ? constrain((int)(((delta / maxCh) * 100.0f) + 0.5f), 0, 100)
                 : 0;
  int topGapPct = (maxCh > 1.0f)
                    ? constrain((int)((((maxCh - midCh) / maxCh) * 100.0f) + 0.5f), 0, 100)
                    : 0;
  LastSat = satPct;

  // 6) Accurate HSV Hue (0..359 deg); suppress unstable Hue noise on nearly achromatic targets
  int hueDeg = 0;
  if (satPct > 2 && delta > 0.5f && maxCh > 1.0f) {
    float hueF = 0.0f;
    if (maxCh == r_bal) {
      hueF = 60.0f * ((g_bal - b_bal) / delta);
    } else if (maxCh == g_bal) {
      hueF = 60.0f * (((b_bal - r_bal) / delta) + 2.0f);
    } else {
      hueF = 60.0f * (((r_bal - g_bal) / delta) + 4.0f);
    }
    while (hueF < 0.0f) {
      hueF += 360.0f;
    }
    while (hueF >= 360.0f) {
      hueF -= 360.0f;
    }
    hueDeg = ((int)(hueF + 0.5f)) % 360;
  }
  LastHue = hueDeg;

  float darkFloor = (blackChRef > 5.0f && blackChRef < whiteChRef * 0.65f) ? (blackChRef * 0.65f) : 0.0f;
  float scale255 = 255.0f / max(1.0f, whiteChRef - darkFloor);
  int calR = constrain((int)((max(0.0f, r_bal - darkFloor) * scale255) + 0.5f), 0, 255);
  int calG = constrain((int)((max(0.0f, g_bal - darkFloor) * scale255) + 0.5f), 0, 255);
  int calB = constrain((int)((max(0.0f, b_bal - darkFloor) * scale255) + 0.5f), 0, 255);
  int calMax = max(calR, max(calG, calB));
  int calMin = min(calR, min(calG, calB));
  int calDelta = calMax - calMin;
  int8_t catIdx = matchRgbCategoryIndex(calR, calG, calB);

  // 7) Strict classification order: WALANG OBJECT -> BLACK -> WHITE -> Chromatic -> Achromatic fallback
  // 7a) WALANG OBJECT (never drop ultrasonic-confirmed dark/pitch-black objects)
  if (!objectDetected && (c < NO_OBJECT_MAX_C || c < 8.0f)) {
    Red = 0;
    Green = 0;
    Blue = 0;
    LastHue = 0;
    LastSat = 0;
    printColorTelemetry(F("WALANG OBJECT"));
    return COLOR_NO_OBJECT;
  }

  // Guard both bright AND dark saturated colors without falsely triggering on any of the 24 Black shades (calDelta <= 25)
  bool isClearlyChromatic =
    ((calDelta >= 26) && ((satPct >= 34) || (satPct >= 25 && topGapPct >= 18))) ||
    ((c >= 45.0f) && (calDelta >= 18) && (satPct >= 46) && (topGapPct >= 24));

  // 7b) BLACK (24 Black shades have calMax <= 79 and calDelta <= 25)
  bool inBlackCategory = (catIdx >= 12 && catIdx <= 14);
  bool isPitchBlack = (c < blackSolidMaxC) || (calMax <= 18);
  bool isBlackShade = isPitchBlack || (!isClearlyChromatic && ((calMax <= 55) || (inBlackCategory && satPct <= 35)));
  if (isPitchBlack || (!isClearlyChromatic && c < blackMaxC && satPct < DARK_COLOR_MIN_SAT_PCT) || (c < whiteMinC && isBlackShade)) {
    Red = calR;
    Green = calG;
    Blue = calB;
    LastHue = 0;
    printAchromaticTelemetry(F("BLACK [LEFT BUCKET]"), catIdx, BLACK_SHADES, NUM_BLACK_SHADES);
    return COLOR_BLACK;
  }

  // 7c) WHITE
  bool inWhiteCategory = (catIdx >= 0 && catIdx <= 10);
  if (!isClearlyChromatic && c >= whiteMinC && ((inWhiteCategory && satPct <= 28 && topGapPct <= 18) || (satPct <= WHITE_MAX_SAT_PCT && topGapPct <= WHITE_MAX_TOPGAP_PCT) || (c >= whiteBrightMinC && satPct <= WHITE_BRIGHT_MAX_SAT_PCT && topGapPct <= 18) || (satPct <= 22))) {
    Red = calR;
    Green = calG;
    Blue = calB;
    LastHue = 0;
    printAchromaticTelemetry(F("WHITE [MID BUCKET]"), catIdx, WHITE_SHADES, NUM_WHITE_SHADES);
    return COLOR_WHITE;
  }

  // 7d) Chromatic 0-255 RGB scaling
  float floorCh = minCh * 0.65f;
  float span = max(1.0f, maxCh - floorCh);
  Red = constrain((int)((((r_bal - floorCh) / span) * 255.0f) + 0.5f), 0, 255);
  Green = constrain((int)((((g_bal - floorCh) / span) * 255.0f) + 0.5f), 0, 255);
  Blue = constrain((int)((((b_bal - floorCh) / span) * 255.0f) + 0.5f), 0, 255);

  if (satPct >= COLOR_MIN_SAT_PCT || topGapPct >= 14) {
    if (hueDeg >= 330 || hueDeg <= 25 || (r_bal == maxCh && hueDeg <= 32 && topGapPct >= 12)) {
      printColorTelemetry(F("RED [RIGHT BUCKET]"));
      return COLOR_RED;
    } else if (hueDeg >= 70 && hueDeg <= 168) {
      printColorTelemetry(F("GREEN [RIGHT BUCKET]"));
      return COLOR_GREEN;
    } else if (hueDeg >= 170 && hueDeg <= 265) {
      printColorTelemetry(F("BLUE [RIGHT BUCKET]"));
      return COLOR_BLUE;
    } else {
      printColorTelemetry(F("COLOR [RIGHT BUCKET]"));
      return COLOR_OTHER;
    }
  }

  // 7e) Achromatic fallback (split transition band cleanly between BLACK and WHITE)
  int achromaticMidC = (blackMaxC + whiteMinC) / 2;
  Red = calR;
  Green = calG;
  Blue = calB;
  LastHue = 0;
  if (c >= achromaticMidC) {
    printAchromaticTelemetry(F("WHITE [MID BUCKET]"), catIdx, WHITE_SHADES, NUM_WHITE_SHADES);
    return COLOR_WHITE;
  } else {
    printAchromaticTelemetry(F("BLACK [LEFT BUCKET]"), catIdx, BLACK_SHADES, NUM_BLACK_SHADES);
    return COLOR_BLACK;
  }
}

void updateColorThresholds() {
  if (blackRefC > 0 && blackRefC < whiteRefC - 60) {
    int spanC = whiteRefC - blackRefC;
    blackMaxC = blackRefC + max(40, (int)(spanC * 0.35f));
    whiteMinC = blackRefC + max(80, (int)(spanC * 0.55f));
    if (blackMaxC >= whiteMinC) {
      whiteMinC = blackMaxC + 20;
    }
    blackSolidMaxC = constrain((int)(blackRefC * 0.15f), 25, 60);
  } else {
    blackSolidMaxC = constrain((int)(whiteRefC * 0.03f), 25, 55);
    blackMaxC = constrain((int)(whiteRefC * 0.48f), 55, 6000);
    whiteMinC = constrain((int)(whiteRefC * 0.58f), 70, 8000);
    if (blackMaxC >= whiteMinC) {
      whiteMinC = blackMaxC + 20;
    }
  }
  whiteBrightMinC = max(whiteMinC + 60, (int)(whiteRefC * 0.75f));
}

bool sampleCalibrationReference(float& r, float& g, float& b, float& c) {
  float sumR = 0.0f, sumG = 0.0f, sumB = 0.0f, sumC = 0.0f;
  for (int i = 0; i < 5; i++) {
    float r_s, g_s, b_s, c_s;
    sampleColorAverages(r_s, g_s, b_s, c_s);
    if (!colorSensorReady) {
      Serial.println(F(">>> Calibration aborted: TCS34725 I2C timeout during sampling."));
      return false;
    }
    sumR += r_s;
    sumG += g_s;
    sumB += b_s;
    sumC += c_s;
    if (i < 4) {
      delay(30);
    }
  }
  r = sumR / 5.0f;
  g = sumG / 5.0f;
  b = sumB / 5.0f;
  c = sumC / 5.0f;
  return true;
}

void calibrateWhiteBalance() {
  if (!ensureColorSensorReady()) {
    Serial.println(F(">>> White calibration aborted: TCS34725 not detected (check SDA=A4, SCL=A5)."));
    return;
  }

  Serial.println(F(">>> Calibrating white balance (keep WHITE object steady in front of TCS34725)..."));
  float r, g, b, c;
  if (!sampleCalibrationReference(r, g, b, c)) {
    return;
  }

  if (c < NO_OBJECT_MAX_C) {
    Serial.print(F(">>> White calibration aborted: C="));
    Serial.print((int)c);
    Serial.println(F(" is too low (walang object)."));
    return;
  }

  float ir = computeIrOffset(r, g, b, c);
  float r_ir = max(1.0f, r - ir);
  float g_ir = max(1.0f, g - ir);
  float b_ir = max(1.0f, b - ir);
  float avg_ir = (r_ir + g_ir + b_ir) / 3.0f;

  rGain = avg_ir / r_ir;
  gGain = avg_ir / g_ir;
  bGain = avg_ir / b_ir;

  whiteRefC = (int)(c + 0.5f);
  whiteChRef = avg_ir;
  if (blackRefC >= whiteRefC - 60) {
    blackRefC = -1;
    blackChRef = 0.0f;
  }
  updateColorThresholds();

  Serial.print(F(">>> White balance updated! Gains -> R="));
  Serial.print(rGain, 2);
  Serial.print(F(" G="));
  Serial.print(gGain, 2);
  Serial.print(F(" B="));
  Serial.print(bGain, 2);
  Serial.print(F(" | whiteRefC="));
  Serial.print(whiteRefC);
  Serial.print(F(" whiteChRef="));
  Serial.print(whiteChRef, 1);
  Serial.print(F(" | Thresholds -> blackSolidMaxC="));
  Serial.print(blackSolidMaxC);
  Serial.print(F(" blackMaxC="));
  Serial.print(blackMaxC);
  Serial.print(F(" whiteMinC="));
  Serial.println(whiteMinC);
  readAndDetectColor(true);
}

void calibrateBlackReference() {
  if (!ensureColorSensorReady()) {
    Serial.println(F(">>> Black calibration aborted: TCS34725 not detected (check SDA=A4, SCL=A5)."));
    return;
  }

  Serial.println(F(">>> Calibrating BLACK threshold (keep BLACK object steady in front of TCS34725)..."));
  float r, g, b, c;
  if (!sampleCalibrationReference(r, g, b, c)) {
    return;
  }

  if (c < 8.0f) {
    Serial.print(F(">>> Black calibration aborted: C="));
    Serial.print((int)c);
    Serial.println(F(" is too low (sensor blocked or no light)."));
    return;
  }

  if (c >= whiteRefC - 60) {
    Serial.print(F(">>> Black calibration aborted: measured C="));
    Serial.print((int)c);
    Serial.print(F(" is not darker than whiteRefC="));
    Serial.print(whiteRefC);
    Serial.println(F(" (calibrate WHITE with 'w' first)."));
    return;
  }

  blackRefC = (int)(c + 0.5f);
  float ir = computeIrOffset(r, g, b, c);
  float avgBal = (max(0.0f, r - ir) * rGain + max(0.0f, g - ir) * gGain + max(0.0f, b - ir) * bGain) / 3.0f;
  blackChRef = (avgBal < whiteChRef * 0.65f) ? avgBal : 0.0f;
  updateColorThresholds();

  Serial.print(F(">>> Black threshold updated! blackRefC="));
  Serial.print(blackRefC);
  Serial.print(F(" blackChRef="));
  Serial.print(blackChRef, 1);
  Serial.print(F(" whiteRefC="));
  Serial.print(whiteRefC);
  Serial.print(F(" | Thresholds -> blackSolidMaxC="));
  Serial.print(blackSolidMaxC);
  Serial.print(F(" blackMaxC="));
  Serial.print(blackMaxC);
  Serial.print(F(" whiteMinC="));
  Serial.println(whiteMinC);
  readAndDetectColor(true);
}


// ============================================================================
// 4. 6-SERVO MOTION & CALIBRATED 5-STAGE PICK-AND-PLACE SEQUENCE
// ============================================================================

/**
 * @brief Smoothly steps one servo (index 0..5) 1 degree per step to targetAngle.
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
 *        1) Lift upper arm to clearance posture (175, 25, 65, 55)
 *        2) Rotate Base (Pin 9) to targetPose[0]
 *        3) Lower/tilt upper arm to targetPose[1..4]
 */
void moveToPoseSmooth(const int targetPose[NUM_SERVOS]) {
  moveUpperArmSmooth(CLEARANCE_SHOULDER, CLEARANCE_ELBOW, CLEARANCE_WRIST_PITCH, CLEARANCE_WRIST_ROLL);
  int baseDelta = abs(targetPose[0] - currentAngles[0]);
  moveJointSmooth(0, targetPose[0]);
  if (stepDelayMs <= 2 && baseDelta >= 50) {
    delay(90);
  }
  moveUpperArmSmooth(targetPose[1], targetPose[2], targetPose[3], targetPose[4]);
}

/**
 * @brief Smoothly steps one servo (index 0..5) 1 degree per 2 ms to targetAngle
 *        and prints live position telemetry for the Serial Monitor adjuster.
 */
void moveServoSmooth(uint8_t idx, int targetAngle) {
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
    delay(2);
  }

  if (idx == 5) {
    gripperClosed = (currentAngles[5] >= (GRIP_OPEN_ANGLE + GRIP_CLOSE_ANGLE) / 2);
  }

  Serial.print(F(">> ["));
  Serial.print(servoNames[idx]);
  Serial.print(F("] = "));
  Serial.print(currentAngles[idx]);
  Serial.println(F(" deg"));
}

/**
 * @brief Sets the gripper claw (Pin 3) and waits briefly for jaw travel.
 */
void setGripper(int angle, unsigned long settleMs = 200) {
  angle = constrain(angle, 0, 180);
  currentAngles[5] = angle;
  gripperClosed = (angle >= (GRIP_OPEN_ANGLE + GRIP_CLOSE_ANGLE) / 2);
  servos[5].write(angle);
  delay(settleMs);
}

/**
 * @brief Moves smoothly to targetPose[0..4] via clearance posture and sets Gripper to targetPose[5].
 */
void moveToCalibratedPose(const int targetPose[NUM_SERVOS], const __FlashStringHelper* poseName) {
  Serial.print(F(">>> Moving smoothly to "));
  Serial.println(poseName);
  moveToPoseSmooth(targetPose);
  setGripper(targetPose[5], 100);
}

/**
 * @brief Runs the complete calibrated 5-stage Pick-and-Place cycle:
 *        BASE_POSE (160) -> PICKUP_POSE (140) -> Lift to BASE_POSE (160)
 *        -> Target Bucket Pose (60 / 100 / 15) -> Drop -> Return to BASE_POSE (160).
 */
void executePickAndPlace(const int targetBucketPose[NUM_SERVOS], const __FlashStringHelper* bucketName) {
  Serial.print(F(">>> Sorting -> "));
  Serial.print(bucketName);
  Serial.print(F(" (Base = "));
  Serial.print(targetBucketPose[0]);
  Serial.println(F(" deg)"));

  // Stage 1: Ensure arm is at BASE_POSE { 160, 175, 25, 65, 55 }, open Gripper to GRIP_FULL_OPEN (20),
  //          then move smoothly via moveToPoseSmooth(PICKUP_POSE) to { 140, 120, 40, 45, 40 }
  moveToPoseSmooth(BASE_POSE);
  setGripper(GRIP_FULL_OPEN, 150);
  moveToPoseSmooth(PICKUP_POSE);
  delay(80);

  // Stage 2: Close Gripper to GRIP_FULL_CLOSE (180) to grab object,
  //          then lift smoothly back to BASE_POSE { 160, 175, 25, 65, 55 } while holding Gripper at 180
  setGripper(GRIP_FULL_CLOSE, 220);
  moveToPoseSmooth(BASE_POSE);

  // Stage 3: Rotate Base (Pin 9) at clearance { 175, 25, 65, 55 } to targetBucketPose[0] (60 / 100 / 15),
  //          then move upper arm smoothly to targetBucketPose[1..4] while holding Gripper at 180
  moveToPoseSmooth(targetBucketPose);
  delay(80);

  // Stage 4: Open Gripper to GRIP_FULL_OPEN (20) to release the object
  setGripper(GRIP_FULL_OPEN, 180);

  // Stage 5: Lift upper arm smoothly back to { 175, 25, 65, 55 }, rotate Base back to 160,
  //          and restore BASE_POSE { 160, 175, 25, 65, 55, 180 }
  moveToPoseSmooth(BASE_POSE);
  setGripper(BASE_POSE[5], 100);

  Serial.println(F(">>> Cycle complete! Returned to BASE_POSE {160, 175, 25, 65, 55, 180}."));
}


// ============================================================================
// 5. LIVE ANGLE ADJUSTER, SERIAL MONITOR MENU & MAIN ARDUINO PROGRAM
// ============================================================================

void printAllAngles() {
  Serial.println(F("================ CURRENT ANGLES ================"));
  Serial.print(F("int POSE[6] = { "));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(currentAngles[i]);
    if (i < NUM_SERVOS - 1) {
      Serial.print(F(", "));
    }
  }
  Serial.println(F(" };"));

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print(i == selectedServo ? F(" -> ") : F("    "));
    Serial.print(servoNames[i]);
    Serial.print(F(": "));
    Serial.print(currentAngles[i]);
    Serial.println(F(" deg"));
  }
  Serial.println(F("================================================"));
}

void printHelpMenu() {
  Serial.println(F("================== COMMAND MENU =================="));
  Serial.println(F("[LIVE ANGLE ADJUSTER]"));
  Serial.println(F("  1..6         : Select servo 1..6 (Base..Gripper)"));
  Serial.println(F("  + / -        : Adjust selected servo by +5 / -5 deg"));
  Serial.println(F("  s1..s6 <ang> : Move servo 1..6 to <ang> (e.g. s2 105)"));
  Serial.println(F("  o / x        : Open (20 deg) / Close (180 deg) gripper"));
  Serial.println(F("  c            : Close gripper (if open) or read color (if closed)"));
  Serial.println(F("  v            : Read & print color sensor anytime"));
  Serial.println(F("  t            : Toggle auto gripper open/close loop (500 ms)"));
  Serial.println(F("  p            : Print CURRENT ANGLES block"));
  Serial.println(F("  ?            : Show this menu & current angles"));
  Serial.println(F("[PRESET POSES & SORTING TESTS]"));
  Serial.println(F("  p1..p5       : Move to BASE / PICKUP / MID / LEFT / RIGHT pose"));
  Serial.println(F("  m            : Test Pick & Place -> MID BUCKET (White, Base 60)"));
  Serial.println(F("  l / k        : Test Pick & Place -> LEFT BUCKET (Black, Base 100)"));
  Serial.println(F("  r / g / b    : Test Pick & Place -> RIGHT BUCKET (Colors, Base 15)"));
  Serial.println(F("  d            : Dip test to PICKUP_POSE and back to BASE_POSE"));
  Serial.println(F("  h            : Return smoothly to BASE_POSE (Home)"));
  Serial.println(F("[COLOR SENSOR & SYSTEM]"));
  Serial.println(F("  w            : Calibrate White Balance (hold WHITE object steady)"));
  Serial.println(F("  z            : Calibrate Black Threshold (hold BLACK object steady)"));
  Serial.println(F("  s / a        : Pause / Resume automatic ultrasonic sorting"));
  Serial.println(F("  f / n        : Speed Max Turbo (1 ms/deg) / Max (2 ms/deg)"));
  Serial.println(F("=================================================="));
}

void toggleAutoSorting() {
  isRunning = !isRunning;
  if (isRunning) {
    gripperLoopRunning = false;
    if (!colorSensorReady) {
      ensureColorSensorReady();
    }
    if (!colorSensorReady) {
      Serial.println(F(">>> WARNING: TCS34725 still offline (SDA=A4, SCL=A5). Auto-sorting will retry on object detection."));
    }
    Serial.println(F(">>> Auto Sorting: ON"));
  } else {
    Serial.println(F(">>> Auto Sorting: PAUSED"));
  }
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == ' ' || c == '\r' || c == '\n' || c == '\t') {
      continue;
    }

    if (c == 's' || c == 'S') {
      delay(15);
      int nextChar = Serial.peek();
      if (nextChar >= '1' && nextChar <= '6') {
        uint8_t sNum = (uint8_t)(Serial.read() - '0');
        delay(15);
        while (Serial.available() > 0 && (Serial.peek() == ' ' || Serial.peek() == '\t')) {
          Serial.read();
        }
        if (Serial.available() > 0 && (isDigit(Serial.peek()) || Serial.peek() == '-')) {
          int ang = Serial.parseInt();
          while (Serial.available() > 0) {
            char dump = Serial.read();
            if (dump == '\n' || dump == '\r') {
              break;
            }
          }
          selectedServo = sNum - 1;
          moveServoSmooth(selectedServo, ang);
        } else {
          while (Serial.available() > 0) {
            char dump = Serial.read();
            if (dump == '\n' || dump == '\r') {
              break;
            }
          }
          selectedServo = sNum - 1;
          Serial.print(F(">>> Usage: s"));
          Serial.print(sNum);
          Serial.print(F(" <0-180> (Selected: "));
          Serial.print(servoNames[selectedServo]);
          Serial.println(F(")"));
        }
      } else {
        toggleAutoSorting();
      }
    } else if (c == 'p' || c == 'P') {
      delay(15);
      int nextChar = Serial.peek();
      if (nextChar >= '1' && nextChar <= '5') {
        char p = (char)Serial.read();
        if (p == '1') {
          moveToCalibratedPose(BASE_POSE, F("BASE_POSE {160, 175, 25, 65, 55, 180}"));
        } else if (p == '2') {
          moveToCalibratedPose(PICKUP_POSE, F("PICKUP_POSE {140, 120, 40, 45, 40, 20}"));
        } else if (p == '3') {
          moveToCalibratedPose(MID_BUCKET_POSE, F("MID_BUCKET_POSE {60, 150, 25, 60, 35, 180}"));
        } else if (p == '4') {
          moveToCalibratedPose(LEFT_BUCKET_POSE, F("LEFT_BUCKET_POSE {100, 150, 30, 35, 35, 180}"));
        } else if (p == '5') {
          moveToCalibratedPose(RIGHT_BUCKET_POSE, F("RIGHT_BUCKET_POSE {15, 135, 5, 60, 35, 180}"));
        }
        printAllAngles();
      } else {
        printAllAngles();
      }
    } else if (c >= '1' && c <= '6') {
      selectedServo = (uint8_t)(c - '1');
      Serial.print(F("Napili: "));
      Serial.print(servoNames[selectedServo]);
      Serial.print(F(" (Current = "));
      Serial.print(currentAngles[selectedServo]);
      Serial.println(F(" deg)"));
    } else if (c == '+') {
      moveServoSmooth(selectedServo, currentAngles[selectedServo] + 5);
    } else if (c == '-') {
      moveServoSmooth(selectedServo, currentAngles[selectedServo] - 5);
    } else if (c == 'o' || c == 'O') {
      moveServoSmooth(5, GRIP_OPEN_ANGLE);
    } else if (c == 'x' || c == 'X') {
      moveServoSmooth(5, GRIP_CLOSE_ANGLE);
    } else if (c == 'c' || c == 'C') {
      if (currentAngles[5] != GRIP_CLOSE_ANGLE) {
        moveServoSmooth(5, GRIP_CLOSE_ANGLE);
      } else {
        readAndDetectColor();
      }
    } else if (c == 'v' || c == 'V') {
      readAndDetectColor();
    } else if (c == 't' || c == 'T') {
      gripperLoopRunning = !gripperLoopRunning;
      if (gripperLoopRunning) {
        isRunning = false;
        gripperClosed = (currentAngles[5] >= (GRIP_OPEN_ANGLE + GRIP_CLOSE_ANGLE) / 2);
        lastGripStepTime = millis();
        Serial.println(F(">>> Gripper Auto-Loop: ON (500 ms, Auto-Sort PAUSED)"));
      } else {
        Serial.println(F(">>> Gripper Auto-Loop: OFF"));
      }
    } else if (c == 'w' || c == 'W') {
      calibrateWhiteBalance();
    } else if (c == 'z' || c == 'Z') {
      calibrateBlackReference();
    } else if (c == 'a' || c == 'A') {
      toggleAutoSorting();
    } else if (c == 'h' || c == 'H') {
      moveToCalibratedPose(BASE_POSE, F("BASE_POSE {160, 175, 25, 65, 55, 180}"));
    } else if (c == 'd' || c == 'D') {
      Serial.println(F(">>> Testing dip to PICKUP_POSE {140, 120, 40, 45, 40, 20} (opening claw first) and back to BASE_POSE..."));
      setGripper(GRIP_FULL_OPEN, 120);
      moveToPoseSmooth(PICKUP_POSE);
      delay(300);
      moveToPoseSmooth(BASE_POSE);
      setGripper(BASE_POSE[5], 100);
    } else if (c == 'm' || c == 'M') {
      executePickAndPlace(MID_BUCKET_POSE, F("MID BUCKET / WHITE ONLY (Manual Test)"));
    } else if (c == 'l' || c == 'L' || c == 'k' || c == 'K') {
      executePickAndPlace(LEFT_BUCKET_POSE, F("LEFT BUCKET / BLACK (Manual Test)"));
    } else if (c == 'r' || c == 'R' || c == 'g' || c == 'G' || c == 'b' || c == 'B') {
      executePickAndPlace(RIGHT_BUCKET_POSE, F("RIGHT BUCKET / ALL COLORS (Manual Test)"));
    } else if (c == 'f' || c == 'F') {
      stepDelayMs = 1;
      Serial.println(F(">>> Speed: MAX TURBO (1 ms/deg)"));
    } else if (c == 'n' || c == 'N') {
      stepDelayMs = 2;
      Serial.println(F(">>> Speed: MAX (2 ms/deg)"));
    } else if (c == '?') {
      printHelpMenu();
      printAllAngles();
    }
  }
}

void setup() {
  Serial.begin(9600);
  delay(500);  // Allow USB-UART / Optiboot handoff and 5V rail to settle cleanly

  Serial.println(F("=================================================="));
  Serial.println(F("  6-Servo Pick-and-Place Color-Sorting Robot Arm  "));
  Serial.println(F("=================================================="));
  Serial.println(F("Starting 6-Servo Arm at calibrated BASE_POSE..."));

  initUltrasonic();
  initColorSensor();

  float bootDist = readDistanceCM();
  Serial.println(F("[BOOT DIAGNOSTIC]"));
  Serial.print(F("  TCS34725: "));
  Serial.println(colorSensorReady ? F("ONLINE") : F("OFFLINE (Disconnected)"));
  Serial.print(F("  HC-SR04 (D8/D7): "));
  if (bootDist < 0.0f) {
    Serial.println(F("NO ECHO / DISCONNECTED"));
  } else {
    Serial.print(bootDist, 1);
    Serial.println(F(" cm"));
  }

  // Attach all 6 servos one-by-one (600 ms gap to prevent power brownout)
  // directly initialized to BASE_POSE[i] { 160, 175, 25, 65, 55, 180 }
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    currentAngles[i] = BASE_POSE[i];
    servos[i].write(BASE_POSE[i]);
    servos[i].attach(servoPins[i], MG996R_MIN_US, MG996R_MAX_US);
    servos[i].write(BASE_POSE[i]);
    delay(600);
  }

  Serial.println(F("Ready at BASE_POSE {160, 175, 25, 65, 55, 180}!"));
  printHelpMenu();
  printAllAngles();
  Serial.print(F("Auto-Sorting: "));
  if (isRunning) {
    Serial.println(F("ACTIVE"));
    Serial.println(F("Sensor Ready. Waiting for close object..."));
  } else {
    Serial.println(F("PAUSED (TCS34725 offline - ready for manual Serial testing)"));
  }
}

uint8_t getTargetBucket(ObjectColor color) {
  if (color == COLOR_WHITE) return 1;
  if (color == COLOR_BLACK) return 2;
  if (color == COLOR_RED || color == COLOR_GREEN || color == COLOR_BLUE || color == COLOR_OTHER) return 3;
  return 0;
}

void loop() {
  handleSerialCommands();

  if (gripperLoopRunning && (millis() - lastGripStepTime >= GRIP_INTERVAL_MS)) {
    lastGripStepTime = millis();
    gripperClosed = !gripperClosed;
    int targetAngle = gripperClosed ? GRIP_CLOSE_ANGLE : GRIP_OPEN_ANGLE;
    currentAngles[5] = targetAngle;
    servos[5].write(targetAngle);
    Serial.print(F(">> ["));
    Serial.print(servoNames[5]);
    Serial.print(F("] = "));
    Serial.print(targetAngle);
    Serial.println(gripperClosed ? F(" deg (CLOSED)") : F(" deg (OPEN)"));
  }

  if (!isRunning) {
    delay(50);
    return;
  }

  float distance = readDistanceCM();

  if (distance >= MIN_RANGE && distance <= MAX_RANGE) {
    hitCount++;

    // Mag-trigger LANG kapag 2 sunod-sunod na reading ang nasa range
    if (hitCount >= REQUIRED_HITS) {
      Serial.print(F("OBJECT DETECTED! Distance: "));
      Serial.print(distance, 1);
      Serial.println(F(" cm"));

      // Wait briefly so a newly dropped object rests flat before reading TCS34725
      delay(PRE_READ_SETTLE_MS);
      if (!isObjectInRangeWithRetry()) {
        hitCount = 0;
        return;
      }

      // --- ACTION: Basahin ang kulay sa TCS34725 at i-sort gamit ang 6-servo arm ---
      ObjectColor firstRead = readAndDetectColor(true);
      uint8_t bucket1 = getTargetBucket(firstRead);

      delay(SETTLE_DELAY_MS);
      bool stillPresent = isObjectInRangeWithRetry();
      if (!stillPresent) {
        hitCount = 0;
        return;
      }

      ObjectColor confirmed = readAndDetectColor(true);
      int secondC = LastClear;
      uint8_t bucket2 = getTargetBucket(confirmed);

      // Bucket-level consensus (RED vs COLOR agree immediately on Right Bucket with zero tie-breaker delay)
      if (bucket1 != bucket2) {
        delay(20);
        ObjectColor tieBreaker = readAndDetectColor(true);
        int thirdC = LastClear;
        uint8_t bucket3 = getTargetBucket(tieBreaker);

        if (bucket3 == bucket1) {
          confirmed = firstRead;
        } else if (bucket3 == bucket2) {
          // confirmed already holds the majority bucket vote
        } else if (bucket3 == 3) {
          confirmed = tieBreaker;
        } else if ((confirmed == COLOR_WHITE && secondC >= whiteMinC) || (tieBreaker == COLOR_WHITE && thirdC >= whiteMinC)) {
          confirmed = COLOR_WHITE;
        } else if (bucket3 != 0) {
          confirmed = tieBreaker;
        }
      }

      // Kung nag-flicker sa NONE/NO_OBJECT sa pangalawang basa pero nandiyan pa rin ang object,
      // gamitin ang unang valid color para hindi mag-hesitate ang arm
      if (getTargetBucket(confirmed) == 0 && bucket1 != 0) {
        confirmed = firstRead;
      }

      switch (confirmed) {
        case COLOR_WHITE:
          hitCount = 0;
          executePickAndPlace(MID_BUCKET_POSE, F("WHITE -> MID BUCKET"));
          break;

        case COLOR_BLACK:
          hitCount = 0;
          executePickAndPlace(LEFT_BUCKET_POSE, F("BLACK -> LEFT BUCKET"));
          break;

        case COLOR_RED:
          hitCount = 0;
          executePickAndPlace(RIGHT_BUCKET_POSE, F("RED -> RIGHT BUCKET (ALL COLORS)"));
          break;

        case COLOR_GREEN:
          hitCount = 0;
          executePickAndPlace(RIGHT_BUCKET_POSE, F("GREEN -> RIGHT BUCKET (ALL COLORS)"));
          break;

        case COLOR_BLUE:
          hitCount = 0;
          executePickAndPlace(RIGHT_BUCKET_POSE, F("BLUE -> RIGHT BUCKET (ALL COLORS)"));
          break;

        case COLOR_OTHER:
          hitCount = 0;
          executePickAndPlace(RIGHT_BUCKET_POSE, F("COLOR -> RIGHT BUCKET (ALL COLORS)"));
          break;

        case COLOR_NO_OBJECT:
        case COLOR_NONE:
        default:
          break;
      }
    }
  } else {
    hitCount = 0;  // Reset kapag wala sa range
  }

  delay(30);
}
