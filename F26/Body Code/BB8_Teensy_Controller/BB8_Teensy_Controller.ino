/*
  BB-8 Teensy 4.1 Motor / Navigation Controller

  Pi sends:
      T,<distance_ft>,<angle_deg>\n
  Example:
      T,15.7,-32.5

  Teensy replies periodically:
      S,<state>,<distance>,<angle>,<pitch>,<roll>,<drive>,<swing>,<pivot>

  IMPORTANT:
  - Verify all Teensy pin assignments below before connecting motor drivers.
  - Teensy 4.1 GPIO is 3.3 V logic.
  - Pi and Teensy MUST share GND.
  - Motor power must NOT come from the Teensy.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_BNO08x.h>

// ============================================================
// PIN ASSIGNMENTS - CHANGE TO MATCH NEW WIRING
// ============================================================

// Pi UART: Teensy Serial1 RX=0, TX=1
constexpr uint8_t DRIVE1_PHASE_PIN  = 2;
constexpr uint8_t DRIVE1_PWM_PIN    = 3;
constexpr uint8_t DRIVE2_PHASE_PIN  = 4;
constexpr uint8_t DRIVE2_PWM_PIN    = 5;
constexpr uint8_t PIVOT_PHASE_PIN   = 6;
constexpr uint8_t PIVOT_PWM_PIN     = 7;

constexpr uint8_t RELAY1_PIN = 8;

// PCA9685
constexpr uint8_t PCA_ADDRESS = 0x40;
constexpr uint8_t SWING1_CH = 0;
constexpr uint8_t SWING2_CH = 4;

// Head servos (same PCA9685 channels as the original Pi controller)
constexpr uint8_t HEAD_FB_CH  = 1;  // head forward/back
constexpr uint8_t HEAD_STS_CH = 2;  // head side-to-side

// ============================================================
// CONFIGURATION
// ============================================================

constexpr uint32_t PI_BAUD = 115200;
constexpr uint32_t USB_BAUD = 115200;

constexpr float FOLLOW_START_DISTANCE_FT = 10.0f;
constexpr float FOLLOW_STOP_DISTANCE_FT  = 8.5f;
constexpr float HARD_MIN_DISTANCE_FT     = 6.0f;
constexpr float MAX_VALID_DISTANCE_FT    = 100.0f;

constexpr float MAX_DRIVE_SPEED = 0.30f;
constexpr float MIN_DRIVE_SPEED = 0.12f;
constexpr float FULL_SPEED_DISTANCE_FT = 18.0f;
constexpr float DRIVE_ACCEL_RATE = 0.40f;
constexpr float DRIVE_DECEL_RATE = 0.70f;

constexpr float ANGLE_DEADBAND_DEG = 6.0f;
constexpr float CLOSE_PIVOT_START_ANGLE_DEG = 22.0f;
constexpr float PIVOT_STOP_ANGLE_DEG = 8.0f;
constexpr float REAR_PIVOT_ANGLE_DEG = 65.0f;
constexpr float CLOSE_NAV_DISTANCE_FT = 14.0f;

constexpr float SWING_CENTER_DEG = 90.0f;
constexpr float MAX_SWING_OFFSET_DEG = 14.0f;
constexpr float SWING_KP = 0.35f;
constexpr float SWING_DIRECTION = -1.0f;
constexpr float MIN_SWING_OFFSET_DEG = 1.5f;
constexpr float SWING_SLEW_RATE_DEG_PER_SEC = 35.0f;

constexpr float PIVOT_KP = 0.022f;
constexpr float MIN_PIVOT_COMMAND = 0.10f;
constexpr float MAX_PIVOT_COMMAND = 0.25f;
constexpr float PIVOT_DIRECTION = -1.0f;

constexpr float DISTANCE_FILTER_ALPHA = 0.25f;
constexpr float ANGLE_FILTER_ALPHA = 0.30f;

constexpr uint32_t PI_TIMEOUT_MS = 500;
constexpr uint32_t CONTROL_PERIOD_US = 20000; // 50 Hz
constexpr uint32_t STATUS_PERIOD_MS = 250;

// Servo calibration.
// Tune these to your actual servos if necessary.
constexpr uint16_t SERVO_MIN_US = 500;
constexpr uint16_t SERVO_MAX_US = 2500;

// Head leveling configuration.
// Original Pi behavior was FB = 90 + pitch and STS = 90 + roll.
constexpr float HEAD_CENTER_DEG = 90.0f;
constexpr float HEAD_MIN_DEG = 55.0f;
constexpr float HEAD_MAX_DEG = 125.0f;
constexpr float HEAD_FB_OFFSET_DEG = 2.5f;
constexpr float HEAD_STS_OFFSET_DEG = -2.0f;
constexpr float HEAD_EMA_ALPHA = 0.15f;
constexpr float HEAD_PITCH_DIRECTION = 1.0f;
constexpr float HEAD_ROLL_DIRECTION  = 1.0f;

// ============================================================
// HARDWARE OBJECTS
// ============================================================

Adafruit_PWMServoDriver pca(PCA_ADDRESS);
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

bool imuOK = false;
float imuPitch = 0.0f;
float imuRoll = 0.0f;
float imuYaw = 0.0f;

// Smoothed head commands.
float headFBCommand = HEAD_CENTER_DEG;
float headSTSCommand = HEAD_CENTER_DEG;
bool headFilterInitialized = false;

// ============================================================
// CONTROLLER STATE
// ============================================================

enum NavState {
  STARTUP,
  WAITING_FOR_PI,
  IN_LEASH_ZONE,
  TARGET_REACHED,
  TOO_CLOSE,
  STRAIGHT,
  ARC,
  PIVOT,
  INVALID_TARGET,
  COMMS_LOST
};

NavState navState = STARTUP;

bool following = false;
bool pivoting = false;
bool systemEnabled = false;

bool haveFilteredTarget = false;
float filteredDistance = 0.0f;
float filteredAngle = 0.0f;

float currentDrive = 0.0f;
float currentSwing = SWING_CENTER_DEG;
float currentPivot = 0.0f;

uint32_t lastPacketMs = 0;
uint32_t lastControlUs = 0;
uint32_t lastStatusMs = 0;

char rxBuffer[80];
size_t rxIndex = 0;

// ============================================================
// HELPERS
// ============================================================

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float moveToward(float current, float target, float maxChange) {
  if (target > current) return min(current + maxChange, target);
  if (target < current) return max(current - maxChange, target);
  return current;
}

float normalizeAngle(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

const char* stateName(NavState s) {
  switch (s) {
    case STARTUP: return "STARTUP";
    case WAITING_FOR_PI: return "WAIT_PI";
    case IN_LEASH_ZONE: return "LEASH";
    case TARGET_REACHED: return "REACHED";
    case TOO_CLOSE: return "TOO_CLOSE";
    case STRAIGHT: return "STRAIGHT";
    case ARC: return "ARC";
    case PIVOT: return "PIVOT";
    case INVALID_TARGET: return "INVALID";
    case COMMS_LOST: return "COMMS_LOST";
    default: return "UNKNOWN";
  }
}

// ============================================================
// MOTOR OUTPUT
// Phase/Enable behavior equivalent to gpiozero PhaseEnableMotor.
// ============================================================

void setPhaseEnableMotor(uint8_t phasePin, uint8_t pwmPin, float command) {
  command = clampf(command, -1.0f, 1.0f);

  if (fabsf(command) < 0.001f) {
    analogWrite(pwmPin, 0);
    digitalWrite(phasePin, LOW);
    return;
  }

  bool direction = command >= 0.0f;
  digitalWrite(phasePin, direction ? HIGH : LOW);

  int pwm = (int)roundf(fabsf(command) * 255.0f);
  pwm = constrain(pwm, 0, 255);
  analogWrite(pwmPin, pwm);
}

void drive(float command) {
  command = clampf(command, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
  setPhaseEnableMotor(DRIVE1_PHASE_PIN, DRIVE1_PWM_PIN, -command);
  setPhaseEnableMotor(DRIVE2_PHASE_PIN, DRIVE2_PWM_PIN, -command);
}

void steer(float command) {
  command = clampf(command, -1.0f, 1.0f);
  if (fabsf(command) < 0.01f) command = 0.0f;
  setPhaseEnableMotor(PIVOT_PHASE_PIN, PIVOT_PWM_PIN, command);
}

uint16_t servoPulseFromDegrees(float degrees) {
  degrees = clampf(degrees, 0.0f, 180.0f);
  float us = SERVO_MIN_US +
             (degrees / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);

  // PCA9685 at 50 Hz => 20,000 us period / 4096 counts.
  return (uint16_t)roundf(us * 4096.0f / 20000.0f);
}

void setServoDegrees(uint8_t channel, float degrees) {
  pca.setPWM(channel, 0, servoPulseFromDegrees(degrees));
}

void setHeadForwardBack(float degrees) {
  degrees = clampf(degrees, HEAD_MIN_DEG, HEAD_MAX_DEG);
  setServoDegrees(HEAD_FB_CH, degrees + HEAD_FB_OFFSET_DEG);
}

void setHeadSideToSide(float degrees) {
  degrees = clampf(degrees, HEAD_MIN_DEG, HEAD_MAX_DEG);
  setServoDegrees(HEAD_STS_CH, degrees + HEAD_STS_OFFSET_DEG);
}

void centerHead() {
  headFBCommand = HEAD_CENTER_DEG;
  headSTSCommand = HEAD_CENTER_DEG;
  headFilterInitialized = false;
  setHeadForwardBack(HEAD_CENTER_DEG);
  setHeadSideToSide(HEAD_CENTER_DEG);
}

void updateHeadLeveling() {
  if (!imuOK) return;

  // Match the original Raspberry Pi behavior:
  // forward/back follows pitch; side-to-side follows roll.
  float targetFB = HEAD_CENTER_DEG + HEAD_PITCH_DIRECTION * imuPitch;
  float targetSTS = HEAD_CENTER_DEG + HEAD_ROLL_DIRECTION * imuRoll;

  targetFB = clampf(targetFB, HEAD_MIN_DEG, HEAD_MAX_DEG);
  targetSTS = clampf(targetSTS, HEAD_MIN_DEG, HEAD_MAX_DEG);

  if (!headFilterInitialized) {
    headFBCommand = targetFB;
    headSTSCommand = targetSTS;
    headFilterInitialized = true;
  } else {
    headFBCommand += HEAD_EMA_ALPHA * (targetFB - headFBCommand);
    headSTSCommand += HEAD_EMA_ALPHA * (targetSTS - headSTSCommand);
  }

  setHeadForwardBack(headFBCommand);
  setHeadSideToSide(headSTSCommand);
}

void setSwing(float degrees) {
  // Preserve original Movement_Functions behavior:
  // difference = current_swing - 90 + swing_offset
  // servo1 = 90 + difference
  // servo2 = 90 - difference
  constexpr float swingOffset = -1.0f;

  degrees = clampf(degrees, 70.0f, 117.0f);
  float difference = degrees - 90.0f + swingOffset;

  setServoDegrees(SWING1_CH, 90.0f + difference);
  setServoDegrees(SWING2_CH, 90.0f - difference);
}

void enableSystem() {
  // Outputs are already zero before relays close.
  drive(0.0f);
  steer(0.0f);
  setSwing(90.0f);
  delay(20);

  digitalWrite(RELAY1_PIN, HIGH);
  systemEnabled = true;
}

void disableSystem() {
  drive(0.0f);
  steer(0.0f);
  setSwing(90.0f);

  digitalWrite(RELAY1_PIN, LOW);
  systemEnabled = false;
}

void emergencyStop(NavState reason) {
  currentDrive = 0.0f;
  currentPivot = 0.0f;
  currentSwing = SWING_CENTER_DEG;

  drive(0.0f);
  steer(0.0f);
  setSwing(SWING_CENTER_DEG);

  following = false;
  pivoting = false;
  navState = reason;
}

// ============================================================
// BNO085
// ============================================================

void setReports() {
  if (!imuOK) return;

  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {
    Serial.println("WARNING: Could not enable BNO085 rotation vector");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk,
                       float &yaw, float &pitch, float &roll) {
  float sqr = qr * qr;
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;

  yaw = atan2f(2.0f * (qi * qj + qk * qr),
               (sqi - sqj - sqk + sqr));

  float sinp = -2.0f * (qi * qk - qj * qr);
  sinp = clampf(sinp, -1.0f, 1.0f);
  pitch = asinf(sinp);

  roll = atan2f(2.0f * (qj * qk + qi * qr),
                (-sqi - sqj + sqk + sqr));

  constexpr float RAD_TO_DEG_F = 57.2957795f;
  yaw *= RAD_TO_DEG_F;
  pitch *= RAD_TO_DEG_F;
  roll *= RAD_TO_DEG_F;
}

void updateIMU() {
  if (!imuOK) return;

  if (bno08x.wasReset()) {
    setReports();
  }

  while (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      quaternionToEuler(
        sensorValue.un.rotationVector.real,
        sensorValue.un.rotationVector.i,
        sensorValue.un.rotationVector.j,
        sensorValue.un.rotationVector.k,
        imuYaw, imuPitch, imuRoll
      );
    }
  }
}

// ============================================================
// TARGET FILTER
// ============================================================

void resetTargetFilter() {
  haveFilteredTarget = false;
}

void filterTarget(float distance, float angle) {
  angle = normalizeAngle(angle);

  if (!haveFilteredTarget) {
    filteredDistance = distance;
    filteredAngle = angle;
    haveFilteredTarget = true;
    return;
  }

  filteredDistance =
      DISTANCE_FILTER_ALPHA * distance +
      (1.0f - DISTANCE_FILTER_ALPHA) * filteredDistance;

  float error = normalizeAngle(angle - filteredAngle);

  filteredAngle = normalizeAngle(
      filteredAngle + ANGLE_FILTER_ALPHA * error
  );
}

// ============================================================
// PI SERIAL PROTOCOL
// ============================================================

void handleCommand(char *line) {
  // Target packet: T,distance_ft,angle_deg
  if (line[0] == 'T' && line[1] == ',') {
    char *p1 = strchr(line + 2, ',');
    if (!p1) return;

    *p1 = '\0';

    float distance = atof(line + 2);
    float angle = atof(p1 + 1);

    if (!isfinite(distance) || !isfinite(angle) ||
        distance < 0.0f || distance > MAX_VALID_DISTANCE_FT) {
      emergencyStop(INVALID_TARGET);
      return;
    }

    filterTarget(distance, angle);
    lastPacketMs = millis();

    // Do not energize motor power until first valid command.
    if (!systemEnabled) enableSystem();

    return;
  }

  // Explicit stop from Pi.
  if (strcmp(line, "STOP") == 0) {
    emergencyStop(WAITING_FOR_PI);
    disableSystem();
    resetTargetFilter();
    return;
  }

  // Optional heartbeat that does NOT command motion.
  if (strcmp(line, "PING") == 0) {
    Serial1.println("PONG");
    return;
  }
}

void readPiSerial() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();

    if (c == '\r') continue;

    if (c == '\n') {
      rxBuffer[rxIndex] = '\0';
      if (rxIndex > 0) handleCommand(rxBuffer);
      rxIndex = 0;
      continue;
    }

    if (rxIndex < sizeof(rxBuffer) - 1) {
      rxBuffer[rxIndex++] = c;
    } else {
      // Overflow = discard packet safely.
      rxIndex = 0;
    }
  }
}

// ============================================================
// NAVIGATION
// ============================================================

float calculateDriveSpeed(float distance) {
  if (distance <= FOLLOW_STOP_DISTANCE_FT) return 0.0f;
  if (distance >= FULL_SPEED_DISTANCE_FT) return MAX_DRIVE_SPEED;

  float progress =
      (distance - FOLLOW_STOP_DISTANCE_FT) /
      (FULL_SPEED_DISTANCE_FT - FOLLOW_STOP_DISTANCE_FT);

  float speed =
      MIN_DRIVE_SPEED +
      progress * (MAX_DRIVE_SPEED - MIN_DRIVE_SPEED);

  return clampf(speed, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
}

float calculateSwingTarget(float angle) {
  if (fabsf(angle) <= ANGLE_DEADBAND_DEG)
    return SWING_CENTER_DEG;

  float offset = angle * SWING_KP * SWING_DIRECTION;
  offset = clampf(offset, -MAX_SWING_OFFSET_DEG,
                  MAX_SWING_OFFSET_DEG);

  if (fabsf(offset) > 0.0f &&
      fabsf(offset) < MIN_SWING_OFFSET_DEG) {
    offset = copysignf(MIN_SWING_OFFSET_DEG, offset);
  }

  return SWING_CENTER_DEG + offset;
}

float calculatePivotCommand(float angle) {
  if (fabsf(angle) <= PIVOT_STOP_ANGLE_DEG)
    return 0.0f;

  float command = angle * PIVOT_KP * PIVOT_DIRECTION;

  if (fabsf(command) > 0.0f &&
      fabsf(command) < MIN_PIVOT_COMMAND) {
    command = copysignf(MIN_PIVOT_COMMAND, command);
  }

  return clampf(command, -MAX_PIVOT_COMMAND,
                MAX_PIVOT_COMMAND);
}

void commandDrive(float target, float dt) {
  target = clampf(target, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

  float rate =
      fabsf(target) > fabsf(currentDrive)
      ? DRIVE_ACCEL_RATE
      : DRIVE_DECEL_RATE;

  currentDrive = moveToward(
      currentDrive, target, rate * dt
  );

  drive(currentDrive);
}

void commandSwing(float target, float dt) {
  target = clampf(
      target,
      SWING_CENTER_DEG - MAX_SWING_OFFSET_DEG,
      SWING_CENTER_DEG + MAX_SWING_OFFSET_DEG
  );

  currentSwing = moveToward(
      currentSwing,
      target,
      SWING_SLEW_RATE_DEG_PER_SEC * dt
  );

  setSwing(currentSwing);
}

void controlledStop(float dt) {
  currentPivot = 0.0f;
  steer(0.0f);
  commandDrive(0.0f, dt);
  commandSwing(SWING_CENTER_DEG, dt);
}

void navigationUpdate(float dt) {
  if (!haveFilteredTarget) {
    controlledStop(dt);
    navState = WAITING_FOR_PI;
    return;
  }

  float distance = filteredDistance;
  float angle = normalizeAngle(filteredAngle);

  if (distance <= HARD_MIN_DISTANCE_FT) {
    emergencyStop(TOO_CLOSE);
    return;
  }

  // Follow-distance hysteresis
  if (!following) {
    if (distance < FOLLOW_START_DISTANCE_FT) {
      navState = IN_LEASH_ZONE;
      controlledStop(dt);
      return;
    }
    following = true;
  } else {
    if (distance <= FOLLOW_STOP_DISTANCE_FT) {
      following = false;
      pivoting = false;
      navState = TARGET_REACHED;
      controlledStop(dt);
      return;
    }
  }

  // Continue an existing pivot until aligned.
  if (pivoting) {
    if (fabsf(angle) <= PIVOT_STOP_ANGLE_DEG) {
      pivoting = false;
      currentPivot = 0.0f;
      steer(0.0f);
    } else {
      navState = PIVOT;
      commandDrive(0.0f, dt);
      commandSwing(SWING_CENTER_DEG, dt);
      currentPivot = calculatePivotCommand(angle);
      steer(currentPivot);
      return;
    }
  }

  // Target behind BB-8.
  if (fabsf(angle) >= REAR_PIVOT_ANGLE_DEG) {
    navState = PIVOT;
    pivoting = true;
    commandDrive(0.0f, dt);
    commandSwing(SWING_CENTER_DEG, dt);
    currentPivot = calculatePivotCommand(angle);
    steer(currentPivot);
    return;
  }

  // Straight.
  if (fabsf(angle) <= ANGLE_DEADBAND_DEG) {
    navState = STRAIGHT;
    currentPivot = 0.0f;
    steer(0.0f);
    commandSwing(SWING_CENTER_DEG, dt);
    commandDrive(calculateDriveSpeed(distance), dt);
    return;
  }

  // Close + significant angle => pivot.
  if (distance <= CLOSE_NAV_DISTANCE_FT &&
      fabsf(angle) >= CLOSE_PIVOT_START_ANGLE_DEG) {
    navState = PIVOT;
    pivoting = true;
    commandDrive(0.0f, dt);
    commandSwing(SWING_CENTER_DEG, dt);
    currentPivot = calculatePivotCommand(angle);
    steer(currentPivot);
    return;
  }

  // Otherwise arc.
  navState = ARC;
  currentPivot = 0.0f;
  steer(0.0f);
  commandSwing(calculateSwingTarget(angle), dt);
  commandDrive(calculateDriveSpeed(distance), dt);
}

// ============================================================
// STATUS
// ============================================================

void sendStatus() {
  Serial1.print("S,");
  Serial1.print(stateName(navState));
  Serial1.print(",");
  Serial1.print(filteredDistance, 2);
  Serial1.print(",");
  Serial1.print(filteredAngle, 2);
  Serial1.print(",");
  Serial1.print(imuPitch, 2);
  Serial1.print(",");
  Serial1.print(imuRoll, 2);
  Serial1.print(",");
  Serial1.print(currentDrive, 3);
  Serial1.print(",");
  Serial1.print(currentSwing, 1);
  Serial1.print(",");
  Serial1.println(currentPivot, 3);

  Serial.print("State=");
  Serial.print(stateName(navState));
  Serial.print(" D=");
  Serial.print(filteredDistance, 1);
  Serial.print(" A=");
  Serial.print(filteredAngle, 1);
  Serial.print(" Pitch=");
  Serial.print(imuPitch, 1);
  Serial.print(" Roll=");
  Serial.print(imuRoll, 1);
  Serial.print(" Drive=");
  Serial.print(currentDrive, 2);
  Serial.print(" Swing=");
  Serial.print(currentSwing, 1);
  Serial.print(" Pivot=");
  Serial.print(currentPivot, 2);
  Serial.print(" HeadFB=");
  Serial.print(headFBCommand, 1);
  Serial.print(" HeadSTS=");
  Serial.println(headSTSCommand, 1);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(USB_BAUD);
  Serial1.begin(PI_BAUD);

  pinMode(DRIVE1_PHASE_PIN, OUTPUT);
  pinMode(DRIVE1_PWM_PIN, OUTPUT);
  pinMode(DRIVE2_PHASE_PIN, OUTPUT);
  pinMode(DRIVE2_PWM_PIN, OUTPUT);
  pinMode(PIVOT_PHASE_PIN, OUTPUT);
  pinMode(PIVOT_PWM_PIN, OUTPUT);

  pinMode(RELAY1_PIN, OUTPUT);

  // SAFETY: establish safe outputs before doing anything else.
  digitalWrite(RELAY1_PIN, LOW);
  analogWrite(DRIVE1_PWM_PIN, 0);
  analogWrite(DRIVE2_PWM_PIN, 0);
  analogWrite(PIVOT_PWM_PIN, 0);

  Wire.begin();

  pca.begin();
  pca.setPWMFreq(50);
  setSwing(SWING_CENTER_DEG);
  centerHead();

  // BNO085 on default I2C address.
  imuOK = bno08x.begin_I2C();
  if (imuOK) {
    Serial.println("BNO085 detected.");
    setReports();
  } else {
    Serial.println("WARNING: BNO085 not detected. Motor control will still run.");
  }

  navState = WAITING_FOR_PI;

  Serial.println("BB-8 Teensy controller ready.");
  Serial.println("Waiting for Pi target packets...");
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  readPiSerial();
  updateIMU();

  uint32_t nowMs = millis();

  // Hardware-level communication watchdog.
  if (systemEnabled &&
      (uint32_t)(nowMs - lastPacketMs) > PI_TIMEOUT_MS) {
    emergencyStop(COMMS_LOST);
    disableSystem();
    resetTargetFilter();
  }

  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastControlUs) >= CONTROL_PERIOD_US) {
    float dt;

    if (lastControlUs == 0) {
      dt = 0.02f;
    } else {
      dt = (nowUs - lastControlUs) / 1000000.0f;
      dt = clampf(dt, 0.001f, 0.10f);
    }

    lastControlUs = nowUs;

    if (systemEnabled) {
      navigationUpdate(dt);
    }

    // Keep the head level from the latest BNO085 pitch/roll reading.
    // This is independent of the navigation state.
    updateHeadLeveling();
  }

  if ((uint32_t)(nowMs - lastStatusMs) >= STATUS_PERIOD_MS) {
    lastStatusMs = nowMs;
    sendStatus();
  }
}
