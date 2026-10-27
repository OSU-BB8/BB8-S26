/*
  EXPERIMENTAL PENDULUM FEEDBACK VERSION
  Derived from BB8_Teensy_Controller_RollBalance.ino.
  ENABLE_PENDULUM_STABILIZATION defaults to false for direction verification.
  Physical lean axes and servo polarity MUST be tested with robot supported.
*/
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
constexpr uint8_t DRIVE1_PHASE_PIN  = 3; //Direction 1
constexpr uint8_t DRIVE1_PWM_PIN    = 2; //PWM 1
constexpr uint8_t DRIVE2_PHASE_PIN  = 5; //Direction 2
constexpr uint8_t DRIVE2_PWM_PIN    = 4; //PWM 2
constexpr uint8_t PIVOT_PHASE_PIN   = 7;
constexpr uint8_t PIVOT_PWM_PIN     = 6;

// Relays are now driven by Body Pi GPIO20/GPIO21, not the Teensy.

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

constexpr float FOLLOW_START_DISTANCE_FT = 5.0f;
constexpr float FOLLOW_STOP_DISTANCE_FT  = 4.0f;
constexpr float HARD_MIN_DISTANCE_FT     = 3.0f;
constexpr float MAX_VALID_DISTANCE_FT    = 40.0f;

constexpr float MAX_DRIVE_SPEED = 0.30f;
constexpr float MIN_DRIVE_SPEED = 0.12f;
constexpr float FULL_SPEED_DISTANCE_FT = 18.0f;
constexpr float DRIVE_ACCEL_RATE = 0.40f;
constexpr float DRIVE_DECEL_RATE = 0.70f;

// IMU pitch stabilization. Enable only after verifying correction direction
// with the robot securely supported and drive wheels clear of the ground.
// SAFETY: Leave false until drive wheels are raised and feedback direction verified.
// After successful testing, change to true to activate idle and moving damping.
constexpr bool ENABLE_DRIVE_STABILIZATION = true;
constexpr float BALANCE_DIRECTION = 1.0f; // Reverse to -1 if feedback reinforces rocking
constexpr float BALANCE_PITCH_ZERO_DEG = 0.0f; // Calibrate at physical rest
constexpr float BALANCE_KP = 0.008f; // PWM fraction / degree
constexpr float BALANCE_KD = 0.0015f; // PWM fraction / (degree/second)
constexpr float BALANCE_MAX_STILL = 0.10f;
constexpr float BALANCE_MAX_MOVING = 0.06f;
constexpr float BALANCE_DEADBAND_DEG = 0.6f;
constexpr float BALANCE_MAX_TILT_DEG = 20.0f;
constexpr uint32_t BALANCE_IMU_TIMEOUT_MS = 100;
constexpr float BALANCE_RATE_FILTER_ALPHA = 0.25f;
// Additional output filtering prevents abrupt PWM changes due to IMU noise.
constexpr float BALANCE_CORRECTION_SLEW_PER_SEC = 0.40f;
constexpr float BALANCE_MOVING_TILT_KP_SCALE = 0.45f;
constexpr float BALANCE_MOVING_RATE_KD_SCALE = 1.0f;
constexpr float BALANCE_IDLE_RATE_KD_SCALE = 1.0f;
// Disable corrections near a large tilt; this is damping, not fall recovery.
constexpr float BALANCE_IDLE_ACTIVATION_DEG = 0.4f;


constexpr float ANGLE_DEADBAND_DEG = 6.0f;
constexpr float CLOSE_PIVOT_START_ANGLE_DEG = 30.0f;
constexpr float PIVOT_STOP_ANGLE_DEG = 10.0f;
constexpr float REAR_PIVOT_ANGLE_DEG = 65.0f;
constexpr float CLOSE_NAV_DISTANCE_FT = 10.0f;

constexpr float SWING_CENTER_DEG = 90.0f; // Calibrate mechanical center with feedback OFF
constexpr float MAX_SWING_OFFSET_DEG = 14.0f;
constexpr float SWING_KP = 0.35f;
constexpr float SWING_DIRECTION = 1.0f;
constexpr float MIN_SWING_OFFSET_DEG = 1.5f;
constexpr float SWING_SLEW_RATE_DEG_PER_SEC = 35.0f;

// Closed-loop lateral body tilt control using the pendulum servos.
// Body coordinates: imuPitch = forward/back; imuRoll = left/right.
// Keep false until servo correction polarity is verified while supported.
constexpr bool ENABLE_PENDULUM_STABILIZATION = true;
constexpr float PENDULUM_TILT_ZERO_DEG = 0.0f;
// Target body lean in ARC: signed beacon angle * gain, limited to safe lean.
constexpr float ARC_BODY_LEAN_DEG_PER_TARGET_DEG = 0.08f;
constexpr float MAX_ARC_BODY_LEAN_DEG = 5.0f;
constexpr float ARC_LEAN_DIRECTION = 1.0f; // Verify inside-of-turn direction
constexpr float BODY_TILT_TARGET_SLEW_DEG_PER_SEC = 90.0f;
// PD output is a servo-angle offset, NOT a motor PWM command.
constexpr float PENDULUM_KP = 0.20f; // servo deg / body tilt deg
constexpr float PENDULUM_KD = 0.20f; // servo deg / (body tilt deg/s)
constexpr float PENDULUM_KI = 0.05f; // servo deg / (body tilt deg * second)
constexpr float PENDULUM_INTEGRAL_LIMIT = 100.0f; // deg * seconds
constexpr float PENDULUM_INTEGRAL_RESET_TARGET_CHANGE_DEG = 1.0f;
constexpr float PENDULUM_CORRECTION_DIRECTION = -1.0f; // verify physical sign
constexpr float PENDULUM_MAX_CORRECTION_DEG = 4.0f;
constexpr float PENDULUM_TILT_RATE_FILTER_ALPHA = 0.45f;
constexpr float PENDULUM_MAX_VALID_TILT_DEG = 30.0f;
constexpr uint32_t PENDULUM_IMU_TIMEOUT_MS = 100;


constexpr float PIVOT_KP = 0.022f;
constexpr float MIN_PIVOT_COMMAND = 0.10f;
constexpr float MAX_PIVOT_COMMAND = 1.0f;
constexpr float PIVOT_DIRECTION = -1.0f;

constexpr float DISTANCE_FILTER_ALPHA = 0.25f;
constexpr float ANGLE_FILTER_ALPHA = 0.30f;

constexpr uint32_t PI_TIMEOUT_MS = 500;
constexpr uint32_t CONTROL_PERIOD_US = 20000; // 50 Hz
constexpr uint32_t STATUS_PERIOD_MS = 250;

// Time allowed for the physical relay contacts to close before any motor
// command is applied. The first received target is retained during this wait.
// Relay contact settling is handled by the Body Pi before sending targets.

// Servo calibration.
// Tune these to your actual servos if necessary.
constexpr uint16_t SERVO_MIN_US = 500;
constexpr uint16_t SERVO_MAX_US = 2500;

// Head leveling configuration.
// Original Pi behavior was FB = 90 + pitch and STS = 90 + roll.
constexpr float HEAD_CENTER_DEG = 90.0f;
constexpr float HEAD_MIN_DEG = 65.0f;
constexpr float HEAD_MAX_DEG = 125.0f;
constexpr float HEAD_FB_OFFSET_DEG = 2.5f;
constexpr float HEAD_STS_OFFSET_DEG = -2.0f;
constexpr float HEAD_EMA_ALPHA = 0.90f;

// Startup safety: do not immediately command the head to the IMU angle.
// Wait briefly, then ramp the commanded head angles toward level at a
// bounded speed. This prevents a sudden snap when the Teensy powers up.
constexpr uint32_t HEAD_STARTUP_HOLD_MS = 5000;
constexpr float HEAD_STARTUP_SLEW_DEG_PER_SEC = 90.0f;

constexpr float HEAD_PITCH_DIRECTION = -1.0f; // Head servo direction independent of IMU coordinates
constexpr float HEAD_ROLL_DIRECTION  = 1.0f;

// ============================================================
// HARDWARE OBJECTS
// ============================================================

Adafruit_PWMServoDriver pca(PCA_ADDRESS);
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

bool imuOK = true;
float imuPitch = 0.0f;
float imuRoll = 0.0f;
float imuYaw = 0.0f;

// Smoothed head commands.
float headFBCommand = HEAD_CENTER_DEG;
float headSTSCommand = HEAD_CENTER_DEG;
bool headFilterInitialized = false;
uint32_t headStartupMs = 0;
uint32_t lastHeadUpdateMs = 0;

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
bool systemEnabled = true;

bool haveFilteredTarget = false;
float filteredDistance = 0.0f;
float filteredAngle = 0.0f;

float currentDrive = 0.0f;
float actualDriveOutput = 0.0f;
float smoothedBalanceCorrection = 0.0f;
float filteredPitchRate = 0.0f;
float lastBalancePitch = 0.0f;
uint32_t lastBalanceImuMs = 0;
uint32_t lastBalanceSampleUs = 0;
bool balanceRateReady = true;
bool balanceImuFresh = false;
float currentSwing = SWING_CENTER_DEG;
float targetBodyTiltDeg = 0.0f;
float commandedBodyTiltDeg = 0.0f;
float lateralTiltRateDegS = 0.0f;
float lastLateralTiltDeg = 0.0f;
uint32_t lastLateralTiltSampleUs = 0;
uint32_t lastLateralTiltMs = 0;
bool lateralTiltRateReady = false;
float pendulumCorrectionDeg = 0.0f;
float pendulumIntegral = 0.0f;
float previousPendulumTargetDeg = 0.0f;
bool pendulumWasActive = false;
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

float slewToward(float current, float target, float maxStep) {
  float error = target - current;
  if (error > maxStep) return current + maxStep;
  if (error < -maxStep) return current - maxStep;
  return target;
}

void updateHeadLeveling() {
  if (!imuOK) return;

  uint32_t now = millis();

  // Give the person powering up BB-8 time to move their hands away.
  // During this hold the head remains at the startup command.
  if ((uint32_t)(now - headStartupMs) < HEAD_STARTUP_HOLD_MS) {
    lastHeadUpdateMs = now;
    return;
  }

  float dt = 0.02f;
  if (lastHeadUpdateMs != 0) {
    dt = (now - lastHeadUpdateMs) / 1000.0f;
    dt = clampf(dt, 0.001f, 0.10f);
  }
  lastHeadUpdateMs = now;

  // Body-frame axes: forward/back uses pitch; side-to-side uses roll.
  // Verify physical servo polarity with motor power disabled.
  float targetFB = HEAD_CENTER_DEG + HEAD_PITCH_DIRECTION * imuPitch;
  float targetSTS = HEAD_CENTER_DEG + HEAD_ROLL_DIRECTION * imuRoll;

  targetFB = clampf(targetFB, HEAD_MIN_DEG, HEAD_MAX_DEG);
  targetSTS = clampf(targetSTS, HEAD_MIN_DEG, HEAD_MAX_DEG);

  // Keep the normal EMA filtering, but never allow the physical command
  // to move faster than the configured startup-safe slew rate.
  float filteredFB = headFBCommand + HEAD_EMA_ALPHA * (targetFB - headFBCommand);
  float filteredSTS = headSTSCommand + HEAD_EMA_ALPHA * (targetSTS - headSTSCommand);

  float maxStep = HEAD_STARTUP_SLEW_DEG_PER_SEC * dt;
  headFBCommand = slewToward(headFBCommand, filteredFB, maxStep);
  headSTSCommand = slewToward(headSTSCommand, filteredSTS, maxStep);
  headFilterInitialized = true;

  setHeadForwardBack(headSTSCommand);
  setHeadSideToSide(headFBCommand);
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

// This flag arms/disarms the motor-control algorithm only.
// Physical relay power is controlled exclusively by the Body Pi.
void enableSystem() {
  if (systemEnabled) return;
  currentDrive = 0.0f;
  actualDriveOutput = 0.0f;
  smoothedBalanceCorrection = 0.0f;
  currentPivot = 0.0f;
  drive(0.0f);
  steer(0.0f);
  setSwing(SWING_CENTER_DEG);
  systemEnabled = true;
}

void disableSystem() {
  drive(0.0f);
  steer(0.0f);
  setSwing(SWING_CENTER_DEG);
  currentDrive = 0.0f;
  actualDriveOutput = 0.0f;
  smoothedBalanceCorrection = 0.0f;
  currentPivot = 0.0f;
  targetBodyTiltDeg = 0.0f;
  commandedBodyTiltDeg = 0.0f;
  pendulumCorrectionDeg = 0.0f;
  pendulumIntegral = 0.0f;
  pendulumWasActive = false;
  systemEnabled = false;
}

void emergencyStop(NavState reason) {
  currentDrive = 0.0f;
  actualDriveOutput = 0.0f;
  smoothedBalanceCorrection = 0.0f;
  currentPivot = 0.0f;
  currentSwing = SWING_CENTER_DEG;
  targetBodyTiltDeg = 0.0f;
  commandedBodyTiltDeg = 0.0f;
  pendulumCorrectionDeg = 0.0f;
  pendulumIntegral = 0.0f;
  pendulumWasActive = false;

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
      
      // BNO085 rotated 90 degrees clockwise on new PCB.
      // Compensate for the mounting rotation.

      const float c = 0.70710678118f; // cos(45 degrees)
      const float s = 0.70710678118f; // sin(45 degrees)

      float qr = sensorValue.un.rotationVector.real;
      float qi = sensorValue.un.rotationVector.i;
      float qj = sensorValue.un.rotationVector.j;
      float qk = sensorValue.un.rotationVector.k;

      // Apply +90 degree Z-axis correction.
      // q_corrected = q_sensor * q_mount
      float correctedR = qr * c - qk * s;
      float correctedI = qi * c + qj * s;
      float correctedJ = qj * c - qi * s;
      float correctedK = qk * c + qr * s;

      // Convert the mounting-corrected quaternion once, then name body axes.
      // The mounting quaternion is corrected above. This particular PCB's
      // physical forward axis corresponds to the converter's roll output.
      // Assign body-frame names once, here; controllers never swap axes.
      float sensorPitchDeg, sensorRollDeg;
      quaternionToEuler(correctedR, correctedI, correctedJ, correctedK,
                        imuYaw, sensorPitchDeg, sensorRollDeg);
      imuPitch = sensorRollDeg;  // forward/back body tilt
      imuRoll  = sensorPitchDeg; // left/right body tilt

      // Lateral body tilt and angular rate for pendulum feedback.
      // Lateral roll rate for pendulum control.
      if (isfinite(imuRoll)) {
        uint32_t tiltNowUs = micros();
        if (lastLateralTiltSampleUs != 0) {
          float sampleDt = (uint32_t)(tiltNowUs - lastLateralTiltSampleUs) * 1.0e-6f;
          if (sampleDt >= 0.002f && sampleDt <= 0.2f) {
            float rawRate = (imuRoll - lastLateralTiltDeg) / sampleDt;
            if (isfinite(rawRate)) {
              lateralTiltRateDegS += PENDULUM_TILT_RATE_FILTER_ALPHA *
                                     (rawRate - lateralTiltRateDegS);
              lateralTiltRateReady = true;
            }
          } else {
            lateralTiltRateReady = false;
            lateralTiltRateDegS = 0.0f;
          }
        }
        lastLateralTiltDeg = imuRoll;
        lastLateralTiltSampleUs = tiltNowUs;
        lastLateralTiltMs = millis();
      }

      if (isfinite(imuPitch)) {
        uint32_t sampleUs = micros();
        if (lastBalanceSampleUs != 0) {
          float sampleDt = (uint32_t)(sampleUs - lastBalanceSampleUs) * 1.0e-6f;
          if (sampleDt >= 0.002f && sampleDt <= 0.1f) {
            float rawRate = (imuPitch - lastBalancePitch) / sampleDt;
            if (fabsf(rawRate) < 1000.0f) {
              filteredPitchRate += BALANCE_RATE_FILTER_ALPHA * (rawRate - filteredPitchRate);
              balanceRateReady = true;
            }
          } else {
            balanceRateReady = false;
            filteredPitchRate = 0.0f;
          }
        }
        lastBalancePitch = imuPitch;
        lastBalanceSampleUs = sampleUs;
        lastBalanceImuMs = millis();
        balanceImuFresh = true;
      }
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

    // Enable navigation after first valid command; Pi owns physical relays.
    if (!systemEnabled) enableSystem();

    return;
  }

  // Pause movement without changing Pi-controlled relays.
  if (strcmp(line, "PAUSE") == 0) {
      emergencyStop(WAITING_FOR_PI);

      // Throw away the old target so BB-8 cannot resume
      // toward stale beacon data.
      resetTargetFilter();

      // Keep navigation armed. The Pi decides whether to keep relays on.
      if (!systemEnabled) {
          enableSystem();
      }

      // PAUSE counts as valid Pi communication.
      lastPacketMs = millis();

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

// Forward/backward rocking uses body pitch.
// Damping in idle and during forward driving. This does NOT measure speed.
// Returns zero if IMU data are stale or tilt is outside the safe control range.
float calculateBalanceCorrection(float target) {
  if (!ENABLE_DRIVE_STABILIZATION || !systemEnabled || !imuOK ||
      !balanceImuFresh || !balanceRateReady ||
      (uint32_t)(millis() - lastBalanceImuMs) > BALANCE_IMU_TIMEOUT_MS)
    return 0.0f;

  float tilt = imuPitch - BALANCE_PITCH_ZERO_DEG;
  if (!isfinite(tilt) || !isfinite(filteredPitchRate) ||
      fabsf(tilt) > BALANCE_MAX_TILT_DEG)
    return 0.0f;

  const bool moving = fabsf(target) >= 0.001f;
  float p = tilt;
  if (fabsf(p) < (moving ? BALANCE_DEADBAND_DEG : BALANCE_IDLE_ACTIVATION_DEG))
    p = 0.0f;

  // While moving, emphasize pitch-rate damping rather than forcing pitch to
  // zero; normal acceleration may require a nonzero body lean.
  float kp = BALANCE_KP * (moving ? BALANCE_MOVING_TILT_KP_SCALE : 1.0f);
  float kd = BALANCE_KD * (moving ? BALANCE_MOVING_RATE_KD_SCALE
                                 : BALANCE_IDLE_RATE_KD_SCALE);
  float correction = BALANCE_DIRECTION * (kp * p + kd * filteredPitchRate);
  float limit = moving ? BALANCE_MAX_MOVING : BALANCE_MAX_STILL;
  return clampf(correction, -limit, limit);
}

void commandDrive(float target, float dt) {
  target = clampf(target, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

  float rate = fabsf(target) > fabsf(currentDrive)
                   ? DRIVE_ACCEL_RATE : DRIVE_DECEL_RATE;
  currentDrive = moveToward(currentDrive, target, rate * dt);

  // Use actual ramped command to select idle vs driving control.
  // Balance angle and its derivative both come from body pitch.
  float desiredCorrection = calculateBalanceCorrection(currentDrive);
  // Immediately drop correction on disabled/stale IMU or unsafe tilt.
  bool correctionValid = ENABLE_DRIVE_STABILIZATION && systemEnabled && imuOK &&
      balanceImuFresh && balanceRateReady &&
      (uint32_t)(millis() - lastBalanceImuMs) <= BALANCE_IMU_TIMEOUT_MS &&
      isfinite(imuPitch) && isfinite(filteredPitchRate) &&
      fabsf(imuPitch - BALANCE_PITCH_ZERO_DEG) <= BALANCE_MAX_TILT_DEG;
  if (!correctionValid) {
    smoothedBalanceCorrection = 0.0f;
  } else {
    smoothedBalanceCorrection = moveToward(smoothedBalanceCorrection,
        desiredCorrection, BALANCE_CORRECTION_SLEW_PER_SEC * dt);
  }
  actualDriveOutput = clampf(currentDrive + smoothedBalanceCorrection,
                              -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
  drive(actualDriveOutput);
}

// target is the existing feed-forward pendulum servo position.
// The target body lean is chosen separately from the ARC state.
void commandSwing(float target, float dt) {
  target = clampf(target,
      SWING_CENTER_DEG - MAX_SWING_OFFSET_DEG,
      SWING_CENTER_DEG + MAX_SWING_OFFSET_DEG);

  commandedBodyTiltDeg = moveToward(commandedBodyTiltDeg,
      targetBodyTiltDeg, BODY_TILT_TARGET_SLEW_DEG_PER_SEC * dt);

  pendulumCorrectionDeg = 0.0f;
  bool imuValid = imuOK && lateralTiltRateReady &&
      (uint32_t)(millis() - lastLateralTiltMs) <= PENDULUM_IMU_TIMEOUT_MS &&
      isfinite(imuRoll) && isfinite(lateralTiltRateDegS) &&
      fabsf(imuRoll - PENDULUM_TILT_ZERO_DEG) <= PENDULUM_MAX_VALID_TILT_DEG;
  bool active = ENABLE_PENDULUM_STABILIZATION && systemEnabled && imuValid &&
      isfinite(dt) && dt > 0.0f && dt <= 0.1f;

  if (!active) {
    pendulumIntegral = 0.0f;
    pendulumWasActive = false;
  } else {
    if (!pendulumWasActive ||
        fabsf(commandedBodyTiltDeg - previousPendulumTargetDeg) >
            PENDULUM_INTEGRAL_RESET_TARGET_CHANGE_DEG) {
      pendulumIntegral = 0.0f;
    }
    pendulumWasActive = true;
    previousPendulumTargetDeg = commandedBodyTiltDeg;

    float measuredTilt = imuRoll - PENDULUM_TILT_ZERO_DEG;
    float tiltError = commandedBodyTiltDeg - measuredTilt;

    // Do not integrate if the correction or final servo target would saturate.
    // This prevents the integral from building up against mechanical limits.
    float candidateIntegral = clampf(pendulumIntegral + tiltError * dt,
        -PENDULUM_INTEGRAL_LIMIT, PENDULUM_INTEGRAL_LIMIT);
    float pd = PENDULUM_KP * tiltError - PENDULUM_KD * lateralTiltRateDegS;
    float candidateOutput = PENDULUM_CORRECTION_DIRECTION *
        (pd + PENDULUM_KI * candidateIntegral);
    float lowerServo = SWING_CENTER_DEG - MAX_SWING_OFFSET_DEG;
    float upperServo = SWING_CENTER_DEG + MAX_SWING_OFFSET_DEG;
    float candidateServo = target + candidateOutput;
    bool saturated = candidateOutput < -PENDULUM_MAX_CORRECTION_DEG ||
                     candidateOutput > PENDULUM_MAX_CORRECTION_DEG ||
                     candidateServo < lowerServo || candidateServo > upperServo;
    if (!saturated) pendulumIntegral = candidateIntegral;

    pendulumCorrectionDeg = clampf(PENDULUM_CORRECTION_DIRECTION *
        (pd + PENDULUM_KI * pendulumIntegral),
        -PENDULUM_MAX_CORRECTION_DEG, PENDULUM_MAX_CORRECTION_DEG);
  }

  // Preserve arc feed-forward and apply bounded PID correction.
  float servoTarget = clampf(target + pendulumCorrectionDeg,
      SWING_CENTER_DEG - MAX_SWING_OFFSET_DEG,
      SWING_CENTER_DEG + MAX_SWING_OFFSET_DEG);
  currentSwing = moveToward(currentSwing, servoTarget,
      SWING_SLEW_RATE_DEG_PER_SEC * dt);
  setSwing(currentSwing);
}

void controlledStop(float dt) {
  currentPivot = 0.0f;
  steer(0.0f);
  commandDrive(0.0f, dt);
  commandSwing(SWING_CENTER_DEG, dt);
}

void navigationUpdate(float dt) {
  targetBodyTiltDeg = 0.0f;
  if (!haveFilteredTarget) {
    controlledStop(dt);
    navState = WAITING_FOR_PI;
    return;
  }

  float distance = filteredDistance;
  float angle = normalizeAngle(filteredAngle);

  if (distance <= HARD_MIN_DISTANCE_FT) {
    // Close target: halt navigation, but keep IMU rocking damping active.
    // controlledStop() calls commandDrive(0, dt), which computes and
    // actually sends the balance-corrected motor command every cycle.
    // Unlike emergencyStop(), it does not erase balance correction.
    navState = TOO_CLOSE;
    following = false;
    pivoting = false;
    controlledStop(dt);
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

  // Otherwise arc. Target a modest lean into the turn.
  navState = ARC;
  targetBodyTiltDeg = clampf(filteredAngle *
      ARC_BODY_LEAN_DEG_PER_TARGET_DEG * ARC_LEAN_DIRECTION,
      -MAX_ARC_BODY_LEAN_DEG, MAX_ARC_BODY_LEAN_DEG);
  currentPivot = 0.0f;
  steer(0.0f);
  commandSwing(calculateSwingTarget(angle), dt);
  commandDrive(calculateDriveSpeed(distance), dt);
}

// ============================================================
// STATUS
// ============================================================

void sendStatus() {
  Serial1.print("S=");
  Serial1.print(stateName(navState));
  Serial1.print(", Distance=");
  Serial1.print(filteredDistance, 2);
  Serial1.print(", Angle=");
  Serial1.print(filteredAngle, 2);
  Serial1.print(", Drive=");
  //Serial1.print(imuPitch, 2);
  //Serial1.print(", ");
  //Serial1.print(imuRoll, 2);
  //Serial1.print(", ");
  Serial1.print(currentDrive, 3);
  Serial1.print(", Swing=");
  Serial1.print(currentSwing, 1);
  Serial1.print(", Pivot=");
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
  Serial.print(" Applied=");
  Serial.print(actualDriveOutput, 2);
  Serial.print(" PitchRate=");
  Serial.print(filteredPitchRate, 1);
  Serial.print(" Swing=");
  Serial.print(currentSwing, 1);
  Serial.print(" TiltTarget=");
  Serial.print(commandedBodyTiltDeg, 1);
  Serial.print(" TiltRate=");
  Serial.print(lateralTiltRateDegS, 1);
  Serial.print(" PendCorr=");
  Serial.print(pendulumCorrectionDeg, 2);
  Serial.print(" Pivot=");
  Serial.print(currentPivot, 2);
  Serial.print(" HeadFB=");
  Serial.print(headFBCommand, 1);
  Serial.print(" HeadSTS=");
  Serial.print(headSTSCommand, 1);
  Serial.print(" Enabled=");
  Serial.print(systemEnabled);

  Serial.print(" IMU=");
  Serial.print(imuOK);

  Serial.print(" Fresh=");
  Serial.print(
    balanceImuFresh &&
    (millis() - lastBalanceImuMs <= BALANCE_IMU_TIMEOUT_MS)
  );

  Serial.print(" RateReady=");
  Serial.print(balanceRateReady);

  Serial.print(" Balance=");
  Serial.print(smoothedBalanceCorrection, 4);
  Serial.println();
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


  // SAFETY: motor PWM starts at zero; Pi owns the physical relays.
  analogWrite(DRIVE1_PWM_PIN, 0);
  analogWrite(DRIVE2_PWM_PIN, 0);
  analogWrite(PIVOT_PWM_PIN, 0);

  Wire.begin();

  pca.begin();
  pca.setPWMFreq(50);
  setSwing(SWING_CENTER_DEG);
  centerHead();

  // Start the head startup safety timer after its initial center command.
  headStartupMs = millis();
  lastHeadUpdateMs = headStartupMs;

  // BNO085 on default I2C address.
  imuOK = bno08x.begin_I2C();
  if (imuOK) {
    Serial.println("BNO085 detected.");
    setReports();
  } else {
    Serial.println("WARNING: BNO085 not detected. Motor control will still run without stabilization.");
  }

  navState = WAITING_FOR_PI;
  Serial.println(ENABLE_DRIVE_STABILIZATION ?
    "IMU drive stabilization ENABLED" : "IMU drive stabilization DISABLED (safety default)");

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
