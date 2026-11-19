/*
  EXPERIMENTAL PENDULUM FEEDBACK VERSION
  Derived from BB8_Teensy_Controller_RollBalance.ino.
  Verify ENABLE_PENDULUM_STABILIZATION below before powered testing.
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
constexpr uint8_t SERVO_DRIVER_I2C_ADDRESS = 0x40;
constexpr uint8_t PENDULUM_SERVO_1_CHANNEL = 0;
constexpr uint8_t PENDULUM_SERVO_2_CHANNEL = 4;

// Head servos (same PCA9685 channels as the original Pi controller)
constexpr uint8_t HEAD_FORWARD_BACK_CHANNEL  = 1;  // head forward/back
constexpr uint8_t HEAD_SIDE_TO_SIDE_CHANNEL = 2;  // head side-to-side

// ============================================================
// CONFIGURATION
// ============================================================

// Serial link speeds (bits per second).
constexpr uint32_t PI_BAUD = 115200;
constexpr uint32_t USB_BAUD = 115200;
constexpr uint8_t IMU_I2C_ADDRESSES[] = {0x4A, 0x4B};
constexpr uint32_t IMU_INIT_RETRY_MS = 2000;

// Following distance thresholds (feet); separate start/stop values prevent chatter.
constexpr float FOLLOW_START_DISTANCE_FT = 5.0f;
constexpr float FOLLOW_STOP_DISTANCE_FT  = 4.0f;
constexpr float HARD_MIN_DISTANCE_FT     = 3.0f;
constexpr float MAX_VALID_DISTANCE_FT    = 40.0f;

// Drive commands are signed PWM fractions, not measured wheel speeds.
constexpr float MAX_DRIVE_COMMAND = 0.30f;
constexpr float MIN_DRIVE_COMMAND = 0.12f;
constexpr float FULL_SPEED_DISTANCE_FT = 18.0f;
constexpr float DRIVE_ACCEL_COMMAND_PER_SEC = 0.25f; // Gentler drive speed ramp (PWM fraction/second)
constexpr float DRIVE_DECEL_COMMAND_PER_SEC = 0.70f;

// Forward/back rocking damping through the drive motors (body pitch).
// IMU pitch stabilization. Enable only after verifying correction direction
// with the robot securely supported and drive wheels clear of the ground.
// SAFETY: Leave false until drive wheels are raised and feedback direction verified.
// After successful testing, change to true to activate idle and moving damping.
constexpr bool ENABLE_DRIVE_STABILIZATION = false;
constexpr float BALANCE_DIRECTION = -1.0f; // Correction polarity; reverse sign if feedback reinforces rocking
constexpr float BALANCE_PITCH_ZERO_DEG = 0.0f; // Calibrate at physical rest
constexpr float BALANCE_KP = 0.2f; // PWM fraction / degree
constexpr float BALANCE_KD = 0.0015f; // PWM fraction / (degree/second)
constexpr float BALANCE_MAX_IDLE_CORRECTION = 0.10f;
constexpr float BALANCE_MAX_MOVING_CORRECTION = 0.06f;
constexpr float BALANCE_DEADBAND_DEG = 0.6f;
constexpr float BALANCE_MAX_TILT_DEG = 20.0f;
constexpr uint32_t BALANCE_IMU_TIMEOUT_MS = 100;
constexpr float BALANCE_RATE_FILTER_ALPHA = 0.25f;
// Additional output filtering prevents abrupt PWM changes due to IMU noise.
constexpr float BALANCE_CORRECTION_COMMAND_PER_SEC = 0.40f;
constexpr float BALANCE_MOVING_TILT_KP_SCALE = 0.45f;
constexpr float BALANCE_MOVING_RATE_KD_SCALE = 1.0f;
constexpr float BALANCE_IDLE_RATE_KD_SCALE = 1.0f;
// Minimum pitch magnitude before the idle proportional term activates.
constexpr float BALANCE_IDLE_ACTIVATION_DEG = 0.4f;


// Navigation mode thresholds: target bearing in degrees and range in feet.
constexpr float ANGLE_DEADBAND_DEG = 6.0f;
constexpr float CLOSE_PIVOT_START_ANGLE_DEG = 30.0f;
constexpr float PIVOT_STOP_ANGLE_DEG = 10.0f;
constexpr float REAR_PIVOT_ANGLE_DEG = 65.0f;
constexpr float CLOSE_NAV_DISTANCE_FT = 10.0f;

// Pendulum feed-forward steering: servo angles and motion limits.
constexpr float SWING_CENTER_DEG = 90.0f; // Calibrate mechanical center with feedback OFF
constexpr float MAX_SWING_OFFSET_DEG = 14.0f;
constexpr float SWING_SERVO_DEG_PER_TARGET_DEG = 0.35f;
constexpr float SWING_DIRECTION = 1.0f;
constexpr float MIN_SWING_OFFSET_DEG = 1.5f;
constexpr float SWING_SLEW_RATE_DEG_PER_SEC = 35.0f;

// Closed-loop lateral body tilt control using the pendulum servos.
// Body coordinates: bodyPitchDeg = forward/back; bodyRollDeg = left/right.
// Keep false until servo correction polarity is verified while supported.
constexpr bool ENABLE_PENDULUM_STABILIZATION = true;
constexpr float PENDULUM_TILT_ZERO_DEG = 0.0f;
// Target body lean in ARC: signed beacon angle * gain, limited to safe lean.
constexpr float ARC_BODY_LEAN_DEG_PER_TARGET_DEG = 0.08f;
constexpr float MAX_ARC_BODY_LEAN_DEG = 20.0f;
constexpr float ARC_LEAN_DIRECTION = 1.0f; // Verify inside-of-turn direction
constexpr float BODY_TILT_TARGET_SLEW_DEG_PER_SEC = 90.0f;
// PID output is a servo-angle offset in degrees.
constexpr float PENDULUM_KP = 0.10f; // servo deg / body tilt deg
constexpr float PENDULUM_KD = 0.00f; // servo deg / (body tilt deg/s)
constexpr float PENDULUM_KI = 0.00f; // servo deg / (body tilt deg * second)
constexpr float PENDULUM_INTEGRAL_LIMIT_DEG_SEC = 100.0f; // deg * seconds
constexpr float PENDULUM_INTEGRAL_RESET_TARGET_CHANGE_DEG = 1.0f;
constexpr float PENDULUM_CORRECTION_DIRECTION = 1.0f; // verify physical sign
constexpr float PENDULUM_MAX_CORRECTION_DEG = 4.0f;
constexpr float PENDULUM_TILT_RATE_FILTER_ALPHA = 0.45f;
constexpr float PENDULUM_MAX_VALID_TILT_DEG = 30.0f;
constexpr uint32_t PENDULUM_IMU_TIMEOUT_MS = 100;


// Reaction-wheel pivot: full-command kick, gradual rundown, then zero-command rest.
// These are PWM commands; without an encoder they do not measure wheel speed.
constexpr float MAX_PIVOT_COMMAND = 1.0f;
constexpr float PIVOT_DIRECTION = -1.0f;
constexpr uint32_t PIVOT_KICK_DURATION_MS = 150;
constexpr uint32_t PIVOT_RAMP_DOWN_MS = 300;
constexpr uint32_t PIVOT_REST_DURATION_MS = 200;

// Target smoothing: 0 favors history; 1 uses only the newest sample.
constexpr float DISTANCE_FILTER_ALPHA = 0.35f;
constexpr float ANGLE_FILTER_ALPHA = 0.40f;

// Communication watchdog and periodic task intervals (units in names).
constexpr uint32_t PI_TIMEOUT_MS = 500;
constexpr uint32_t CONTROL_PERIOD_US = 20000; // 50 Hz
constexpr uint32_t STATUS_PERIOD_MS = 250;

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
constexpr float HEAD_FORWARD_BACK_OFFSET_DEG = 8.0f;
constexpr float HEAD_SIDE_TO_SIDE_OFFSET_DEG = -10.0f;
constexpr float HEAD_TARGET_FILTER_TIME_SEC = 0.10f; // Half the filter delay for twice-as-fast leveling
constexpr float HEAD_LEVELING_DEADBAND_DEG = 0.3f; // Ignore small errors that cause servo hunting
constexpr float HEAD_LEVELING_RESPONSE_PER_SEC = 8.0f; // Twice the damped response rate
constexpr float HEAD_LEVELING_ACCEL_DEG_PER_SEC2 = 160.0f; // 4x acceleration preserves the profile at 2x speed
constexpr uint32_t HEAD_IMU_TIMEOUT_MS = 500; // Neck tolerates short gaps independently of drive damping

// Startup safety: do not immediately command the head to the IMU angle.
// Wait briefly, then ramp the commanded head angles toward level at a
// bounded speed. This prevents a sudden snap when the Teensy powers up.
constexpr uint32_t HEAD_STARTUP_HOLD_MS = 5000;
constexpr float HEAD_LEVELING_SLEW_DEG_PER_SEC = 40.0f; // Twice the previous loaded-neck speed limit

constexpr float HEAD_PITCH_DIRECTION = -1.0f; // Head servo direction independent of IMU coordinates
constexpr float HEAD_ROLL_DIRECTION  = 1.0f;

// ============================================================
// HARDWARE INTERFACES AND RUNTIME STATE
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

// Hardware interfaces.
Adafruit_PWMServoDriver servoDriver(SERVO_DRIVER_I2C_ADDRESS);
Adafruit_BNO08x bodyImu(-1);
sh2_SensorValue_t imuSensorEvent;

// Body orientation in degrees: pitch = forward/back, roll = left/right.
bool imuAvailable = false;
uint32_t lastImuInitAttemptMs = 0;
float bodyPitchDeg = 0.0f;
float bodyRollDeg = 0.0f;
float bodyYawDeg = 0.0f;

// Navigation mode and following hysteresis; Pi owns physical relays.
NavState navigationState = STARTUP;
bool isFollowingTarget = false;
bool isPivoting = false;
bool navigationEnabled = true;

// Filtered beacon target: range in feet, signed bearing in degrees.
bool targetFilterInitialized = false;
float filteredTargetDistanceFt = 0.0f;
float filteredTargetAngleDeg = 0.0f;

// Drive and pivot outputs: signed PWM fractions (-1 to +1).
float rampedDriveCommand = 0.0f; // Navigation demand after acceleration/deceleration ramp
float appliedDriveCommand = 0.0f; // Ramped demand plus pitch correction, clamped to drive limits
float pivotMotorCommand = 0.0f;

enum PivotPulsePhase { PIVOT_IDLE, PIVOT_KICK, PIVOT_RAMP_DOWN, PIVOT_REST };
PivotPulsePhase pivotPulsePhase = PIVOT_IDLE;
uint32_t pivotPhaseStartedMs = 0;
float pivotPulseDirection = 0.0f;
float pivotRampStartMagnitude = 0.0f;

// Pitch damping: PWM correction, degrees/second rate, sample timestamps.
float smoothedPitchCorrectionCommand = 0.0f;
float filteredPitchRateDegPerSec = 0.0f;
float previousPitchDeg = 0.0f;
uint32_t lastPitchUpdateMs = 0;
uint32_t lastPitchSampleUs = 0;
bool pitchRateReady = true;
bool hasPitchSample = false; // Freshness also requires checking lastPitchUpdateMs

// Pendulum steering: servo position and body-roll targets in degrees.
float pendulumServoCommandDeg = SWING_CENTER_DEG;
float targetBodyRollDeg = 0.0f;
float rampedBodyRollTargetDeg = 0.0f;

// Roll feedback: degrees/second rate and IMU sample timestamps.
float filteredRollRateDegPerSec = 0.0f;
float previousRollDeg = 0.0f;
uint32_t lastRollSampleUs = 0;
uint32_t lastRollUpdateMs = 0;
bool rollRateReady = false;

// Pendulum PID memory: servo correction (deg), integral (deg * seconds).
float pendulumFeedbackCorrectionDeg = 0.0f;
float rollErrorIntegralDegSec = 0.0f;
float previousBodyRollTargetDeg = 0.0f;
bool pendulumFeedbackWasActive = false;

// Head leveling: body pitch/roll commands in degrees, before output mapping.
float headPitchCommandDeg = HEAD_CENTER_DEG;
float headRollCommandDeg = HEAD_CENTER_DEG;
float filteredHeadPitchTargetDeg = HEAD_CENTER_DEG;
float filteredHeadRollTargetDeg = HEAD_CENTER_DEG;
float headPitchVelocityDegPerSec = 0.0f;
float headRollVelocityDegPerSec = 0.0f;
bool headFilterInitialized = false;
bool hasHeadOrientationSample = false;
uint32_t lastHeadOrientationMs = 0;
const char *headLevelingStatus = "STARTUP";
uint32_t headStartupMs = 0;
uint32_t lastHeadUpdateMs = 0;

// Task timing: milliseconds (Ms) or microseconds (Us).
uint32_t lastPiCommandMs = 0;
uint32_t lastControlUpdateUs = 0;
uint32_t lastStatusSentMs = 0;

// Pi UART receive buffer: partial line and number of buffered characters.
char piReceiveBuffer[80];
size_t piReceiveLength = 0;
bool piDiscardingLine = false;

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
  command = clampf(command, -MAX_DRIVE_COMMAND, MAX_DRIVE_COMMAND);
  setPhaseEnableMotor(DRIVE1_PHASE_PIN, DRIVE1_PWM_PIN, -command);
  setPhaseEnableMotor(DRIVE2_PHASE_PIN, DRIVE2_PWM_PIN, -command);
}

void steer(float command) {
  command = clampf(command, -1.0f, 1.0f);
  if (fabsf(command) < 0.01f) command = 0.0f;
  setPhaseEnableMotor(PIVOT_PHASE_PIN, PIVOT_PWM_PIN, command);
}

void stopPivotPulse() {
  pivotPulsePhase = PIVOT_IDLE;
  pivotPhaseStartedMs = 0;
  pivotPulseDirection = 0.0f;
  pivotRampStartMagnitude = 0.0f;
  pivotMotorCommand = 0.0f;
  steer(0.0f);
}

void commandPivotPulse(float targetAngleDeg) {
  if (!navigationEnabled || !isfinite(targetAngleDeg) ||
      fabsf(targetAngleDeg) <= PIVOT_STOP_ANGLE_DEG) {
    stopPivotPulse();
    return;
  }

  const uint32_t nowMs = millis();
  const float requestedDirection = copysignf(1.0f, targetAngleDeg * PIVOT_DIRECTION);
  if (pivotPulsePhase == PIVOT_IDLE) {
    pivotPulseDirection = requestedDirection;
    pivotPulsePhase = PIVOT_KICK;
    pivotPhaseStartedMs = nowMs;
  }

  // A changed turn direction first runs the old pulse down and rests at zero.
  // Never switch directly from a full-power kick to opposite motor polarity.
  if (pivotPulsePhase == PIVOT_KICK &&
      (requestedDirection != pivotPulseDirection ||
       (uint32_t)(nowMs - pivotPhaseStartedMs) >= PIVOT_KICK_DURATION_MS)) {
    pivotRampStartMagnitude = fabsf(pivotMotorCommand);
    pivotPulsePhase = PIVOT_RAMP_DOWN;
    pivotPhaseStartedMs = nowMs;
  }

  switch (pivotPulsePhase) {
    case PIVOT_KICK:
      pivotMotorCommand = pivotPulseDirection * MAX_PIVOT_COMMAND;
      break;
    case PIVOT_RAMP_DOWN: {
      const uint32_t elapsedMs = (uint32_t)(nowMs - pivotPhaseStartedMs);
      if (elapsedMs >= PIVOT_RAMP_DOWN_MS) {
        pivotMotorCommand = 0.0f;
        pivotPulsePhase = PIVOT_REST;
        pivotPhaseStartedMs = nowMs;
      } else {
        pivotMotorCommand = pivotPulseDirection * pivotRampStartMagnitude *
            (1.0f - (float)elapsedMs / (float)PIVOT_RAMP_DOWN_MS);
      }
      break;
    }
    case PIVOT_REST:
      pivotMotorCommand = 0.0f;
      if ((uint32_t)(nowMs - pivotPhaseStartedMs) >= PIVOT_REST_DURATION_MS) {
        pivotPulseDirection = requestedDirection;
        pivotPulsePhase = PIVOT_KICK;
        pivotPhaseStartedMs = nowMs;
        pivotMotorCommand = pivotPulseDirection * MAX_PIVOT_COMMAND;
      }
      break;
    default:
      stopPivotPulse();
      return;
  }
  steer(pivotMotorCommand);
}

uint16_t servoPulseFromDegrees(float degrees) {
  degrees = clampf(degrees, 0.0f, 180.0f);
  float us = SERVO_MIN_US +
             (degrees / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);

  // PCA9685 at 50 Hz => 20,000 us period / 4096 counts.
  return (uint16_t)roundf(us * 4096.0f / 20000.0f);
}

void setServoDegrees(uint8_t channel, float degrees) {
  servoDriver.setPWM(channel, 0, servoPulseFromDegrees(degrees));
}

void setHeadForwardBack(float degrees) {
  degrees = clampf(degrees, HEAD_MIN_DEG, HEAD_MAX_DEG);
  setServoDegrees(HEAD_FORWARD_BACK_CHANNEL, degrees + HEAD_FORWARD_BACK_OFFSET_DEG);
}

void setHeadSideToSide(float degrees) {
  degrees = clampf(degrees, HEAD_MIN_DEG, HEAD_MAX_DEG);
  setServoDegrees(HEAD_SIDE_TO_SIDE_CHANNEL, degrees + HEAD_SIDE_TO_SIDE_OFFSET_DEG);
}

void centerHead() {
  headPitchCommandDeg = HEAD_CENTER_DEG;
  headRollCommandDeg = HEAD_CENTER_DEG;
  filteredHeadPitchTargetDeg = HEAD_CENTER_DEG;
  filteredHeadRollTargetDeg = HEAD_CENTER_DEG;
  headPitchVelocityDegPerSec = 0.0f;
  headRollVelocityDegPerSec = 0.0f;
  headFilterInitialized = false;
  setHeadForwardBack(HEAD_CENTER_DEG);
  setHeadSideToSide(HEAD_CENTER_DEG);
}

void smoothHeadAxis(float targetDeg, float &commandDeg,
                    float &velocityDegPerSec, float dtSec) {
  // A damped spring eases both position and velocity. Substeps keep the
  // response stable if a control cycle runs late.
  while (dtSec > 0.0f) {
    const float stepSec = min(dtSec, 0.01f);
    float errorDeg = targetDeg - commandDeg;
    if (fabsf(errorDeg) <= HEAD_LEVELING_DEADBAND_DEG) {
      errorDeg = 0.0f;
    } else {
      errorDeg -= copysignf(HEAD_LEVELING_DEADBAND_DEG, errorDeg);
    }
    const float response = HEAD_LEVELING_RESPONSE_PER_SEC;
    float acceleration = response * response * errorDeg -
                         2.0f * response * velocityDegPerSec;
    acceleration = clampf(acceleration, -HEAD_LEVELING_ACCEL_DEG_PER_SEC2,
                          HEAD_LEVELING_ACCEL_DEG_PER_SEC2);
    velocityDegPerSec = clampf(velocityDegPerSec + acceleration * stepSec,
        -HEAD_LEVELING_SLEW_DEG_PER_SEC, HEAD_LEVELING_SLEW_DEG_PER_SEC);
    commandDeg += velocityDegPerSec * stepSec;
    // Mechanical travel limits take precedence over the motion profile.
    if (commandDeg < HEAD_MIN_DEG || commandDeg > HEAD_MAX_DEG) {
      commandDeg = clampf(commandDeg, HEAD_MIN_DEG, HEAD_MAX_DEG);
      velocityDegPerSec = 0.0f;
    }
    dtSec -= stepSec;
  }
}

void updateHeadLeveling() {
  uint32_t now = millis();
  // Head freshness comes directly from valid orientation events, independent
  // of the drive/pendulum derivative estimators. Report why leveling is held.
  if (!imuAvailable) headLevelingStatus = "NO_IMU";
  else if (!hasHeadOrientationSample) headLevelingStatus = "WAIT_IMU";
  else if (!isfinite(bodyPitchDeg) || !isfinite(bodyRollDeg)) headLevelingStatus = "INVALID_IMU";
  else if ((uint32_t)(now - lastHeadOrientationMs) > HEAD_IMU_TIMEOUT_MS)
    headLevelingStatus = "STALE_IMU";
  else headLevelingStatus = "LEVELING";

  if (strcmp(headLevelingStatus, "LEVELING") != 0) {
    headPitchVelocityDegPerSec = 0.0f;
    headRollVelocityDegPerSec = 0.0f;
    // Keep filter progress across packet gaps. Repeatedly resetting it to
    // the servo position can prevent small commands from ever building up.
    lastHeadUpdateMs = now;
    return;
  }

  // Give the person powering up BB-8 time to move their hands away.
  // During this hold the head remains at the startup command.
  if ((uint32_t)(now - headStartupMs) < HEAD_STARTUP_HOLD_MS) {
    headLevelingStatus = "STARTUP";
    lastHeadUpdateMs = now;
    return;
  }

  float controlDtSec = 0.02f;
  if (lastHeadUpdateMs != 0) {
    controlDtSec = (now - lastHeadUpdateMs) / 1000.0f;
    controlDtSec = clampf(controlDtSec, 0.001f, 0.10f);
  }
  lastHeadUpdateMs = now;

  // Body-frame axes: forward/back uses pitch; side-to-side uses roll.
  // Verify physical servo polarity with motor power disabled.
  float targetFB = HEAD_CENTER_DEG + HEAD_PITCH_DIRECTION * bodyPitchDeg;
  float targetSTS = HEAD_CENTER_DEG + HEAD_ROLL_DIRECTION * bodyRollDeg;

  targetFB = clampf(targetFB, HEAD_MIN_DEG, HEAD_MAX_DEG);
  targetSTS = clampf(targetSTS, HEAD_MIN_DEG, HEAD_MAX_DEG);

  if (!headFilterInitialized) {
    filteredHeadPitchTargetDeg = headPitchCommandDeg;
    filteredHeadRollTargetDeg = headRollCommandDeg;
    headFilterInitialized = true;
  }
  // Filter the target independently of the output; the time constant keeps
  // smoothing consistent even when control loop timing varies.
  const float alpha = 1.0f - expf(-controlDtSec / HEAD_TARGET_FILTER_TIME_SEC);
  filteredHeadPitchTargetDeg += alpha * (targetFB - filteredHeadPitchTargetDeg);
  filteredHeadRollTargetDeg += alpha * (targetSTS - filteredHeadRollTargetDeg);
  smoothHeadAxis(filteredHeadPitchTargetDeg, headPitchCommandDeg,
                 headPitchVelocityDegPerSec, controlDtSec);
  smoothHeadAxis(filteredHeadRollTargetDeg, headRollCommandDeg,
                 headRollVelocityDegPerSec, controlDtSec);

  // Preserve the existing physical channel mapping on this neck assembly.
  setHeadForwardBack(headRollCommandDeg);
  setHeadSideToSide(headPitchCommandDeg);
}

void setSwing(float degrees) {
  // Preserve original Movement_Functions behavior:
  // difference = current_swing - 90 + swing_offset
  // servo1 = 90 + difference
  // servo2 = 90 - difference
  constexpr float swingOffset = -1.0f;

  degrees = clampf(degrees, 70.0f, 117.0f);
  float difference = degrees - 90.0f + swingOffset;

  setServoDegrees(PENDULUM_SERVO_1_CHANNEL, 90.0f + difference);
  setServoDegrees(PENDULUM_SERVO_2_CHANNEL, 90.0f - difference);
}

// This flag arms/disarms the motor-control algorithm only.
// Physical relay power is controlled exclusively by the Body Pi.
void enableSystem() {
  if (navigationEnabled) return;
  rampedDriveCommand = 0.0f;
  appliedDriveCommand = 0.0f;
  smoothedPitchCorrectionCommand = 0.0f;
  stopPivotPulse();
  drive(0.0f);
  setSwing(SWING_CENTER_DEG);
  navigationEnabled = true;
}

void disableSystem() {
  drive(0.0f);
  stopPivotPulse();
  setSwing(SWING_CENTER_DEG);
  rampedDriveCommand = 0.0f;
  appliedDriveCommand = 0.0f;
  smoothedPitchCorrectionCommand = 0.0f;
  targetBodyRollDeg = 0.0f;
  rampedBodyRollTargetDeg = 0.0f;
  pendulumFeedbackCorrectionDeg = 0.0f;
  rollErrorIntegralDegSec = 0.0f;
  pendulumFeedbackWasActive = false;
  navigationEnabled = false;
}

void emergencyStop(NavState reason) {
  rampedDriveCommand = 0.0f;
  appliedDriveCommand = 0.0f;
  smoothedPitchCorrectionCommand = 0.0f;
  stopPivotPulse();
  pendulumServoCommandDeg = SWING_CENTER_DEG;
  targetBodyRollDeg = 0.0f;
  rampedBodyRollTargetDeg = 0.0f;
  pendulumFeedbackCorrectionDeg = 0.0f;
  rollErrorIntegralDegSec = 0.0f;
  pendulumFeedbackWasActive = false;

  drive(0.0f);
  setSwing(SWING_CENTER_DEG);

  isFollowingTarget = false;
  isPivoting = false;
  navigationState = reason;
}

// ============================================================
// BNO085
// ============================================================

bool setReports() {
  if (!imuAvailable) return false;

  if (!bodyImu.enableReport(SH2_ROTATION_VECTOR, 10000)) {
    Serial.println("WARNING: Could not enable BNO085 rotation vector");
    return false;
  }
  return true;
}

void initializeIMU() {
  lastImuInitAttemptMs = millis();
  imuAvailable = false;
  hasHeadOrientationSample = false;
  hasPitchSample = false;
  pitchRateReady = false;
  rollRateReady = false;
  lastPitchSampleUs = 0;
  lastRollSampleUs = 0;
  filteredPitchRateDegPerSec = 0.0f;
  filteredRollRateDegPerSec = 0.0f;

  for (uint8_t address : IMU_I2C_ADDRESSES) {
    // Probe first: distinguish a missing device from a failed SH-2 handshake.
    Wire.beginTransmission(address);
    const uint8_t busResult = Wire.endTransmission();
    Serial.print("BNO085 probe 0x");
    Serial.print(address, HEX);
    Serial.print(" I2C result=");
    Serial.println(busResult);
    if (busResult != 0) continue;

    if (!bodyImu.begin_I2C(address, &Wire)) {
      Serial.println("WARNING: Address responds, but BNO085 initialization failed.");
      continue;
    }
    imuAvailable = true;
    if (!setReports()) {
      imuAvailable = false;
      continue;
    }
    Serial.print("BNO085 ready at 0x");
    Serial.println(address, HEX);
    return;
  }
  Serial.println("WARNING: No usable BNO085. Check IMU power, GND, SDA/SCL and interface mode. Retrying while navigation is disabled.");
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
  if (!imuAvailable) {
    // Library initialization can block. Retry only with navigation disabled
    // and motor outputs zero, so it cannot delay an active motion watchdog.
    if (!navigationEnabled && fabsf(appliedDriveCommand) < 0.001f &&
        fabsf(pivotMotorCommand) < 0.001f &&
        (uint32_t)(millis() - lastImuInitAttemptMs) >= IMU_INIT_RETRY_MS) {
      initializeIMU();
    }
    return;
  }

  if (bodyImu.wasReset()) {
    setReports();
  }

  while (bodyImu.getSensorEvent(&imuSensorEvent)) {
    if (imuSensorEvent.sensorId == SH2_ROTATION_VECTOR) {
      
      // BNO085 rotated 90 degrees clockwise on new PCB.
      // Compensate for the mounting rotation.

      const float c = 0.70710678118f; // cos(45 degrees)
      const float s = 0.70710678118f; // sin(45 degrees)

      float qr = imuSensorEvent.un.rotationVector.real;
      float qi = imuSensorEvent.un.rotationVector.i;
      float qj = imuSensorEvent.un.rotationVector.j;
      float qk = imuSensorEvent.un.rotationVector.k;

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
                        bodyYawDeg, sensorPitchDeg, sensorRollDeg);
      bodyPitchDeg = sensorRollDeg;  // forward/back body tilt
      bodyRollDeg  = sensorPitchDeg; // left/right body tilt

      if (isfinite(bodyPitchDeg) && isfinite(bodyRollDeg)) {
        hasHeadOrientationSample = true;
        lastHeadOrientationMs = millis();
      }

      // Lateral body tilt and angular rate for pendulum feedback.
      // Lateral roll rate for pendulum control.
      if (isfinite(bodyRollDeg)) {
        uint32_t tiltNowUs = micros();
        if (lastRollSampleUs != 0) {
          float sampleDt = (uint32_t)(tiltNowUs - lastRollSampleUs) * 1.0e-6f;
          if (sampleDt >= 0.002f && sampleDt <= 0.2f) {
            float rawRate = (bodyRollDeg - previousRollDeg) / sampleDt;
            if (isfinite(rawRate)) {
              filteredRollRateDegPerSec += PENDULUM_TILT_RATE_FILTER_ALPHA *
                                     (rawRate - filteredRollRateDegPerSec);
              rollRateReady = true;
            }
          } else {
            rollRateReady = false;
            filteredRollRateDegPerSec = 0.0f;
          }
        }
        previousRollDeg = bodyRollDeg;
        lastRollSampleUs = tiltNowUs;
        lastRollUpdateMs = millis();
      }

      if (isfinite(bodyPitchDeg)) {
        uint32_t sampleUs = micros();
        if (lastPitchSampleUs != 0) {
          float sampleDt = (uint32_t)(sampleUs - lastPitchSampleUs) * 1.0e-6f;
          if (sampleDt >= 0.002f && sampleDt <= 0.1f) {
            float rawRate = (bodyPitchDeg - previousPitchDeg) / sampleDt;
            if (fabsf(rawRate) < 1000.0f) {
              filteredPitchRateDegPerSec += BALANCE_RATE_FILTER_ALPHA * (rawRate - filteredPitchRateDegPerSec);
              pitchRateReady = true;
            }
          } else {
            pitchRateReady = false;
            filteredPitchRateDegPerSec = 0.0f;
          }
        }
        previousPitchDeg = bodyPitchDeg;
        lastPitchSampleUs = sampleUs;
        lastPitchUpdateMs = millis();
        hasPitchSample = true;
      }
    }
  }
}

// ============================================================
// TARGET FILTER
// ============================================================

void resetTargetFilter() {
  targetFilterInitialized = false;
}

void filterTarget(float distance, float angle) {
  angle = normalizeAngle(angle);

  if (!targetFilterInitialized) {
    filteredTargetDistanceFt = distance;
    filteredTargetAngleDeg = angle;
    targetFilterInitialized = true;
    return;
  }

  filteredTargetDistanceFt =
      DISTANCE_FILTER_ALPHA * distance +
      (1.0f - DISTANCE_FILTER_ALPHA) * filteredTargetDistanceFt;

  float error = normalizeAngle(angle - filteredTargetAngleDeg);

  filteredTargetAngleDeg = normalizeAngle(
      filteredTargetAngleDeg + ANGLE_FILTER_ALPHA * error
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

    // A released stick (T,0,0) must cancel a pulse immediately, rather than
    // letting a previously distant/sideways target decay through the EMA.
    if (distance <= HARD_MIN_DISTANCE_FT) {
      emergencyStop(TOO_CLOSE);
      resetTargetFilter();
    } else if (isPivoting && fabsf(normalizeAngle(angle)) <= PIVOT_STOP_ANGLE_DEG) {
      // New alignment/straight command supersedes the old filtered bearing.
      stopPivotPulse();
      isPivoting = false;
      filteredTargetAngleDeg = normalizeAngle(angle);
    }
    filterTarget(distance, angle);
    lastPiCommandMs = millis();

    // Enable navigation after first valid command; Pi owns physical relays.
    if (!navigationEnabled) enableSystem();

    return;
  }

  // Pause movement without changing Pi-controlled relays.
  if (strcmp(line, "PAUSE") == 0) {
      emergencyStop(WAITING_FOR_PI);

      // Throw away the old target so BB-8 cannot resume
      // toward stale beacon data.
      resetTargetFilter();

      // Keep navigation armed. The Pi decides whether to keep relays on.
      if (!navigationEnabled) {
          enableSystem();
      }

      // PAUSE counts as valid Pi communication.
      lastPiCommandMs = millis();

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
  // Targets describe the current control position, not motions to queue.
  // Apply only the newest complete target buffered during this pass.
  char latestTarget[sizeof(piReceiveBuffer)];
  bool haveLatestTarget = false;
  while (Serial1.available()) {
    char c = (char)Serial1.read();

    if (c == '\r') continue;

    if (c == '\n') {
      if (!piDiscardingLine && piReceiveLength > 0) {
        piReceiveBuffer[piReceiveLength] = '\0';
        if (piReceiveBuffer[0] == 'T' && piReceiveBuffer[1] == ',') {
          memcpy(latestTarget, piReceiveBuffer, piReceiveLength + 1);
          haveLatestTarget = true;
        } else {
          // Stop/pause are barriers: no earlier buffered target may re-arm
          // navigation after them. A later target is a new explicit request.
          if (strcmp(piReceiveBuffer, "STOP") == 0 ||
              strcmp(piReceiveBuffer, "PAUSE") == 0) {
            haveLatestTarget = false;
          }
          handleCommand(piReceiveBuffer);
        }
      }
      piReceiveLength = 0;
      piDiscardingLine = false;
      continue;
    }

    if (piDiscardingLine) continue;
    if (piReceiveLength < sizeof(piReceiveBuffer) - 1) {
      piReceiveBuffer[piReceiveLength++] = c;
    } else {
      // Discard the whole oversized line, including any command-like tail.
      piReceiveLength = 0;
      piDiscardingLine = true;
    }
  }
  if (haveLatestTarget) handleCommand(latestTarget);
}

// ============================================================
// NAVIGATION
// ============================================================

float calculateDriveSpeed(float distance) {
  if (distance <= FOLLOW_STOP_DISTANCE_FT) return 0.0f;
  if (distance >= FULL_SPEED_DISTANCE_FT) return MAX_DRIVE_COMMAND;

  float progress =
      (distance - FOLLOW_STOP_DISTANCE_FT) /
      (FULL_SPEED_DISTANCE_FT - FOLLOW_STOP_DISTANCE_FT);

  float speed =
      MIN_DRIVE_COMMAND +
      progress * (MAX_DRIVE_COMMAND - MIN_DRIVE_COMMAND);

  return clampf(speed, MIN_DRIVE_COMMAND, MAX_DRIVE_COMMAND);
}

float calculateSwingTarget(float angle) {
  if (fabsf(angle) <= ANGLE_DEADBAND_DEG)
    return SWING_CENTER_DEG;

  float offset = angle * SWING_SERVO_DEG_PER_TARGET_DEG * SWING_DIRECTION;
  offset = clampf(offset, -MAX_SWING_OFFSET_DEG,
                  MAX_SWING_OFFSET_DEG);

  if (fabsf(offset) > 0.0f &&
      fabsf(offset) < MIN_SWING_OFFSET_DEG) {
    offset = copysignf(MIN_SWING_OFFSET_DEG, offset);
  }

  return SWING_CENTER_DEG + offset;
}

// Forward/backward rocking uses body pitch.
// Damping in idle and during forward driving. This does NOT measure speed.
// Returns zero if IMU data are stale or tilt is outside the safe control range.
float calculateBalanceCorrection(float target) {
  if (!ENABLE_DRIVE_STABILIZATION || !navigationEnabled || !imuAvailable ||
      !hasPitchSample || !pitchRateReady ||
      (uint32_t)(millis() - lastPitchUpdateMs) > BALANCE_IMU_TIMEOUT_MS)
    return 0.0f;

  float tilt = bodyPitchDeg - BALANCE_PITCH_ZERO_DEG;
  if (!isfinite(tilt) || !isfinite(filteredPitchRateDegPerSec) ||
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
  float correction = BALANCE_DIRECTION * (kp * p + kd * filteredPitchRateDegPerSec);
  float limit = moving ? BALANCE_MAX_MOVING_CORRECTION : BALANCE_MAX_IDLE_CORRECTION;
  return clampf(correction, -limit, limit);
}

void commandDrive(float target, float controlDtSec) {
  target = clampf(target, -MAX_DRIVE_COMMAND, MAX_DRIVE_COMMAND);

  float rate = fabsf(target) > fabsf(rampedDriveCommand)
                   ? DRIVE_ACCEL_COMMAND_PER_SEC : DRIVE_DECEL_COMMAND_PER_SEC;
  rampedDriveCommand = moveToward(rampedDriveCommand, target, rate * controlDtSec);

  // Use actual ramped command to select idle vs driving control.
  // Balance angle and its derivative both come from body pitch.
  float desiredCorrection = calculateBalanceCorrection(rampedDriveCommand);
  // Immediately drop correction on disabled/stale IMU or unsafe tilt.
  bool correctionValid = ENABLE_DRIVE_STABILIZATION && navigationEnabled && imuAvailable &&
      hasPitchSample && pitchRateReady &&
      (uint32_t)(millis() - lastPitchUpdateMs) <= BALANCE_IMU_TIMEOUT_MS &&
      isfinite(bodyPitchDeg) && isfinite(filteredPitchRateDegPerSec) &&
      fabsf(bodyPitchDeg - BALANCE_PITCH_ZERO_DEG) <= BALANCE_MAX_TILT_DEG;
  if (!correctionValid) {
    smoothedPitchCorrectionCommand = 0.0f;
  } else {
    smoothedPitchCorrectionCommand = moveToward(smoothedPitchCorrectionCommand,
        desiredCorrection, BALANCE_CORRECTION_COMMAND_PER_SEC * controlDtSec);
  }
  appliedDriveCommand = clampf(rampedDriveCommand + smoothedPitchCorrectionCommand,
                              -MAX_DRIVE_COMMAND, MAX_DRIVE_COMMAND);
  drive(appliedDriveCommand);
}

// target is the existing feed-forward pendulum servo position.
// The target body lean is chosen separately from the ARC state.
void commandSwing(float target, float controlDtSec) {
  target = clampf(target,
      SWING_CENTER_DEG - MAX_SWING_OFFSET_DEG,
      SWING_CENTER_DEG + MAX_SWING_OFFSET_DEG);

  rampedBodyRollTargetDeg = moveToward(rampedBodyRollTargetDeg,
      targetBodyRollDeg, BODY_TILT_TARGET_SLEW_DEG_PER_SEC * controlDtSec);

  pendulumFeedbackCorrectionDeg = 0.0f;
  bool imuValid = imuAvailable && rollRateReady &&
      (uint32_t)(millis() - lastRollUpdateMs) <= PENDULUM_IMU_TIMEOUT_MS &&
      isfinite(bodyRollDeg) && isfinite(filteredRollRateDegPerSec) &&
      fabsf(bodyRollDeg - PENDULUM_TILT_ZERO_DEG) <= PENDULUM_MAX_VALID_TILT_DEG;
  bool active = ENABLE_PENDULUM_STABILIZATION && navigationEnabled && imuValid &&
      isfinite(controlDtSec) && controlDtSec > 0.0f && controlDtSec <= 0.1f;

  if (!active) {
    rollErrorIntegralDegSec = 0.0f;
    pendulumFeedbackWasActive = false;
  } else {
    if (!pendulumFeedbackWasActive ||
        fabsf(rampedBodyRollTargetDeg - previousBodyRollTargetDeg) >
            PENDULUM_INTEGRAL_RESET_TARGET_CHANGE_DEG) {
      rollErrorIntegralDegSec = 0.0f;
    }
    pendulumFeedbackWasActive = true;
    previousBodyRollTargetDeg = rampedBodyRollTargetDeg;

    float measuredTilt = bodyRollDeg - PENDULUM_TILT_ZERO_DEG;
    float tiltError = rampedBodyRollTargetDeg - measuredTilt;

    // Do not integrate if the correction or final servo target would saturate.
    // This prevents the integral from building up against mechanical limits.
    float candidateIntegral = clampf(rollErrorIntegralDegSec + tiltError * controlDtSec,
        -PENDULUM_INTEGRAL_LIMIT_DEG_SEC, PENDULUM_INTEGRAL_LIMIT_DEG_SEC);
    float pd = PENDULUM_KP * tiltError - PENDULUM_KD * filteredRollRateDegPerSec;
    float candidateOutput = PENDULUM_CORRECTION_DIRECTION *
        (pd + PENDULUM_KI * candidateIntegral);
    float lowerServo = SWING_CENTER_DEG - MAX_SWING_OFFSET_DEG;
    float upperServo = SWING_CENTER_DEG + MAX_SWING_OFFSET_DEG;
    float candidateServo = target + candidateOutput;
    bool saturated = candidateOutput < -PENDULUM_MAX_CORRECTION_DEG ||
                     candidateOutput > PENDULUM_MAX_CORRECTION_DEG ||
                     candidateServo < lowerServo || candidateServo > upperServo;
    if (!saturated) rollErrorIntegralDegSec = candidateIntegral;

    pendulumFeedbackCorrectionDeg = clampf(PENDULUM_CORRECTION_DIRECTION *
        (pd + PENDULUM_KI * rollErrorIntegralDegSec),
        -PENDULUM_MAX_CORRECTION_DEG, PENDULUM_MAX_CORRECTION_DEG);
  }

  // Preserve arc feed-forward and apply bounded PID correction.
  float servoTarget = clampf(target + pendulumFeedbackCorrectionDeg,
      SWING_CENTER_DEG - MAX_SWING_OFFSET_DEG,
      SWING_CENTER_DEG + MAX_SWING_OFFSET_DEG);
  pendulumServoCommandDeg = moveToward(pendulumServoCommandDeg, servoTarget,
      SWING_SLEW_RATE_DEG_PER_SEC * controlDtSec);
  setSwing(pendulumServoCommandDeg);
}

void controlledStop(float controlDtSec) {
  stopPivotPulse();
  commandDrive(0.0f, controlDtSec);
  commandSwing(SWING_CENTER_DEG, controlDtSec);
}

void navigationUpdate(float controlDtSec) {
  targetBodyRollDeg = 0.0f;
  if (!targetFilterInitialized) {
    controlledStop(controlDtSec);
    navigationState = WAITING_FOR_PI;
    return;
  }

  float distance = filteredTargetDistanceFt;
  float angle = normalizeAngle(filteredTargetAngleDeg);

  if (distance <= HARD_MIN_DISTANCE_FT) {
    // Close target: halt navigation, but keep IMU rocking damping active.
    // controlledStop() calls commandDrive(0, controlDtSec), which computes and
    // actually sends the balance-corrected motor command every cycle.
    // Unlike emergencyStop(), it does not erase balance correction.
    navigationState = TOO_CLOSE;
    isFollowingTarget = false;
    isPivoting = false;
    controlledStop(controlDtSec);
    return;
  }

  // Follow-distance hysteresis
  if (!isFollowingTarget) {
    if (distance < FOLLOW_START_DISTANCE_FT) {
      navigationState = IN_LEASH_ZONE;
      controlledStop(controlDtSec);
      return;
    }
    isFollowingTarget = true;
  } else {
    if (distance <= FOLLOW_STOP_DISTANCE_FT) {
      isFollowingTarget = false;
      isPivoting = false;
      navigationState = TARGET_REACHED;
      controlledStop(controlDtSec);
      return;
    }
  }

  // Continue an existing pivot until aligned.
  if (isPivoting) {
    if (fabsf(angle) <= PIVOT_STOP_ANGLE_DEG) {
      isPivoting = false;
      stopPivotPulse();
    } else {
      navigationState = PIVOT;
      commandDrive(0.0f, controlDtSec);
      commandSwing(SWING_CENTER_DEG, controlDtSec);
      commandPivotPulse(angle);
      return;
    }
  }

  // Target behind BB-8.
  if (fabsf(angle) >= REAR_PIVOT_ANGLE_DEG) {
    navigationState = PIVOT;
    isPivoting = true;
    commandDrive(0.0f, controlDtSec);
    commandSwing(SWING_CENTER_DEG, controlDtSec);
    commandPivotPulse(angle);
    return;
  }

  // Straight.
  if (fabsf(angle) <= ANGLE_DEADBAND_DEG) {
    navigationState = STRAIGHT;
    stopPivotPulse();
    commandSwing(SWING_CENTER_DEG, controlDtSec);
    commandDrive(calculateDriveSpeed(distance), controlDtSec);
    return;
  }

  // Close + significant angle => pivot.
  if (distance <= CLOSE_NAV_DISTANCE_FT &&
      fabsf(angle) >= CLOSE_PIVOT_START_ANGLE_DEG) {
    navigationState = PIVOT;
    isPivoting = true;
    commandDrive(0.0f, controlDtSec);
    commandSwing(SWING_CENTER_DEG, controlDtSec);
    commandPivotPulse(angle);
    return;
  }

  // Otherwise arc. Target a modest lean into the turn.
  navigationState = ARC;
  targetBodyRollDeg = clampf(filteredTargetAngleDeg *
      ARC_BODY_LEAN_DEG_PER_TARGET_DEG * ARC_LEAN_DIRECTION,
      -MAX_ARC_BODY_LEAN_DEG, MAX_ARC_BODY_LEAN_DEG);
  stopPivotPulse();
  commandSwing(calculateSwingTarget(angle), controlDtSec);
  commandDrive(calculateDriveSpeed(distance), controlDtSec);
}

// ============================================================
// STATUS
// ============================================================

void sendStatus() {
  Serial1.print("S=");
  Serial1.print(stateName(navigationState));
  Serial1.print(", Distance=");
  Serial1.print(filteredTargetDistanceFt, 2);
  Serial1.print(", Angle=");
  Serial1.print(filteredTargetAngleDeg, 2);
  Serial1.print(", Drive=");
  //Serial1.print(bodyPitchDeg, 2);
  //Serial1.print(", ");
  //Serial1.print(bodyRollDeg, 2);
  //Serial1.print(", ");
  Serial1.print(rampedDriveCommand, 3);
  Serial1.print(", Swing=");
  Serial1.print(pendulumServoCommandDeg, 1);
  Serial1.print(", Pivot=");
  Serial1.println(pivotMotorCommand, 3);

  Serial.print("State=");
  Serial.print(stateName(navigationState));
  Serial.print(" D=");
  Serial.print(filteredTargetDistanceFt, 1);
  Serial.print(" A=");
  Serial.print(filteredTargetAngleDeg, 1);
  Serial.print(" Pitch=");
  Serial.print(bodyPitchDeg, 1);
  Serial.print(" Roll=");
  Serial.print(bodyRollDeg, 1);
  Serial.print(" Drive=");
  Serial.print(rampedDriveCommand, 2);
  Serial.print(" Applied=");
  Serial.print(appliedDriveCommand, 2);
  Serial.print(" PitchRate=");
  Serial.print(filteredPitchRateDegPerSec, 1);
  Serial.print(" Swing=");
  Serial.print(pendulumServoCommandDeg, 1);
  Serial.print(" TiltTarget=");
  Serial.print(rampedBodyRollTargetDeg, 1);
  Serial.print(" TiltRate=");
  Serial.print(filteredRollRateDegPerSec, 1);
  Serial.print(" PendCorr=");
  Serial.print(pendulumFeedbackCorrectionDeg, 2);
  Serial.print(" Pivot=");
  Serial.print(pivotMotorCommand, 2);
  Serial.print(" HeadFB=");
  Serial.print(headPitchCommandDeg, 1);
  Serial.print(" HeadSTS=");
  Serial.print(headRollCommandDeg, 1);
  Serial.print(" HeadMode=");
  Serial.print(headLevelingStatus);
  Serial.print(" HeadAgeMs=");
  if (hasHeadOrientationSample) Serial.print((uint32_t)(millis() - lastHeadOrientationMs));
  else Serial.print("NONE");
  Serial.print(" Enabled=");
  Serial.print(navigationEnabled);

  Serial.print(" IMU=");
  Serial.print(imuAvailable);

  Serial.print(" Fresh=");
  Serial.print(
    hasPitchSample &&
    (millis() - lastPitchUpdateMs <= BALANCE_IMU_TIMEOUT_MS)
  );

  Serial.print(" RateReady=");
  Serial.print(pitchRateReady);

  Serial.print(" Balance=");
  Serial.print(smoothedPitchCorrectionCommand, 4);
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

  servoDriver.begin();
  servoDriver.setPWMFreq(50);
  setSwing(SWING_CENTER_DEG);
  centerHead();

  // Start the head startup safety timer after its initial center command.
  headStartupMs = millis();
  lastHeadUpdateMs = headStartupMs;

  // Support either BNO085 address and retry a late-powered sensor while idle.
  initializeIMU();

  navigationState = WAITING_FOR_PI;
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
  if (navigationEnabled &&
      (uint32_t)(nowMs - lastPiCommandMs) > PI_TIMEOUT_MS) {
    emergencyStop(COMMS_LOST);
    disableSystem();
    resetTargetFilter();
  }

  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastControlUpdateUs) >= CONTROL_PERIOD_US) {
    float controlDtSec;

    if (lastControlUpdateUs == 0) {
      controlDtSec = 0.02f;
    } else {
      controlDtSec = (nowUs - lastControlUpdateUs) / 1000000.0f;
      controlDtSec = clampf(controlDtSec, 0.001f, 0.10f);
    }

    lastControlUpdateUs = nowUs;

    if (navigationEnabled) {
      navigationUpdate(controlDtSec);
    }

    // Keep the head level from the latest BNO085 pitch/roll reading.
    // This is independent of the navigation state.
    updateHeadLeveling();
  }

  if ((uint32_t)(nowMs - lastStatusSentMs) >= STATUS_PERIOD_MS) {
    lastStatusSentMs = nowMs;
    sendStatus();
  }
}
