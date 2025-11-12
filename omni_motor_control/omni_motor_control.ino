#include <PID_v1.h>
#include <MotorDriver.h>
#include <MPU6050.h>
#include "config.h"

MotorDriver motorDriver = MotorDriver(YF_IIC_RZ);
MPU6050 mpu6050;

struct EncoderPins {
  int encA;
  int encB;
};

EncoderPins encoders[4] = {
  {3, 5},  // Motor 1 - Lifter
  {2, 4},  // Motor 2 - Front Right
  {7, 6},  // Motor 3 - Front Left
  {9, 8}   // Motor 4 - Back
};

volatile long encoderCount[4] = {0, 0, 0, 0};
long lastEncoderCount[4] = {0, 0, 0, 0};
unsigned long lastEncoderTime[4] = {0, 0, 0, 0};

double rpmHistory[4][RPM_FILTER_SIZE] = {0};
int rpmHistoryIndex[4] = {0};
double filteredRPM[4] = {0, 0, 0, 0};
double smoothedRPM[4] = {0, 0, 0, 0};
double rpmAlpha = RPM_ALPHA;
double rpmAlphaFast = RPM_ALPHA_FAST;

double lastRPM[4] = {0, 0, 0, 0};
double maxRPMChange = MAX_RPM_CHANGE;
double maxRPMChangeRotation = MAX_RPM_CHANGE_ROTATION;

long totalEncoderCount[4] = {0, 0, 0, 0};
double motorPosition[4] = {0, 0, 0, 0};

double syncError[4] = {0, 0, 0, 0};
double syncKp = SYNC_KP;
double targetSyncRPM = 0;
bool synchronizationActive = false;
int activeMotorCount = 0;
bool motorIntendedActive[4] = {false, false, false, false};

unsigned long lastRotationCommand = 0;
bool fastRotationMode = false;

double setpoint[4] = {0, 0, 0, 0};
double input[4] = {0, 0, 0, 0};
double output[4] = {0, 0, 0, 0};
bool motorsStopped = true;
bool lifterActive = false;

double prev_setpoint[4] = {0, 0, 0, 0};
float speedMultiplier = 1.0;

int16_t rawGyroX, rawGyroY, rawGyroZ;
int16_t rawAccelX, rawAccelY, rawAccelZ;
float gyroX, gyroY, gyroZ;
float accelX, accelY, accelZ;
float temperature;
double robotHeading = 0.0;
double targetHeading = 0.0;
double headingError = 0.0;
double headingKp = HEADING_KP;
double headingCorrection = 0.0;
unsigned long lastMPUUpdate = 0;
bool imuInitialized = false;
bool headingCorrectionEnabled = true;
double gyroDriftX = 0.0;
double gyroDriftY = 0.0;
double gyroDriftZ = 0.0;
int gyroCalibrationSamples = GYRO_CALIBRATION_SAMPLES;
bool gyroCalibrated = false;

PID pid[4] = {
  PID(&input[0], &output[0], &setpoint[0], Lifter_Kp, Lifter_Ki, Lifter_Kd, DIRECT),
  PID(&input[1], &output[1], &setpoint[1], Omni_Kp, Omni_Ki, Omni_Kd, DIRECT),
  PID(&input[2], &output[2], &setpoint[2], Omni_Kp, Omni_Ki, Omni_Kd, DIRECT),
  PID(&input[3], &output[3], &setpoint[3], Omni_Kp, Omni_Ki, Omni_Kd, DIRECT)
};

struct IRDistanceData {
  float voltage;
  float distance;
  bool valid;
};

IRDistanceData irLeft1, irLeft2;
IRDistanceData irRight1, irRight2;
IRDistanceData irBack1, irBack2;

struct UltrasonicData {
  long duration;
  float distance;
  bool valid;
};

UltrasonicData ultrasonicFrontLeft;
UltrasonicData ultrasonicFrontRight;

struct LineSensorData {
  int rawValue;
  bool onLine;
};

LineSensorData lineLeft, lineCenter, lineRight;

unsigned long lastSensorUpdate = 0;
const unsigned long SENSOR_UPDATE_INTERVAL = 100;

void setup() {
  Serial.begin(115200);
  Serial.println("Omni Wheel Robot Control Starting...");

  motorDriver.begin();
  motorDriver.motorConfig(1, 1, 1, 1);
  delay(500);

  initializeMPU6050();

  for (int i = 0; i < 4; i++) {
    pinMode(encoders[i].encA, INPUT_PULLUP);
    pinMode(encoders[i].encB, INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(encoders[0].encA), encoderISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[1].encA), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[2].encA), encoderISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[3].encA), encoderISR3, CHANGE);

  for (int i = 0; i < 4; i++) {
    pid[i].SetMode(AUTOMATIC);
    pid[i].SetSampleTime(PID_SAMPLE_TIME);
    pid[i].SetOutputLimits(PID_OUTPUT_LIMIT_MIN, PID_OUTPUT_LIMIT_MAX);
  }

  Serial.println("Initializing sensors...");
  pinMode(ULTRASONIC_FRONT_LEFT_TRIG, OUTPUT);
  pinMode(ULTRASONIC_FRONT_LEFT_ECHO, INPUT);
  pinMode(ULTRASONIC_FRONT_RIGHT_TRIG, OUTPUT);
  pinMode(ULTRASONIC_FRONT_RIGHT_ECHO, INPUT);
  Serial.println("Sensors initialized.");

  Serial.println("Setup complete. Ready for commands.");
  Serial.println("=== MOVEMENT COMMANDS ===");
  Serial.println("Basic: f(forward), b(backward), l(left), r(right)");
  Serial.println("Turn: t(turn left), y(turn right), c(rotate CW), w(rotate CCW)");
  Serial.println("Diagonal: q(forward-left), e(forward-right), z(backward-left), x(backward-right)");
  Serial.println("Arc: a(arc left), j(arc right)");
  Serial.println("Control: s(stop), p(status), o(turbo mode toggle)");
  Serial.println("=== SPECIAL COMMANDS ===");
  Serial.println("Test: 1-4(motor test), g(calibration), h(figure-8)");
  Serial.println("Lifter: u(lift up), d(lift down)");
    Serial.println("Speed: 5-9(speed 50%-90%), 0(speed 100%)");
    Serial.println("IMU: ir(status), ic(calibrate), im(toggle correction), ih(reset heading)");
    Serial.println("Sensors: sr(detailed sensor readings)");
    Serial.println("Emergency: v(emergency stop)");
}

void loop() {
  // Update motor control every PID sample time
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= PID_SAMPLE_TIME) {
    updateMotorControl();
    lastUpdate = millis();
  }

  // Update MPU6050 sensor readings continuously
  updateMPU6050();

  // Update all sensors periodically
  updateAllSensors();

  // Check for serial commands
  static String commandBuffer = "";
  static unsigned long lastCommandTime = 0;

  while (Serial.available()) {
    char incomingChar = Serial.read();

    // Skip newline, carriage return, and other non-command characters
    if (incomingChar != '\n' && incomingChar != '\r' && incomingChar != ' ' && incomingChar != '\t') {
      commandBuffer += incomingChar;
      lastCommandTime = millis();

      // Check if we have a complete command
      if (commandBuffer.length() >= 2) {
        // Two-character sensor commands
        if (commandBuffer == "ir" || commandBuffer == "ic" || commandBuffer == "im" || commandBuffer == "ih" || commandBuffer == "sr") {
          executeCommand(commandBuffer);
          commandBuffer = "";
        }
      } else if (commandBuffer.length() == 1) {
        // Single-character motor commands - execute immediately
        executeCommand(commandBuffer);
        commandBuffer = "";
      }
    }
  }

  // Timeout for incomplete commands (clear buffer after 500ms)
  if (commandBuffer.length() > 0 && millis() - lastCommandTime > 500) {
    Serial.println("Command timeout - incomplete command cleared");
    commandBuffer = "";
  }
}

// Encoder interrupt service routines
void encoderISR0() {
  encoderCount[0] += readEncoder(0);
}

void encoderISR1() {
  encoderCount[1] += readEncoder(1);
}

void encoderISR2() {
  encoderCount[2] += readEncoder(2);
}

void encoderISR3() {
  encoderCount[3] += readEncoder(3);
}

// Read encoder state
int readEncoder(int motorIndex) {
  int state = digitalRead(encoders[motorIndex].encA) * 2 + digitalRead(encoders[motorIndex].encB);
  static int lastState[4] = {0, 0, 0, 0};

  int direction = 0;
  switch (lastState[motorIndex]) {
    case 0:
      if (state == 1) direction = 1;
      else if (state == 2) direction = -1;
      break;
    case 1:
      if (state == 3) direction = 1;
      else if (state == 0) direction = -1;
      break;
    case 2:
      if (state == 0) direction = 1;
      else if (state == 3) direction = -1;
      break;
    case 3:
      if (state == 2) direction = 1;
      else if (state == 1) direction = -1;
      break;
  }
  lastState[motorIndex] = state;
  return direction;
}

// Enhanced encoder processing functions
double applyMovingAverageFilter(int motorIndex, double newRPM) {
  // Add new RPM to history buffer
  rpmHistory[motorIndex][rpmHistoryIndex[motorIndex]] = newRPM;
  rpmHistoryIndex[motorIndex] = (rpmHistoryIndex[motorIndex] + 1) % RPM_FILTER_SIZE;

  // Calculate moving average
  double sum = 0;
  for (int i = 0; i < RPM_FILTER_SIZE; i++) {
    sum += rpmHistory[motorIndex][i];
  }

  filteredRPM[motorIndex] = sum / RPM_FILTER_SIZE;
  return filteredRPM[motorIndex];
}

double applyExponentialSmoothing(int motorIndex, double newRPM) {
  // Adaptive exponential smoothing - ultra-responsive during fast rotation
  double alpha = fastRotationMode ? rpmAlphaFast : rpmAlpha;
  smoothedRPM[motorIndex] = alpha * newRPM + (1.0 - alpha) * smoothedRPM[motorIndex];
  return smoothedRPM[motorIndex];
}

double applyAccelerationLimiting(int motorIndex, double targetRPM) {
  // Check if we're in fast rotation mode (recent rotation command) - EXTENDED to 5 seconds
  bool isFastRotation = false;
  if (fastRotationMode && (millis() - lastRotationCommand) < 5000) {  // 5 seconds of MAXIMUM fast response
    isFastRotation = true;
  } else if ((millis() - lastRotationCommand) > 6000) {  // Disable fast mode after 6 seconds
    fastRotationMode = false;
  }

  // Determine if this is a rotation movement (all motors have same speed)
  bool isRotation = false;
  if (motorIndex >= 1 && motorIndex <= 3) {  // Only for omni motors
    double absSetpoint = abs(setpoint[motorIndex]);
    // Check if all omni motors have the same absolute setpoint (indicates rotation)
    if (absSetpoint > 10 && abs(setpoint[1]) == absSetpoint && abs(setpoint[2]) == absSetpoint && abs(setpoint[3]) == absSetpoint) {
      isRotation = true;
    }
  }

  // Skip acceleration limiting for fast rotation mode
  if (isFastRotation) {
    lastRPM[motorIndex] = targetRPM;
    return targetRPM;
  }

  // Use faster acceleration for rotation movements
  double maxChange = isRotation ? maxRPMChangeRotation : maxRPMChange;

  // Limit rate of RPM change for smooth acceleration/deceleration
  double rpmChange = targetRPM - lastRPM[motorIndex];

  if (abs(rpmChange) > maxChange) {
    rpmChange = (rpmChange > 0) ? maxChange : -maxChange;
    targetRPM = lastRPM[motorIndex] + rpmChange;
  }

  lastRPM[motorIndex] = targetRPM;
  return targetRPM;
}

void updateMotorPosition(int motorIndex, long countDiff) {
  // Track total encoder counts and position
  totalEncoderCount[motorIndex] += countDiff;
  motorPosition[motorIndex] = (double)totalEncoderCount[motorIndex] / (ENCODER_CPR * GEAR_RATIO);
}

// Motor synchronization functions
void updateMotorSynchronization() {
  // Calculate average RPM of active motors for synchronization
  double totalRPM = 0;
  activeMotorCount = 0;

  // Count active motors based on intended active flags
  for (int i = 1; i < 4; i++) {  // Skip lifter motor
    if (motorIntendedActive[i] && !motorsStopped) {
      totalRPM += smoothedRPM[i];
      activeMotorCount++;
    }
  }

  // Calculate target synchronized RPM
  if (activeMotorCount > 0) {
    targetSyncRPM = totalRPM / activeMotorCount;
    synchronizationActive = true;
  } else {
    targetSyncRPM = 0;
    synchronizationActive = false;
  }

  // Calculate synchronization errors only for intended active motors
  for (int i = 1; i < 4; i++) {
    if (motorIntendedActive[i] && !motorsStopped && synchronizationActive) {
      syncError[i] = targetSyncRPM - smoothedRPM[i];
    } else {
      syncError[i] = 0;
    }
  }
}

double applySynchronizationCorrection(int motorIndex, double targetRPM) {
  // Apply synchronization correction only to intended active motors
  if (synchronizationActive && motorIntendedActive[motorIndex] && !motorsStopped) {
    double correction = syncError[motorIndex] * syncKp;
    targetRPM += correction;

    // Constrain correction to reasonable limits
    targetRPM = constrain(targetRPM, -MAX_RPM * 1.2, MAX_RPM * 1.2);
  }

  return targetRPM;
}

void updateMotorControl() {
  unsigned long currentTime = millis();

  for (int i = 0; i < 4; i++) {
    long countDiff = encoderCount[i] - lastEncoderCount[i];
    unsigned long timeDiff = currentTime - lastEncoderTime[i];

    if (timeDiff > 0) {
      double revolutions = (double)countDiff / (ENCODER_CPR * GEAR_RATIO);
      double rawRPM = (revolutions * 60000.0) / timeDiff;

      double filteredRPM = applyMovingAverageFilter(i, rawRPM);
      double smoothedRPM = applyExponentialSmoothing(i, filteredRPM);
      input[i] = smoothedRPM;

      updateMotorPosition(i, countDiff);
    }

    lastEncoderCount[i] = encoderCount[i];
    lastEncoderTime[i] = currentTime;
  }

  if (!fastRotationMode) {
    updateMotorSynchronization();
  }

  applyIMUCorrections();

  if (lifterActive) {
    pid[0].Compute();
    int lifterSpeed = map(output[0], -255, 255, -4096, 4096);
    motorDriver.setSingleMotor(1, lifterSpeed);
  } else {
    motorDriver.stopMotor(M1);
  }

  if (!motorsStopped) {
    for (int i = 1; i < 4; i++) {
      if (!motorIntendedActive[i]) {
        motorDriver.stopMotor(i + 1);
        setpoint[i] = 0;
        continue;
      }
      pid[i].Compute();
      int motorSpeed = map(output[i], -255, 255, -4096, 4096);
      motorDriver.setSingleMotor(i+1, motorSpeed);
    }
  } else {
    motorDriver.stopMotor(M2);
    motorDriver.stopMotor(M3);
    motorDriver.stopMotor(M4);
  }
}

void setOmniSpeeds(double vx, double vy, double omega) {
  motorsStopped = false;
  lifterActive = false;

  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  const double wheel_distance = 1.0;
  double motor_speeds[4] = {0, 0, 0, 0};

  double forward_component = abs(vx);
  double sideways_component = abs(vy);
  double rotation_component = abs(omega);
  bool use_back_wheel = (sideways_component > 0.1) || (rotation_component > 0.1);

  for (int i = 1; i < 4; i++) {
    double angle_rad = MOTOR_ANGLES[i] * PI / 180.0;

    if (!use_back_wheel && i == 3) {
      motor_speeds[i] = 0;
      motorIntendedActive[i] = false;
    } else {
      motor_speeds[i] = ((-vx) * cos(angle_rad) + vy * sin(angle_rad) + (omega * wheel_distance)) * speedMultiplier;
    }
  }

  double max_speed = 0.0;
  for (int i = 1; i < 4; i++) {
    max_speed = max(max_speed, abs(motor_speeds[i]));
  }

  double scale_factor = (max_speed > 1.0) ? (1.0 / max_speed) : 1.0;

  for (int i = 1; i < 4; i++) {
    double normalized_speed = motor_speeds[i] * scale_factor;
    double target_rpm = normalized_speed * BASE_SPEED;
    target_rpm = constrain(target_rpm, -MAX_RPM, MAX_RPM);

    motorIntendedActive[i] = (abs(normalized_speed) > 0.01);
    target_rpm = applySynchronizationCorrection(i, target_rpm);
    target_rpm = applyAccelerationLimiting(i, target_rpm);

    setpoint[i] = target_rpm;
    prev_setpoint[i] = target_rpm;
  }
}

// Movement functions
void moveForward() {
  Serial.println("Moving Forward (FL & FR wheels only - back wheel idle)");
  setOmniSpeeds(1.0, 0.0, 0.0);
  setpoint[0] = 0;  // Lifter stays still
  setTargetHeading(); // Set target heading for IMU correction
}

void moveBackward() {
  Serial.println("Moving Backward (FL & FR wheels only - back wheel idle)");
  setOmniSpeeds(-1.0, 0.0, 0.0);
  setpoint[0] = 0;
  setTargetHeading(); // Set target heading for IMU correction
}

void moveLeft() {
  Serial.println("Moving Left (FL + Back wheels - 2-wheel strafe)");
  motorsStopped = false;
  lifterActive = false;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Left strafe: FL + Back wheels (2-wheel combination)
  setpoint[1] = 0;                                    // FR (Motor 2) - idle
  setpoint[2] = -0.8 * BASE_SPEED * speedMultiplier;  // FL (Motor 3)
  setpoint[3] = 0.8 * BASE_SPEED * speedMultiplier;   // Back (Motor 4)
  setpoint[0] = 0;                                    // Lifter (Motor 1) - idle

  // Set intended active flags
  motorIntendedActive[2] = true;  // FL
  motorIntendedActive[3] = true;  // Back

  setTargetHeading(); // Set target heading for IMU correction
}

void moveRight() {
  Serial.println("Moving Right (FR + Back wheels - 2-wheel strafe)");
  motorsStopped = false;
  lifterActive = false;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Right strafe: FR + Back wheels (2-wheel combination)
  setpoint[1] = 0.8 * BASE_SPEED * speedMultiplier;   // FR (Motor 2)
  setpoint[2] = 0;                                    // FL (Motor 3) - idle
  setpoint[3] = -0.8 * BASE_SPEED * speedMultiplier;  // Back (Motor 4)
  setpoint[0] = 0;                                    // Lifter (Motor 1) - idle

  // Set intended active flags
  motorIntendedActive[1] = true;  // FR
  motorIntendedActive[3] = true;  // Back

  setTargetHeading(); // Set target heading for IMU correction
}

// Diagonal movement functions - REFINED: specific wheel combinations
void moveForwardLeft() {
  Serial.println("Moving Forward-Left (FR + Back wheels)");
  motorsStopped = false;
  lifterActive = false;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Forward-left diagonal: FR + Back wheels
  setpoint[1] = 0.8 * BASE_SPEED * speedMultiplier;  // FR (Motor 2)
  setpoint[2] = 0;                                    // FL (Motor 3) - idle
  setpoint[3] = 0.8 * BASE_SPEED * speedMultiplier;  // Back (Motor 4)
  setpoint[0] = 0;                                    // Lifter (Motor 1) - idle

  // Set intended active flags
  motorIntendedActive[1] = true;  // FR
  motorIntendedActive[3] = true;  // Back

  setTargetHeading(); // Set target heading for IMU correction
}

void moveForwardRight() {
  Serial.println("Moving Forward-Right (FL + Back wheels)");
  motorsStopped = false;
  lifterActive = false;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Forward-right diagonal: FL + Back wheels
  setpoint[1] = 0;                                    // FR (Motor 2) - idle
  setpoint[2] = 0.8 * BASE_SPEED * speedMultiplier;  // FL (Motor 3)
  setpoint[3] = 0.8 * BASE_SPEED * speedMultiplier;  // Back (Motor 4)
  setpoint[0] = 0;                                    // Lifter (Motor 1) - idle

  // Set intended active flags
  motorIntendedActive[2] = true;  // FL
  motorIntendedActive[3] = true;  // Back

  setTargetHeading(); // Set target heading for IMU correction
}

void moveBackwardLeft() {
  Serial.println("Moving Backward-Left (FL + Back wheels)");
  motorsStopped = false;
  lifterActive = false;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Backward-left diagonal: FL + Back wheels
  setpoint[1] = 0;                                     // FR (Motor 2) - idle
  setpoint[2] = -0.8 * BASE_SPEED * speedMultiplier;  // FL (Motor 3)
  setpoint[3] = -0.8 * BASE_SPEED * speedMultiplier;  // Back (Motor 4)
  setpoint[0] = 0;                                     // Lifter (Motor 1) - idle

  // Set intended active flags
  motorIntendedActive[2] = true;  // FL
  motorIntendedActive[3] = true;  // Back

  setTargetHeading(); // Set target heading for IMU correction
}

void moveBackwardRight() {
  Serial.println("Moving Backward-Right (FR + Back wheels)");
  motorsStopped = false;
  lifterActive = false;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Backward-right diagonal: FR + Back wheels
  setpoint[1] = -0.8 * BASE_SPEED * speedMultiplier;  // FR (Motor 2)
  setpoint[2] = 0;                                     // FL (Motor 3) - idle
  setpoint[3] = -0.8 * BASE_SPEED * speedMultiplier;  // Back (Motor 4)
  setpoint[0] = 0;                                     // Lifter (Motor 1) - idle

  // Set intended active flags
  motorIntendedActive[1] = true;  // FR
  motorIntendedActive[3] = true;  // Back

  setTargetHeading(); // Set target heading for IMU correction
}

// Arc movement functions (forward + rotation)
void arcLeft() {
  Serial.println("Arc Left");
  setOmniSpeeds(0.8, 0.0, -0.5);  // Forward + left rotation
  setpoint[0] = 0;
  setTargetHeading(); // Set target heading for IMU correction
}

void arcRight() {
  Serial.println("Arc Right");
  setOmniSpeeds(0.8, 0.0, 0.5);   // Forward + right rotation
  setpoint[0] = 0;
  setTargetHeading(); // Set target heading for IMU correction
}

// Turn functions (slight rotation while moving forward)
void turnLeft() {
  Serial.println("Turning Left");
  setOmniSpeeds(0.5, 0.0, -0.8);  // Slow forward + turn
  setpoint[0] = 0;
  setTargetHeading(); // Set target heading for IMU correction
}

void turnRight() {
  Serial.println("Turning Right");
  setOmniSpeeds(0.5, 0.0, 0.8);   // Slow forward + turn
  setpoint[0] = 0;
  setTargetHeading(); // Set target heading for IMU correction
}

void rotateCW() {
  Serial.println("Rotating Clockwise (TURBO MODE - MAXIMUM RESPONSE)");
  lastRotationCommand = millis();
  fastRotationMode = true;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Use maximum speed for rotation (1.0 = 100% of BASE_SPEED)
  setOmniSpeeds(0.0, 0.0, 1.0);
  setpoint[0] = 0;

  // Explicitly set all omni motors as intended active for rotation
  motorIntendedActive[1] = true;  // FR
  motorIntendedActive[2] = true;  // FL
  motorIntendedActive[3] = true;  // Back
}

void rotateCCW() {
  Serial.println("Rotating Counter-Clockwise (TURBO MODE - MAXIMUM RESPONSE)");
  lastRotationCommand = millis();
  fastRotationMode = true;

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Use maximum speed for rotation (1.0 = 100% of BASE_SPEED)
  setOmniSpeeds(0.0, 0.0, -1.0);
  setpoint[0] = 0;

  // Explicitly set all omni motors as intended active for rotation
  motorIntendedActive[1] = true;  // FR
  motorIntendedActive[2] = true;  // FL
  motorIntendedActive[3] = true;  // Back
}

void stopMotors() {
  Serial.println("Stopping - All motors disabled");
  motorsStopped = true;   // Disable omni motor control
  lifterActive = false;   // Disable lifter control
  fastRotationMode = false; // Disable fast rotation mode

  // Reset intended active flags
  for (int i = 0; i < 4; i++) {
    motorIntendedActive[i] = false;
  }

  // Use YFROBOT library stop function for immediate stop
  motorDriver.stopMotor(MAll);  // Stop all motors immediately

  // Reset PID setpoints and acceleration tracking
  for (int i = 0; i < 4; i++) {
    setpoint[i] = 0;
    prev_setpoint[i] = 0;  // Reset acceleration tracking
    input[i] = 0;
    output[i] = 0;

    // Reset encoder smoothing data for clean restart
    smoothedRPM[i] = 0;
    lastRPM[i] = 0;
    syncError[i] = 0;
    for (int j = 0; j < RPM_FILTER_SIZE; j++) {
      rpmHistory[i][j] = 0;
    }
    rpmHistoryIndex[i] = 0;
  }
}

// Control lifter motor
void liftUp() {
  Serial.println("Lifting Up");
  lifterActive = true;    // Enable lifter control
  motorsStopped = true;   // Stop omni motors
  setpoint[0] = LIFT_SPEED;
  // Other motors will be stopped by the control logic
}

void liftDown() {
  Serial.println("Lifting Down");
  lifterActive = true;    // Enable lifter control
  motorsStopped = true;   // Stop omni motors
  setpoint[0] = -LIFT_SPEED;
  // Other motors will be stopped by the control logic
}

// Execute commands from serial
void executeCommand(String command) {
  // Handle two-character sensor commands first
  if (command == "ir") {
    // IMU status
    if (imuInitialized) {
      Serial.println("IMU Status: ENABLED");
      Serial.print("Heading: ");
      Serial.print(robotHeading);
      Serial.print("°, Target: ");
      Serial.print(targetHeading);
      Serial.print("°, Error: ");
      Serial.print(headingError);
      Serial.print("°, Correction: ");
      Serial.println(headingCorrection);
      Serial.print("Gyro: X=");
      Serial.print(gyroX);
      Serial.print("°, Y=");
      Serial.print(gyroY);
      Serial.print("°, Z=");
      Serial.print(gyroZ);
      Serial.println("°/s");
      Serial.print("Accel: X=");
      Serial.print(accelX);
      Serial.print("m/s², Y=");
      Serial.print(accelY);
      Serial.print("m/s², Z=");
      Serial.print(accelZ);
      Serial.println("m/s²");
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println("°C");
    } else {
      Serial.println("IMU Status: DISABLED - MPU6050 not detected");
    }
    return;
  } else if (command == "ic") {
    // IMU calibration
    if (imuInitialized) {
      calibrateGyroscope();
      resetHeading();
    } else {
      Serial.println("IMU not initialized - cannot calibrate");
    }
    return;
  } else if (command == "im") {
    // Toggle IMU heading correction
    headingCorrectionEnabled = !headingCorrectionEnabled;
    Serial.print("IMU Heading Correction: ");
    Serial.println(headingCorrectionEnabled ? "ENABLED" : "DISABLED");
    return;
  } else if (command == "ih") {
    // Reset heading to 0°
    resetHeading();
    return;
  } else if (command == "sr") {
    // Detailed sensor readings
    Serial.println("=== Detailed Sensor Readings ===");

    // Force update sensors for fresh readings
    updateIRDistanceSensors();
    updateUltrasonicSensors();
    updateLineSensors();

    Serial.println("IR Distance Sensors (Sharp GP2Y0A02YK0F):");
    Serial.print("  Left 1: ");
    Serial.print(irLeft1.distance, 1);
    Serial.print("mm (");
    Serial.print(irLeft1.voltage, 2);
    Serial.print("V) - ");
    Serial.println(irLeft1.valid ? "VALID" : "INVALID");

    Serial.print("  Left 2: ");
    Serial.print(irLeft2.distance, 1);
    Serial.print("mm (");
    Serial.print(irLeft2.voltage, 2);
    Serial.print("V) - ");
    Serial.println(irLeft2.valid ? "VALID" : "INVALID");

    Serial.print("  Right 1: ");
    Serial.print(irRight1.distance, 1);
    Serial.print("mm (");
    Serial.print(irRight1.voltage, 2);
    Serial.print("V) - ");
    Serial.println(irRight1.valid ? "VALID" : "INVALID");

    Serial.print("  Right 2: ");
    Serial.print(irRight2.distance, 1);
    Serial.print("mm (");
    Serial.print(irRight2.voltage, 2);
    Serial.print("V) - ");
    Serial.println(irRight2.valid ? "VALID" : "INVALID");

    Serial.print("  Back 1: ");
    Serial.print(irBack1.distance, 1);
    Serial.print("mm (");
    Serial.print(irBack1.voltage, 2);
    Serial.print("V) - ");
    Serial.println(irBack1.valid ? "VALID" : "INVALID");

    Serial.print("  Back 2: ");
    Serial.print(irBack2.distance, 1);
    Serial.print("mm (");
    Serial.print(irBack2.voltage, 2);
    Serial.print("V) - ");
    Serial.println(irBack2.valid ? "VALID" : "INVALID");

    Serial.println("HC-SR04 Ultrasonic Sensors:");
    Serial.print("  Front Left: ");
    Serial.print(ultrasonicFrontLeft.distance, 1);
    Serial.print("cm (");
    Serial.print(ultrasonicFrontLeft.duration);
    Serial.print("us) - ");
    Serial.println(ultrasonicFrontLeft.valid ? "VALID" : "TIMEOUT");

    Serial.print("  Front Right: ");
    Serial.print(ultrasonicFrontRight.distance, 1);
    Serial.print("cm (");
    Serial.print(ultrasonicFrontRight.duration);
    Serial.print("us) - ");
    Serial.println(ultrasonicFrontRight.valid ? "VALID" : "TIMEOUT");

    Serial.println("Line Sensors (Threshold: 512):");
    Serial.print("  Left: ");
    Serial.print(lineLeft.rawValue);
    Serial.print(" - ");
    Serial.println(lineLeft.onLine ? "ON LINE" : "OFF LINE");

    Serial.print("  Center: ");
    Serial.print(lineCenter.rawValue);
    Serial.print(" - ");
    Serial.println(lineCenter.onLine ? "ON LINE" : "OFF LINE");

    Serial.print("  Right: ");
    Serial.print(lineRight.rawValue);
    Serial.print(" - ");
    Serial.println(lineRight.onLine ? "ON LINE" : "OFF LINE");

    Serial.println("========================");
    return;
  }

  // Handle single-character motor commands
  char singleCommand = command.charAt(0);
  switch (singleCommand) {
    case 'f':
    case 'F':
      moveForward();
      break;
    case 'b':
    case 'B':
      moveBackward();
      break;
    case 'l':
    case 'L':
      moveLeft();
      break;
    case 'r':
    case 'R':
      moveRight();
      break;
    // Diagonal movements
    case 'q':
    case 'Q':
      moveForwardLeft();
      break;
    case 'e':
    case 'E':
      moveForwardRight();
      break;
    case 'z':
    case 'Z':
      moveBackwardLeft();
      break;
    case 'x':
    case 'X':
      moveBackwardRight();
      break;
    // Arc movements
    case 'a':
    case 'A':
      arcLeft();
      break;
    case 'j':
    case 'J':
      arcRight();
      break;
    // Turns
    case 't':
      turnLeft();
      break;
    case 'y':
      turnRight();
      break;
    case 'c':
    case 'C':
      rotateCW();
      break;
    case 'w':
    case 'W':
      rotateCCW();
      break;
    // Speed control
    case '0':
      setSpeed(1.0);  // 100% speed
      Serial.println("Speed set to 100%");
      break;
    case '5':
      setSpeed(0.5);  // 50% speed
      Serial.println("Speed set to 50%");
      break;
    case '6':
      setSpeed(0.6);  // 60% speed
      Serial.println("Speed set to 60%");
      break;
    case '7':
      setSpeed(0.7);  // 70% speed
      Serial.println("Speed set to 70%");
      break;
    case '8':
      setSpeed(0.8);  // 80% speed
      Serial.println("Speed set to 80%");
      break;
    case '9':
      setSpeed(0.9);  // 90% speed
      Serial.println("Speed set to 90%");
      break;
    // Special commands
    case 'g':
    case 'G':
      calibrateEncoders();
      break;
    case 'h':
    case 'H':
      figureEight();
      break;
    case 'o':
    case 'O':
      // Toggle TURBO mode (permanent fast response)
      if (fastRotationMode) {
        fastRotationMode = false;
        Serial.println("TURBO MODE DISABLED - Normal response");
      } else {
        fastRotationMode = true;
        lastRotationCommand = millis(); // Prevent auto-disable
        Serial.println("TURBO MODE ENABLED - Maximum responsiveness for ALL movements");
      }
      break;
    case 'u':
    case 'U':
      liftUp();
      break;
    case 'd':
    case 'D':
      liftDown();
      break;
    case 's':
    case 'S':
      stopMotors();
      break;
    case 'p':
    case 'P':
      printStatus();
      break;
    case '1':
    case '2':
    case '3':
    case '4':
      testMotor(singleCommand - '0');
      break;
    case 'v':
    case 'V':
      // Force stop all motors (emergency stop)
      Serial.println("EMERGENCY STOP - All motors forced off");
      motorDriver.stopMotor(MAll);
      motorsStopped = true;
      lifterActive = false;
      fastRotationMode = false;
      // Reset all setpoints and flags
      for (int i = 0; i < 4; i++) {
        setpoint[i] = 0;
        prev_setpoint[i] = 0;
        motorIntendedActive[i] = false;
      }
      break;
    default:
      Serial.println("Unknown command. Available: f,b,l,r,t,y,c,w,u,d,s,p,1-4(motor test), ir(imu status), ic(calibrate), im(toggle IMU), ih(reset heading), sr(sensor readings), v(emergency stop)");
      break;
  }
}

// =============== SENSOR FUNCTIONS ===============

IRDistanceData readIRDistanceSensor(int pin) {
  IRDistanceData data;
  int rawValue = analogRead(pin);
  data.voltage = (float)rawValue * 5.0 / 1023.0;

  if (data.voltage >= IR_VOLTAGE_MIN && data.voltage <= IR_VOLTAGE_MAX) {
    data.distance = 1.0 / (data.voltage * 0.0004 + 0.0002);
    data.valid = (data.distance >= IR_DISTANCE_MIN && data.distance <= IR_DISTANCE_MAX);
  } else {
    data.distance = 0;
    data.valid = false;
  }
  return data;
}

void updateIRDistanceSensors() {
  irLeft1 = readIRDistanceSensor(IR_LEFT_1_PIN);
  irLeft2 = readIRDistanceSensor(IR_LEFT_2_PIN);
  irRight1 = readIRDistanceSensor(IR_RIGHT_1_PIN);
  irRight2 = readIRDistanceSensor(IR_RIGHT_2_PIN);
  irBack1 = readIRDistanceSensor(IR_BACK_1_PIN);
  irBack2 = readIRDistanceSensor(IR_BACK_2_PIN);
}

UltrasonicData readUltrasonicSensor(int trigPin, int echoPin) {
  UltrasonicData data;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT);
  if (duration > 0) {
    data.duration = duration;
    data.distance = (duration * SOUND_SPEED) / 2.0;
    data.valid = true;
  } else {
    data.duration = 0;
    data.distance = 0;
    data.valid = false;
  }
  return data;
}

void updateUltrasonicSensors() {
  ultrasonicFrontLeft = readUltrasonicSensor(ULTRASONIC_FRONT_LEFT_TRIG, ULTRASONIC_FRONT_LEFT_ECHO);
  ultrasonicFrontRight = readUltrasonicSensor(ULTRASONIC_FRONT_RIGHT_TRIG, ULTRASONIC_FRONT_RIGHT_ECHO);
}

LineSensorData readLineSensor(int pin) {
  LineSensorData data;
  data.rawValue = analogRead(pin);
  data.onLine = (data.rawValue > LINE_SENSOR_THRESHOLD);
  return data;
}

void updateLineSensors() {
  lineLeft = readLineSensor(LINE_SENSOR_LEFT);
  lineCenter = readLineSensor(LINE_SENSOR_CENTER);
  lineRight = readLineSensor(LINE_SENSOR_RIGHT);
}

void updateAllSensors() {
  if (millis() - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
    updateIRDistanceSensors();
    updateUltrasonicSensors();
    updateLineSensors();
    lastSensorUpdate = millis();
  }
}

// =============== MPU6050 IMU FUNCTIONS ===============

void initializeMPU6050() {
  Serial.println("Initializing MPU6050 IMU...");
  Wire.begin();
  Wire.setClock(400000);
  mpu6050.initialize();

  Serial.print("Testing MPU6050 connection...");
  bool connectionOK = false;

  for (int attempt = 0; attempt < 5; attempt++) {
    if (mpu6050.testConnection()) {
      connectionOK = true;
      break;
    }
    delay(200);
    Serial.print(".");
  }

  if (connectionOK) {
    Serial.println("MPU6050 connection successful!");
    imuInitialized = true;

    calibrateGyroscope();
    robotHeading = 0.0;
    targetHeading = 0.0;
    Serial.println(" SUCCESS!");
    Serial.println("MPU6050 IMU ready for precise movement control!");
  } else {
    Serial.println(" FAILED!");
    Serial.println("MPU6050 connection failed! IMU features disabled.");
    imuInitialized = false;
  }
}

void calibrateGyroscope() {
  Serial.println("Calibrating gyroscope... Keep robot stationary!");
  double sumX = 0.0, sumY = 0.0, sumZ = 0.0;

  for (int i = 0; i < gyroCalibrationSamples; i++) {
    mpu6050.getRotation(&rawGyroX, &rawGyroY, &rawGyroZ);
    gyroX = rawGyroX / 131.0;
    gyroY = rawGyroY / 131.0;
    gyroZ = rawGyroZ / 131.0;

    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;

    if (i % 25 == 0) Serial.print(".");
    delay(5);
  }

  gyroDriftX = sumX / gyroCalibrationSamples;
  gyroDriftY = sumY / gyroCalibrationSamples;
  gyroDriftZ = sumZ / gyroCalibrationSamples;

  gyroCalibrated = true;
  Serial.println("\nGyroscope calibration complete!");
  Serial.print("Drift compensation: X=");
  Serial.print(gyroDriftX);
  Serial.print("°, Y=");
  Serial.print(gyroDriftY);
  Serial.print("°, Z=");
  Serial.println(gyroDriftZ);
}

// Update MPU6050 sensor readings and heading
void updateMPU6050() {
  if (!imuInitialized) return;

  unsigned long currentTime = millis();
  static unsigned long lastUpdate = 0;
  float dt = (currentTime - lastUpdate) / 1000.0; // Time step in seconds
  lastUpdate = currentTime;

  // Get raw sensor data
  mpu6050.getMotion6(&rawAccelX, &rawAccelY, &rawAccelZ, &rawGyroX, &rawGyroY, &rawGyroZ);
  temperature = mpu6050.getTemperature() / 340.0 + 36.53; // Convert to Celsius

  // Convert accelerometer readings to m/s² (approximate conversion)
  accelX = rawAccelX / 16384.0 * 9.81; // MPU6050 accel sensitivity scale factor * gravity
  accelY = rawAccelY / 16384.0 * 9.81;
  accelZ = rawAccelZ / 16384.0 * 9.81;

  // Convert gyroscope readings to degrees per second
  gyroX = rawGyroX / 131.0; // MPU6050 gyro sensitivity scale factor
  gyroY = rawGyroY / 131.0;
  gyroZ = rawGyroZ / 131.0;

  // Apply drift compensation if calibrated
  if (gyroCalibrated) {
    gyroX -= gyroDriftX;
    gyroY -= gyroDriftY;
    gyroZ -= gyroDriftZ;
  }

  // Integrate gyroscope Z-axis for heading (yaw)
  // Only update heading during movement to avoid drift accumulation when stopped
  if (!motorsStopped && dt > 0 && dt < 1.0) { // Valid time step
    robotHeading += gyroZ * dt;
    // Normalize heading to -180 to 180 degrees
    while (robotHeading > 180.0) robotHeading -= 360.0;
    while (robotHeading < -180.0) robotHeading += 360.0;
  }

  // Calculate heading correction if enabled
  if (headingCorrectionEnabled && !motorsStopped) {
    headingError = targetHeading - robotHeading;
    // Normalize error to -180 to 180
    while (headingError > 180.0) headingError -= 360.0;
    while (headingError < -180.0) headingError += 360.0;

    // Apply PID correction
    headingCorrection = headingKp * headingError;
    // Limit correction to reasonable values
    headingCorrection = constrain(headingCorrection, -30.0, 30.0);
  } else {
    headingCorrection = 0.0;
  }
}

// Set target heading for heading correction (called when starting movement)
void setTargetHeading() {
  if (imuInitialized) {
    targetHeading = robotHeading;
    Serial.print("Target heading set to: ");
    Serial.print(targetHeading);
    Serial.println("°");
  }
}

// Apply IMU-based corrections to motor setpoints
void applyIMUCorrections() {
  if (!imuInitialized || !headingCorrectionEnabled || motorsStopped) return;

  // Apply heading correction to rotation component
  // This adds a small rotation correction to maintain straight line movement
  for (int i = 1; i <= 3; i++) { // Only apply to omni motors (2, 3, 4)
    if (motorIntendedActive[i]) { // Only correct motors that are intended to be active
      // Add heading correction to the motor setpoint
      setpoint[i] += headingCorrection * 0.1; // Small correction factor
      // Ensure setpoint stays within limits
      setpoint[i] = constrain(setpoint[i], -BASE_SPEED * speedMultiplier, BASE_SPEED * speedMultiplier);
    }
  }
}

// Reset heading and target (useful for recalibration)
void resetHeading() {
  robotHeading = 0.0;
  targetHeading = 0.0;
  headingCorrection = 0.0;
  Serial.println("Heading reset to 0°");
}

// Print enhanced status with encoder data and synchronization
void printStatus() {
  Serial.println("=== Enhanced Motor Status ===");
  Serial.print("Omni Motors Stopped: ");
  Serial.println(motorsStopped ? "YES" : "NO");
  Serial.print("Lifter Active: ");
  Serial.println(lifterActive ? "YES" : "NO");
  Serial.print("Synchronization: ");
  Serial.print(synchronizationActive ? "ACTIVE" : "INACTIVE");
  if (synchronizationActive) {
    Serial.print(" (Motors: ");
    for (int i = 1; i < 4; i++) {
      if (motorIntendedActive[i]) {
        Serial.print(i);
        Serial.print(" ");
      }
    }
    Serial.print("active)");
  }
  Serial.print(" | FastMode: ");
  Serial.print(fastRotationMode ? "ON" : "OFF");
  Serial.print(" | ForceStop: ");
  bool anyForceStop = false;
  for (int i = 1; i < 4; i++) {
    if (!motorIntendedActive[i] && !motorsStopped) {
      Serial.print(i);
      Serial.print(" ");
      anyForceStop = true;
    }
  }
  if (!anyForceStop) Serial.print("none");
  Serial.println("");
  Serial.println("Control Status: " + String((!motorsStopped || lifterActive) ? "ACTIVE" : "DISABLED"));

  // IMU Status
  Serial.print("IMU: ");
  if (imuInitialized) {
    Serial.print("ENABLED, Heading=");
    Serial.print(robotHeading, 1);
    Serial.print("°, Target=");
    Serial.print(targetHeading, 1);
    Serial.print("°, Correction=");
    Serial.print(headingCorrectionEnabled ? "ON" : "OFF");
    if (headingCorrectionEnabled) {
      Serial.print(" (");
      Serial.print(headingCorrection, 2);
      Serial.print(")");
    }
  } else {
    Serial.print("DISABLED");
  }
  Serial.println("");

  // Sensor Status
  Serial.println("=== Sensor Status ===");

  // IR Distance Sensors
  Serial.println("IR Distance Sensors (mm):");
  Serial.print("  Left: ");
  Serial.print(irLeft1.valid ? String(irLeft1.distance, 0) : "INVALID");
  Serial.print(" | ");
  Serial.println(irLeft2.valid ? String(irLeft2.distance, 0) : "INVALID");

  Serial.print("  Right: ");
  Serial.print(irRight1.valid ? String(irRight1.distance, 0) : "INVALID");
  Serial.print(" | ");
  Serial.println(irRight2.valid ? String(irRight2.distance, 0) : "INVALID");

  Serial.print("  Back: ");
  Serial.print(irBack1.valid ? String(irBack1.distance, 0) : "INVALID");
  Serial.print(" | ");
  Serial.println(irBack2.valid ? String(irBack2.distance, 0) : "INVALID");

  // Ultrasonic Sensors
  Serial.println("Ultrasonic Sensors (cm):");
  Serial.print("  Front Left: ");
  Serial.print(ultrasonicFrontLeft.valid ? String(ultrasonicFrontLeft.distance, 1) : "INVALID");
  Serial.print(" | Front Right: ");
  Serial.println(ultrasonicFrontRight.valid ? String(ultrasonicFrontRight.distance, 1) : "INVALID");

  // Line Sensors
  Serial.println("Line Sensors:");
  Serial.print("  Left: ");
  Serial.print(lineLeft.onLine ? "ON_LINE" : "OFF_LINE");
  Serial.print(" (");
  Serial.print(lineLeft.rawValue);
  Serial.print(") | Center: ");
  Serial.print(lineCenter.onLine ? "ON_LINE" : "OFF_LINE");
  Serial.print(" (");
  Serial.print(lineCenter.rawValue);
  Serial.print(") | Right: ");
  Serial.print(lineRight.onLine ? "ON_LINE" : "OFF_LINE");
  Serial.print(" (");
  Serial.print(lineRight.rawValue);
  Serial.println(")");
  Serial.println("==================");

  if (synchronizationActive) {
    Serial.print("Target Sync RPM: ");
    Serial.print(targetSyncRPM, 1);
    Serial.print(", Active Motors: ");
    Serial.println(activeMotorCount);
  }

  for (int i = 0; i < 4; i++) {
    Serial.print("Motor ");
    Serial.print(i + 1);
    Serial.print(": Setpoint=");
    Serial.print(setpoint[i], 1);
    Serial.print(" RPM, Smoothed=");
    Serial.print(smoothedRPM[i], 1);
    Serial.print(" RPM, SyncErr=");
    Serial.print(syncError[i], 1);
    Serial.print(", Position=");
    Serial.print(motorPosition[i], 2);
    Serial.print(" rev, Output=");
    Serial.println(output[i], 1);
  }
  Serial.println("==================");
}

void setSpeed(float multiplier) {
  speedMultiplier = constrain(multiplier, 0.1, 1.0);
}

// Calibration sequence
void calibrateMotors() {
  Serial.println("Starting motor calibration...");

  // Test each motor individually
  for (int i = 1; i <= 4; i++) {
    Serial.print("Calibrating Motor ");
    Serial.println(i);
    testMotor(i);
    delay(1000);
  }

  // Test basic movements
  Serial.println("Testing forward movement...");
  moveForward();
  delay(2000);
  stopMotors();

  Serial.println("Testing rotation...");
  rotateCW();
  delay(3000);
  stopMotors();

  Serial.println("Calibration complete!");
}

// Encoder calibration function
void calibrateEncoders() {
  Serial.println("Calibrating encoders and resetting position tracking...");

  // Reset all encoder tracking data
  for (int i = 0; i < 4; i++) {
    encoderCount[i] = 0;
    lastEncoderCount[i] = 0;
    totalEncoderCount[i] = 0;
    motorPosition[i] = 0;
    smoothedRPM[i] = 0;
    filteredRPM[i] = 0;
    lastRPM[i] = 0;
    syncError[i] = 0;

    // Reset history buffers
    for (int j = 0; j < RPM_FILTER_SIZE; j++) {
      rpmHistory[i][j] = 0;
    }
    rpmHistoryIndex[i] = 0;
  }

  // Reset synchronization data
  targetSyncRPM = 0;
  synchronizationActive = false;
  activeMotorCount = 0;

  Serial.println("Encoder calibration complete - all positions and sync data reset to zero");
}

// Figure-8 pattern for testing
void figureEight() {
  Serial.println("Starting Figure-8 pattern with smooth encoder-controlled movements...");

  // First loop - right turn with smooth transitions
  Serial.println("Right loop...");
  setOmniSpeeds(0.6, 0.0, 0.8);  // Forward + right turn
  delay(3000);

  // Smooth transition
  stopMotors();
  delay(1000);  // Longer pause for encoder stabilization

  // Second loop - left turn
  Serial.println("Left loop...");
  setOmniSpeeds(0.6, 0.0, -0.8); // Forward + left turn
  delay(3000);

  stopMotors();
  Serial.println("Figure-8 complete - encoder data stabilized!");
}

// Test individual motor at FULL POWER
void testMotor(int motorIndex) {
  Serial.print("FULL POWER Test - Motor ");
  Serial.print(motorIndex);
  switch(motorIndex) {
    case 1: Serial.println(" (Lifter)"); break;
    case 2: Serial.println(" (Front Right)"); break;
    case 3: Serial.println(" (Front Left)"); break;
    case 4: Serial.println(" (Back)"); break;
  }
  Serial.println("Power level: 4000/4096 (97% max) - 3 seconds each direction");

  bool wasStopped = motorsStopped;    // Remember previous omni state
  bool wasLifterActive = lifterActive; // Remember previous lifter state

  // Temporarily disable all automatic control for testing
  motorsStopped = true;
  lifterActive = false;

  // Test forward - FULL POWER
  Serial.println("Forward (FULL POWER)...");
  motorDriver.setSingleMotor(motorIndex, 4000);  // Near maximum speed
  delay(3000);

  // Stop
  motorDriver.stopMotor(motorIndex);
  delay(1000);

  // Test backward - FULL POWER
  Serial.println("Backward (FULL POWER)...");
  motorDriver.setSingleMotor(motorIndex, -4000); // Near maximum speed reverse
  delay(3000);

  // Stop
  motorDriver.stopMotor(motorIndex);

  // Restore previous states
  motorsStopped = wasStopped;
  lifterActive = wasLifterActive;

  Serial.println("Test complete");
}
