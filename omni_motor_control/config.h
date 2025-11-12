// Configuration file for Omni Wheel Robot Control
// Modify these values according to your robot setup

#ifndef CONFIG_H
#define CONFIG_H

// === MOTOR PIN CONFIGURATION ===
// R27889 Shield connections to Arduino Mega
#define MOTOR1_PWM    3    // Lifter motor PWM
#define MOTOR1_DIR    5    // Lifter motor direction
#define MOTOR2_PWM    2    // Front Left motor PWM
#define MOTOR2_DIR    4    // Front Left motor direction
#define MOTOR3_PWM    7    // Front Right motor PWM
#define MOTOR3_DIR    6    // Front Right motor direction
#define MOTOR4_PWM    9    // Back motor PWM
#define MOTOR4_DIR    8    // Back motor direction

// === ENCODER PIN CONFIGURATION ===
// Encoder Data A and Data B pins
#define MOTOR1_ENCA   A0   // Lifter encoder A
#define MOTOR1_ENCB   A1   // Lifter encoder B
#define MOTOR2_ENCA   A2   // Front Left encoder A
#define MOTOR2_ENCB   A3   // Front Left encoder B
#define MOTOR3_ENCA   A4   // Front Right encoder A
#define MOTOR3_ENCB   A5   // Front Right encoder B
#define MOTOR4_ENCA   A6   // Back encoder A
#define MOTOR4_ENCB   A7   // Back encoder B

// === MOTOR SPECIFICATIONS ===
#define CONFIG_ENCODER_CPR      28    // Counts per revolution for PG28
#define CONFIG_GEAR_RATIO       1.0   // Gear ratio (adjust if motors are geared)
#define CONFIG_MAX_RPM          100.0 // Maximum RPM for motors
#define CONFIG_BASE_SPEED       60.0  // Base speed for movements (RPM)
#define CONFIG_TURN_SPEED       40.0  // Speed for turning movements (RPM)
#define CONFIG_LIFT_SPEED       50.0  // Speed for lifter motor (RPM)

// === PID PARAMETERS ===
// Tune these values for your specific motors and load
#define CONFIG_PID_KP           2.0   // Proportional gain
#define CONFIG_PID_KI           0.5   // Integral gain
#define CONFIG_PID_KD           0.1   // Derivative gain
#define CONFIG_PID_SAMPLE_TIME  100   // PID sample time in milliseconds
#define CONFIG_PID_OUTPUT_MIN   -255  // Minimum PWM output
#define CONFIG_PID_OUTPUT_MAX   255   // Maximum PWM output

// === OMNI WHEEL CONFIGURATION ===
// Motor angles in degrees from forward direction (counter-clockwise)
// CORRECTED for hexagonal 3-wheel robot with 135° wheel spacing
// FL=135°, FR=45°, Back=180° for proper forward movement (FL and FR counter-rotate)
#define MOTOR1_ANGLE        0     // Lifter (not used for omni)
#define MOTOR2_ANGLE        45    // Front Right (45°)
#define MOTOR3_ANGLE        135   // Front Left (135°)
#define MOTOR4_ANGLE        180   // Back (180°)

// Array format for calculations (use MOTOR_ANGLES_ARRAY in code initialization)
#define MOTOR_ANGLES_ARRAY_VALUES 0, 45, 135, 180

// === ENCODER PROCESSING ===
#define RPM_FILTER_SIZE     5     // Moving average filter size
#define RPM_ALPHA           0.6   // Exponential smoothing factor (normal)
#define RPM_ALPHA_FAST      0.9   // Ultra-responsive smoothing (fast rotation)

// === ACCELERATION LIMITING ===
#define MAX_RPM_CHANGE      200.0 // Maximum RPM change per sample (normal)
#define MAX_RPM_CHANGE_ROTATION 1000.0 // Maximum RPM change for rotation
#define MAX_RPM_CHANGE_FAST 50.0  // Maximum RPM change during fast rotation

// === MOTOR SYNCHRONIZATION ===
#define SYNC_KP             0.3   // Synchronization PID gain

// === IMU CONFIGURATION ===
#define GYRO_CALIBRATION_SAMPLES 100  // Number of samples for gyro calibration
#define GYRO_CALIBRATION_DELAY  5     // Delay between calibration samples (ms)
#define CONFIG_HEADING_KP            2.0     // Heading correction PID gain
#define IMU_UPDATE_INTERVAL   10      // IMU update interval (ms)

// === FAST ROTATION MODE ===
#define FAST_ROTATION_DURATION 5000   // Fast mode duration (ms)
#define FAST_ROTATION_TIMEOUT  6000   // Auto-disable after this time (ms)

// === SERIAL COMMUNICATION ===
#define SERIAL_BAUD_RATE    115200

// === DEBUG SETTINGS ===
#define ENABLE_SERIAL_DEBUG true
#define DEBUG_UPDATE_RATE   500   // Debug print interval in ms

#endif // CONFIG_H
