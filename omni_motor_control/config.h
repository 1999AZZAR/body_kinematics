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
#define ENCODER_CPR         28    // Counts per revolution for PG28
#define GEAR_RATIO          1.0   // Gear ratio (adjust if motors are geared)
#define MAX_RPM             100.0 // Maximum RPM for motors
#define BASE_SPEED          60.0  // Base speed for movements (RPM)
#define TURN_SPEED          40.0  // Speed for turning movements (RPM)
#define LIFT_SPEED          50.0  // Speed for lifter motor (RPM)

// === PID PARAMETERS ===
// Tune these values for your specific motors and load
#define PID_KP              2.0   // Proportional gain
#define PID_KI              0.5   // Integral gain
#define PID_KD              0.1   // Derivative gain
#define PID_SAMPLE_TIME     100   // PID sample time in milliseconds
#define PID_OUTPUT_MIN      -255  // Minimum PWM output
#define PID_OUTPUT_MAX      255   // Maximum PWM output

// === OMNI WHEEL CONFIGURATION ===
// Motor angles in degrees from forward direction (counter-clockwise)
// Adjust these based on your physical wheel placement
#define MOTOR1_ANGLE        0     // Lifter (not used for omni)
#define MOTOR2_ANGLE        315   // Front Left
#define MOTOR3_ANGLE        45    // Front Right
#define MOTOR4_ANGLE        180   // Back

// === SERIAL COMMUNICATION ===
#define SERIAL_BAUD_RATE    115200

// === DEBUG SETTINGS ===
#define ENABLE_SERIAL_DEBUG true
#define DEBUG_UPDATE_RATE   500   // Debug print interval in ms

#endif // CONFIG_H
