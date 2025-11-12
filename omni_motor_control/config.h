// Configuration file for Hexagonal 3-Wheel Omni Robot Control
// Contains all configurable constants and PID tunings

#ifndef CONFIG_H
#define CONFIG_H

// === ENCODER CONFIGURATION ===
#define RPM_FILTER_SIZE 5                    // Number of samples for RPM filtering
const int ENCODER_CPR = 28;                  // Counts per revolution (for PG28 motors)
const double GEAR_RATIO = 1.0;              // Gear ratio (adjust if geared motors)

// === MOTOR SPEED LIMITS ===
const double MAX_RPM = 100.0;               // Maximum RPM for motors
const double BASE_SPEED = 60.0;             // Base speed for movements
const double TURN_SPEED = 40.0;             // Speed for turning movements
const double LIFT_SPEED = 50.0;             // Speed for lifter motor

// === PID CONFIGURATION ===
const double PID_SAMPLE_TIME = 100;         // PID sample time in ms
const int PID_OUTPUT_LIMIT_MIN = -255;      // PID output minimum (PWM)
const int PID_OUTPUT_LIMIT_MAX = 255;       // PID output maximum (PWM)

// PID Tunings (Kp, Ki, Kd) for each motor
const double Lifter_Kp = 2.0;               // Lifter PID - conservative
const double Lifter_Ki = 0.5;
const double Lifter_Kd = 0.1;

const double Omni_Kp = 8.0;                 // Omni motors - aggressive for instant response
const double Omni_Ki = 3.0;
const double Omni_Kd = 0.2;

// === MOTOR SYNCHRONIZATION ===
const double SYNC_KP = 0.3;                 // Synchronization PID gain (lower for stability)

// === ACCELERATION LIMITING ===
const double MAX_RPM_CHANGE = 200.0;        // Maximum RPM change per sample (for smooth acceleration)
const double MAX_RPM_CHANGE_ROTATION = 1000.0; // Aggressive changes for rotation (c/w commands)

// === FILTERING AND SMOOTHING ===
const double RPM_ALPHA = 0.6;               // Exponential smoothing factor (higher = more responsive, 0.1-0.9)
const double RPM_ALPHA_FAST = 0.9;          // Ultra-responsive smoothing during fast rotation

// === IMU CONFIGURATION ===
const double HEADING_KP = 2.0;              // Heading correction PID gain
const int GYRO_CALIBRATION_SAMPLES = 100;   // Number of samples for gyro calibration

// === MOTOR ANGLES (degrees) ===
// Hexagonal 3-wheel robot with 135° wheel spacing
const double MOTOR_ANGLES[4] = {0, 45, 135, 180};  // Lifter, FR, FL, Back

#endif // CONFIG_H
