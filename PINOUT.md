# Arduino Mega Pinout Reference - Hexagonal 3-Wheel Omni Robot

## Overview
Complete pinout documentation for the Arduino Mega-based omni robot control system with motor control, sensors, and IMU integration.

---

## Arduino Mega Pin Assignments

### Digital Pins (0-53)

#### Motor Encoder Inputs (Direct Connection from Motors)
```
D2  → Motor 2 (Front Right) Encoder A
D3  → Motor 1 (Lifter) Encoder A
D4  → Motor 2 (Front Right) Encoder B
D5  → Motor 1 (Lifter) Encoder B
D6  → Motor 3 (Front Left) Encoder B
D7  → Motor 3 (Front Left) Encoder A
D8  → Motor 4 (Back) Encoder B
D9  → Motor 4 (Back) Encoder A
```

#### Ultrasonic Sensor Control (HC-SR04)
```
D22 → Ultrasonic Front Left TRIGGER
D23 → Ultrasonic Front Left ECHO
D24 → Ultrasonic Front Right TRIGGER
D25 → Ultrasonic Front Right ECHO
```

#### Unused Digital Pins (Available for Expansion)
```
D0, D1   → Serial RX/TX (Reserved)
D10-D13  → SPI (Available)
D14-D21  → I2C/Timer (D20=D20/SDA, D21=D21/SCL - Reserved for I2C)
D26-D53  → Available for expansion
```

---

### Analog Pins (A0-A15)

#### IR Distance Sensors (Sharp GP2Y0A02YK0F)
```
A0  → IR Distance Left 1
A1  → IR Distance Left 2
A2  → IR Distance Right 1
A3  → IR Distance Right 2
A9  → IR Distance Back 1  (moved from A4 - I2C conflict on Uno shields)
A10 → IR Distance Back 2  (moved from A5 - I2C conflict on Uno shields)
```

#### Line Sensors (Analog Input)
```
A6  → Line Sensor Left
A7  → Line Sensor Center
A8  → Line Sensor Right
```

#### Unused Analog Pins (Available for Expansion)
```
A9-A15 → Available for expansion
```

---

## YFROBOT v2 Motor Driver Shield Connections

### I2C Communication (Required)
```
YFROBOT SDA → Arduino D20 (SDA)
YFROBOT SCL → Arduino D21 (SCL)
YFROBOT GND → Arduino GND
YFROBOT VCC → Arduino 5V
```

**Important Notes:**
- Motor control is handled entirely via I2C communication
- No direct PWM or DIR pin connections needed
- Shield has onboard microcontroller for motor control
- Ensure proper I2C pull-up resistors if needed

---

## MPU6050 IMU Sensor Connections

### I2C Communication (Shares bus with motor driver)
```
MPU6050 SDA → Arduino D20 (SDA)
MPU6050 SCL → Arduino D21 (SCL)
MPU6050 GND → Arduino GND
MPU6050 VCC → Arduino 3.3V (IMPORTANT: Use 3.3V, NOT 5V!)
```

**Important Notes:**
- MPU6050 requires 3.3V power supply (5V will damage the sensor)
- Default I2C address: 0x68
- May have I2C bus conflicts with motor driver
- IMU is optional - robot functions without it

---

## Motor Connections (PG28 Motors with Encoders)

### Motor 1 - Lifter Motor
```
Motor Terminals → YFROBOT Shield Motor 1 (M1+/M1-)
Encoder d3 → Arduino D3 (Data A)
Encoder d5 → Arduino D5 (Data B)
Power → YFROBOT Shield (from Arduino 5V/GND via I2C)
```

### Motor 2 - Front Right Wheel
```
Motor Terminals → YFROBOT Shield Motor 2 (M2+/M2-)
Encoder d2 → Arduino D2 (Data A)
Encoder d4 → Arduino D4 (Data B)
Power → YFROBOT Shield (from Arduino 5V/GND via I2C)
```

### Motor 3 - Front Left Wheel
```
Motor Terminals → YFROBOT Shield Motor 3 (M3+/M3-)
Encoder d7 → Arduino D7 (Data A)
Encoder d6 → Arduino D6 (Data B)
Power → YFROBOT Shield (from Arduino 5V/GND via I2C)
```

### Motor 4 - Back Wheel
```
Motor Terminals → YFROBOT Shield Motor 4 (M4+/M4-)
Encoder d9 → Arduino D9 (Data A)
Encoder d8 → Arduino D8 (Data B)
Power → YFROBOT Shield (from Arduino 5V/GND via I2C)
```

---

## Sensor Connections Summary

### IR Distance Sensors (Sharp GP2Y0A02YK0F)
```
Sensor  | Arduino Pin | Purpose
---------|-------------|---------
IR Left 1  | A0         | Left wall alignment
IR Left 2  | A1         | Left wall alignment (redundant)
IR Right 1 | A2         | Right wall alignment
IR Right 2 | A3         | Right wall alignment (redundant)
IR Back 1  | A9         | Back wall alignment (moved from A4 - I2C conflict)
IR Back 2  | A10        | Back wall alignment (redundant, moved from A5 - I2C conflict)
```

**IR Sensor Specifications:**
- Range: 20-150cm (200-1500mm)
- Output: Analog voltage 0.4V-2.7V
- Power: Connect to Arduino 5V/GND

### Ultrasonic Sensors (HC-SR04)
```
Sensor          | Trigger Pin | Echo Pin | Purpose
----------------|-------------|----------|---------
Front Left      | D22        | D23     | Front left obstacle detection
Front Right     | D24        | D25     | Front right obstacle detection
```

**Ultrasonic Sensor Specifications:**
- Range: 2-400cm
- Trigger: 10μs pulse to start measurement
- Echo: Pulse width proportional to distance
- Power: Connect to Arduino 5V/GND

### Line Sensors
```
Sensor  | Arduino Pin | Purpose
---------|-------------|---------
Left    | A6         | Left line detection
Center  | A7         | Center line detection
Right   | A8         | Right line detection
```

**Line Sensor Specifications:**
- Input: Analog voltage (0-5V)
- Output: Higher values = darker surface
- Threshold: Configurable (default 512)
- Power: Connect to Arduino 5V/GND

---

## Power Connections

### Main Power (Arduino Mega)
```
VIN → External power supply (7-12V recommended)
GND → Power ground
5V → Logic power output (for sensors)
3.3V → IMU power (MPU6050 only)
```

### Motor Power (Via YFROBOT Shield)
```
Motor Power → YFROBOT Shield motor terminals (M+/M-)
Logic Power → Arduino 5V/GND (via I2C)
```

### Sensor Power
```
All Sensors → Arduino 5V/GND (except IMU which uses 3.3V)
IR Sensors → Arduino 5V/GND
Ultrasonic → Arduino 5V/GND
Line Sensors → Arduino 5V/GND
MPU6050 IMU → Arduino 3.3V/GND (IMPORTANT!)
```

---

## Pin Usage Summary

### Used Pins (35 total)
```
Digital: D2, D3, D4, D5, D6, D7, D8, D9, D20, D21, D22, D23, D24, D25
Analog: A0, A1, A2, A3, A6, A7, A8, A9, A10
```

### Reserved Pins (Serial/I2C)
```
D0, D1 → Serial communication
D20, D21 → I2C (SDA/SCL) - shared between motor driver and IMU
```

### Available Pins (18 digital + 5 analog = 23 total)
```
Digital: D10-D19, D26-D53
Analog: A4, A5, A11-A15  (A4/A5 now available - moved IR sensors to A9/A10)
```

---

## Connection Checklist

### Required Connections
- [ ] YFROBOT Shield I2C (SDA, SCL, GND, 5V)
- [ ] Motor encoder wires to Arduino digital pins
- [ ] Motor power to YFROBOT shield terminals
- [ ] Arduino main power (VIN/GND)

### Optional Connections
- [ ] MPU6050 IMU (SDA, SCL, 3.3V, GND) - requires 3.3V power
- [ ] IR Distance sensors (A0-A5, 5V, GND)
- [ ] Ultrasonic sensors (D22-D25, 5V, GND)
- [ ] Line sensors (A6-A8, 5V, GND)

### Verification Steps
1. Check all encoder connections (8 wires total)
2. Verify I2C connections (4 wires for shield)
3. Confirm motor power connections
4. Test sensor power (3.3V for IMU, 5V for others)
5. Verify no pin conflicts

---

## Troubleshooting Pin Issues

### Common Problems
- **Motor not moving**: Check encoder connections and I2C wiring
- **IMU not detected**: Verify 3.3V power and I2C connections
- **Invalid sensor readings**: Check sensor power and pin assignments
- **I2C conflicts**: Motor driver and IMU share the same bus

### Testing Commands
- `1-4`: Individual motor tests
- `p`: Status display (includes sensor readings)
- `i`: IMU status
- `!`: Detailed sensor readings
- `k`: IMU calibration retry

---

## Expansion Options

### Available Digital Pins (D10-D19, D26-D53)
- Additional ultrasonic sensors
- Servo motors for mechanisms
- LED indicators
- Button inputs
- Additional motor encoders

### Available Analog Pins (A9-A15)
- Additional IR distance sensors
- Additional line sensors
- Potentiometers for manual tuning
- Battery voltage monitoring
- Temperature sensors

---

## Wiring Diagram Summary

```
Arduino Mega
├── Digital Pins
│   ├── D2-D9: Motor encoders (8 pins)
│   ├── D20-D21: I2C bus (shared)
│   ├── D22-D25: Ultrasonic control (4 pins)
│   └── D0-D1, D10-D19, D26-D53: Available
├── Analog Pins
│   ├── A0-A5: IR distance sensors (6 pins)
│   ├── A6-A8: Line sensors (3 pins)
│   └── A9-A15: Available (7 pins)
└── Power
    ├── VIN: Main power input
    ├── 5V: Sensor power output
    ├── 3.3V: IMU power output
    └── GND: Common ground
```

---
*Last updated: November 12, 2025*
*Total pins used: 35/69 (51% utilization)*
