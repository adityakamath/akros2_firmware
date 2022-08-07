// Copyright (c) 2022 Aditya Kamath
// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AKROS2_CALIB_CONFIG_H
#define AKROS2_CALIB_CONFIG_H

//DRIVE: uncomment the base you're building
#define AKROS2_BASE MECANUM                 // Mecanum drive robot
//#define AKROS2_BASE DIFFERENTIAL_DRIVE      // 2WD and Tracked robot w/ 2 motors
//#define AKROS2_BASE SKID_STEER              // 4WD robot

//MOTOR DRIVER: uncomment the motor driver you're using
#define USE_BTS7960_MOTOR_DRIVER            // BTS7970 Motor Driver
//#define USE_GENERIC_2_IN_MOTOR_DRIVER       // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin ie. L298, L293, VNH5019
//#define USE_GENERIC_1_IN_MOTOR_DRIVER       // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
//#define USE_ESC_MOTOR_DRIVER                // Motor ESC for brushless motors

//IMU: uncomment the IMU you're using
#define USE_GY85_IMU
//#define USE_MPU6050_IMU
//#define USE_MPU9150_IMU
//#define USE_MPU9250_IMU

//PID
#define K_P 0.6                             // P constant //TODO
#define K_I 0.8                             // I constant //TODO
#define K_D 0.5                             // D constant //TODO
#define UPDATE_RATE 20                      // Control timer timeout in ms (20ms = 50Hz)

//ROBOT ORIENTATION
/*
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

//ROBOT SPECS
#define MOTOR_MAX_RPM 180                   // motor's max RPM
#define MAX_RPM_RATIO 1                     // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO //TODO
#define MOTOR_OPERATING_VOLTAGE 12          // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 9           // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 9      // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 288                 // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 304                 // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 294                 // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 296                 // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.0800               // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.210            // distance between left and right wheels
#define PWM_BITS 8                          // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 50000                 // PWM Frequency

//INVERT ENCODER COUNTS
#define MOTOR1_ENCODER_INV false
#define MOTOR2_ENCODER_INV true
#define MOTOR3_ENCODER_INV false
#define MOTOR4_ENCODER_INV true

//ENCODER PINS
#define MOTOR1_ENCODER_A 31
#define MOTOR1_ENCODER_B 29

#define MOTOR2_ENCODER_A 33
#define MOTOR2_ENCODER_B 8

#define MOTOR3_ENCODER_A 30
#define MOTOR3_ENCODER_B 28

#define MOTOR4_ENCODER_A 32
#define MOTOR4_ENCODER_B 9

//INVERT MOTOR DIRECTIONS
#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV false
#define MOTOR4_INV false

//MOTOR PINS
#ifdef USE_GENERIC_2_IN_MOTOR_DRIVER
  #define MOTOR1_PWM 21 //Pin no 21 is not a PWM pin on Teensy 4.x, you can swap it with pin no 1 instead.
  #define MOTOR1_IN_A 20
  #define MOTOR1_IN_B 1

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A 6
  #define MOTOR2_IN_B 8

  #define MOTOR3_PWM 22
  #define MOTOR3_IN_A 23
  #define MOTOR3_IN_B 0

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 3
  #define MOTOR4_IN_B 2

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

#ifdef USE_GENERIC_1_IN_MOTOR_DRIVER
  #define MOTOR1_PWM 21 //Pin no 21 is not a PWM pin on Teensy 4.x, you can use pin no 1 instead.
  #define MOTOR1_IN_A 20
  #define MOTOR1_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A 6
  #define MOTOR2_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR3_PWM 22
  #define MOTOR3_IN_A 23
  #define MOTOR3_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 3
  #define MOTOR4_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

#ifdef USE_BTS7960_MOTOR_DRIVER
  #define MOTOR1_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 3 // Pin no 21 is not a PWM pin on Teensy 4.x, you can use pin no 1 instead.
  #define MOTOR1_IN_B 2 // Pin no 20 is not a PWM pin on Teensy 4.x, you can use pin no 0 instead.

  #define MOTOR2_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 7
  #define MOTOR2_IN_B 6

  #define MOTOR3_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_A 37
  #define MOTOR3_IN_B 36

  #define MOTOR4_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_A 5
  #define MOTOR4_IN_B 4

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

#ifdef USE_ESC_MOTOR_DRIVER
  #define MOTOR1_PWM 21 //Pin no 21 is not a PWM pin on Teensy 4.x. You can use pin no 1 instead.
  #define MOTOR1_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR3_PWM 22
  #define MOTOR3_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define PWM_MAX 400
  #define PWM_MIN -PWM_MAX
#endif

#endif
