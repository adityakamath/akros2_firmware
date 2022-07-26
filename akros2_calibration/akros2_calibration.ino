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

#include "akros2_calib_config.h"
#include "src/motor/motor.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "src/encoder/encoder.h"
#include "src/kinematics/kinematics.h"

#define SAMPLE_TIME 10 //seconds
#define LED_PIN 13
#define MEASURED_VOLTAGE constrain(MOTOR_POWER_MEASURED_VOLTAGE, 0, MOTOR_OPERATING_VOLTAGE)
#define MOTOR_POWER_RATIO MEASURED_VOLTAGE/MOTOR_OPERATING_VOLTAGE

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

Kinematics kinematics(
    Kinematics::AKROS2_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE
);

int total_motors = 4;
Motor *motors[4] = {&motor1_controller, &motor2_controller, &motor3_controller, &motor4_controller};
Encoder *encoders[4] = {&motor1_encoder, &motor2_encoder, &motor3_encoder, &motor4_encoder};
String labels[4] = {"FRONT LEFT  - M1: ", "FRONT RIGHT - M2: ", "REAR  LEFT  - M3: ", "REAR  RIGHT - M4: "};

void setup()
{
  Serial.begin(9600);
  while (!Serial){}

  Serial.println("Please ensure that the robot is ELEVATED and there are NO OBSTRUCTIONS to the wheels.");
  Serial.println("");
  Serial.println("1: SPIN: Type 'spin' + enter");
  Serial.println("\t Used to check motor directions.");
  Serial.println("\t All motors must rotate in the forward direction.");
  Serial.println("\t Update config.h if any motor is rotating in the opposite direction");
  Serial.println("2: SAMPLE: Type 'sample' + enter. Encoder CPRs must already be set in config.h");
  Serial.println("\t Used to calculate max RPMs of each motor");
  Serial.println("\t Motor directions and Encoder CPRs must be set correctly in config.h");
  Serial.println("\t Update config.h with the average max RPM value and run again");

  Serial.println("");
  Serial.println("If config.h is updated, the sketch must be compiled/uploaded again");
  Serial.println("\r\n=======================================================");
}

void loop()
{
  static String cmd = "";

  while (Serial.available())
  {
    char character = Serial.read();
    cmd.concat(character);
    Serial.print(character);
    delay(1);
    if(character == '\n' and cmd.equals("spin\n"))
      {
        cmd = "";
        Serial.println("\r\n");
        sampleMotors(0);
      }
      else if(character == '\n' and cmd.equals("sample\n"))
      {
        cmd = "";
        Serial.println("\r\n");
        sampleMotors(1);
      }
      else if(character == '\n')
      {
        Serial.println("");
        cmd = "";
      }
  }
}

void sampleMotors(bool show_summary)
{
  if(Kinematics::AKROS2_BASE == Kinematics::DIFFERENTIAL_DRIVE)
  {
    total_motors = 2;
  }

  Serial.print("The motors will spin one by one at PWM_MAX (");
  Serial.print((int)PWM_MAX);
  Serial.print(") for ");
  Serial.print(SAMPLE_TIME);
  Serial.println(" seconds.");

  for(int i=0; i<total_motors; i++)
  {
    Serial.print("SPINNING ");
    Serial.print(labels[i]);

    unsigned long start_time = micros();
    unsigned long last_status = micros();

    encoders[i]->write(0);
    while(true)
    {
      if(micros() - start_time >= SAMPLE_TIME * 1000000)
      {
        motors[i]->spin(0);
        Serial.println("");
        break;
      }

      if(micros() - last_status >= 1000000)
      {
        last_status = micros();
        Serial.print(".");
      }

      motors[i]->spin(PWM_MAX);
    }
  }
  if(show_summary)
  {
    printSummary();
  }

}

void printSummary()
{
  float cpr[4];
  float rpm[4];
  float cprc[4];

  cprc[0] = COUNTS_PER_REV1;
  cprc[1] = COUNTS_PER_REV2;
  cprc[2] = COUNTS_PER_REV3;
  cprc[3] = COUNTS_PER_REV4;

  for(int i=0; i<total_motors; i++)
  {
    rpm[i] = (encoders[i]->read()*(60.0/SAMPLE_TIME))/(cprc[i]*MOTOR_POWER_RATIO);
  }

  float average_max_rpm = (rpm[0]+rpm[1]+rpm[2]+rpm[3])/total_motors;
  float total_rev = average_max_rpm*MOTOR_POWER_RATIO*(SAMPLE_TIME/60.0);

  for(int j=0; j<total_motors; j++)
  {
    cpr[j] = encoders[j]->read()/total_rev;
  }

  Serial.println("\r\n====================MOTOR_MAX_RPM=====================");
  for(int j=0; j<total_motors; j+=2)
  {
    Serial.print(labels[j]);
    Serial.print(rpm[j]);
    Serial.print("\t");
    Serial.print(labels[j+1]);
    Serial.println(rpm[j+1]);
  }

  Serial.print("Average MOTOR_MAX_RPM: ");
  Serial.println(average_max_rpm);

  Serial.println("Update MOTOR_MAX_RPM in config.h and build/run calibration sketch again");

  Serial.println("\r\n====================COUNTS_PER_REVx====================");
  for(int k=0; k<total_motors; k++)
  {
    Serial.print(labels[k]);
    Serial.print(" Calculated: +- ");
    Serial.print(cpr[k]);
    Serial.print(" Config: ");
    Serial.print(cprc[k]);
    Serial.print(" Deviation: ~ ");
    Serial.print((cprc[k] - cpr[k])*100/cprc[k]);
    Serial.println("%");
  }

  Serial.println("Calculated values must match config.h values (COUNTS_PER_REVx) with +/- 5% acceptable deviation");

  Serial.println("\r\n====================MAX_VELOCITIES====================");
  float max_rpm = kinematics.getMaxRPM();

  Kinematics::velocities max_linear = kinematics.getVelocities(max_rpm, max_rpm, max_rpm, max_rpm);
  Kinematics::velocities max_angular = kinematics.getVelocities(-max_rpm, max_rpm,-max_rpm, max_rpm);

  Serial.print("Linear Velocity X: +- ");
  Serial.print(max_linear.linear_x);
  Serial.println(" m/s");

  Serial.print("Linear Velocity Y: +- ");
  Serial.print(max_linear.linear_y);
  Serial.println(" m/s");

  Serial.print("Angular Velocity Z: +- ");
  Serial.print(max_angular.angular_z);
  Serial.println(" rad/s");

  Serial.println("Update config.h if necessary. If changed, build/run calibration sketch again");
}