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

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <FastLED.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

#include <akros2_msgs/msg/Mode.h>
#include <akros2_msgs/msg/Velocities.h>
#include <akros2_msgs/msg/Feedback.h>

#include "akros2_base_config.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "src/encoder/encoder.h"
#include "src/motor/motor.h"
#include "src/kinematics/kinematics.h"
#include "src/pid/pid.h"
#include "src/odometry/odometry.h"
#include "src/imu/imu.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){errorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rcl_publisher_t    odom_publisher;
rcl_publisher_t    imu_publisher;
rcl_publisher_t    feedback_publisher;
rcl_subscription_t twist_subscriber;
rcl_subscription_t mode_subscriber;

akros2_msgs__msg__Mode       mode_msg;
akros2_msgs__msg__Velocities cur_rpm_msg;
akros2_msgs__msg__Velocities req_rpm_msg;
akros2_msgs__msg__Feedback   feedback_msg;
nav_msgs__msg__Odometry      odom_msg;
sensor_msgs__msg__Imu        imu_msg;
geometry_msgs__msg__Twist    twist_msg;

rclc_executor_t          executor;
rclc_support_t           support;
rcl_init_options_t       init_options;
rcl_allocator_t          allocator;
rcl_node_t               node;
rcl_timer_t              timer;
rclc_parameter_server_t  param_server;
rclc_parameter_options_t param_options;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

CRGB neopixel[NEOPIXEL_COUNT];

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
  Kinematics::LINO_BASE,
  MOTOR_MAX_RPM,
  MAX_RPM_RATIO,
  MOTOR_OPERATING_VOLTAGE,
  MOTOR_POWER_MAX_VOLTAGE,
  WHEEL_DIAMETER,
  LR_WHEELS_DISTANCE
);

Odometry odometry;
IMU imu;

const char * kp_name = "motor_kp";
const char * ki_name = "motor_ki";
const char * kd_name = "motor_kd";

double kp, ki, kd;
float cur_rpm1, cur_rpm2, cur_rpm3, cur_rpm4;
float req_rpm1, req_rpm2, req_rpm3, req_rpm4;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  FastLED.addLeds<NEOPIXEL, NEOPIXEL_PIN>(neopixel, NEOPIXEL_COUNT);

  bool imu_ok = imu.init();
  if(!imu_ok)
  {
    while(1)
    {
      flashLED(3);
    }
  }

  #ifdef TRANSPORT_SERIAL
    set_microros_transports();
  #endif

  #ifdef TRANSPORT_ETHERNET
    byte mac[6];
    getTeensyMAC(mac);

    IPAddress teensy_ip(192, 168, 1, 177);
    IPAddress agent_ip(192, 168, 1, 113);

    set_microros_native_ethernet_udp_transports(mac, teensy_ip, agent_ip, 9999);
  #endif

  setNeopixel(toCRGB(0, 255, 255)); // STARTUP: Cyan
  FastLED.show();
}

void loop() {
  switch (state)
  {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT)
      {
        destroyEntities();
      }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED)
      {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  state == AGENT_CONNECTED ? digitalWrite(LED_PIN, LOW) : digitalWrite(LED_PIN, HIGH);
}

void timerCallback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL)
  {
    updateMode();
    FastLED.show();
    moveBase();
    publishData();
  }
}

void twistCallback(const void * msgin)
{
  prev_cmd_time = millis();
}

void modeCallback(const void * msgin){}

void paramCallback(Parameter * param)
{
  // Get parameter values from parameter server
  RCSOFTCHECK(rclc_parameter_get_double(&param_server, kp_name, &kp));
  RCSOFTCHECK(rclc_parameter_get_double(&param_server, ki_name, &ki));
  RCSOFTCHECK(rclc_parameter_get_double(&param_server, kd_name, &kd));

  motor1_pid.updateConstants(kp, ki, kd);
  motor2_pid.updateConstants(kp, ki, kd);
  motor3_pid.updateConstants(kp, ki, kd);
  motor4_pid.updateConstants(kp, ki, kd);
}

bool createEntities()
{
  //create node
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, (size_t)ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE, NAMESPACE, &support));

  // create odometry publisher
  RCCHECK(rclc_publisher_init_default(
          &odom_publisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
          "odom"));

  // create imu publisher
  RCCHECK(rclc_publisher_init_default(
          &imu_publisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
          "imu"));

  // create pid control msg
  RCCHECK(rclc_publisher_init_default(
          &feedback_publisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(akros2_msgs, msg, Feedback),
          "feedback"));

  // create twist command subscriber
  RCCHECK(rclc_subscription_init_default(
          &twist_subscriber,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          "cmd_vel"));

  // create mode subscriber
  RCCHECK(rclc_subscription_init_default(
          &mode_subscriber,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(akros2_msgs, msg, Mode),
          "mode"));

  // create timer
  const unsigned int timeout_ms = UPDATE_RATE;
  RCCHECK(rclc_timer_init_default(
          &timer,
          &support,
          RCL_MS_TO_NS(timeout_ms),
          timerCallback));

  // create parameter server
  param_options = {.notify_changed_over_dds = true, .max_params = 3};
  RCCHECK(rclc_parameter_server_init_with_option(
          &param_server,
          &node,
          &param_options));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER + 3, & allocator));
  RCCHECK(rclc_executor_add_subscription(
          &executor,
          &twist_subscriber,
          &twist_msg,
          &twistCallback,
          ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
          &executor,
          &mode_subscriber,
          &mode_msg,
          &modeCallback,
          ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_parameter_server(
          &executor,
          &param_server,
          paramCallback));

  // Add parameters to the server
  RCCHECK(rclc_add_parameter(&param_server, kp_name, RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, ki_name, RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, kd_name, RCLC_PARAMETER_DOUBLE));

  // Set parameter default values
  RCCHECK(rclc_parameter_set_double(&param_server, kp_name, K_P));
  RCCHECK(rclc_parameter_set_double(&param_server, ki_name, K_I));
  RCCHECK(rclc_parameter_set_double(&param_server, kd_name, K_D));

  // synchronize time with the agent
  RCCHECK(rmw_uros_sync_session(10));
  calculateOffset();

  return true;
}

bool destroyEntities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_ret_t rc = rclc_executor_fini(&executor);
  rc += rcl_publisher_fini(&odom_publisher, &node);
  rc += rcl_publisher_fini(&imu_publisher, &node);
  rc += rcl_publisher_fini(&feedback_publisher, &node);
  rc += rclc_executor_fini(&executor);
  rc += rcl_timer_fini(&timer);
  rc += rcl_subscription_fini(&twist_subscriber, &node);
  rc += rcl_subscription_fini(&mode_subscriber, &node);
  rc += rclc_parameter_server_fini(&param_server, &node);
  rc += rcl_node_fini(&node);
  rc += rclc_support_fini(&support);
  rc += rcl_init_options_fini(&init_options);

  fullStop();
  setNeopixel(toCRGB(0, 255, 255)); // DISCONNECTED: Cyan
  FastLED.show();

  return (rc != RCL_RET_OK) ? false : true;
}

void fullStop()
{
  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.angular.z = 0.0;

  motor1_controller.brake();
  motor2_controller.brake();
  motor3_controller.brake();
  motor4_controller.brake();

  motor1_pid.resetAll();
  motor2_pid.resetAll();
  motor3_pid.resetAll();
  motor4_pid.resetAll();
}

void updateMode()
{
  if (mode_msg.estop)
  {
    setNeopixel(toCRGB(255, 0, 0)); // STOP: Red
    fullStop();
  }
  else
  {
    mode_msg.auto_t ? setNeopixel(toCRGB(0, 55, 255)) : setNeopixel(toCRGB(0, 255, 55)); // AUTO: Blue, TELEOP: Green
  }
  FastLED.show();
}

void moveBase()
{
  //TODO:
  // brake if there's no command received, or when it's only the first command sent
  //if((millis() - prev_cmd_time) >= 200)
  //{
    //twist_msg.linear.x = 0.0;
    //twist_msg.linear.y = 0.0;
    //twist_msg.angular.z = 0.0;

    //digitalWrite(LED_PIN, HIGH);
  //}
  
  if( twist_msg.linear.x != 0 || twist_msg.linear.y != 0 || twist_msg.angular.z != 0 )
  {
    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(
                                  twist_msg.linear.x,
                                  twist_msg.linear.y,
                                  twist_msg.angular.z);

    req_rpm1 = req_rpm.motor1;
    req_rpm2 = req_rpm.motor2;
    req_rpm3 = req_rpm.motor3;
    req_rpm4 = req_rpm.motor4;

    cur_rpm1 = motor1_encoder.getRPM();
    cur_rpm2 = motor2_encoder.getRPM();
    cur_rpm3 = motor3_encoder.getRPM();
    cur_rpm4 = motor4_encoder.getRPM();

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm1, cur_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm2, cur_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm3, cur_rpm3));
    motor4_controller.spin(motor4_pid.compute(req_rpm4, cur_rpm4));
  }
  else
  {
    fullStop();
    req_rpm1 = req_rpm2 = req_rpm3 = req_rpm4 = 0;
    cur_rpm1 = cur_rpm2 = cur_rpm3 = cur_rpm4 = 0;
  }

  Kinematics::velocities current_vel = kinematics.getVelocities(
                                        cur_rpm1,
                                        cur_rpm2,
                                        cur_rpm3,
                                        cur_rpm4);

  unsigned long now = millis();
  float vel_dt = (now - prev_odom_update) / 1000.0;
  prev_odom_update = now;
  odometry.update(
    vel_dt,
    current_vel.linear_x,
    current_vel.linear_y,
    current_vel.angular_z);
}

void publishData()
{
  imu_msg  = imu.getData();
  odom_msg = odometry.getData();

  cur_rpm_msg.motor1 = cur_rpm1;
  cur_rpm_msg.motor2 = cur_rpm2;
  cur_rpm_msg.motor3 = cur_rpm3;
  cur_rpm_msg.motor4 = cur_rpm4;

  req_rpm_msg.motor1 = req_rpm1;
  req_rpm_msg.motor2 = req_rpm2;
  req_rpm_msg.motor3 = req_rpm3;
  req_rpm_msg.motor4 = req_rpm4;

  feedback_msg.current  = cur_rpm_msg;
  feedback_msg.required = req_rpm_msg;

  struct timespec time_stamp = getTime();

  imu_msg.header.stamp.sec = time_stamp.tv_sec;
  imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  
  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

  feedback_msg.header.stamp.sec = time_stamp.tv_sec;
  feedback_msg.header.stamp.nanosec = time_stamp.tv_nsec;

  RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
  RCSOFTCHECK(rcl_publish(&feedback_publisher, &feedback_msg, NULL));
}

void calculateOffset()
{
  unsigned long now = millis();
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
  struct timespec tp = {0};
  // add time difference between uC time and ROS time to
  // synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;

  return tp;
}

void errorLoop()
{
  while(1)
  {
    flashLED(2);
  }
}

void flashLED(int n_times)
{
  for(int i=0; i<n_times; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  delay(1000);
}

void getTeensyMAC(uint8_t *mac)
{
  for(uint8_t by=0; by<2; by++)
  {
    mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
  }
  for(uint8_t by=0; by<4; by++)
  {
    mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
  }
}

CRGB toCRGB(uint8_t Cdata, uint8_t Mdata, uint8_t Ydata)
{
  CRGB output(0, 0, 0);

  output.r = 255-Cdata;
  output.g = 255-Mdata;
  output.b = 255-Ydata;

  return output;
}

void setNeopixel(CRGB in_led)
{
  for(int i=0; i<NEOPIXEL_COUNT; i++)
  {
    neopixel[i] = in_led;
  }
}
