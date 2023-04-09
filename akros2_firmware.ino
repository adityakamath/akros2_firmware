// Copyright (c) 2023 Aditya Kamath
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

#include <stdio.h>
#include <FastLED.h>

#include "akros2_base_config.h"
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>

#include <akros2_msgs/msg/Mode.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>

#include <rosidl_runtime_c/string.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

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

rcl_subscription_t mode_subscriber;
rcl_subscription_t twist_subscriber;
rcl_publisher_t    odom_publisher;
rcl_publisher_t    imu_publisher;
rcl_publisher_t    joint_state_publisher;
rcl_publisher_t    req_state_publisher;

akros2_msgs__msg__Mode       mode_msg;
geometry_msgs__msg__Twist    twist_msg;
nav_msgs__msg__Odometry      odom_msg;
sensor_msgs__msg__Imu        imu_msg;
sensor_msgs__msg__JointState joint_state_msg;
sensor_msgs__msg__JointState req_state_msg;

rosidl_runtime_c__String base_frame_str;
rosidl_runtime_c__String odom_frame_str;
rosidl_runtime_c__String imu_frame_str;
rosidl_runtime_c__String__Sequence joint_name_seq;
rosidl_runtime_c__double__Sequence joint_vel_seq;
rosidl_runtime_c__double__Sequence joint_pos_seq;
rosidl_runtime_c__double__Sequence req_vel_seq;
rosidl_runtime_c__double__Sequence req_pos_seq;

rclc_executor_t          executor;
rclc_support_t           support;
rcl_init_options_t       init_options;
rcl_allocator_t          allocator;
rcl_node_t               node;
rcl_timer_t              timer;
rclc_parameter_server_t  param_server;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state = WAITING_AGENT;

float joint_rpm[NR_OF_JOINTS];
float req_rpm[NR_OF_JOINTS];

CRGB neopixel[NEOPIXEL_COUNT];

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN*MAX_PWM_RATIO, PWM_MAX*MAX_PWM_RATIO, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN*MAX_PWM_RATIO, PWM_MAX*MAX_PWM_RATIO, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN*MAX_PWM_RATIO, PWM_MAX*MAX_PWM_RATIO, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN*MAX_PWM_RATIO, PWM_MAX*MAX_PWM_RATIO, K_P, K_I, K_D);

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

const char * kp_name        = "kp";
const char * ki_name        = "ki";
const char * kd_name        = "kd";
const char * rpm_ratio_name = "scale";

double kp = K_P;
double ki = K_I;
double kd = K_D;
double rpm_ratio = MAX_RPM_RATIO;

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
    while(1){ flashLED(3); }
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

  setNeopixel(CRGB(0, 255, 255)); // STARTUP: Cyan
  FastLED.show();
}

void loop() 
{
  switch (state)
  {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT){ destroyEntities(); }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED){ rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); }
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

void twistCallback(const void * msgin){ prev_cmd_time = millis(); }

void modeCallback(const void * msgin){}

bool paramCallback(const Parameter * old_param, const Parameter * new_param, void * context)
{
  if (old_param != NULL && new_param != NULL) 
  {
    if(strcmp(new_param->name.data, rpm_ratio_name) == 0){ rpm_ratio = new_param->value.double_value; }
    else if(strcmp(new_param->name.data, kp_name) == 0){ kp = new_param->value.double_value; }
    else if(strcmp(new_param->name.data, ki_name) == 0){ ki = new_param->value.double_value; }
    else if(strcmp(new_param->name.data, kd_name) == 0){ kd = new_param->value.double_value; }

    kinematics.setMaxRPM(rpm_ratio);
    motor1_pid.updateConstants(kp, ki, kd);
    motor2_pid.updateConstants(kp, ki, kd);
    motor3_pid.updateConstants(kp, ki, kd);
    motor4_pid.updateConstants(kp, ki, kd);

    return true;
  }
  else return false;
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

  // create combined measured joint state publisher
  RCCHECK(rclc_publisher_init_default(
          &joint_state_publisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          "joint_states"));

  // create combined required state publisher
  RCCHECK(rclc_publisher_init_default(
          &req_state_publisher,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          "req_states"));

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
  const unsigned int timeout_ms = 1000 / UPDATE_FREQ;
  RCCHECK(rclc_timer_init_default(
          &timer,
          &support,
          RCL_MS_TO_NS(timeout_ms),
          timerCallback));

  // create parameter server
  rclc_parameter_options_t param_options = {
        .notify_changed_over_dds = true,
        .max_params = 4,
        .allow_undeclared_parameters = false,
        .low_mem_mode = true};
  
  RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node, &param_options));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES+3, &allocator));
  RCCHECK(rclc_executor_add_parameter_server(
          &executor,
          &param_server,
          paramCallback));
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

  // Add parameters to the server
  RCCHECK(rclc_add_parameter(&param_server, kp_name, RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, ki_name, RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, kd_name, RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, rpm_ratio_name, RCLC_PARAMETER_DOUBLE));

  // Set parameter default values
  RCCHECK(rclc_parameter_set_double(&param_server, kp_name, K_P));
  RCCHECK(rclc_parameter_set_double(&param_server, ki_name, K_I));
  RCCHECK(rclc_parameter_set_double(&param_server, kd_name, K_D));
  RCCHECK(rclc_parameter_set_double(&param_server, rpm_ratio_name, MAX_RPM_RATIO));

  rosidl_runtime_c__String__init(&base_frame_str);
  rosidl_runtime_c__String__init(&odom_frame_str);
  rosidl_runtime_c__String__init(&imu_frame_str);
  rosidl_runtime_c__String__Sequence__init(&joint_name_seq, NR_OF_JOINTS);
  rosidl_runtime_c__double__Sequence__init(&joint_vel_seq, NR_OF_JOINTS);
  rosidl_runtime_c__double__Sequence__init(&joint_pos_seq, NR_OF_JOINTS);
  rosidl_runtime_c__double__Sequence__init(&req_vel_seq, NR_OF_JOINTS);
  rosidl_runtime_c__double__Sequence__init(&req_pos_seq, NR_OF_JOINTS);

  // populate frame_id and joint names
  rosidl_runtime_c__String__assign(&base_frame_str, BASE_FRAME_ID);
  rosidl_runtime_c__String__assign(&odom_frame_str, ODOM_FRAME_ID);
  rosidl_runtime_c__String__assign(&imu_frame_str, IMU_FRAME_ID);
  rosidl_runtime_c__String__assign(&joint_name_seq.data[0], "joint_lf"); // motor 1
  rosidl_runtime_c__String__assign(&joint_name_seq.data[1], "joint_rf"); // motor 2
  rosidl_runtime_c__String__assign(&joint_name_seq.data[2], "joint_lb"); // motor 3
  rosidl_runtime_c__String__assign(&joint_name_seq.data[3], "joint_rb"); // motor 4

  for(int i=0; i<NR_OF_JOINTS; i++)
  {
    joint_rpm[i] = 0.0;
    joint_vel_seq.data[i] = 0.0;
    joint_pos_seq.data[i] = 0.0;
    req_rpm[i] = 0.0;
    req_vel_seq.data[i] = 0.0;
    req_pos_seq.data[i] = 0.0;
  }

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
  rc += rcl_publisher_fini(&joint_state_publisher, &node);
  rc += rcl_publisher_fini(&req_state_publisher, &node);
  rc += rcl_timer_fini(&timer);
  rc += rcl_subscription_fini(&twist_subscriber, &node);
  rc += rcl_subscription_fini(&mode_subscriber, &node);
  rc += rclc_parameter_server_fini(&param_server, &node);
  rc += rcl_node_fini(&node);
  rc += rclc_support_fini(&support);
  rc += rcl_init_options_fini(&init_options);

  rosidl_runtime_c__String__fini(&base_frame_str);
  rosidl_runtime_c__String__fini(&odom_frame_str);
  rosidl_runtime_c__String__fini(&imu_frame_str);
  rosidl_runtime_c__String__Sequence__fini(&joint_name_seq);
  rosidl_runtime_c__double__Sequence__fini(&joint_vel_seq);
  rosidl_runtime_c__double__Sequence__fini(&joint_pos_seq);
  rosidl_runtime_c__double__Sequence__fini(&req_vel_seq);
  rosidl_runtime_c__double__Sequence__fini(&req_pos_seq);

  fullStop();
  setNeopixel(CRGB(0, 255, 255)); // DISCONNECTED: Cyan
  FastLED.show();

  return (rc != RCL_RET_OK) ? false : true;
}

void fullStop()
{
  twist_msg = {0.0};

  motor1_controller.brake();
  motor2_controller.brake();
  motor3_controller.brake();
  motor4_controller.brake();

  motor1_pid.resetAll();
  motor2_pid.resetAll();
  motor3_pid.resetAll();
  motor4_pid.resetAll();

  for(int i=0; i<NR_OF_JOINTS; i++)
  {
    joint_rpm[i] = 0.0;
    req_rpm[i] = 0.0;
  }
}

void updateMode()
{
  if (mode_msg.estop)
  {
    setNeopixel(CRGB(255, 0, 0)); // STOP: Red
    fullStop();
  }
  else mode_msg.auto_t ? setNeopixel(CRGB(0, 55, 255)) : setNeopixel(CRGB(0, 255, 55)); // AUTO: Blue, TELEOP: Green
  FastLED.show();
}

void moveBase()
{
  if(twist_msg.linear.x != 0 || twist_msg.linear.y != 0 || twist_msg.angular.z != 0)
  {
    // brake if there's no command received, or when it's only the first command sent
    if(((millis() - prev_cmd_time) >= 200))
    {
      twist_msg = {0.0};
    }

    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm required_rpm = kinematics.getRPM(twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z);

    req_rpm[0] = required_rpm.motor1;
    req_rpm[1] = required_rpm.motor2;
    req_rpm[2] = required_rpm.motor3;
    req_rpm[3] = required_rpm.motor4;
    
    joint_rpm[0] = motor1_encoder.getRPM();
    joint_rpm[1] = motor2_encoder.getRPM();
    joint_rpm[2] = motor3_encoder.getRPM();
    joint_rpm[3] = motor4_encoder.getRPM();

    float pwm_arr[NR_OF_JOINTS];
    pwm_arr[0] = motor1_pid.compute(req_rpm[0], joint_rpm[0]);
    pwm_arr[1] = motor2_pid.compute(req_rpm[1], joint_rpm[1]);
    pwm_arr[2] = motor3_pid.compute(req_rpm[2], joint_rpm[2]);
    pwm_arr[3] = motor4_pid.compute(req_rpm[3], joint_rpm[3]);

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    float smooth_pwm[NR_OF_JOINTS];
    for(int i=0; i<NR_OF_JOINTS; i++)
    {
      smooth_pwm[i] += (pwm_arr[i] - smooth_pwm[i])*SMOOTHING_CONST;
    }

    motor1_controller.spin(constrain(smooth_pwm[0], PWM_MIN*MAX_PWM_RATIO, PWM_MAX*MAX_PWM_RATIO));
    motor2_controller.spin(constrain(smooth_pwm[1], PWM_MIN*MAX_PWM_RATIO, PWM_MAX*MAX_PWM_RATIO));
    motor3_controller.spin(constrain(smooth_pwm[2], PWM_MIN*MAX_PWM_RATIO, PWM_MAX*MAX_PWM_RATIO));
    motor4_controller.spin(constrain(smooth_pwm[3], PWM_MIN*MAX_PWM_RATIO, PWM_MAX*MAX_PWM_RATIO));
  }
  else fullStop();

  Kinematics::velocities current_vel = kinematics.getVelocities(
                                        joint_rpm[0],
                                        joint_rpm[1],
                                        joint_rpm[2],
                                        joint_rpm[3]);

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
  struct timespec time_stamp = getTime();

  // populate IMU data
  imu_msg  = imu.getData();
  imu_msg.header.stamp.sec = time_stamp.tv_sec;
  imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  imu_msg.header.frame_id = imu_frame_str;
  
  // populate odom data
  odom_msg = odometry.getData();
  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  odom_msg.header.frame_id = odom_frame_str;

  // populate required and measured joint states: velocities (rad/s) and positions (rads, in range[-pi, pi])
  for(int i=0; i<NR_OF_JOINTS; i++)
  {
    joint_vel_seq.data[i] = joint_rpm[i] * M_PI * 2 / 60; // rpm to rad/s
    joint_pos_seq.data[i] += joint_vel_seq.data[i] / UPDATE_FREQ;  // rad/s to rad
    if (joint_pos_seq.data[i] > M_PI){ joint_pos_seq.data[i] = -1.0 * M_PI; }
    if (joint_pos_seq.data[i] < -1.0 * M_PI){ joint_pos_seq.data[i] = M_PI; }

    req_vel_seq.data[i] = req_rpm[i] * M_PI * 2 / 60; // rpm to rad/s
    req_pos_seq.data[i] += req_vel_seq.data[i] / UPDATE_FREQ;  // rad/s to rad
    if (req_pos_seq.data[i] > M_PI){ req_pos_seq.data[i] = -1.0 * M_PI; }
    if (req_pos_seq.data[i] < -1.0 * M_PI){ req_pos_seq.data[i] = M_PI; }    
  }

  // populate measured joint state data
  joint_state_msg.header.stamp.sec = time_stamp.tv_sec;
  joint_state_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  joint_state_msg.header.frame_id = base_frame_str;
  joint_state_msg.name = joint_name_seq;
  joint_state_msg.velocity = joint_vel_seq;
  joint_state_msg.position = joint_pos_seq;

  // populate required joint state data
  req_state_msg.header.stamp.sec = time_stamp.tv_sec;
  req_state_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  req_state_msg.header.frame_id = base_frame_str;
  req_state_msg.name = joint_name_seq;
  req_state_msg.velocity = req_vel_seq;
  req_state_msg.position = req_pos_seq;  
  
  // publish
  RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
  RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
  RCSOFTCHECK(rcl_publish(&req_state_publisher, &req_state_msg, NULL));
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
  // add time difference between uC time and ROS time to synchronize time with ROS
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

void setNeopixel(CRGB in_led)
{
  for(int i=0; i<NEOPIXEL_COUNT; i++)
  {
    neopixel[i] = in_led;
  }
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
