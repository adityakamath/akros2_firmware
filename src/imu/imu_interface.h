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

#ifndef IMU_INTERFACE
#define IMU_INTERFACE

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h>

extern "C" {
#include "MahonyAHRS.h"
}

class IMUInterface
{
    protected:
        sensor_msgs__msg__Imu imu_msg_;
        const float g_to_accel_ = 9.81;
        const float mgauss_to_utesla_ = 0.1;
        const float utesla_to_tesla_ = 0.000001;

        float accel_cov_ = 0.00001;
        float gyro_cov_ = 0.00001;
        const int sample_size_ = 40;

        geometry_msgs__msg__Vector3 gyro_cal_;

        void calibrateGyro()
        {
            geometry_msgs__msg__Vector3 gyro;

            for(int i=0; i<sample_size_; i++)
            {
                gyro = readGyroscope();
                gyro_cal_.x += gyro.x;
                gyro_cal_.y += gyro.y;
                gyro_cal_.z += gyro.z;

                delay(50);
            }

            gyro_cal_.x = gyro_cal_.x / (float)sample_size_;
            gyro_cal_.y = gyro_cal_.y / (float)sample_size_;
            gyro_cal_.z = gyro_cal_.z / (float)sample_size_;
        }

    public:
        IMUInterface()
        {
            imu_msg_.header.frame_id = micro_ros_string_utilities_set(imu_msg_.header.frame_id, "imu_link");
        }

        virtual geometry_msgs__msg__Vector3 readAccelerometer() = 0;
        virtual geometry_msgs__msg__Vector3 readGyroscope() = 0;
        virtual bool startSensor() = 0;

        bool init()
        {
            bool sensor_ok = startSensor();
            if(sensor_ok)
                calibrateGyro();

            return sensor_ok;
        }

        sensor_msgs__msg__Imu getData(bool ned_to_enu)
        {
            imu_msg_.angular_velocity = readGyroscope();
            imu_msg_.angular_velocity.x -= gyro_cal_.x;
            imu_msg_.angular_velocity.y -= gyro_cal_.y;
            imu_msg_.angular_velocity.z -= gyro_cal_.z;

            if(imu_msg_.angular_velocity.x > -0.01 && imu_msg_.angular_velocity.x < 0.01 )
                imu_msg_.angular_velocity.x = 0;

            if(imu_msg_.angular_velocity.y > -0.01 && imu_msg_.angular_velocity.y < 0.01 )
                imu_msg_.angular_velocity.y = 0;

            if(imu_msg_.angular_velocity.z > -0.01 && imu_msg_.angular_velocity.z < 0.01 )
                imu_msg_.angular_velocity.z = 0;

            imu_msg_.angular_velocity_covariance[0] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[4] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[8] = gyro_cov_;

            imu_msg_.linear_acceleration = readAccelerometer();
            imu_msg_.linear_acceleration_covariance[0] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[4] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[8] = accel_cov_;

            MahonyAHRSupdateIMU(
                    imu_msg_.angular_velocity.x,
                    imu_msg_.angular_velocity.y,
                    imu_msg_.angular_velocity.z,
                    imu_msg_.linear_acceleration.x,
                    imu_msg_.linear_acceleration.y,
                    imu_msg_.linear_acceleration.z);

            sensor_msgs__msg__Imu tmp_msg = imu_msg_;
            
            if(ned_to_enu)
            {
                // Convert NED → ENU: (x y z) → (y x -z)  or (w x y z) → (y x -z w)
                tmp_msg.angular_velocity.x = imu_msg_.angular_velocity.y;
                tmp_msg.angular_velocity.y = imu_msg_.angular_velocity.x;
                tmp_msg.angular_velocity.z = -1 * imu_msg_.angular_velocity.z;
                
                tmp_msg.linear_acceleration.x = imu_msg_.linear_acceleration.y;
                tmp_msg.linear_acceleration.y = imu_msg_.linear_acceleration.x;
                tmp_msg.linear_acceleration.z = -1 * imu_msg_.linear_acceleration.z;
            }
            
            imu_msg_.angular_velocity = tmp_msg.angular_velocity;
            imu_msg_.linear_acceleration = tmp_msg.linear_acceleration;

            return imu_msg_;
        }
};

#endif
