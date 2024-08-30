# akros2_firmware
![](https://img.shields.io/badge/ROS%202%20Humble-Ubuntu%2022.04-blue) ![GitHub License](https://img.shields.io/github/license/adityakamath/pan_tilt_ros)
 ![X (formerly Twitter) Follow](https://img.shields.io/twitter/follow/kamathsblog)
 
Firmware and libraries for the akros2 robot, based on [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware) repository and the `MECANUM` config. Using a Teensy 4.1 with an [expansion board from Tindie](https://www.tindie.com/products/cburgess129/arduino-teensy41-teensy-41-expansion-board/). I'm using the same libraries as linorobot2, but have added/changed a few things to the firmware:

* Option for native ethernet transport (UDP4).
* Option to set  ```ROS_DOMAIN_ID```.
* Neopixel status and mode indicator using [FastLED](https://github.com/FastLED/FastLED).
* Compiling/uploading using Arduino IDE and [modified micro_ros_arduino libraries](https://github.com/adityakamath/micro_ros_arduino/tree/akros2_galactic) instead of PlatformIO.
* Mode subscriber with custom [akros2_msgs/Mode](https://github.com/adityakamath/micro_ros_arduino/tree/akros2_galactic/extras/library_generation/extra_packages/akros2_msgs/msg/Mode.msg) type.
* Joint State publishers to publish the measured and required joint states (velocities and positions) as separate [sensor_msgs/JointState](https://docs.ros2.org/galactic/api/sensor_msgs/msg/JointState.html) messages.
* Parameter server reads the following:
    * Gain values (```kp```, ```ki```, ```kd```, ```scale```) which are set on the ROS2 host, used for PID tuning.
        * Parameters are initialized using values defined in the configuration. These values are also used during re-initialization (when agent is reconnected)
        * Config needs to be updated and then the firmware needs to be recompiled with the tuned PID gains.
        * ```scale``` value is limited to the range [0.0, 1.0] with a step size of 0.01.
    * Boolean value ```ned_to_enu``` which is used to convert IMU representation from NED (North-East-Down) to ENU (East-North-Up) convention [according to REP-103](https://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions), if needed. The parameter value is passed down to the [IMU interface](https://github.com/adityakamath/akros2_firmware/blob/akros2_humble/src/imu/imu_interface.h), which does the conversion. Defaults to ```false```, as the sensor already provides measurements in ENU convention.
* TODO - Refactoring: Calibration sketch updated and moved to a separate repo [arduino_sketchbook_ros](https://github.com/adityakamath/arduino_sketchbook_ros/tree/main/akros2_calibration/).
    * Reports max RPMs (revolutions per minute) using pre-defined CPRs (counts per revolution), instead of the other way around.
    * Also reports the calculated CPRs and its deviation from the pre-defined CPRs.

Generic linorobot2 installation and setup instructions can be found on the [galactic](https://github.com/adityakamath/akros2_firmware/tree/galactic) branch.
