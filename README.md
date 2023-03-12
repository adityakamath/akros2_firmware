# akros2_firmware
Firmware and libraries for the akros2 robot, based on [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware) repository and the `MECANUM` config. Using a Teensy 4.1 with an [expansion board from Tindie](https://www.tindie.com/products/cburgess129/arduino-teensy41-teensy-41-expansion-board/). I'm using the same libraries as linorobot2, but have added/changed a few things to the firmware:

* Option for native ethernet transport (UDP4).
* Option to set  ```ROS_DOMAIN_ID```.
* Neopixel status and mode indicator using [FastLED](https://github.com/FastLED/FastLED).
* Compiling/uploading using Arduino IDE and [modified micro_ros_arduino libraries](https://github.com/adityakamath/micro_ros_arduino/tree/akros2_galactic) instead of PlatformIO.
* Calibration sketch updated and moved to a separate repo [arduino_sketchbook_ros](https://github.com/adityakamath/arduino_sketchbook_ros/tree/main/akros2_calibration/).
    * Reports max RPMs (revolutions per minute) using pre-defined CPRs (counts per revolution), instead of the other way around.
    * Also reports the calculated CPRs and its deviation from the pre-defined CPRs.
* Mode subscriber with custom [akros2_msgs/Mode](https://github.com/adityakamath/micro_ros_arduino/tree/akros2_galactic/extras/library_generation/extra_packages/akros2_msgs/msg/Mode.msg) type.
* Joint State publishers to publish the measured and required joint states (velocities and positions) as separate [sensor_msgs/JointState](https://docs.ros2.org/galactic/api/sensor_msgs/msg/JointState.html) messages.
* TODO: Parameter server reads gain values (```kp```, ```ki```, ```kd```, ```scale```) set on the ROS2 host, and reacts to these values when they change. Used for PID tuning.
    * Parameters are initialized using values defined in the configuration. These values are also used during re-initialization (when agent is reconnected)
    * Config needs to be updated and then the firmware needs to be recompiled with the tuned PID gains.
* TODO: Fuse encoder and IMU measurements.

Generic linorobot2 installation and setup instructions can be found on the [galactic](https://github.com/adityakamath/akros2_firmware/tree/galactic) branch.