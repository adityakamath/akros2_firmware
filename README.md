# AKROS2 Firmware
Firmware and libraries for the akros2 robot, based on [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware) repository and the `MECANUM` config. Using a Teensy 4.1 with an [expansion board from Tindie](https://www.tindie.com/products/cburgess129/arduino-teensy41-teensy-41-expansion-board/). I'm using the same libraries as linorobot2, but have added/changed a few things to the firmware:

* Option for native ethernet transport (UDP4)
* Option to set ROS_DOMAIN_ID
* Neopixel status and mode indicator
* Compiling/uploading using Arduino IDE and [micro_ros_arduino][https://github.com/micro-ROS/micro_ros_arduino] instead of PlatformIO
* Calibration sketch moved to a separate branch [akros2_calibration](https://github.com/adityakamath/akros2_firmware/tree/akros2_calibration)
* Mode subscriber with custom message type (TODO)
* Fuse encoder and IMU measurements (TODO)
* Add parameters (TODO)

Generic linorobot2 installation and setup instructions can be found on the [galactic](https://github.com/adityakamath/akros2_firmware/tree/galactic) branch.