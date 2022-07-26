# AKROS2 Firmware
Firmware and libraries for the akros2 robot, based on [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware) repository and the `MECANUM` config. Using a Teensy 4.1 with an [expansion board from Tindie](https://www.tindie.com/products/cburgess129/arduino-teensy41-teensy-41-expansion-board/). I'm using the same libraries as linorobot2, but have added/changed a few things to the firmware:

* Option for native ethernet transport (UDP4).
* Option to set ROS_DOMAIN_ID.
* Neopixel status and mode indicator using [FastLED](https://github.com/FastLED/FastLED).
* Compiling/uploading using Arduino IDE and [modified micro_ros_arduino libraries](https://github.com/adityakamath/micro_ros_arduino/tree/akros2_galactic) instead of PlatformIO.
* Calibration sketch updated and moved to a separate directory [akros2_calibration](https://github.com/adityakamath/akros2_firmware/tree/akros2_galactic/akros2_calibration/).
    * Reports max RPMs using pre-defined CPRs, instead of the other way around.
    * Also reports the calculated CPRs and its deviation from the pre-defined CPRs.
* Mode subscriber with custom [akros2_msgs/Mode](https://github.com/adityakamath/micro_ros_arduino/tree/akros2_galactic/extras/library_generation/extra_packages/akros2_msgs) type.
* Fuse encoder and IMU measurements (TODO).
* Add parameters (TODO).

Generic linorobot2 installation and setup instructions can be found on the [galactic](https://github.com/adityakamath/akros2_firmware/tree/galactic) branch.