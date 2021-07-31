# odrive_ros2_control
ENGLISH / [中文](<README_CN.md>)
## Introduction
ODrive driver for ros2_control
## Prerequisites
* ROS Foxy
* ODrive Firmware v0.5.1 (subsequent versions will be supported soon)
## Done
- [x] Support native protocol on USB
- [x] Support position, speed, torque commands
- [x] Support position, speed, torque feedbacks
- [x] Unit conversion adheres to [REP-103](<https://www.ros.org/reps/rep-0103.html>)
- [x] Support using any or both of axes on a single ODrive
- [x] Allow two axes running in different control modes
## Todo
- [ ] Support control mode switch
- [ ] Support using multiple ODrives
- [ ] Support serial port and CAN
- [ ] Support sensorless mode
- [ ] Support feedforward control inputs
- [ ] Automatic configuration of ODrives based on URDF and YAML files
- [ ] Safety improvements
- [ ] Wiki page