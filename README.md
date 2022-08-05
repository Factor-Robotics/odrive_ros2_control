# odrive_ros2_control

## Introduction
ODrive driver for ros2_control
## Prerequisites
* ROS Foxy
* ODrive Firmware v0.5.3
## Documentation
- [Wiki](https://github.com/aifarm-dev/odrive_ros2_control/wiki/Getting-Started)
## Done
- [x] Support native protocol on USB
- [x] Support position, speed, torque commands
- [x] Support position, speed, torque feedbacks
- [x] Support using multiple ODrives
- [x] Support smooth switching of control modes
- [x] Unit conversion adheres to [REP-103](<https://www.ros.org/reps/rep-0103.html>)
- [x] Support using any or both of axes on each ODrive
- [x] Allow multiple axes running in different control modes
- [x] Provide sensor data (error, voltage, temperature)
- [x] Auto watchdog feeding
## Todo
- [ ] Support serial port and CAN
- [ ] Support feedforward control inputs
- [ ] Automatic configuration of ODrives based on URDF and YAML files
