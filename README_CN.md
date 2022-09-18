# odrive_ros2_control
[ENGLISH](<README.md>) / 中文
## 简介
ODrive 的 ros2_control 驱动
## 兼容性
|  | ROS 2 Foxy Fitzroy | ROS 2 Humble Hawksbill |
|---|---|---|
| ODrive Firmware v0.5.3 | [foxy-fw-v0.5.3](../../tree/foxy-fw-v0.5.3) | [humble-fw-v0.5.3](../../tree/humble-fw-v0.5.3) |
| ODrive Firmware v0.5.1 | [foxy-fw-v0.5.1](../../tree/foxy-fw-v0.5.1) | [humble-fw-v0.5.1](../../tree/humble-fw-v0.5.1) |
## 文档
- [Wiki](https://github.com/Factor-Robotics/odrive_ros2_control/wiki/%E6%96%87%E6%A1%A3)
## Done
- [x] 支持 USB 上的原生协议
- [x] 支持位置、速度、力矩命令
- [x] 支持位置、速度、力矩反馈
- [x] 支持前馈控制输入
- [x] 单位换算遵循 [REP-103](<https://www.ros.org/reps/rep-0103.html>)
- [x] 支持使用多个 ODrive
- [x] 支持使用每个 ODrive 上的一个或两个轴
- [x] 允许多轴在不同的控制模式下运行
- [x] 支持控制模式平滑切换
- [x] 提供传感器数据（错误、电压、温度）
- [x] 自动喂狗
- [x] 受[ros2_control_demos](<https://github.com/ros-controls/ros2_control_demos>)启发的硬件在环演示
## Todo
- [ ] 支持串口和CAN
- [ ] 根据URDF和YAML文件自动配置ODrive