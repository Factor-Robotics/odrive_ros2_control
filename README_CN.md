# odrive_ros2_control
[ENGLISH](<README.md>) / 中文
## 简介
ODrive 的 ros2_control 驱动
## 前置依赖
* ROS Foxy
* ODrive 固件 v0.5.3
## 文档
- [Wiki](https://github.com/Factor-Robotics/odrive_ros2_control/wiki/%E6%96%87%E6%A1%A3)
## Done
- [x] 支持 USB 上的原生协议
- [x] 支持位置、速度、力矩命令
- [x] 支持位置、速度、力矩反馈
- [x] 支持使用多个 ODrive
- [x] 支持控制模式切换
- [x] 单位换算遵循 [REP-103](<https://www.ros.org/reps/rep-0103.html>)
- [x] 支持使用单个 ODrive 上的任一个或两个轴
- [x] 允许两轴在不同的控制模式下运行
- [x] 自动喂狗
## Todo
- [ ] 支持串口和CAN
- [ ] 支持前馈控制输入
- [ ] 根据URDF和YAML文件自动配置ODrive
- [ ] 安全性改进