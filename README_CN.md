# odrive_ros2_control
[ENGLISH](<README.md>) / 中文
## 简介
ODrive 的 ros2_control 驱动
## 前置依赖
* ROS Foxy
* ODrive 固件 v0.5.1（很快将支持后续版本）
## Done
- [x] 支持 USB 上的原生协议
- [x] 支持位置、速度、力矩控制
- [x] 支持位置、速度、力矩反馈
- [x] 单位换算遵循 [REP-103](<https://www.ros.org/reps/rep-0103.html>)
- [x] 支持使用单个 ODrive 上的任一个或两个轴
- [x] 允许两轴在不同的控制模式下运行
## Todo
- [ ] 支持控制模式切换
- [ ] 支持使用多个 ODrive
- [ ] 支持串口和CAN
- [ ] 支持无感模式
- [ ] 支持前馈控制输入
- [ ] 根据URDF和YAML文件自动配置ODrive
- [ ] 安全性改进
- [ ] Wiki 页面