// Copyright 2021 Factor Robotics
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

#pragma once

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_hardware_interface/odrive_usb.hpp"
#include "odrive_hardware_interface/visibility_control.hpp"

#define AXIS_STATE_IDLE 1
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8

#define CHECK(status)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    int ret = (status);                                                                                                \
    if (ret != 0)                                                                                                      \
    {                                                                                                                  \
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), libusb_error_name(ret));                             \
      return return_type::ERROR;                                                                                       \
    }                                                                                                                  \
  } while (0)

using namespace odrive;
using hardware_interface::return_type;

namespace odrive_hardware_interface
{
class ODriveHardwareInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ODriveHardwareInterface)

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo& info) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                          const std::vector<std::string>& stop_interfaces) override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  return_type start() override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  return_type stop() override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  return_type read() override;

  ODRIVE_HARDWARE_INTERFACE_PUBLIC
  return_type write() override;

private:
  ODriveUSB* odrive;

  std::vector<uint64_t> serial_numbers_;
  std::vector<int> axis_;
  std::vector<float> torque_constant_;
  std::vector<bool> enable_watchdog_;
  std::vector<float> watchdog_timeout_;

  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_axis_errors_;
  std::vector<double> hw_motor_errors_;
  std::vector<double> hw_encoder_errors_;
  std::vector<double> hw_controller_errors_;
  std::vector<double> hw_fet_temperatures_;
  std::vector<double> hw_motor_temperatures_;

  enum class integration_level_t : int32_t
  {
    UNDEFINED = 0,
    EFFORT = 1,
    VELOCITY = 2,
    POSITION = 3
  };

  std::vector<integration_level_t> control_level_;
};
}  // namespace odrive_hardware_interface
