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

#include "odrive_hardware_interface/odrive_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace odrive_hardware_interface
{
return_type ODriveHardwareInterface::configure(const hardware_interface::HardwareInfo& info)
{
  if (configure_default(info) != return_type::OK)
  {
    return return_type::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_axis_errors_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_errors_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());
  hw_encoder_errors_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());
  hw_controller_errors_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());
  hw_fet_temperatures_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_temperatures_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 3)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "Joint '%s' has %d command interfaces. 3 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return return_type::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"),
                   "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "Joint '%s'has %d state interfaces. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return return_type::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"),
                   "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return return_type::ERROR;
    }

    axis_.push_back(std::stoi(joint.parameters.at("axis")));
    enable_watchdog_.push_back(std::stoi(joint.parameters.at("enable_watchdog")));
  }

  odrive = new ODriveUSB();
  CHECK(odrive->init());

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    float torque_constant;
    CHECK(odrive->read(odrive->odrive_handle_, AXIS__MOTOR__CONFIG__TORQUE_CONSTANT + per_axis_offset * axis_[i],
                       torque_constant));
    torque_constant_.push_back(torque_constant);

    if (enable_watchdog_[i])
    {
      CHECK(odrive->write(odrive->odrive_handle_, AXIS__CONFIG__WATCHDOG_TIMEOUT + per_axis_offset * axis_[i],
                          std::stof(info_.joints[i].parameters.at("watchdog_timeout"))));
    }
    CHECK(odrive->write(odrive->odrive_handle_, AXIS__CONFIG__ENABLE_WATCHDOG + per_axis_offset * axis_[i],
                        (bool)enable_watchdog_[i]));
  }

  control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);
  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.sensors[i].name, "motor_temperature", &hw_motor_temperatures_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.sensors[i].name, "fet_temperature", &hw_fet_temperatures_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.sensors[i].name, "motor_error", &hw_motor_errors_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.sensors[i].name, "encoder_error", &hw_encoder_errors_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.sensors[i].name, "controller_error", &hw_controller_errors_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.sensors[i].name, "axis_error", &hw_axis_errors_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
  }

  return command_interfaces;
}

return_type ODriveHardwareInterface::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                 const std::vector<std::string>& stop_interfaces)
{
  std::vector<integration_level_t> new_modes = control_level_;
  for (std::string key : stop_interfaces)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        new_modes[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  for (std::string key : start_interfaces)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
      {
        new_modes[i] = integration_level_t::POSITION;
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        new_modes[i] = integration_level_t::VELOCITY;
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        new_modes[i] = integration_level_t::EFFORT;
      }
    }
  }

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (control_level_[i] != new_modes[i])
    {
      int32_t requested_state, control_mode;

      switch (new_modes[i])
      {
        case integration_level_t::UNDEFINED:
          requested_state = AXIS_STATE_IDLE;
          CHECK(odrive->write(odrive->odrive_handle_, AXIS__REQUESTED_STATE + per_axis_offset * axis_[i],
                              requested_state));
          break;

        case integration_level_t::EFFORT:
          hw_commands_efforts_[i] = hw_efforts_[i];
          control_mode = (int32_t)new_modes[i];
          CHECK(odrive->write(odrive->odrive_handle_,
                              AXIS__CONTROLLER__CONFIG__CONTROL_MODE + per_axis_offset * axis_[i], control_mode));
          requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          CHECK(odrive->write(odrive->odrive_handle_, AXIS__REQUESTED_STATE + per_axis_offset * axis_[i],
                              requested_state));
          break;

        case integration_level_t::VELOCITY:
          hw_commands_velocities_[i] = hw_velocities_[i];
          hw_commands_efforts_[i] = 0;
          control_mode = (int32_t)new_modes[i];
          CHECK(odrive->write(odrive->odrive_handle_,
                              AXIS__CONTROLLER__CONFIG__CONTROL_MODE + per_axis_offset * axis_[i], control_mode));
          requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          CHECK(odrive->write(odrive->odrive_handle_, AXIS__REQUESTED_STATE + per_axis_offset * axis_[i],
                              requested_state));
          break;

        case integration_level_t::POSITION:
          hw_commands_positions_[i] = hw_positions_[i];
          hw_commands_velocities_[i] = 0;
          hw_commands_efforts_[i] = 0;
          control_mode = (int32_t)new_modes[i];
          CHECK(odrive->write(odrive->odrive_handle_,
                              AXIS__CONTROLLER__CONFIG__CONTROL_MODE + per_axis_offset * axis_[i], control_mode));
          requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          CHECK(odrive->write(odrive->odrive_handle_, AXIS__REQUESTED_STATE + per_axis_offset * axis_[i],
                              requested_state));
          break;
      }
    }
    control_level_[i] = new_modes[i];
  }

  return return_type::OK;
}

return_type ODriveHardwareInterface::start()
{
  int32_t requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (enable_watchdog_[i])
    {
      CHECK(odrive->call(odrive->odrive_handle_, AXIS__WATCHDOG_FEED + per_axis_offset * axis_[i]));
    }
    CHECK(odrive->call(odrive->odrive_handle_, AXIS__CLEAR_ERRORS + per_axis_offset * axis_[i]));
    CHECK(odrive->write(odrive->odrive_handle_, AXIS__REQUESTED_STATE + per_axis_offset * axis_[i], requested_state));
  }

  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type ODriveHardwareInterface::stop()
{
  int32_t requested_state = AXIS_STATE_IDLE;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    CHECK(odrive->write(odrive->odrive_handle_, AXIS__REQUESTED_STATE + per_axis_offset * axis_[i], requested_state));
  }

  status_ = hardware_interface::status::STOPPED;
  return return_type::OK;
}

return_type ODriveHardwareInterface::read()
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    float Iq_measured, vel_estimate, pos_estimate, fet_temperature, motor_temperature;
    int32_t axis_error, motor_error, encoder_error, controller_error;

    CHECK(odrive->read(odrive->odrive_handle_, AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED + per_axis_offset * axis_[i],
                       Iq_measured));
    hw_efforts_[i] = Iq_measured * torque_constant_[i];

    CHECK(odrive->read(odrive->odrive_handle_, AXIS__ENCODER__VEL_ESTIMATE + per_axis_offset * axis_[i], vel_estimate));
    hw_velocities_[i] = vel_estimate * 2 * M_PI;

    CHECK(odrive->read(odrive->odrive_handle_, AXIS__ENCODER__POS_ESTIMATE + per_axis_offset * axis_[i], pos_estimate));
    hw_positions_[i] = pos_estimate * 2 * M_PI;

    CHECK(odrive->read(odrive->odrive_handle_, AXIS__ERROR + per_axis_offset * axis_[i], axis_error));
    hw_axis_errors_[i] = axis_error;

    CHECK(odrive->read(odrive->odrive_handle_, AXIS__MOTOR__ERROR + per_axis_offset * axis_[i], motor_error));
    hw_motor_errors_[i] = motor_error;

    CHECK(odrive->read(odrive->odrive_handle_, AXIS__ENCODER__ERROR + per_axis_offset * axis_[i], encoder_error));
    hw_encoder_errors_[i] = encoder_error;

    CHECK(odrive->read(odrive->odrive_handle_, AXIS__CONTROLLER__ERROR + per_axis_offset * axis_[i], controller_error));
    hw_controller_errors_[i] = controller_error;

    CHECK(odrive->read(odrive->odrive_handle_, AXIS__FET_THERMISTOR__TEMPERATURE + per_axis_offset * axis_[i],
                       fet_temperature));
    hw_fet_temperatures_[i] = fet_temperature;

    CHECK(odrive->read(odrive->odrive_handle_, AXIS__MOTOR_THERMISTOR__TEMPERATURE + per_axis_offset * axis_[i],
                       motor_temperature));
    hw_motor_temperatures_[i] = motor_temperature;
  }

  return return_type::OK;
}

return_type ODriveHardwareInterface::write()
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    float Iq_setpoint, input_vel, input_pos;

    switch (control_level_[i])
    {
      case integration_level_t::POSITION:
        input_pos = hw_commands_positions_[i] / 2 / M_PI;
        CHECK(
            odrive->write(odrive->odrive_handle_, AXIS__CONTROLLER__INPUT_POS + per_axis_offset * axis_[i], input_pos));

      case integration_level_t::VELOCITY:
        input_vel = hw_commands_velocities_[i] / 2 / M_PI;
        CHECK(
            odrive->write(odrive->odrive_handle_, AXIS__CONTROLLER__INPUT_VEL + per_axis_offset * axis_[i], input_vel));

      case integration_level_t::EFFORT:
        Iq_setpoint = hw_commands_efforts_[i] / torque_constant_[i];
        CHECK(odrive->write(odrive->odrive_handle_,
                            AXIS__MOTOR__CURRENT_CONTROL__IQ_SETPOINT + per_axis_offset * axis_[i], Iq_setpoint));

      case integration_level_t::UNDEFINED:
        if (enable_watchdog_[i])
        {
          CHECK(odrive->call(odrive->odrive_handle_, AXIS__WATCHDOG_FEED + per_axis_offset * axis_[i]));
        }
    }
  }

  return return_type::OK;
}
}  // namespace odrive_hardware_interface

PLUGINLIB_EXPORT_CLASS(odrive_hardware_interface::ODriveHardwareInterface, hardware_interface::SystemInterface)
