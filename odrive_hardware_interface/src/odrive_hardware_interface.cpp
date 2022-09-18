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

  serial_numbers_.resize(2);

  hw_vbus_voltages_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  hw_axis_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_encoder_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_controller_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_fet_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo& sensor : info_.sensors)
  {
    serial_numbers_[0].emplace_back(std::stoull(sensor.parameters.at("serial_number"), 0, 16));
  }

  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    serial_numbers_[1].emplace_back(std::stoull(joint.parameters.at("serial_number"), 0, 16));
    axes_.emplace_back(std::stoi(joint.parameters.at("axis")));
    enable_watchdogs_.emplace_back(std::stoi(joint.parameters.at("enable_watchdog")));
  }

  odrive = new ODriveUSB();
  CHECK(odrive->init(serial_numbers_));

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    float torque_constant;
    CHECK(odrive->read(serial_numbers_[1][i], AXIS__MOTOR__CONFIG__TORQUE_CONSTANT + per_axis_offset * axes_[i],
                       torque_constant));
    torque_constants_.emplace_back(torque_constant);

    if (enable_watchdogs_[i])
    {
      CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONFIG__WATCHDOG_TIMEOUT + per_axis_offset * axes_[i],
                          std::stof(info_.joints[i].parameters.at("watchdog_timeout"))));
    }
    CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONFIG__ENABLE_WATCHDOG + per_axis_offset * axes_[i],
                        (bool)enable_watchdogs_[i]));
  }

  control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);
  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.sensors.size(); i++)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.sensors[i].name, "vbus_voltage", &hw_vbus_voltages_[i]));
  }

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "axis_error", &hw_axis_errors_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "motor_error", &hw_motor_errors_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "encoder_error", &hw_encoder_errors_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "controller_error", &hw_controller_errors_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "fet_temperature", &hw_fet_temperatures_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "motor_temperature", &hw_motor_temperatures_[i]));
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
  for (std::string key : stop_interfaces)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        control_level_[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  for (std::string key : start_interfaces)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      switch (control_level_[i])
      {
        case integration_level_t::UNDEFINED:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT)
          {
            control_level_[i] = integration_level_t::EFFORT;
          }

        case integration_level_t::EFFORT:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
          {
            control_level_[i] = integration_level_t::VELOCITY;
          }

        case integration_level_t::VELOCITY:
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
          {
            control_level_[i] = integration_level_t::POSITION;
          }

        case integration_level_t::POSITION:
          break;
      }
    }
  }

  return return_type::OK;
}

return_type ODriveHardwareInterface::perform_command_mode_switch(const std::vector<std::string>&,
                                                                 const std::vector<std::string>&)
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    float input_torque, input_vel, input_pos;
    int32_t requested_state;

    switch (control_level_[i])
    {
      case integration_level_t::UNDEFINED:
        requested_state = AXIS_STATE_IDLE;
        CHECK(
            odrive->write(serial_numbers_[1][i], AXIS__REQUESTED_STATE + per_axis_offset * axes_[i], requested_state));
        break;

      case integration_level_t::EFFORT:
        hw_commands_efforts_[i] = hw_efforts_[i];
        CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__CONFIG__CONTROL_MODE + per_axis_offset * axes_[i],
                            (int32_t)control_level_[i]));
        input_torque = hw_commands_efforts_[i];
        CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_TORQUE + per_axis_offset * axes_[i],
                            input_torque));
        requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        CHECK(
            odrive->write(serial_numbers_[1][i], AXIS__REQUESTED_STATE + per_axis_offset * axes_[i], requested_state));
        break;

      case integration_level_t::VELOCITY:
        hw_commands_velocities_[i] = hw_velocities_[i];
        hw_commands_efforts_[i] = 0;
        CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__CONFIG__CONTROL_MODE + per_axis_offset * axes_[i],
                            (int32_t)control_level_[i]));
        input_vel = hw_commands_velocities_[i] / 2 / M_PI;
        CHECK(
            odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_VEL + per_axis_offset * axes_[i], input_vel));
        input_torque = hw_commands_efforts_[i];
        CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_TORQUE + per_axis_offset * axes_[i],
                            input_torque));
        requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        CHECK(
            odrive->write(serial_numbers_[1][i], AXIS__REQUESTED_STATE + per_axis_offset * axes_[i], requested_state));
        break;

      case integration_level_t::POSITION:
        hw_commands_positions_[i] = hw_positions_[i];
        hw_commands_velocities_[i] = 0;
        hw_commands_efforts_[i] = 0;
        CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__CONFIG__CONTROL_MODE + per_axis_offset * axes_[i],
                            (int32_t)control_level_[i]));
        input_pos = hw_commands_positions_[i] / 2 / M_PI;
        CHECK(
            odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_POS + per_axis_offset * axes_[i], input_pos));
        input_vel = hw_commands_velocities_[i] / 2 / M_PI;
        CHECK(
            odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_VEL + per_axis_offset * axes_[i], input_vel));
        input_torque = hw_commands_efforts_[i];
        CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_TORQUE + per_axis_offset * axes_[i],
                            input_torque));
        requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
        CHECK(
            odrive->write(serial_numbers_[1][i], AXIS__REQUESTED_STATE + per_axis_offset * axes_[i], requested_state));
        break;
    }
  }

  return return_type::OK;
}

return_type ODriveHardwareInterface::start()
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (enable_watchdogs_[i])
    {
      CHECK(odrive->call(serial_numbers_[1][i], AXIS__WATCHDOG_FEED + per_axis_offset * axes_[i]));
    }
    CHECK(odrive->call(serial_numbers_[1][i], AXIS__CLEAR_ERRORS + per_axis_offset * axes_[i]));
  }

  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type ODriveHardwareInterface::stop()
{
  int32_t requested_state = AXIS_STATE_IDLE;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    CHECK(odrive->write(serial_numbers_[1][i], AXIS__REQUESTED_STATE + per_axis_offset * axes_[i], requested_state));
  }

  status_ = hardware_interface::status::STOPPED;
  return return_type::OK;
}

return_type ODriveHardwareInterface::read()
{
  for (size_t i = 0; i < info_.sensors.size(); i++)
  {
    float vbus_voltage;

    CHECK(odrive->read(serial_numbers_[0][i], VBUS_VOLTAGE, vbus_voltage));
    hw_vbus_voltages_[i] = vbus_voltage;
  }

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    float Iq_measured, vel_estimate, pos_estimate, fet_temperature, motor_temperature;
    int32_t axis_error, motor_error, encoder_error, controller_error;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED + per_axis_offset * axes_[i],
                       Iq_measured));
    hw_efforts_[i] = Iq_measured * torque_constants_[i];

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__ENCODER__VEL_ESTIMATE + per_axis_offset * axes_[i], vel_estimate));
    hw_velocities_[i] = vel_estimate * 2 * M_PI;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__ENCODER__POS_ESTIMATE + per_axis_offset * axes_[i], pos_estimate));
    hw_positions_[i] = pos_estimate * 2 * M_PI;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__ERROR + per_axis_offset * axes_[i], axis_error));
    hw_axis_errors_[i] = axis_error;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__MOTOR__ERROR + per_axis_offset * axes_[i], motor_error));
    hw_motor_errors_[i] = motor_error;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__ENCODER__ERROR + per_axis_offset * axes_[i], encoder_error));
    hw_encoder_errors_[i] = encoder_error;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__CONTROLLER__ERROR + per_axis_offset * axes_[i], controller_error));
    hw_controller_errors_[i] = controller_error;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__FET_THERMISTOR__TEMPERATURE + per_axis_offset * axes_[i],
                       fet_temperature));
    hw_fet_temperatures_[i] = fet_temperature;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__MOTOR_THERMISTOR__TEMPERATURE + per_axis_offset * axes_[i],
                       motor_temperature));
    hw_motor_temperatures_[i] = motor_temperature;
  }

  return return_type::OK;
}

return_type ODriveHardwareInterface::write()
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    float input_torque, input_vel, input_pos;

    switch (control_level_[i])
    {
      case integration_level_t::POSITION:
        input_pos = hw_commands_positions_[i] / 2 / M_PI;
        CHECK(
            odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_POS + per_axis_offset * axes_[i], input_pos));

      case integration_level_t::VELOCITY:
        input_vel = hw_commands_velocities_[i] / 2 / M_PI;
        CHECK(
            odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_VEL + per_axis_offset * axes_[i], input_vel));

      case integration_level_t::EFFORT:
        input_torque = hw_commands_efforts_[i];
        CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_TORQUE + per_axis_offset * axes_[i],
                            input_torque));

      case integration_level_t::UNDEFINED:
        if (enable_watchdogs_[i])
        {
          CHECK(odrive->call(serial_numbers_[1][i], AXIS__WATCHDOG_FEED + per_axis_offset * axes_[i]));
        }
    }
  }

  return return_type::OK;
}
}  // namespace odrive_hardware_interface

PLUGINLIB_EXPORT_CLASS(odrive_hardware_interface::ODriveHardwareInterface, hardware_interface::SystemInterface)
