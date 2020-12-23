// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
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
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andy Zelenak zelenak@picknik.ai
 * \date    2020-11-9
 *
 */
//----------------------------------------------------------------------
#include <ur_robot_driver/hardware_interface.h>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <ur_client_library/ur/tool_communication.h>
#include <ur_client_library/exceptions.h>

#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <math.h>

namespace ur_robot_driver
{
hardware_interface::return_type URPositionHardwareInterface::configure(const HardwareInfo& system_info)
{
  info_ = system_info;

  ramp_time_ = stod(info_.hardware_parameters["ramp_time"]);
  rate_ = stod(info_.hardware_parameters["rate"]);

  // resize and initialize
  commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  force_torque_.resize(info_.sensors[0].state_interfaces.size(), 8.0);
  // TODO all the checking from HardwareInfo which holds urdf info
  //  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  //  {
  //    if (joint.type.compare("ros2_control_components/PositionJoint") != 0)
  //    {
  //      status_ = status::UNKNOWN;
  //      return return_type::ERROR;
  //    }
  //  }

  // TODO fetch parameters (robot_ip, write&read params, ...), this can also be done in start

  status_ = status::CONFIGURED;

  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> URPositionHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
            hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &states_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));

    state_interfaces.emplace_back(
            hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
                                               &joint_efforts_[i]));
  }

  for (uint i = 0; i < info_.sensors.size(); i++){

    for (uint j = 0; j< info_.sensors[i].state_interfaces.size(); ++j) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &force_torque_[i]));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> URPositionHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &commands_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}

return_type URPositionHardwareInterface::start()
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Starting ...please wait...");

  std::this_thread::sleep_for(std::chrono::seconds(2));

  for (uint i = 0; i < states_.size(); i++)
  {
    if (std::isnan(states_[i]) || std::isnan(commands_[i]))
    {
      states_[i] = 0;
      commands_[i] = 0;
    }
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System successfully started!");

  return return_type::OK;
}

return_type URPositionHardwareInterface::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Stopping ...please wait...");

  std::this_thread::sleep_for(std::chrono::seconds(2));

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System successfully stopped!");

  return return_type::OK;
}

return_type URPositionHardwareInterface::read()
{

  for (uint i = 0; i < states_.size(); i++) {
    // Simulate RRBot's movement
    double diff = commands_[i] - states_[i];
    double ramp_increment = M_PI/ramp_time_*(1.0/rate_);
    states_[i] += std::copysign(std::min(std::abs(diff), ramp_increment), diff );
  }
    return return_type::OK;

}

return_type URPositionHardwareInterface::write()
{
  return return_type::OK;

}

  URPositionHardwareInterface::URPositionHardwareInterface(): SystemInterface() {

      status_ = status::CONFIGURED;

  }

}  // namespace ur_robot_driver
