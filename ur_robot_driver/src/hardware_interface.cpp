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
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \author  Andy Zelenak zelenak@picknik.ai
 * \date    2020-11-9
 *
 */
//----------------------------------------------------------------------
#include <ur_robot_driver/hardware_interface.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <ur_client_library/ur/tool_communication.h>
#include <ur_client_library/exceptions.h>

#include "rclcpp/rclcpp.hpp"

namespace rtde = urcl::rtde_interface;

namespace ur_robot_driver
{
hardware_interface::return_type URPositionHardwareInterface::configure(const HardwareInfo& system_info)
{
  info_ = system_info;

  // initialize
  urcl_joint_positions_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_joint_velocities_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_joint_efforts_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_ft_sensor_measurements_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_tcp_pose_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_position_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' has %d command interfaces found. 2 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"), "Joint '%s' has %d state interface. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }
  }

  status_ = status::CONFIGURED;

  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> URPositionHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &urcl_joint_positions_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &urcl_joint_velocities_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &urcl_joint_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> URPositionHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &urcl_position_commands_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &urcl_velocity_commands_[i]));
  }

  return command_interfaces;
}

return_type URPositionHardwareInterface::start()
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Starting ...please wait...");

  std::this_thread::sleep_for(std::chrono::seconds(2));

  ramp_time_ = stod(info_.hardware_parameters["ramp_time"]);
  rate_ = stod(info_.hardware_parameters["rate"]);

  // The robot's IP address.
  std::string robot_ip = info_.hardware_parameters["robot_ip"];
  // Path to the urscript code that will be sent to the robot
  std::string script_filename = info_.hardware_parameters["script_filename"];
  // Path to the file containing the recipe used for requesting RTDE outputs.
  std::string output_recipe_filename = info_.hardware_parameters["output_recipe_filename"];
  // Path to the file containing the recipe used for requesting RTDE inputs.
  std::string input_recipe_filename = info_.hardware_parameters["input_recipe_filename"];
  // Start robot in headless mode. This does not require the 'External Control' URCap to be running
  // on the robot, but this will send the URScript to the robot directly. On e-Series robots this
  // requires the robot to run in 'remote-control' mode.
  bool headless_mode = static_cast<bool>(stoi(info_.hardware_parameters["headless_mode"]));
  // Port that will be opened to communicate between the driver and the robot controller.
  int reverse_port = stoi(info_.hardware_parameters["reverse_port"]);
  // The driver will offer an interface to receive the program's URScript on this port.
  int script_sender_port = stoi(info_.hardware_parameters["script_sender_port"]);
  //  std::string tf_prefix = info_.hardware_parameters["tf_prefix"];
  std::string tf_prefix;

  // Enables non_blocking_read mode. Should only be used with combined_robot_hw. Disables error generated when read
  // returns without any data, sets the read timeout to zero, and synchronises read/write operations. Enabling this when
  // not used with combined_robot_hw can suppress important errors and affect real-time performance.
  non_blocking_read_ = static_cast<bool>(stoi(info_.hardware_parameters["non_blocking_read"]));

  // Specify gain for servoing to position in joint space.
  // A higher gain can sharpen the trajectory.
  int servoj_gain = stoi(info_.hardware_parameters["servoj_gain"]);
  // Specify lookahead time for servoing to position in joint space.
  // A longer lookahead time can smooth the trajectory.
  double servoj_lookahead_time = stod(info_.hardware_parameters["servoj_lookahead_time"]);

  bool use_tool_communication = static_cast<bool>(stoi(info_.hardware_parameters["use_tool_communication"]));

  // Hash of the calibration reported by the robot. This is used for validating the robot
  // description is using the correct calibration. If the robot's calibration doesn't match this
  // hash, an error will be printed. You can use the robot as usual, however Cartesian poses of the
  // endeffector might be inaccurate. See the "ur_calibration" package on help how to generate your
  // own hash matching your actual robot.
  std::string calibration_checksum = info_.hardware_parameters["kinematics/hash"];

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Initializing driver...");

  try
  {
    {}
  }
  catch (urcl::ToolCommNotAvailable& e)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("URPositionHardwareInterface"), "See parameter use_tool_communication");

    return return_type::ERROR;
  }
  catch (urcl::UrException& e)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("URPositionHardwareInterface"), e.what());
    return return_type::ERROR;
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

template <typename T>
void URPositionHardwareInterface::readData(const std::unique_ptr<rtde::DataPackage>& data_pkg,
                                           const std::string& var_name, T& data)
{
  if (!data_pkg->getData(var_name, data))
  {
    // This throwing should never happen unless misconfigured
    std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
    throw std::runtime_error(error_msg);
  }
}

template <typename T, size_t N>
void URPositionHardwareInterface::readBitsetData(const std::unique_ptr<rtde::DataPackage>& data_pkg,
                                                 const std::string& var_name, std::bitset<N>& data)
{
  if (!data_pkg->getData<T, N>(var_name, data))
  {
    // This throwing should never happen unless misconfigured
    std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
    throw std::runtime_error(error_msg);
  }
}

return_type URPositionHardwareInterface::read()
{

  for (uint i = 0; i < urcl_position_commands_.size(); i++) {
    // Simulate RRBot's movement
    double diff = urcl_position_commands_[i] - urcl_joint_positions_[i];
    double ramp_increment = M_PI/ramp_time_*(1.0/rate_);
    urcl_joint_positions_[i] += std::copysign(std::min(std::abs(diff), ramp_increment), diff );
  }
  return return_type::OK;

}

return_type URPositionHardwareInterface::write()
{

    // create a lambda substract functor
    std::function<double(double, double)> substractor = [](double a, double b) { return std::abs(a - b); };

    // create a position difference vector
    std::vector<double> pos_diff;
    pos_diff.resize(urcl_position_commands_.size());
    std::transform(urcl_position_commands_.begin(), urcl_position_commands_.end(), urcl_position_commands_old_.begin(),
                   pos_diff.begin(), substractor);

    // create a velocity difference vector
    std::vector<double> vel_diff;
    vel_diff.resize(urcl_velocity_commands_.size());
    std::transform(urcl_velocity_commands_.begin(), urcl_velocity_commands_.end(), urcl_velocity_commands_old_.begin(),
                   vel_diff.begin(), substractor);

    double pos_diff_sum = 0.0;
    double vel_diff_sum = 0.0;
    std::for_each(pos_diff.begin(), pos_diff.end(), [&pos_diff_sum](double a) { return pos_diff_sum += a; });
    std::for_each(vel_diff.begin(), vel_diff.end(), [&vel_diff_sum](double a) { return vel_diff_sum += a; });

    if (pos_diff_sum != 0.0)
    {
      {}
    }
    else if (vel_diff_sum != 0.0)
    {
      {}
    }
    else
    {
      {}
    }

    packet_read_ = false;

    // remember old values
    urcl_position_commands_old_ = urcl_position_commands_;
    urcl_velocity_commands_old_ = urcl_velocity_commands_;

    return return_type::OK;

}

void URPositionHardwareInterface::handleRobotProgramState(bool program_running)
{
  if (!robot_program_running_ && program_running)
  {
    // TODO how to set controller reset flag
  }
  robot_program_running_ = program_running;
}
}  // namespace ur_robot_driver
