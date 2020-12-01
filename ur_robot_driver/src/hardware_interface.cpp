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

namespace rtde = urcl::rtde_interface;

namespace ur_robot_driver
{
hardware_interface::return_type URPositionHardwareInterface::configure(const HardwareInfo& system_info)
{
  info_ = system_info;

  // resize and initialize
  commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

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

//  urcl_joint_positions_ = {100,200,300,400,500,600};
    urcl_joint_positions_.at(0) = 100.0;
    urcl_joint_positions_.at(1) = 200.0;
    urcl_joint_positions_.at(2) = 300.0;
    urcl_joint_positions_.at(3) = 400.0;
    urcl_joint_positions_.at(4) = 500.0;
    urcl_joint_positions_.at(5) = 600.0;

  status_ = status::CONFIGURED;

  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> URPositionHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &states_[i]));
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
  }

  return command_interfaces;
}

return_type URPositionHardwareInterface::start()
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Starting ...please wait...");

  std::this_thread::sleep_for(std::chrono::seconds(2));

  // TODO initialize driver
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Initializing dashboard client");

  // The robot's IP address.
  std::string robot_ip_("192.168.0.3");
  // Path to the urscript code that will be sent to the robot
  std::string script_filename("script_filename");
  // Path to the file containing the recipe used for requesting RTDE outputs.
  std::string output_recipe_filename("output_recipe_filename");
  // Path to the file containing the recipe used for requesting RTDE inputs.
  std::string input_recipe_filename("input_recipe_filename");
  // Start robot in headless mode. This does not require the 'External Control' URCap to be running
  // on the robot, but this will send the URScript to the robot directly. On e-Series robots this
  // requires the robot to run in 'remote-control' mode.
  bool headless_mode = false;
  // Port that will be opened to communicate between the driver and the robot controller.
  int reverse_port = 50001;
  // The driver will offer an interface to receive the program's URScript on this port.
  int script_sender_port = 50002;
  std::string tf_prefix_("");
  // Enables non_blocking_read mode. Should only be used with combined_robot_hw. Disables error generated when read
  // returns without any data, sets the read timeout to zero, and synchronises read/write operations. Enabling this when
  // not used with combined_robot_hw can suppress important errors and affect real-time performance.
  bool non_blocking_read_ = false;

  // Specify gain for servoing to position in joint space.
  // A higher gain can sharpen the trajectory.
  int servoj_gain = 2000;

  // Specify lookahead time for servoing to position in joint space.
  // A longer lookahead time can smooth the trajectory.
  double servoj_lookahead_time = 0.03;

  // Whenever the runtime state of the "External Control" program node in the UR-program changes, a
  // message gets published here. So this is equivalent to the information whether the robot accepts
  // commands from ROS side.
  //  program_state_pub_ = robot_hw_nh.advertise<std_msgs::Bool>("robot_program_running", 10, true);

  bool use_tool_communication = false;
  std::unique_ptr<urcl::ToolCommSetup> tool_comm_setup;
  if (use_tool_communication)
  {
    tool_comm_setup = std::make_unique<urcl::ToolCommSetup>();
  }

  // Hash of the calibration reported by the robot. This is used for validating the robot
  // description is using the correct calibration. If the robot's calibration doesn't match this
  // hash, an error will be printed. You can use the robot as usual, however Cartesian poses of the
  // endeffector might be inaccurate. See the "ur_calibration" package on help how to generate your
  // own hash matching your actual robot.
  std::string calibration_checksum;

  try
  {
    ur_driver_ = std::make_unique<urcl::UrDriver>(
        robot_ip_, script_filename, output_recipe_filename, input_recipe_filename,
        std::bind(&URPositionHardwareInterface::handleRobotProgramState, this, std::placeholders::_1), headless_mode,
        std::move(tool_comm_setup), calibration_checksum, (uint32_t)reverse_port, (uint32_t)script_sender_port,
        servoj_gain, servoj_lookahead_time, non_blocking_read_);
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

  // TODO initialize dashboard client

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Initializing dashboard client");
  // this node will die so Dashboard Client ROS won't be publishing anything
  rclcpp::Node::SharedPtr dashboard_nh = std::make_shared<rclcpp::Node>("URPositionHardwareInterface", "dashboard");
  dashboard_client_ = std::make_unique<DashboardClientROS>(dashboard_nh, robot_ip_);

  // set some default values
  // TODO replace with reading current state of the joints
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

  // TODO stop/reset driver
  ur_driver_.reset();

  // TODO stop/reset dashboard client
  dashboard_client_.reset();

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
  // TODO add receiving commands from driver

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Reading ...");

  std::unique_ptr<rtde::DataPackage> data_pkg = ur_driver_->getDataPackage();
  if (data_pkg)
  {
    packet_read_ = true;
    readData(data_pkg, "actual_q", urcl_joint_positions_);

    memcpy(&states_[0], &urcl_joint_positions_[0], 6* sizeof(double));

//    for (size_t i=0; i<6;i++) {
//        RCLCPP_INFO_STREAM(rclcpp::get_logger("URPositionHardwareInterface"), "Joint " << i + 1 << " = " << states_[i]);
//    }

    return return_type::OK;
  }

  return return_type::ERROR;
}

return_type URPositionHardwareInterface::write()
{
  // TODO send commands_ to driver
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Writing ...");
  return return_type::OK;

  for (uint i = 0; i < info_.joints.size(); i++)
    urcl_position_commands_[i] = commands_[i];

  ur_driver_->writeJointCommand(urcl_position_commands_, urcl::comm::ControlMode::MODE_SERVOJ);

  packet_read_ = false;

  return return_type::OK;
}

void URPositionHardwareInterface::handleRobotProgramState(bool program_running)
{
  if (program_running)
  {
    RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Robot receives commands from ROS side");
  }
}
}  // namespace ur_robot_driver
