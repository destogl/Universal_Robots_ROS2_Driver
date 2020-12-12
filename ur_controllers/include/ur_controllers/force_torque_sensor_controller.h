//
// Created by livanov93 on 12/12/20.
//

#ifndef UR_CONTROLLERS_FORCE_TORQUE_SENSOR_CONTROLLER_H
#define UR_CONTROLLERS_FORCE_TORQUE_SENSOR_CONTROLLER_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include <controller_interface/controller_interface.hpp>

namespace ur_controllers {

    class ForceTorqueStateController: public controller_interface::ControllerInterface{


    public:
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update() override;



    };
} // namespace ur_controllers


#endif //UR_CONTROLLERS_FORCE_TORQUE_SENSOR_CONTROLLER_H
