//
// Created by root on 12/12/20.
//

#include "ur_controllers/force_torque_sensor_controller.h"





#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        ur_controllers::ForceTorqueStateController, controller_interface::ControllerInterface)

controller_interface::InterfaceConfiguration
ur_controllers::ForceTorqueStateController::command_interface_configuration() const {
    return controller_interface::InterfaceConfiguration();
}

controller_interface::InterfaceConfiguration
ur_controllers::ForceTorqueStateController::state_interface_configuration() const {
    return controller_interface::InterfaceConfiguration();
}

controller_interface::return_type ur_controllers::ForceTorqueStateController::update() {
    return ERROR;
}
