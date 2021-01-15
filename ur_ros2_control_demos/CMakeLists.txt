cmake_minimum_required(VERSION 3.5)
project(ur_ros2_control_demos)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable to publish fake servo commands for testing/demo purposes
set(SERVO_CMDS_NAME servo_command_publisher)
add_executable(${SERVO_CMDS_NAME} src/servo_command_publisher.cpp)
ament_target_dependencies(${SERVO_CMDS_NAME}
        rclcpp
        geometry_msgs
        std_srvs
        )

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

# Install Libraries
install(
        TARGETS
        ${SERVO_CMDS_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
