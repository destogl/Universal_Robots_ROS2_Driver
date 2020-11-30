# Universal Robots ROS2 Driver

Alpha version of the ROS2 Universal Robots driver. Should be transferred to the Universal Robots org when ready.

## Build Instructions

To build this package in a ROS Foxy workspace, clone this repo
into the `src/` directory, then:

```
# Clone source-based dependencies into src/ directory
vcs import --skip-existing --input src/Universal_Robots_ROS2_Driver/.repos.yaml src

# Install package-based dependencies
rosdep install -y --rosdistro $ROS_DISTRO --ignore-src --from-paths src

# Edit file with errors
sed -i '1i # include <algorithm> \n# include <math.h>' /${HOME}/catkin_ws/src/ros2_control/test_robot_hardware/src/test_force_torque_sensor.cpp

# Build sources
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select control_msgs ros2_control_components test_robot_hardware hardware_interface controller_manager controller_interface controller_manager_msgs ros2controlcli  forward_command_controller ur_client_library ur_dashboard_msgs ur_robot_driver ur_description realtime_tools joint_state_controller
```