// ROS
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("servo_command_publisher");

  // ROS objects
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<rclcpp::Node>("servo_command_publisher", node_options);

  // Call the start service to init and start the servo component
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client =
      node->create_client<std_srvs::srv::Trigger>("/start_servo");
  servo_start_client->wait_for_service(1s);
  servo_start_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

  std::this_thread::sleep_for(std::chrono::seconds(2));
  // Create a publisher for publishing the jog commands
  auto pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("servo_server/delta_twist_cmds", 10);
  std::weak_ptr<std::remove_pointer<decltype(pub.get())>::type> captured_pub = pub;
  std::weak_ptr<std::remove_pointer<decltype(node.get())>::type> captured_node = node;
  auto callback = [captured_pub, captured_node]() -> void {
    auto pub_ptr = captured_pub.lock();
    auto node_ptr = captured_node.lock();
    if (!pub_ptr || !node_ptr)
    {
      return;
    }
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_ptr->now();
    msg->header.frame_id = "tool0";
    msg->twist.linear.z = -0.08;
//    msg->twist.angular.z = -0.5;
    pub_ptr->publish(std::move(msg));
  };
  auto timer = node->create_wall_timer(2ms, callback);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
