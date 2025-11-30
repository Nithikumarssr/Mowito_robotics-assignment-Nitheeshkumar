#include "rclcpp/rclcpp.hpp"
#include "ros_comms/srv/add_two_ints.hpp"  // Custom generated header

void add(const std::shared_ptr<ros_comms::srv::AddTwoInts::Request> request,
         std::shared_ptr<ros_comms::srv::AddTwoInts::Response> response) {
  response->sum = request->a + request->b;
  response->success = true;
  response->message = "Addition completed successfully";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp_components_node"), "Incoming request\na: %d b: %d", 
              request->a, request->b);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("add_two_ints_server");
  auto service = node->create_service<ros_comms::srv::AddTwoInts>("add_two_ints", &add);
  RCLCPP_INFO(node->get_logger(), "Custom Add Two Ints Server started. Waiting for requests...");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

