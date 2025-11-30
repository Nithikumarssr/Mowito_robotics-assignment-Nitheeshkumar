#include "rclcpp/rclcpp.hpp"
#include "ros_comms/srv/add_two_ints.hpp"  // Custom generated header
#include <memory>
#include <chrono>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("add_two_ints_client");
  auto client = node->create_client<ros_comms::srv::AddTwoInts>("add_two_ints");

  // Wait for service
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service. Exiting.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
  }

  auto request = std::make_shared<ros_comms::srv::AddTwoInts::Request>();
  request->a = 5;
  request->b = 3;

  auto result_future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->success) {
      RCLCPP_INFO(node->get_logger(), "Result of %d + %d = %d (Message: %s)", 
                  request->a, request->b, result->sum, result->message.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), "Service failed: %s", result->message.c_str());
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}

