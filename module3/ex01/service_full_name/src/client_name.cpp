#include "rclcpp/rclcpp.hpp"
#include "names/srv/full_name_sum_service.hpp"

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if(argc != 3) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: client_name name last_name");
        return 1;
    }

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("summ_full_name_client");
    rclcpp::Client<names::srv::FullNameSumService>::SharedPtr client = 
        node->create_client<names::srv::FullNameSumService>("summ_full_name");
    
    auto request = std::make_shared<names::srv::FullNameSumService::Request>();
    request->name = argv[1];
    request->last_name = argv[2];


    while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %s", result.get()->full_name.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service summ_full_name");
  }

  rclcpp::shutdown();
  return 0;
}