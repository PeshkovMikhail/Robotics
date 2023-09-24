#include "rclcpp/rclcpp.hpp"
#include "names/srv/full_name_sum_service.hpp"

#include <memory>

void summ(const std::shared_ptr<names::srv::FullNameSumService::Request> request,
            std::shared_ptr<names::srv::FullNameSumService::Response> response) {
    response->full_name = request->name + " " + request->last_name;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nname: %s" " last_name: %s",
                request->name.c_str(), request->last_name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->full_name.c_str());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("summ_full_name_server");
    rclcpp::Service<names::srv::FullNameSumService>::SharedPtr service =
        node->create_service<names::srv::FullNameSumService>("summ_full_name", &summ);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to sum full name");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}