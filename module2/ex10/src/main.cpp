#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace geometry_msgs;
using std::placeholders::_1;

void fill_zero_vector3(msg::Vector3& vector) {
	vector.set__x(0);
	vector.set__y(0);
	vector.set__z(0);
}

class TurtleController : public rclcpp::Node
{
public:
	TurtleController()
	: Node("turtle_controller")
	{
		publisher_ = this->create_publisher<msg::Twist>("/turtle1/cmd_vel", 10);
		subscription_ = this->create_subscription<std_msgs::msg::String>("/cmd_text", 10, std::bind(&TurtleController::topic_callback, this, _1));
	}
private:
	void topic_callback(const std_msgs::msg::String & msg) const {
		auto message = msg::Twist();
		auto linear = msg::Vector3();
		fill_zero_vector3(linear);
		auto angular = msg::Vector3();
		fill_zero_vector3(angular);


		if(strcmp("turn_right", msg.data.c_str())==0){
			angular.set__z(-1.5);
		}
		else if(strcmp("turn_left", msg.data.c_str())==0) {
			angular.set__z(1.5);
		}
		else if(strcmp("move_forward", msg.data.c_str())==0) {
			std::cout << "forward\r\n";
			linear.set__x(1);
		}
		else if(strcmp("move_backward", msg.data.c_str())==0){
			linear.set__x(-1);
			std::cout << "backward\r\n";
		}
		else {
			std::cerr << "unexpected cmd: " << msg.data <<std::endl;
		}

		message.set__angular(angular);
		message.set__linear(linear);
		publisher_->publish(message);
	}

	rclcpp::Publisher<msg::Twist>::SharedPtr publisher_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TurtleController>());
	rclcpp::shutdown();
	return 0;
}
