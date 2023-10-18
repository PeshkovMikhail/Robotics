#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
	MinimalPublisher()
	: Node("minimal_publisher"), count_(0)
	{
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
		timer_ = this->create_wall_timer(
		  500ms, std::bind(&MinimalPublisher::timer_callback, this));
	}

private:
	void timer_callback()
	{
		auto message = geometry_msgs::msg::Twist();
		message.linear.set__x(-5); // for some reason linear control must be reversed
        message.angular.set__z(6.28); // full circle
		publisher_->publish(message);
	}
	rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        size_t count_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalPublisher>());
	rclcpp::shutdown();
	return 0;
}
